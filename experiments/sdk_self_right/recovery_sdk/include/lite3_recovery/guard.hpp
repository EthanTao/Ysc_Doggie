#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>

namespace lite3_recovery {

using Millis = std::uint64_t;
constexpr Millis command_ttl = 20;
constexpr Millis feedback_ttl = 200;
constexpr std::size_t joint_count = 12;

enum class StopReason {
    none, handset, cancel, stall, feedback, fault, timeout, command_expired,
    transport, clock, ownership
};
enum class Output { none, motion, damping };

struct Joint {
    double position{}, velocity{}, kp{}, kd{}, feedforward{};
};
using Joints = std::array<Joint, joint_count>;
using Axis = std::array<double, joint_count>;

struct Bounds {
    Axis min_position{}, max_position{}, max_speed{}, max_kp{}, max_kd{}, max_effort{};
    Axis max_acceleration{}, max_gain_rate{};
};

inline bool within(double value, double lower, double upper) {
    return std::isfinite(value) && value >= lower && value <= upper;
}

inline bool valid_bounds(const Bounds& b) {
    for (std::size_t i = 0; i < joint_count; ++i) {
        if (!std::isfinite(b.min_position[i]) || !std::isfinite(b.max_position[i])
            || !(b.min_position[i] < b.max_position[i])) return false;
        for (double bound : {b.max_speed[i], b.max_kp[i], b.max_kd[i], b.max_effort[i],
                             b.max_acceleration[i], b.max_gain_rate[i]}) {
            if (!std::isfinite(bound) || bound <= 0.) return false;
        }
    }
    return true;
}

inline bool valid_motion(const Joints& joints, const Bounds& b) {
    for (std::size_t i = 0; i < joint_count; ++i) {
        const auto& j = joints[i];
        if (!within(j.position, b.min_position[i], b.max_position[i])
            || !within(j.velocity, -b.max_speed[i], b.max_speed[i])
            || !within(j.kp, 0., b.max_kp[i]) || !within(j.kd, 0., b.max_kd[i])
            || !within(j.feedforward, 0., 0.)) return false;
    }
    return true;
}

struct Frame {
    std::uint64_t session{}, sequence{};
    Millis source_ms{};
    Joints joints{};
};

struct Commissioning {
    bool exclusive_authority{};
    bool handset_verified{};
    bool trajectory_reviewed{};
    bool damping_verified{};
};

struct Evidence {
    StopReason reason{StopReason::none};
    Millis stop_ms{}, last_motion_ms{}, first_damping_ms{};
    bool motion_sent{}, damping_sent{};
};

inline bool fresh(Millis now, Millis source, Millis ttl) {
    return source <= now && now-source < ttl;
}

// All actual sends must occur inside tick's synchronous, non-blocking sink.
// Serializing the STOP latch and the transport boundary prevents late cached
// trajectories from escaping after the latch. Sinks must not reenter the guard.
class Guard {
public:
    bool arm(std::uint64_t session, Millis now, const Commissioning& checks,
             bool durable_reservation, const Bounds& bounds, const Axis& damping) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (session_ || evidence_.reason != StopReason::none || !session
            || !durable_reservation || !checks.exclusive_authority || !checks.handset_verified
            || !checks.trajectory_reviewed || !checks.damping_verified || !valid_bounds(bounds))
            return false;
        Joints candidate{};
        for (std::size_t i = 0; i < joint_count; ++i) {
            if (!within(damping[i], 0., bounds.max_kd[i]) || damping[i] <= 0.) return false;
            candidate[i].kd = damping[i];
        }
        damping_ = candidate;
        session_ = session;
        arm_ms_ = clock_ms_ = now;
        bounds_ = bounds;
        return true;
    }

    // Supply a counter and timestamp from the hardware receive callback, not
    // from the DDS timer that republishes the last received data.
    bool feedback(std::uint64_t sequence, Millis received, Millis now,
                  const Axis& position, const Axis& velocity) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!clock_ok(now) || !session_ || !sequence || sequence <= feedback_sequence_
            || (feedback_sequence_ && received <= feedback_ms_)
            || !fresh(now, received, feedback_ttl)) return false;
        // A frame arriving after the previous hardware deadline cannot
        // make a missed watchdog interval retroactively healthy.
        if (feedback_sequence_ && !fresh(now, feedback_ms_, feedback_ttl)) {
            latch(StopReason::feedback, now);
            return false;
        }
        for (std::size_t i=0; i<joint_count; ++i) {
            if (!std::isfinite(position[i]) || !std::isfinite(velocity[i])) {
                latch(StopReason::feedback, now);
                return false;
            }
        }
        actual_q_ = position;
        actual_v_ = velocity;
        feedback_sequence_ = sequence;
        feedback_ms_ = received;
        return true;
    }

    bool accept(const Frame& frame, Millis now) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!clock_ok(now) || !session_ || evidence_.reason != StopReason::none) return false;
        expire(now);
        if (evidence_.reason != StopReason::none || frame.session != session_
            || frame.sequence <= sequence_ || frame.source_ms < arm_ms_
            || (sequence_ && frame.source_ms <= frame_.source_ms)
            || !fresh(now, frame.source_ms, command_ttl)) return false;
        if (!feedback_sequence_ || !valid_motion(frame.joints, bounds_)
            || (sequence_ && !rate_safe(frame))) {
            latch(StopReason::fault, now);
            return false;
        }
        frame_ = frame;
        sequence_ = frame.sequence;
        return true;
    }

    void stop(StopReason reason, Millis now) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (clock_ok(now) && reason != StopReason::none) latch(reason, now);
    }

    // Authority loss inhibits all SDK output. Reacquisition is a separate
    // commissioning operation, never an implicit response to an abort.
    template<class Sink>
    Output tick(Millis now, bool owns_control, Sink&& send) {
        std::lock_guard<std::mutex> lock(mutex_);
        clock_ok(now);
        if (!session_) return Output::none;
        if (!owns_control) {
            latch(StopReason::ownership, clock_ms_);
            authority_lost_ = true;
        }
        if (authority_lost_) return Output::none;
        expire(clock_ms_);
        if (evidence_.reason == StopReason::none && sequence_ && !effort_safe())
            latch(StopReason::fault, clock_ms_);
        if (evidence_.reason == StopReason::none) {
            if (!sequence_) return Output::none;
            if (send(Output::motion, frame_.joints)) {
                evidence_.motion_sent = true;
                evidence_.last_motion_ms = clock_ms_;
                return Output::motion;
            }
            latch(StopReason::transport, clock_ms_);
        }
        if (send(Output::damping, damping_)) {
            if (!evidence_.damping_sent) evidence_.first_damping_ms = clock_ms_;
            evidence_.damping_sent = true;
            return Output::damping;
        }
        return Output::none;
    }

    Evidence evidence() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return evidence_;
    }

private:
    bool effort_safe() const {
        for (std::size_t i=0; i<joint_count; ++i) {
            const auto& j=frame_.joints[i];
            const double effort=j.kp*(j.position-actual_q_[i])+j.kd*(j.velocity-actual_v_[i]);
            if (!within(effort,-bounds_.max_effort[i],bounds_.max_effort[i])) return false;
        }
        return true;
    }

    bool rate_safe(const Frame& next) const {
        const double seconds=(next.source_ms-frame_.source_ms)/1000.;
        constexpr double roundoff=1e-9;
        for (std::size_t i=0; i<joint_count; ++i) {
            const auto& a=frame_.joints[i];
            const auto& b=next.joints[i];
            if (std::abs(a.position-b.position)>bounds_.max_speed[i]*seconds+roundoff
                || std::abs(a.velocity-b.velocity)>bounds_.max_acceleration[i]*seconds+roundoff
                || std::abs(a.kp-b.kp)>bounds_.max_gain_rate[i]*seconds+roundoff
                || std::abs(a.kd-b.kd)>bounds_.max_gain_rate[i]*seconds+roundoff) return false;
        }
        return true;
    }

    void latch(StopReason reason, Millis now) {
        if (evidence_.reason != StopReason::none) return;
        evidence_.reason = reason;
        evidence_.stop_ms = now;
        frame_ = {};
    }

    bool clock_ok(Millis now) {
        if (now < clock_ms_) {
            latch(StopReason::clock, clock_ms_);
            return false;
        }
        clock_ms_ = now;
        return true;
    }

    void expire(Millis now) {
        if (evidence_.reason != StopReason::none) return;
        if (!fresh(now, sequence_ ? frame_.source_ms : arm_ms_, command_ttl)) {
            latch(StopReason::command_expired, now);
        } else if ((!feedback_sequence_ && sequence_)
                   || (feedback_sequence_ && !fresh(now, feedback_ms_, feedback_ttl))) {
            latch(StopReason::feedback, now);
        }
    }

    mutable std::mutex mutex_;
    Bounds bounds_{};
    std::uint64_t session_{}, sequence_{}, feedback_sequence_{};
    Millis arm_ms_{}, clock_ms_{}, feedback_ms_{};
    Frame frame_{};
    Joints damping_{};
    Axis actual_q_{}, actual_v_{};
    Evidence evidence_{};
    bool authority_lost_{};
};

} // namespace lite3_recovery
