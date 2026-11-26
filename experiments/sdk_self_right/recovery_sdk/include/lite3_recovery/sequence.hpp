#pragma once

#include "lite3_recovery/guard.hpp"
#include <deque>
#include <optional>
#include <vector>

namespace lite3_recovery {

using Quaternion = std::array<double, 4>;
constexpr double rad_to_deg = 57.29577951308232;

struct Feedback {
    std::uint64_t sequence{};
    Millis received_ms{};
    Axis position{}, velocity{};
    Quaternion attitude{}; // x, y, z, w
};

inline bool unit(const Quaternion& q) {
    double n{};
    for (double v : q) n += v*v;
    return within(n, .996, 1.004);
}

inline double tilt(const Quaternion& q) {
    return rad_to_deg*std::acos(std::clamp(1.-2.*(q[0]*q[0]+q[1]*q[1]), -1., 1.));
}

inline double attitude_distance(const Quaternion& a, const Quaternion& b) {
    double dot{}, an{}, bn{};
    for (std::size_t i=0; i<a.size(); ++i) {
        dot += a[i]*b[i];
        an += a[i]*a[i];
        bn += b[i]*b[i];
    }
    return 2.*rad_to_deg*std::acos(std::clamp(std::abs(dot)/std::sqrt(an*bn), 0., 1.));
}

inline bool valid_feedback(const Feedback& sample, Millis now) {
    if (!sample.sequence || !fresh(now, sample.received_ms, feedback_ttl) || !unit(sample.attitude))
        return false;
    for (std::size_t i=0; i<joint_count; ++i)
        if (!std::isfinite(sample.position[i]) || !std::isfinite(sample.velocity[i])) return false;
    return true;
}

struct StallLimits {
    Millis hold_ms{};
    double motion_rad{}, speed_rad_s{}, attitude_deg{};
    bool valid() const {
        return hold_ms >= 200 && hold_ms <= 2000
            && std::isfinite(motion_rad) && motion_rad > 0
            && std::isfinite(speed_rad_s) && speed_rad_s > 0
            && std::isfinite(attitude_deg) && attitude_deg > 0;
    }
};

class StallWindow {
public:
    void reset() { samples_.clear(); }

    bool stalled(const Feedback& sample, const StallLimits& limits) {
        if (!samples_.empty()) {
            if (sample.sequence <= samples_.back().sequence
                || sample.received_ms <= samples_.back().received_ms) return false;
            if (!fresh(sample.received_ms, samples_.back().received_ms, feedback_ttl)) reset();
        }
        samples_.push_back(sample);
        while (samples_.size()>1 && sample.received_ms-samples_[1].received_ms >= limits.hold_ms)
            samples_.pop_front();
        if (sample.received_ms-samples_.front().received_ms < limits.hold_ms) return false;
        Axis lo=samples_.front().position, hi=lo;
        for (const auto& point : samples_) {
            if (attitude_distance(samples_.front().attitude, point.attitude)>limits.attitude_deg)
                return false;
            for (std::size_t i=0; i<joint_count; ++i) {
                if (std::abs(point.velocity[i])>limits.speed_rad_s) return false;
                lo[i]=std::min(lo[i],point.position[i]);
                hi[i]=std::max(hi[i],point.position[i]);
                if (hi[i]-lo[i]>limits.motion_rad) return false;
            }
        }
        return true;
    }

private:
    std::deque<Feedback> samples_; // ≤2002 samples, increasing ms, ≤2 s window
};

enum class Phase { disabled, watching, rolling, prone_check, standing, upright_hold, stopped };

struct Status {
    Phase phase{};
    StopReason stop{StopReason::none};
    bool fall_detected{};
    Millis fall_detected_ms{};
};

struct SequenceConfig {
    double fall_tilt_deg{150.};
    double supine_tilt_deg{120.};
    Millis detect_dwell_ms{500};
    Millis stable_dwell_ms{5000};
    bool allow_prone_shortcut{false};
};

struct Input {
    Feedback feedback{};
    StopReason stop{StopReason::none};
    bool exclusive_authority{};
    bool fall_allowed{}; // Approved initial pose and controller readiness.
    bool expects_progress{};
    bool roll_complete{};
    bool prone_confirmed{}; // Must corroborate leg support, not just IMU level.
    bool stand_complete{};
};

class Sequence {
public:
    explicit Sequence(StallLimits limits, SequenceConfig config = {})
        : limits_(limits), config_(config) {}

    bool enable(const Commissioning& checks, bool durable_reservation) {
        if (phase_ != Phase::disabled || !durable_reservation || !limits_.valid()
            || !checks.exclusive_authority || !checks.handset_verified
            || !checks.trajectory_reviewed || !checks.damping_verified) return false;
        phase_ = Phase::watching;
        return true;
    }

    Status tick(Millis now, const Input& in) {
        if (phase_ == Phase::disabled || phase_ == Phase::stopped)
            return {phase_, reason_, fall_detected_, fall_detected_ms_};
        if (in.stop != StopReason::none) return stop(in.stop);
        if (now < last_now_) return stop(StopReason::clock);
        last_now_=now;
        if (!in.exclusive_authority) return stop(StopReason::ownership);
        if (!valid_feedback(in.feedback, now)) {
            if (phase_ != Phase::watching) return stop(StopReason::feedback);
            base_.reset();
            detect_start_ms_.reset();
            fall_detected_ = false;
            fall_detected_ms_ = 0;
            return {phase_, StopReason::none, fall_detected_, fall_detected_ms_};
        }
        const auto& sample=in.feedback;
        if (sample.sequence < last_sequence_
            || (sample.sequence > last_sequence_ && last_sequence_ && sample.received_ms <= last_rx_))
            return stop(StopReason::feedback);
        if (sample.sequence == last_sequence_ && sample.received_ms != last_rx_)
            return stop(StopReason::feedback);
        const bool new_sample = sample.sequence > last_sequence_;
        const bool gap = last_sequence_ && !fresh(sample.received_ms, last_rx_, feedback_ttl);
        if (new_sample) {
            last_sequence_=sample.sequence;
            last_rx_=sample.received_ms;
        }
        if (gap) {
            if (phase_ != Phase::watching) return stop(StopReason::feedback);
            base_.reset();
            detect_start_ms_.reset();
            fall_detected_ = false;
            fall_detected_ms_ = 0;
        }
        if (phase_ == Phase::watching) {
            const double sample_tilt = tilt(sample.attitude);
            const bool tilted_for_fall = sample_tilt >= config_.fall_tilt_deg
                || (config_.allow_prone_shortcut && in.prone_confirmed && in.fall_allowed);
            if (!in.fall_allowed || !tilted_for_fall) {
                base_.reset();
                detect_start_ms_.reset();
                fall_detected_ = false;
                fall_detected_ms_ = 0;
            } else if (new_sample) {
                if (!detect_start_ms_) detect_start_ms_ = sample.received_ms;
                if (!fall_detected_ && sample.received_ms - *detect_start_ms_ >= config_.detect_dwell_ms) {
                    fall_detected_ = true;
                    fall_detected_ms_ = sample.received_ms;
                }
                if (!base_ || attitude_distance(base_->attitude, sample.attitude)>5.) base_=sample;
                if (sample.received_ms-base_->received_ms >= config_.stable_dwell_ms) {
                    if (config_.allow_prone_shortcut && sample_tilt < config_.supine_tilt_deg) {
                        transition(Phase::prone_check, now);
                    } else {
                        transition(Phase::rolling, now);
                    }
                }
            }
            return {phase_, StopReason::none, fall_detected_, fall_detected_ms_};
        }
        if ((phase_==Phase::rolling || phase_==Phase::prone_check || phase_==Phase::standing)
            && now-attempt_ms_>=10000) return stop(StopReason::timeout);
        if (!in.expects_progress || phase_==Phase::prone_check || phase_==Phase::upright_hold)
            stall_.reset();
        else if (new_sample && stall_.stalled(sample, limits_)) return stop(StopReason::stall);
        if (phase_==Phase::rolling && in.roll_complete) transition(Phase::prone_check, now);
        if (phase_==Phase::prone_check && new_sample) {
            if (!in.prone_confirmed || tilt(sample.attitude)>20.) base_.reset();
            else {
                if (!base_ || attitude_distance(base_->attitude,sample.attitude)>5.) base_=sample;
                if (sample.received_ms-base_->received_ms>=500) transition(Phase::standing, now);
            }
        } else if (phase_==Phase::standing && in.stand_complete && tilt(sample.attitude)<=20.) {
            transition(Phase::upright_hold, now);
        }
        return {phase_, StopReason::none, fall_detected_, fall_detected_ms_};
    }

private:
    void transition(Phase next, Millis now) {
        if (next==Phase::rolling || (phase_==Phase::watching && next==Phase::prone_check))
            attempt_ms_=now;
        phase_=next;
        stall_.reset();
        base_.reset();
    }

    Status stop(StopReason why) {
        phase_=Phase::stopped;
        reason_=why;
        stall_.reset();
        return {phase_,reason_,fall_detected_,fall_detected_ms_};
    }

    StallLimits limits_;
    SequenceConfig config_{};
    Phase phase_{Phase::disabled};
    StopReason reason_{StopReason::none};
    Millis attempt_ms_{}, last_now_{}, last_rx_{};
    std::uint64_t last_sequence_{};
    std::optional<Feedback> base_;
    std::optional<Millis> detect_start_ms_;
    bool fall_detected_{};
    Millis fall_detected_ms_{};
    StallWindow stall_;
};

struct Segment {
    Millis duration_ms{};
    Joints target{};
    bool expects_progress{};
};

// Quintic time scaling gives continuous position/velocity/acceleration at each
// segment boundary. Input endpoints are equilibrium poses, not replays of telemetry.
inline bool valid_segment(const Joints& start, const Segment& segment, const Bounds& b) {
    if (segment.duration_ms < 5 || segment.duration_ms>10000 || segment.duration_ms%5
        || !valid_motion(start,b) || !valid_motion(segment.target,b)) return false;
    const double s=segment.duration_ms/1000.;
    for (std::size_t i=0; i<joint_count; ++i) {
        const auto& a=start[i];
        const auto& z=segment.target[i];
        const double distance=std::abs(z.position-a.position);
        if (a.velocity!=0. || z.velocity!=0. || 1.875*distance/s>b.max_speed[i]
            || (10./std::sqrt(3.))*distance/(s*s)>b.max_acceleration[i]
            || 1.875*std::max(std::abs(z.kp-a.kp),std::abs(z.kd-a.kd))/s>b.max_gain_rate[i]
            || (!segment.expects_progress && distance>0.)) return false;
    }
    return true;
}

inline bool valid_trajectory(const Joints& start, const std::vector<Segment>& segments, const Bounds& b) {
    if (segments.empty() || !valid_bounds(b)) return false;
    auto last=start;
    Millis total{};
    bool rolls=false;
    for (const auto& segment : segments) {
        if (!valid_segment(last,segment,b)) return false;
        total+=segment.duration_ms;
        if (total>=9500) return false; // Leave time for the prone gate.
        rolls |= segment.expects_progress;
        last=segment.target;
    }
    return rolls;
}

// Called only with a segment that passed valid_segment.
inline Joints interpolate(const Joints& start, const Segment& segment, Millis elapsed_ms) {
    const double u=std::min(1.,static_cast<double>(elapsed_ms)/segment.duration_ms);
    const double weight=u*u*u*(10.+u*(-15.+6.*u));
    const double slope=30.*u*u*(1.-u)*(1.-u)*1000./segment.duration_ms;
    auto lerp=[weight](double a,double b){return a+(b-a)*weight;};
    Joints out{};
    for (std::size_t i=0; i<joint_count; ++i) {
        const auto& a=start[i];
        const auto& z=segment.target[i];
        out[i]={lerp(a.position,z.position),(z.position-a.position)*slope,
                lerp(a.kp,z.kp),lerp(a.kd,z.kd),0.};
    }
    return out;
}

} // namespace lite3_recovery
