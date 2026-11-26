#pragma once

#include "lite3_recovery/sequence.hpp"
#include <utility>

// No DDS, socket, approved trajectory, or automatic hardware start is defined
// here. The SDK-specific adapter supplies positively identified hardware
// feedback, ownership, the official standing controller, and the checked sink.
namespace lite3_recovery {

struct Profile {
    Bounds bounds{};
    Joints start;
    std::vector<Segment> segments;
    StallLimits stall;
    Axis damping{};
    double initial_position_tolerance{};
    bool smooth_entry_from_actual{false};
};

class Controller {
public:
    explicit Controller(Profile profile, SequenceConfig seq_config = {})
        : profile_(std::move(profile)), sequence_(profile_.stall, seq_config),
          seq_config_(seq_config) {}

    // durable_reservation must be the result of reserve_trial() in the
    // production adapter. No marker is written from the motion loop.
    bool enable(std::uint64_t session, const Commissioning& checks, bool durable_reservation) {
        if (session_ || !session || !std::isfinite(profile_.initial_position_tolerance)
            || profile_.initial_position_tolerance <= 0.
            || !valid_trajectory(profile_.start,profile_.segments,profile_.bounds)) return false;
        for (std::size_t i=0; i<joint_count; ++i) {
            if (!within(profile_.damping[i],0.,profile_.bounds.max_kd[i])
                || profile_.damping[i]<=0.) return false;
        }
        if (!sequence_.enable(checks,durable_reservation)) return false;
        session_=session;
        checks_=checks;
        return true;
    }

    // The controller has a single producer. STOP can separately latch the
    // bridge via guard().stop() even if this producer has stalled.
    template<class Standing>
    Status advance(Millis now, Input input, Standing&& sdk_stand) {
        if (bridge_started_) {
            const auto event=guard_.evidence();
            if (event.reason!=StopReason::none) input.stop=event.reason;
            if (input.stop!=StopReason::none) guard_.stop(input.stop,now);
        }
        const auto profile_phase=phase_;
        if (profile_phase==Phase::watching) {
            const bool prone_bypass = seq_config_.allow_prone_shortcut
                && tilt(input.feedback.attitude) < seq_config_.supine_tilt_deg;
            if (!prone_bypass) {
                for (std::size_t i=0; i<joint_count; ++i) {
                    if (std::abs(input.feedback.position[i]-profile_.start[i].position)
                        >profile_.initial_position_tolerance) input.fall_allowed=false;
                }
            }
        }

        std::size_t index{};
        Millis elapsed{};
        if (profile_phase==Phase::rolling && now>=roll_started_) {
            elapsed=now-roll_started_;
            while (index<profile_.segments.size() && elapsed>=profile_.segments[index].duration_ms) {
                elapsed-=profile_.segments[index].duration_ms;
                ++index;
            }
            input.roll_complete=index==profile_.segments.size();
            input.expects_progress=!input.roll_complete && profile_.segments[index].expects_progress;
        }
        auto status=sequence_.tick(now,input);
        phase_=status.phase;
        if (status.stop!=StopReason::none) {
            if (bridge_started_) guard_.stop(status.stop,now);
            return status;
        }
        if (phase_==Phase::watching || phase_==Phase::disabled) return status;
        if (!bridge_started_) {
            if (!guard_.arm(session_,now,checks_,true,profile_.bounds,profile_.damping))
                return stop_failed(now,StopReason::fault);
            bridge_started_=true;
            roll_started_=now;
            if (profile_.smooth_entry_from_actual) {
                for (std::size_t i=0; i<joint_count; ++i) {
                    const double q_clamped = std::clamp(
                        input.feedback.position[i],
                        profile_.bounds.min_position[i],
                        profile_.bounds.max_position[i]);
                    roll_entry_pose_[i]={q_clamped, 0., 0., profile_.damping[i], 0.};
                }
                has_roll_entry_pose_=true;
            }
        }
        const auto& fb=input.feedback;
        guard_.feedback(fb.sequence,fb.received_ms,now,fb.position,fb.velocity);
        Joints command{};
        if (phase_==Phase::rolling) {
            const auto& start=(index==0)
                ? (has_roll_entry_pose_ ? roll_entry_pose_ : profile_.start)
                : profile_.segments[index-1].target;
            command=interpolate(start,profile_.segments[index],elapsed);
        } else if (phase_==Phase::prone_check) {
            if (profile_phase==Phase::watching) entered_via_prone_shortcut_=true;
            if (entered_via_prone_shortcut_) {
                for (std::size_t i=0; i<joint_count; ++i) {
                    const double q_clamped = std::clamp(
                        fb.position[i],
                        profile_.bounds.min_position[i],
                        profile_.bounds.max_position[i]);
                    command[i]={q_clamped, 0., 0., profile_.damping[i], 0.};
                }
            } else {
                command=profile_.segments.back().target;
            }
        } else {
            // Pass the previous commanded posture in `command` so sdk_stand can
            // maintain C0 position/gain continuity across prone_check -> standing
            // even when actual joints sag slightly under gravity.
            command=last_command_;
            if (!sdk_stand(phase_,command)) return stop_failed(now,StopReason::fault);
        }
        if (!guard_.accept(Frame{session_,++command_sequence_,now,command},now)) {
            const auto reason=guard_.evidence().reason;
            return stop_failed(now,reason==StopReason::none ? StopReason::fault : reason);
        }
        last_command_=command;
        return status;
    }

    Guard& guard() { return guard_; }

private:
    Status stop_failed(Millis now, StopReason reason) {
        guard_.stop(reason,now);
        Input stopped{};
        stopped.stop=reason;
        auto status=sequence_.tick(now,stopped);
        phase_=status.phase;
        return status;
    }

    Profile profile_;
    Sequence sequence_;
    SequenceConfig seq_config_{};
    Guard guard_;
    Commissioning checks_{};
    Phase phase_{Phase::disabled};
    std::uint64_t session_{}, command_sequence_{};
    Millis roll_started_{};
    Joints roll_entry_pose_{};
    Joints last_command_{};
    bool has_roll_entry_pose_{};
    bool entered_via_prone_shortcut_{};
    bool bridge_started_{};
};

} // namespace lite3_recovery
