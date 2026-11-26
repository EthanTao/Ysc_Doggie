#pragma once

#include "lite3_recovery/sequence.hpp"
#include <algorithm>
#include <cmath>

namespace lite3_recovery {

struct OfficialStandUpConfig {
    double kp{100.0};
    double kd{2.5};
    double kp_scale{1.0};
    Millis pre_duration_ms{400};
    Millis stand_duration_ms{1500};
    Millis hold_settle_ms{300};
};

// Mirrors official Lite3 StandUpState (standup_state.hpp + lite3_control_parameters.cpp)
// as a pure, ROS-free callback for Controller::advance(), keeping every tick
// subject to Guard's 20 ms TTL, rate/effort bounds, and immediate damping STOP.
class OfficialStandUp {
public:
    using Config = OfficialStandUpConfig;
    static constexpr double kThighLen = 0.20;
    static constexpr double kShankLen = 0.21;
    static constexpr double kPreHeight = 0.12;
    static constexpr double kStandHeight = 0.30;
    static constexpr double kHipYLower = -3.50;
    static constexpr double kHipYUpper = 0.320;
    static constexpr double kKneeLower = 0.349;
    static constexpr double kKneeUpper = 2.80;
    static constexpr double kPi = 3.14159265358979323846;

    explicit OfficialStandUp(Config cfg = Config{}) : cfg_(cfg) {
        const double hy = hipy_by_height(kPreHeight);
        const double kn = knee_by_height(kPreHeight);
        for (std::size_t leg = 0; leg < 4; ++leg) {
            pre_pose_[3 * leg + 0] = 0.0;
            pre_pose_[3 * leg + 1] = hy;
            pre_pose_[3 * leg + 2] = kn;
        }
    }

    static double hipy_by_height(double h) {
        if (std::abs(h) >= kThighLen + kShankLen || h <= 0.0) return 0.0;
        const double c = std::clamp(
            (kThighLen * kThighLen + h * h - kShankLen * kShankLen) / (2.0 * h * kThighLen),
            -1.0, 1.0);
        return std::clamp(-std::acos(c), kHipYLower, kHipYUpper);
    }

    static double knee_by_height(double h) {
        if (std::abs(h) >= kThighLen + kShankLen || h <= 0.0) return 0.0;
        const double c = std::clamp(
            (kThighLen * kThighLen + kShankLen * kShankLen - h * h) / (2.0 * kThighLen * kShankLen),
            -1.0, 1.0);
        return std::clamp(kPi - std::acos(c), kKneeLower, kKneeUpper);
    }

    const Axis& pre_pose() const { return pre_pose_; }

    void reset() { started_ = false; }

    bool expects_progress(Millis now) const {
        if (!started_ || now < start_ms_) return false;
        const Millis elapsed = now - start_ms_;
        return elapsed > cfg_.pre_duration_ms + 200
            && elapsed < cfg_.pre_duration_ms + cfg_.stand_duration_ms;
    }

    bool complete(Millis now) const {
        if (!started_ || now < start_ms_) return false;
        return (now - start_ms_) >= (cfg_.pre_duration_ms + cfg_.stand_duration_ms + cfg_.hold_settle_ms);
    }

    bool compute(Millis now, const Feedback& fb, double start_kp, Phase phase, Joints& out) {
        if (phase != Phase::standing && phase != Phase::upright_hold) return false;
        if (!started_) {
            started_ = true;
            start_ms_ = now;
            // Use previous commanded position/kp carried in `out` when valid to guarantee
            // C0 continuity across prone_check -> standing (no Guard rate_safe jump on sag).
            bool has_prev_cmd = false;
            for (std::size_t i = 0; i < joint_count; ++i) {
                if (out[i].kd > 0.0 || out[i].kp > 0.0) { has_prev_cmd = true; break; }
            }
            for (std::size_t i = 0; i < joint_count; ++i) {
                init_q_[i] = has_prev_cmd ? out[i].position : fb.position[i];
            }
            const double prev_kp = has_prev_cmd ? out[0].kp : start_kp;
            start_kp_ = std::clamp(prev_kp, 0.0, cfg_.kp * cfg_.kp_scale);
        }
        const double target_kp = cfg_.kp * cfg_.kp_scale;
        const double kd = cfg_.kd;
        const double t_s = (now >= start_ms_) ? static_cast<double>(now - start_ms_) / 1000.0 : 0.0;
        const double t1 = static_cast<double>(cfg_.pre_duration_ms) / 1000.0;
        const double t2 = static_cast<double>(cfg_.stand_duration_ms) / 1000.0;

        if (phase == Phase::upright_hold || t_s >= t1 + t2) {
            const double hy = hipy_by_height(kStandHeight);
            const double kn = knee_by_height(kStandHeight);
            for (std::size_t leg = 0; leg < 4; ++leg) {
                out[3 * leg + 0] = {0.0, 0.0, target_kp, kd, 0.0};
                out[3 * leg + 1] = {hy,  0.0, target_kp, kd, 0.0};
                out[3 * leg + 2] = {kn,  0.0, target_kp, kd, 0.0};
            }
            return true;
        }

        if (t_s <= t1) {
            const double u = (t1 > 0.0) ? std::clamp(t_s / t1, 0.0, 1.0) : 1.0;
            const double w = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
            const double slope = (t1 > 0.0) ? (30.0 * u * u * (1.0 - u) * (1.0 - u) / t1) : 0.0;
            const double kp = start_kp_ + (target_kp - start_kp_) * w;
            for (std::size_t i = 0; i < joint_count; ++i) {
                const double pos = init_q_[i] + (pre_pose_[i] - init_q_[i]) * w;
                const double vel = (pre_pose_[i] - init_q_[i]) * slope;
                out[i] = {pos, vel, kp, kd, 0.0};
            }
            return true;
        }

        const double nt = t_s - t1;
        const double dt = 0.001;
        const double h = cubic_pos(kPreHeight, 0.0, kStandHeight, 0.0, nt, t2);
        const double h_next = cubic_pos(kPreHeight, 0.0, kStandHeight, 0.0, nt + dt, t2);
        const double hy = hipy_by_height(h);
        const double hy_v = (hipy_by_height(h_next) - hy) / dt;
        const double kn = knee_by_height(h);
        const double kn_v = (knee_by_height(h_next) - kn) / dt;
        for (std::size_t leg = 0; leg < 4; ++leg) {
            out[3 * leg + 0] = {0.0, 0.0,  target_kp, kd, 0.0};
            out[3 * leg + 1] = {hy,  hy_v, target_kp, kd, 0.0};
            out[3 * leg + 2] = {kn,  kn_v, target_kp, kd, 0.0};
        }
        return true;
    }

private:
    static double cubic_pos(double x0, double v0, double xf, double vf, double t, double T) {
        if (t >= T) return xf;
        const double d = x0;
        const double c = v0;
        const double a = (vf * T - 2.0 * xf + v0 * T + 2.0 * x0) / (T * T * T);
        const double b = (3.0 * xf - vf * T - 2.0 * v0 * T - 3.0 * x0) / (T * T);
        return a * t * t * t + b * t * t + c * t + d;
    }

    Config cfg_{};
    Axis pre_pose_{};
    Axis init_q_{};
    Millis start_ms_{};
    double start_kp_{};
    bool started_{};
};

} // namespace lite3_recovery
