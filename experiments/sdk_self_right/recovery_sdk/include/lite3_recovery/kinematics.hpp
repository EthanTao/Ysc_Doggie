#pragma once

#include "lite3_recovery/sequence.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace lite3_recovery {

inline Quaternion rpy_deg_to_quat_xyzw(double roll_deg, double pitch_deg, double yaw_deg) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double r = roll_deg * kDegToRad * 0.5;
    const double p = pitch_deg * kDegToRad * 0.5;
    const double y = yaw_deg * kDegToRad * 0.5;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    Quaternion q{
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    };
    const double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-9) {
        for (double& v : q) v /= norm;
    }
    return q;
}

// Lite3 leg forward kinematics matching official Lite3.xml MJCF geometry:
// HipX axis = (-1, 0, 0), HipY axis = (0, -1, 0), Knee axis = (0, -1, 0).
inline std::array<double, 3> leg_foot_in_body(std::size_t leg, double q_hx, double q_hy, double q_kn) {
    constexpr std::array<double, 4> kHipX = { 0.1745,  0.1745, -0.1745, -0.1745};
    constexpr std::array<double, 4> kHipY = { 0.0620, -0.0620,  0.0620, -0.0620};
    constexpr std::array<double, 4> kSide = { 1.0,    -1.0,     1.0,    -1.0};
    constexpr double kHipLen = 0.09735;
    constexpr double kThigh  = 0.20;
    constexpr double kShank  = 0.21012;

    // Rotation around (0, -1, 0) by theta maps (0, 0, -L) -> (-L*sin(theta), 0, -L*cos(theta))
    const double x_leg = -kThigh * std::sin(q_hy) - kShank * std::sin(q_hy + q_kn);
    const double z_leg = -kThigh * std::cos(q_hy) - kShank * std::cos(q_hy + q_kn);
    const double y_leg = kSide[leg] * kHipLen;

    // Rotation around (-1, 0, 0) by q_hx: R_x(-q_hx)
    const double c = std::cos(-q_hx);
    const double s = std::sin(-q_hx);
    const double y_body = kHipY[leg] + c * y_leg - s * z_leg;
    const double z_body = s * y_leg + c * z_leg;
    const double x_body = kHipX[leg] + x_leg;
    return {x_body, y_body, z_body};
}

// Estimates torso height above ground using minimum foot projection along world-up
// rotated into the body frame (verified within ~2.5 cm of MuJoCo truth).
inline double estimate_base_height_m(const Axis& q, const Quaternion& quat_xyzw) {
    const double x = quat_xyzw[0], y = quat_xyzw[1], z = quat_xyzw[2], w = quat_xyzw[3];
    // Third row of R_wb (i.e. R_wb^T * e_z = world +Z expressed in body frame)
    const double nx = 2.0 * (x * z - w * y);
    const double ny = 2.0 * (y * z + w * x);
    const double nz = 1.0 - 2.0 * (x * x + y * y);

    double best = 0.0;
    for (std::size_t leg = 0; leg < 4; ++leg) {
        const auto foot = leg_foot_in_body(leg, q[3*leg], q[3*leg + 1], q[3*leg + 2]);
        const double depth = -(foot[0] * nx + foot[1] * ny + foot[2] * nz);
        if (depth > best) best = depth;
    }
    return best;
}

struct ProneSupportCheck {
    bool confirmed{};
    double tilt_deg{};
    double max_joint_err{};
    double max_joint_vel{};
    double est_height_m{};
    int loaded_feet{};
};

inline ProneSupportCheck assess_prone_support(
    const Feedback& fb,
    const std::array<double, 4>& contact_force_z,
    const Axis& pre_pose) {
    ProneSupportCheck out{};
    out.tilt_deg = tilt(fb.attitude);
    out.est_height_m = estimate_base_height_m(fb.position, fb.attitude);
    for (std::size_t i = 0; i < joint_count; ++i) {
        out.max_joint_err = std::max(out.max_joint_err, std::abs(fb.position[i] - pre_pose[i]));
        out.max_joint_vel = std::max(out.max_joint_vel, std::abs(fb.velocity[i]));
    }
    for (double fz : contact_force_z) {
        if (std::isfinite(fz) && std::abs(fz) > 10.0) ++out.loaded_feet;
    }
    // Prone support requires level attitude, settled joint speeds, legs tucked
    // close to pre-stand posture, and FK height consistent with ground support
    // (not upside down or fully extended in air).
    out.confirmed = out.tilt_deg <= 20.0
        && out.max_joint_err <= 0.65
        && out.max_joint_vel <= 1.2
        && out.est_height_m >= 0.02
        && out.est_height_m <= 0.22;
    return out;
}

} // namespace lite3_recovery
