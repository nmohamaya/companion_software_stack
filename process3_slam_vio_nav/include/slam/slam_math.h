// process3_slam_vio_nav/include/slam/slam_math.h
// Shared SO(3)/SE(3) math utilities for VIO backends.
//
// Provides:
//   exp_map  : so(3) -> SO(3)  (Rodrigues' formula)
//   log_map  : SO(3) -> so(3)  (inverse of exp_map)
//   skew     : R^3   -> so(3)  (skew-symmetric matrix)
//   left/right Jacobian of SO(3) (for covariance propagation)
#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone::slam {

// ── Exp map: so(3) -> SO(3) (Rodrigues' formula) ────────────
// Converts an axis-angle vector omega*dt into a unit quaternion.
// For small angles, uses first-order approximation.
inline Eigen::Quaterniond exp_map(const Eigen::Vector3d& omega_dt) {
    const double theta = omega_dt.norm();
    if (theta < 1e-10) {
        // First-order approximation for small angles
        return Eigen::Quaterniond(1.0, omega_dt.x() * 0.5, omega_dt.y() * 0.5, omega_dt.z() * 0.5)
            .normalized();
    }
    Eigen::Vector3d axis = omega_dt / theta;
    return Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
}

// ── Log map: SO(3) -> so(3) ─────────────────────────────────
// Inverse of exp_map: extracts the axis-angle vector from a quaternion.
inline Eigen::Vector3d log_map(const Eigen::Quaterniond& q) {
    // Ensure w >= 0 for numerical stability (q and -q represent the same rotation)
    Eigen::Quaterniond q_pos = q.w() >= 0 ? q : Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());

    const double sin_half = q_pos.vec().norm();
    if (sin_half < 1e-10) {
        // Small angle: theta ~ 2 * sin(theta/2) ~ 2 * ||q.vec()||
        return 2.0 * q_pos.vec();
    }
    const double theta = 2.0 * std::atan2(sin_half, q_pos.w());
    return theta / sin_half * q_pos.vec();
}

// ── Skew-symmetric matrix [v]x ──────────────────────────────
// Returns the 3x3 skew-symmetric matrix such that skew(v)*u = v x u.
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

// ── Left Jacobian of SO(3) ─────────────────────────────────
// J_l(phi) maps a perturbation in the tangent space to the group.
// For small phi, J_l ~ I. Otherwise uses the closed-form expression:
//   J_l = (sin(t)/t)*I + (1 - sin(t)/t)*a*a^T + ((1 - cos(t))/t)*[a]x
// where t = ||phi||, a = phi/t.
inline Eigen::Matrix3d left_jacobian_SO3(const Eigen::Vector3d& phi) {
    const double theta = phi.norm();
    if (theta < 1e-10) {
        return Eigen::Matrix3d::Identity();
    }
    const Eigen::Vector3d a     = phi / theta;
    const double          sin_t = std::sin(theta);
    const double          cos_t = std::cos(theta);

    // J_l = (sin(t)/t)*I + (1 - sin(t)/t)*a*a^T + ((1 - cos(t))/t)*[a]x
    return (sin_t / theta) * Eigen::Matrix3d::Identity() +
           (1.0 - sin_t / theta) * (a * a.transpose()) + ((1.0 - cos_t) / theta) * skew(a);
}

// ── Right Jacobian of SO(3) ────────────────────────────────
// J_r(phi) = J_l(-phi).
inline Eigen::Matrix3d right_jacobian_SO3(const Eigen::Vector3d& phi) {
    return left_jacobian_SO3(-phi);
}

}  // namespace drone::slam
