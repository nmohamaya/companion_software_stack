// common/util/include/util/sensor_geometry.h
//
// Attitude-aware sensor↔world geometry (Issue #816).
//
// A body-mounted ranging sensor (radar, lidar) reports a return in *body*
// spherical coordinates (range, azimuth, elevation).  Deciding whether that
// return is above the ground plane requires its WORLD-frame vertical position,
// which depends on the vehicle's full attitude — not just yaw.  The historical
// ground filters used `alt = sensor_alt + range·sin(elevation)`, i.e. the raw
// sensor elevation with the vehicle assumed perfectly level.  Under the pitch
// and roll a real airframe reaches (measured p99 ≈ 11–13°, max ≈ 18° in
// scenario 18), that error is `range·sin(attitude)` — several metres at typical
// obstacle ranges — which BOTH admits ground returns as phantom obstacles
// (#815) AND, worse, rejects real obstacles as ground under nose-up pitch
// (#816, a safety-critical suppression).
//
// These helpers compute the correct world-frame quantity from the full
// body→world rotation.  Header-only, Eigen, no allocation, noexcept — safe to
// call on the perception hot path.
//
// Frame conventions — ROS/Gazebo standard, verified empirically against real
// scenario-18 Pose data in #816 (do not change without re-verifying):
//   - World axes: X=North, Y=East, Z=Up (ENU), matching drone::ipc::Pose.
//   - Body frame: FLU — x=forward, y=LEFT, z=up (right-handed). The vehicle
//     Pose quaternion (Gazebo Odometry orientation) maps this body frame to
//     ENU, so applying it DIRECTLY to a body ray is correct: a nose-up drone
//     yields a forward ray with world-z > 0 (verified: real nose-up samples
//     give body-forward → world-z ≈ +0.30).
//   - Body spherical → body Cartesian: x = range·cos(el)·cos(az) (forward),
//     y = range·cos(el)·sin(az) (LEFT for +az), z = range·sin(el) (up for +el).
//     Pass the RAW sensor azimuth (Gazebo gpu_lidar horizontal angle is already
//     CCW/left-positive = FLU); do NOT apply the fusion engine's az_sign (that
//     converts to the UKF's FRD y-right convention, which this helper does not
//     use).
//   - Quaternion order is (w, x, y, z) to match Pose::quaternion[4].
#pragma once

#include <cmath>

#include <Eigen/Geometry>

namespace drone::util {

/// Build a body→world quaternion from roll/pitch/yaw (radians) in the FLU/ENU
/// convention above, with intuitive vehicle signs:
///   +roll  = right wing down,  +pitch = NOSE UP (forward ray rises),
///   +yaw   = CCW from North (does not affect altitude).
/// (+pitch = nose-up matches the real Pose quaternion — verified #816 — hence
/// the −pitch about +Y: in FLU a positive rotation about the left axis would be
/// nose-down.)  Returns a normalized quaternion.  Test/utility constructor;
/// production applies the vehicle's Pose quaternion directly.
[[nodiscard]] inline Eigen::Quaternionf quat_from_rpy(float roll, float pitch, float yaw) noexcept {
    const Eigen::Quaternionf q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                                 Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()) *
                                 Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
    return q.normalized();
}

/// Convert a Pose-style (w, x, y, z) quaternion to a normalized Eigen
/// quaternion.  Falls back to identity if the input is non-finite or
/// degenerate (zero norm) — a fail-safe default: an unknown attitude must never
/// silently rotate a return to a wrong altitude.
[[nodiscard]] inline Eigen::Quaternionf quat_from_wxyz(float w, float x, float y,
                                                       float z) noexcept {
    if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return Eigen::Quaternionf::Identity();
    }
    Eigen::Quaternionf q(w, x, y, z);
    const float        n = q.norm();
    if (!std::isfinite(n) || n < 1e-6f) {
        return Eigen::Quaternionf::Identity();
    }
    q.coeffs() /= n;
    return q;
}

/// Unit ray direction of a (azimuth, elevation) body-spherical bearing,
/// expressed in the WORLD frame via the body→world rotation.
[[nodiscard]] inline Eigen::Vector3f ray_direction_world(
    float azimuth_rad, float elevation_rad, const Eigen::Quaternionf& body_to_world) noexcept {
    const float           cos_el = std::cos(elevation_rad);
    const Eigen::Vector3f body_dir(cos_el * std::cos(azimuth_rad),  // forward (N at level, yaw 0)
                                   cos_el * std::sin(azimuth_rad),  // left    (FLU: +az = left)
                                   std::sin(elevation_rad));        // up      (+el = up)
    return body_to_world * body_dir;
}

/// World-frame altitude (Z-up) of a sensor return at (range, azimuth,
/// elevation) given the sensor's world altitude and the body→world rotation:
///
///     world_alt = sensor_alt + range · (body_to_world · ray_body).z()
///
/// This is the attitude-aware replacement for `sensor_alt + range·sin(el)`.
[[nodiscard]] inline float return_world_altitude_m(
    float sensor_alt_m, float range_m, float azimuth_rad, float elevation_rad,
    const Eigen::Quaternionf& body_to_world) noexcept {
    const Eigen::Vector3f dir_world = ray_direction_world(azimuth_rad, elevation_rad,
                                                          body_to_world);
    return sensor_alt_m + range_m * dir_world.z();
}

}  // namespace drone::util
