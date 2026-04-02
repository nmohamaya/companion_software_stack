// process6_payload_manager/include/payload/auto_tracker.h
// Gimbal auto-tracking: transforms world-frame object positions into body-frame
// bearing using the SLAM pose, then computes gimbal pitch/yaw angles to point
// at the highest-confidence tracked object.
//
// Coordinate-frame contract:
//   - DetectedObjectList positions are in world frame (published by P2).
//   - Pose provides the drone's world-frame translation and orientation.
//   - This module subtracts the pose translation, rotates world→body using
//     pose yaw, and feeds the resulting body-frame vector into compute_bearing().

#pragma once

#include "ipc/ipc_types.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace drone::payload {

/// Local pi constant — avoids dependence on non-standard M_PI macro.
inline constexpr float kPi = 3.14159265358979323846f;

/// Configuration for gimbal auto-tracking.
struct AutoTrackConfig {
    bool  enabled{false};        // Master enable — default off
    float min_confidence{0.5f};  // Ignore objects below this confidence
};

/// Result of auto-tracking computation.
struct AutoTrackResult {
    bool     has_target{false};  // True if a valid target was found
    float    pitch_deg{0.0f};    // Gimbal pitch command (degrees)
    float    yaw_deg{0.0f};      // Gimbal yaw command (degrees)
    float    target_confidence{0.0f};
    uint32_t target_track_id{0};
};

/// Compute pitch and yaw angles (degrees) from a relative position vector.
/// dx/dy/dz are in the vehicle body frame (x=forward, y=left, z=up — FLU).
/// Returns {pitch_deg, yaw_deg}.
[[nodiscard]] inline std::pair<float, float> compute_bearing(float dx, float dy, float dz) {
    const float horizontal_dist = std::sqrt(dx * dx + dy * dy);

    // Pitch: angle above/below horizontal (negative = look down)
    float pitch_deg = 0.0f;
    if (horizontal_dist > 1e-6f || std::fabs(dz) > 1e-6f) {
        pitch_deg = std::atan2(dz, horizontal_dist) * (180.0f / kPi);
    }

    // Yaw: atan2(dy, dx) where dx=forward, dy=left in body frame
    // Positive yaw = left of forward, negative = right
    float yaw_deg = 0.0f;
    if (std::fabs(dx) > 1e-6f || std::fabs(dy) > 1e-6f) {
        yaw_deg = std::atan2(dy, dx) * (180.0f / kPi);
    }

    return {pitch_deg, yaw_deg};
}

/// Extract yaw (radians) from quaternion (w, x, y, z).
[[nodiscard]] inline float yaw_from_quaternion(const double quat[4]) {
    const double qw = quat[0];
    const double qx = quat[1];
    const double qy = quat[2];
    const double qz = quat[3];
    return static_cast<float>(
        std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
}

/// Select the highest-confidence object above min_confidence and compute
/// gimbal angles to point at it, given the drone's current pose.
///
/// Object positions are in world frame (as published by P2). The SLAM pose
/// is used to compute the world→body transform: subtract drone translation,
/// then rotate by -yaw to get the body-frame bearing vector.
[[nodiscard]] inline AutoTrackResult compute_auto_track(
    const drone::ipc::DetectedObjectList& objects, const drone::ipc::Pose& pose,
    const AutoTrackConfig& config) {
    AutoTrackResult result{};

    if (!config.enabled) {
        return result;
    }

    // Find highest-confidence object above threshold
    float    best_conf = -1.0f;
    uint32_t best_idx  = 0;
    bool     found     = false;

    const uint32_t count = std::min(objects.num_objects,
                                    static_cast<uint32_t>(drone::ipc::MAX_DETECTED_OBJECTS));

    for (uint32_t i = 0; i < count; ++i) {
        const auto& obj = objects.objects[i];
        if (!obj.validate()) continue;
        if (obj.confidence < config.min_confidence) continue;

        if (obj.confidence > best_conf) {
            best_conf = obj.confidence;
            best_idx  = i;
            found     = true;
        }
    }

    if (!found) {
        return result;  // No valid target — gimbal holds position
    }

    const auto& target = objects.objects[best_idx];

    // Object positions are in world frame — transform to body frame.
    // 1. Subtract drone position to get world-relative vector.
    const float dx_world = target.position_x - static_cast<float>(pose.translation[0]);
    const float dy_world = target.position_y - static_cast<float>(pose.translation[1]);
    const float dz_world = target.position_z - static_cast<float>(pose.translation[2]);

    // 2. Rotate world→body using negative yaw (R_z(-yaw)).
    //    World frame: [0]=North, [1]=East, [2]=Up (NEU)
    //    Body frame:  x=forward, y=left, z=up (FLU)
    //    Yaw-only rotation (roll/pitch assumed ~0 for gimbal bearing).
    //    Note: this is FLU, not FRD — consistent with gimbal pitch convention
    //    where negative pitch = look down, and with UKF body frame conventions
    //    in the perception pipeline (which separately negates azimuth for FRD).
    const float yaw   = yaw_from_quaternion(pose.quaternion);
    const float cos_y = std::cos(yaw);
    const float sin_y = std::sin(yaw);

    const float body_x = dx_world * cos_y + dy_world * sin_y;   // forward
    const float body_y = -dx_world * sin_y + dy_world * cos_y;  // left
    const float body_z = dz_world;                              // up

    auto [pitch_deg, yaw_deg] = compute_bearing(body_x, body_y, body_z);

    result.has_target        = true;
    result.pitch_deg         = pitch_deg;
    result.yaw_deg           = yaw_deg;
    result.target_confidence = target.confidence;
    result.target_track_id   = target.track_id;

    return result;
}

}  // namespace drone::payload
