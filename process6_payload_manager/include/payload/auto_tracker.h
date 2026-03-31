// process6_payload_manager/include/payload/auto_tracker.h
// Gimbal auto-tracking: points the gimbal at the highest-confidence tracked object.
// Computes bearing from drone pose (SLAM) to object position and converts to
// gimbal pitch/yaw angles.

#pragma once

#include "ipc/ipc_types.h"

#include <cmath>
#include <cstdint>
#include <utility>

namespace drone::payload {

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
/// dx/dy/dz are in the vehicle body frame (x=forward, y=left, z=up).
/// Returns {pitch_deg, yaw_deg}.
[[nodiscard]] inline std::pair<float, float> compute_bearing(float dx, float dy, float dz) {
    const float horizontal_dist = std::sqrt(dx * dx + dy * dy);

    // Pitch: angle below horizontal (negative = look down)
    // atan2(dz, horizontal) gives elevation; gimbal pitch is typically negative for below-horizon
    float pitch_deg = 0.0f;
    if (horizontal_dist > 1e-6f || std::fabs(dz) > 1e-6f) {
        pitch_deg = std::atan2(dz, horizontal_dist) * (180.0f / static_cast<float>(M_PI));
    }

    // Yaw: atan2(dy, dx) where dx=forward, dy=left in body frame
    // Positive yaw = left of forward, negative = right
    float yaw_deg = 0.0f;
    if (std::fabs(dx) > 1e-6f || std::fabs(dy) > 1e-6f) {
        yaw_deg = std::atan2(dy, dx) * (180.0f / static_cast<float>(M_PI));
    }

    return {pitch_deg, yaw_deg};
}

/// Select the highest-confidence object above min_confidence and compute
/// gimbal angles to point at it, given the drone's current pose.
///
/// Object positions are in the vehicle frame (relative to drone), so we
/// use them directly for bearing computation.
[[nodiscard]] inline AutoTrackResult compute_auto_track(
    const drone::ipc::DetectedObjectList& objects, const AutoTrackConfig& config) {
    AutoTrackResult result{};

    if (!config.enabled) {
        return result;
    }

    // Find highest-confidence object above threshold
    float    best_conf = -1.0f;
    uint32_t best_idx  = 0;
    bool     found     = false;

    for (uint32_t i = 0; i < objects.num_objects; ++i) {
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

    // Object positions are in vehicle frame (relative to drone)
    const float dx = target.position_x;  // forward
    const float dy = target.position_y;  // left
    const float dz = target.position_z;  // up

    auto [pitch_deg, yaw_deg] = compute_bearing(dx, dy, dz);

    result.has_target        = true;
    result.pitch_deg         = pitch_deg;
    result.yaw_deg           = yaw_deg;
    result.target_confidence = target.confidence;
    result.target_track_id   = target.track_id;

    return result;
}

}  // namespace drone::payload
