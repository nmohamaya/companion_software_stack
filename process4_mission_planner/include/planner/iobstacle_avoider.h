// process4_mission_planner/include/planner/iobstacle_avoider.h
// Strategy interface for obstacle avoidance.
//
// The obstacle avoider modifies a planned trajectory command to avoid
// detected obstacles.  Swapping the implementation lets us move from
// simple repulsive potential field to velocity obstacles (VO), ORCA,
// MPC-based avoidance, etc.
#pragma once

#include "ipc/shm_types.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <spdlog/spdlog.h>

namespace drone::planner {

/// Abstract obstacle avoider — modifies a trajectory to avoid obstacles.
class IObstacleAvoider {
public:
    virtual ~IObstacleAvoider() = default;

    /// Modify a planned trajectory command to avoid obstacles.
    /// @param planned  The planned trajectory from the path planner.
    /// @param pose     Current drone pose.
    /// @param objects  Detected objects (potential obstacles).
    /// @return         Modified trajectory command.
    virtual drone::ipc::ShmTrajectoryCmd avoid(const drone::ipc::ShmTrajectoryCmd& planned,
                                               const drone::ipc::ShmPose&          pose,
                                               const drone::ipc::ShmDetectedObjectList& objects) = 0;

    /// Human-readable name for logging.
    virtual std::string name() const = 0;
};

// ─────────────────────────────────────────────────────────────
// Potential-field obstacle avoider (repulsive forces from obstacles).
// This is the current behaviour extracted into a strategy object.
// ─────────────────────────────────────────────────────────────

class PotentialFieldAvoider final : public IObstacleAvoider {
public:
    /// @param influence_radius  Max distance (m) at which obstacles exert force.
    /// @param repulsive_gain    Strength of the repulsive field.
    explicit PotentialFieldAvoider(float influence_radius = 5.0f, float repulsive_gain = 2.0f)
        : influence_radius_(influence_radius), repulsive_gain_(repulsive_gain) {}

    drone::ipc::ShmTrajectoryCmd avoid(const drone::ipc::ShmTrajectoryCmd&      planned,
                                       const drone::ipc::ShmPose&               pose,
                                       const drone::ipc::ShmDetectedObjectList& objects) override {
        auto cmd = planned;

        // Only apply avoidance if objects were published recently (< 500ms)
        // and have a non-zero count.  Simulated/random objects change every
        // cycle and cause aggressive jitter — skip them by checking staleness.
        auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::steady_clock::now().time_since_epoch())
                                                .count());
        constexpr uint64_t max_age_ns = 500'000'000ULL;  // 500 ms
        if (objects.num_objects == 0 || objects.timestamp_ns == 0 ||
            (now_ns > objects.timestamp_ns && now_ns - objects.timestamp_ns > max_age_ns)) {
            return cmd;
        }

        float total_rep_x = 0.0f;
        float total_rep_y = 0.0f;

        for (uint32_t i = 0; i < objects.num_objects; ++i) {
            const auto& obj = objects.objects[i];

            // Skip objects with zero/near-zero confidence — likely noise
            if (obj.confidence < 0.3f) continue;

            float ox       = obj.position_x - static_cast<float>(pose.translation[0]);
            float oy       = obj.position_y - static_cast<float>(pose.translation[1]);
            float obj_dist = std::sqrt(ox * ox + oy * oy);

            if (obj_dist < influence_radius_ && obj_dist > 0.1f) {
                float repulsion = repulsive_gain_ / (obj_dist * obj_dist);
                total_rep_x -= (ox / obj_dist) * repulsion;
                total_rep_y -= (oy / obj_dist) * repulsion;
                spdlog::debug("[Avoider] Obstacle at {:.1f}m, repulsion={:.2f}", obj_dist,
                              repulsion);
            }
        }

        // Clamp total repulsive force to avoid extreme corrections
        constexpr float max_repulsion = 2.0f;  // m/s max correction
        total_rep_x                   = std::clamp(total_rep_x, -max_repulsion, max_repulsion);
        total_rep_y                   = std::clamp(total_rep_y, -max_repulsion, max_repulsion);

        cmd.velocity_x += total_rep_x;
        cmd.velocity_y += total_rep_y;

        return cmd;
    }

    std::string name() const override { return "PotentialFieldAvoider"; }

private:
    float influence_radius_;
    float repulsive_gain_;
};

}  // namespace drone::planner
