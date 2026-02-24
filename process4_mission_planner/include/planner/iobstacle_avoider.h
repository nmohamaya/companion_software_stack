// process4_mission_planner/include/planner/iobstacle_avoider.h
// Strategy interface for obstacle avoidance.
//
// The obstacle avoider modifies a planned trajectory command to avoid
// detected obstacles.  Swapping the implementation lets us move from
// simple repulsive potential field to velocity obstacles (VO), ORCA,
// MPC-based avoidance, etc.
#pragma once

#include "ipc/shm_types.h"

#include <string>
#include <memory>
#include <cmath>
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
    virtual drone::ipc::ShmTrajectoryCmd avoid(
        const drone::ipc::ShmTrajectoryCmd& planned,
        const drone::ipc::ShmPose& pose,
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
    explicit PotentialFieldAvoider(float influence_radius = 5.0f,
                                   float repulsive_gain = 2.0f)
        : influence_radius_(influence_radius)
        , repulsive_gain_(repulsive_gain)
    {}

    drone::ipc::ShmTrajectoryCmd avoid(
        const drone::ipc::ShmTrajectoryCmd& planned,
        const drone::ipc::ShmPose& pose,
        const drone::ipc::ShmDetectedObjectList& objects) override
    {
        auto cmd = planned;

        for (uint32_t i = 0; i < objects.num_objects; ++i) {
            const auto& obj = objects.objects[i];
            float ox = obj.position_x -
                       static_cast<float>(pose.translation[0]);
            float oy = obj.position_y -
                       static_cast<float>(pose.translation[1]);
            float obj_dist = std::sqrt(ox * ox + oy * oy);

            if (obj_dist < influence_radius_ && obj_dist > 0.1f) {
                float repulsion = repulsive_gain_ / (obj_dist * obj_dist);
                cmd.velocity_x -= (ox / obj_dist) * repulsion;
                cmd.velocity_y -= (oy / obj_dist) * repulsion;
                spdlog::debug("[Avoider] Obstacle at {:.1f}m, repulsion={:.2f}",
                              obj_dist, repulsion);
            }
        }

        return cmd;
    }

    std::string name() const override { return "PotentialFieldAvoider"; }

private:
    float influence_radius_;
    float repulsive_gain_;
};

/// Factory — creates the appropriate avoider based on config.
inline std::unique_ptr<IObstacleAvoider> create_obstacle_avoider(
    const std::string& backend = "potential_field",
    float influence_radius = 5.0f,
    float repulsive_gain = 2.0f)
{
    if (backend == "potential_field") {
        return std::make_unique<PotentialFieldAvoider>(
            influence_radius, repulsive_gain);
    }
    throw std::runtime_error("Unknown obstacle avoider: " + backend);
}

}  // namespace drone::planner
