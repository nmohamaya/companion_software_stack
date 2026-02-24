// process4_mission_planner/include/planner/ipath_planner.h
// Strategy interface for path/trajectory planning.
//
// The path planner computes a velocity command to move from the current
// pose toward a target waypoint.  Swapping the implementation lets us
// move from potential-field to RRT*, D* Lite, A*, lattice planners, etc.
#pragma once

#include "planner/mission_fsm.h"   // Waypoint
#include "ipc/shm_types.h"

#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace drone::planner {

/// Abstract path planner — produces a trajectory command from pose + target.
class IPathPlanner {
public:
    virtual ~IPathPlanner() = default;

    /// Plan a velocity command toward the target waypoint.
    /// @param pose    Current drone pose.
    /// @param target  Target waypoint.
    /// @return        Trajectory command with velocities and target position.
    virtual drone::ipc::ShmTrajectoryCmd plan(
        const drone::ipc::ShmPose& pose,
        const Waypoint& target) = 0;

    /// Human-readable name for logging.
    virtual std::string name() const = 0;
};

// ─────────────────────────────────────────────────────────────
// Potential-field planner (attractive force toward waypoint).
// This is the current behaviour extracted into a strategy object.
// ─────────────────────────────────────────────────────────────

class PotentialFieldPlanner final : public IPathPlanner {
public:
    drone::ipc::ShmTrajectoryCmd plan(
        const drone::ipc::ShmPose& pose,
        const Waypoint& target) override
    {
        drone::ipc::ShmTrajectoryCmd cmd{};
        cmd.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        cmd.valid = true;

        float dx = target.x - static_cast<float>(pose.translation[0]);
        float dy = target.y - static_cast<float>(pose.translation[1]);
        float dz = target.z - static_cast<float>(pose.translation[2]);
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > 0.01f) {
            float speed = std::min(target.speed, dist);
            cmd.velocity_x = (dx / dist) * speed;
            cmd.velocity_y = (dy / dist) * speed;
            cmd.velocity_z = (dz / dist) * speed;
        }

        cmd.target_x = target.x;
        cmd.target_y = target.y;
        cmd.target_z = target.z;
        cmd.target_yaw = target.yaw;

        return cmd;
    }

    std::string name() const override { return "PotentialFieldPlanner"; }
};

/// Factory — creates the appropriate planner based on config.
inline std::unique_ptr<IPathPlanner> create_path_planner(
    const std::string& backend = "potential_field")
{
    if (backend == "potential_field") {
        return std::make_unique<PotentialFieldPlanner>();
    }
    throw std::runtime_error("Unknown path planner: " + backend);
}

}  // namespace drone::planner
