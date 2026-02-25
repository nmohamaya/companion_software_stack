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
    /// @param smoothing  EMA alpha in [0,1]. 1.0 = no smoothing, 0.1 = very smooth.
    explicit PotentialFieldPlanner(float smoothing = 0.35f)
        : alpha_(std::clamp(smoothing, 0.05f, 1.0f)) {}

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

        float raw_vx = 0.0f, raw_vy = 0.0f, raw_vz = 0.0f;
        if (dist > 0.01f) {
            // Ramp speed linearly from target.speed down to min_speed
            // within the last ramp_dist metres, but never below min_speed
            // so the drone doesn't crawl near waypoints.
            constexpr float min_speed  = 1.0f;   // m/s floor
            constexpr float ramp_dist  = 3.0f;   // metres to begin ramp
            float desired = target.speed;
            if (dist < ramp_dist) {
                desired = min_speed + (target.speed - min_speed)
                          * (dist / ramp_dist);
            }
            float speed = std::min(desired, target.speed);
            raw_vx = (dx / dist) * speed;
            raw_vy = (dy / dist) * speed;
            raw_vz = (dz / dist) * speed;
        }

        // Exponential moving average to smooth velocity commands.
        // Prevents jittery oscillation from noisy pose updates.
        smooth_vx_ = alpha_ * raw_vx + (1.0f - alpha_) * smooth_vx_;
        smooth_vy_ = alpha_ * raw_vy + (1.0f - alpha_) * smooth_vy_;
        smooth_vz_ = alpha_ * raw_vz + (1.0f - alpha_) * smooth_vz_;

        cmd.velocity_x = smooth_vx_;
        cmd.velocity_y = smooth_vy_;
        cmd.velocity_z = smooth_vz_;

        cmd.target_x = target.x;
        cmd.target_y = target.y;
        cmd.target_z = target.z;
        cmd.target_yaw = target.yaw;

        return cmd;
    }

    std::string name() const override { return "PotentialFieldPlanner"; }

private:
    float alpha_;                // EMA smoothing factor
    float smooth_vx_{0.0f};     // smoothed velocity state
    float smooth_vy_{0.0f};
    float smooth_vz_{0.0f};
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
