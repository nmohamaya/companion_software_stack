// process4_mission_planner/include/planner/iobstacle_avoider.h
// Strategy interface for obstacle avoidance.
//
// The obstacle avoider modifies a planned trajectory command to avoid
// detected obstacles.  Swapping the implementation lets us use
// ObstacleAvoider3D, velocity obstacles (VO), ORCA, MPC-based avoidance, etc.
//
// PotentialFieldAvoider removed in Issue #207 — ObstacleAvoider3D supersedes it.
#pragma once

#include "ipc/ipc_types.h"

#include <memory>
#include <string>

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
    virtual drone::ipc::TrajectoryCmd avoid(const drone::ipc::TrajectoryCmd&      planned,
                                            const drone::ipc::Pose&               pose,
                                            const drone::ipc::DetectedObjectList& objects) = 0;

    /// Human-readable name for logging.
    virtual std::string name() const = 0;
};

}  // namespace drone::planner
