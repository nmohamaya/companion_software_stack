// process4_mission_planner/include/planner/ipath_planner.h
// Strategy interface for path/trajectory planning.
//
// The path planner computes a velocity command to move from the current
// pose toward a target waypoint.  Swapping the implementation lets us
// use D* Lite, RRT*, lattice planners, etc.
//
// PotentialFieldPlanner removed in Issue #207 — D* Lite supersedes it.
#pragma once

#include "ipc/ipc_types.h"
#include "planner/mission_fsm.h"  // Waypoint

#include <memory>
#include <string>

namespace drone::planner {

/// Abstract path planner — produces a trajectory command from pose + target.
class IPathPlanner {
public:
    virtual ~IPathPlanner() = default;

    /// Plan a velocity command toward the target waypoint.
    /// @param pose    Current drone pose.
    /// @param target  Target waypoint.
    /// @return        Trajectory command with velocities and target position.
    virtual drone::ipc::TrajectoryCmd plan(const drone::ipc::Pose& pose,
                                           const Waypoint&         target) = 0;

    /// Human-readable name for logging.
    virtual std::string name() const = 0;
};

}  // namespace drone::planner
