// process4_mission_planner/include/planner/planner_factory.h
// Factory for creating path planner instances by backend name.
//
// Supports: "dstar_lite".
// Created as part of Issue #158.  A* removed in Issue #203.
// PotentialFieldPlanner removed in Issue #207 — D* Lite supersedes it.
#pragma once

#include "planner/dstar_lite_planner.h"
#include "planner/grid_planner_base.h"
#include "planner/ipath_planner.h"
#include "util/result.h"

#include <memory>
#include <string>

namespace drone::planner {

/// Create a path planner by backend name.
/// D* Lite accepts a GridPlannerConfig.
[[nodiscard]] inline drone::util::Result<std::unique_ptr<IPathPlanner>> create_path_planner(
    const std::string& backend = "dstar_lite", const GridPlannerConfig& config = {}) {
    using R = drone::util::Result<std::unique_ptr<IPathPlanner>>;
    if (backend == "dstar_lite") {
        return R::ok(std::make_unique<DStarLitePlanner>(config));
    }
    return R::err(drone::util::Error(drone::util::ErrorCode::InvalidValue,
                                     "Unknown path planner: " + backend));
}

}  // namespace drone::planner
