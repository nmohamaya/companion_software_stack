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

#include <memory>
#include <stdexcept>
#include <string>

namespace drone::planner {

/// Create a path planner by backend name.
/// D* Lite accepts a GridPlannerConfig.
inline std::unique_ptr<IPathPlanner> create_path_planner(const std::string& backend = "dstar_lite",
                                                         const GridPlannerConfig& config = {}) {
    if (backend == "dstar_lite") {
        return std::make_unique<DStarLitePlanner>(config);
    }
    throw std::runtime_error("Unknown path planner: " + backend);
}

}  // namespace drone::planner
