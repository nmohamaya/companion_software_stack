// process4_mission_planner/include/planner/planner_factory.h
// Factory for creating path planner instances by backend name.
//
// Supports: "potential_field", "astar", "dstar_lite".
// Created as part of Issue #158.
#pragma once

#include "planner/astar_planner.h"
#include "planner/dstar_lite_planner.h"
#include "planner/grid_planner_base.h"
#include "planner/ipath_planner.h"

#include <memory>
#include <stdexcept>
#include <string>

namespace drone::planner {

/// Create a path planner by backend name.
/// Grid-based planners (astar, dstar_lite) accept a GridPlannerConfig.
/// The potential_field planner ignores the config (uses its own defaults).
inline std::unique_ptr<IPathPlanner> create_path_planner(
    const std::string& backend = "potential_field", const GridPlannerConfig& config = {}) {
    if (backend == "potential_field") {
        return std::make_unique<PotentialFieldPlanner>();
    }
    if (backend == "astar") {
        return std::make_unique<AStarPathPlanner>(config);
    }
    if (backend == "dstar_lite") {
        return std::make_unique<DStarLitePlanner>(config);
    }
    throw std::runtime_error("Unknown path planner: " + backend);
}

}  // namespace drone::planner
