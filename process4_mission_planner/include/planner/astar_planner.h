// process4_mission_planner/include/planner/astar_planner.h
// A* 3D Grid Path Planner — finds optimal paths through a voxel grid
// with obstacle inflation.
//
// Architecture:
//   1. Obstacle map: maintains a lightweight 3D occupancy grid built from
//      DetectedObjectList.  Each detected object inflates a sphere of
//      cells (configurable radius) to provide clearance.
//   2. A* search: classic A* with 26-connected neighbours on the grid.
//      Uses Euclidean heuristic (admissible and consistent).
//   3. Path following: the planner returns a velocity command toward the
//      next waypoint on the A* path, re-planning periodically.
//
// The grid is local and relative to the drone's planning origin.
// Resolution and extent are configurable.
//
// Implements Epic #110 Phase 3 — A*/RRT* global planner.
// Refactored in Issue #158 to extend GridPlannerBase.
#pragma once

#include "planner/grid_planner_base.h"
#include "planner/occupancy_grid_3d.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::planner {

// ─────────────────────────────────────────────────────────────
// A* Search
// ─────────────────────────────────────────────────────────────

/// A* search result.
struct AStarResult {
    bool                              found = false;
    std::vector<GridCell>             path;        // grid cells from start to goal
    std::vector<std::array<float, 3>> world_path;  // world coords
    int                               iterations = 0;
    float                             path_cost  = 0.0f;
};

/// Run A* search on the 3D occupancy grid.
/// @param grid   The obstacle grid.
/// @param start  Start cell.
/// @param goal   Goal cell.
/// @param max_iter  Maximum iterations before giving up.
/// @param max_search_time_ms  Wall-clock timeout in ms (0 = disabled).
/// @return AStarResult with the path (if found).
inline AStarResult astar_search(const OccupancyGrid3D& grid, const GridCell& start,
                                const GridCell& goal, int max_iter = 50000,
                                float max_search_time_ms = 0.0f) {
    AStarResult result;

    // Trivial case: start == goal
    if (start == goal) {
        result.found = true;
        result.path  = {start};
        result.world_path.push_back(grid.grid_to_world(start));
        return result;
    }

    // Check if goal is reachable
    if (grid.is_occupied(goal) || !grid.in_bounds(goal)) {
        return result;  // unreachable
    }
    if (!grid.in_bounds(start)) {
        return result;  // start is out of bounds
    }

    // If the drone is inside an inflated obstacle cell (e.g. just passed through
    // an obstacle's inflation radius), escape to the nearest free cell before
    // running A*.  Without this, A* returns immediately with no path, causing
    // the direct fallback to fly straight into the obstacle.
    GridCell eff_start = start;
    if (grid.is_occupied(start)) {
        bool escaped = false;
        for (int r = 1; r <= 6 && !escaped; ++r) {
            for (int dz = -r; dz <= r && !escaped; ++dz) {
                for (int dy = -r; dy <= r && !escaped; ++dy) {
                    for (int dx = -r; dx <= r && !escaped; ++dx) {
                        if (std::abs(dx) != r && std::abs(dy) != r && std::abs(dz) != r)
                            continue;  // only shell of radius r
                        GridCell c{start.x + dx, start.y + dy, start.z + dz};
                        if (grid.in_bounds(c) && !grid.is_occupied(c)) {
                            eff_start = c;
                            escaped   = true;
                        }
                    }
                }
            }
        }
        if (!escaped) return result;  // completely surrounded
        if (eff_start == goal) {
            result.found = true;
            result.path  = {eff_start};
            result.world_path.push_back(grid.grid_to_world(eff_start));
            return result;
        }
    }

    // Euclidean heuristic (admissible in 3D grid with 26-connectivity)
    auto heuristic = [](const GridCell& a, const GridCell& b) -> float {
        float dx = static_cast<float>(a.x - b.x);
        float dy = static_cast<float>(a.y - b.y);
        float dz = static_cast<float>(a.z - b.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    // Timeout setup
    const bool use_timeout = (max_search_time_ms > 0.0f);
    const auto deadline    = use_timeout ? std::chrono::steady_clock::now() +
                                            std::chrono::microseconds(
                                                static_cast<int64_t>(max_search_time_ms * 1000.0f))
                                         : std::chrono::steady_clock::time_point{};

    struct Node {
        GridCell cell;
        float    f;  // g + h
        bool     operator>(const Node& o) const { return f > o.f; }
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<GridCell, float, GridCellHash>                g_score;
    std::unordered_map<GridCell, GridCell, GridCellHash>             came_from;

    g_score[eff_start] = 0.0f;
    open.push({eff_start, heuristic(eff_start, goal)});

    int iterations = 0;
    while (!open.empty() && iterations < max_iter) {
        ++iterations;

        // Check wall-clock timeout every 64 iterations
        if (use_timeout && (iterations & 63) == 0) {
            if (std::chrono::steady_clock::now() >= deadline) {
                spdlog::debug("[A*] Timeout after {} iterations ({:.1f}ms limit)", iterations,
                              max_search_time_ms);
                break;
            }
        }

        auto current = open.top();
        open.pop();

        if (current.cell == goal) {
            // Reconstruct path
            result.found      = true;
            result.iterations = iterations;
            result.path_cost  = g_score[goal];

            GridCell c = goal;
            while (c != eff_start) {
                result.path.push_back(c);
                c = came_from[c];
            }
            result.path.push_back(eff_start);
            std::reverse(result.path.begin(), result.path.end());

            // Convert to world coordinates
            result.world_path.reserve(result.path.size());
            for (const auto& cell : result.path) {
                result.world_path.push_back(grid.grid_to_world(cell));
            }
            return result;
        }

        float g_current = g_score[current.cell];

        // Skip if we've already found a better path to this node
        auto it = g_score.find(current.cell);
        if (it != g_score.end() && current.f > it->second + heuristic(current.cell, goal) + 0.01f) {
            continue;
        }

        for (int n = 0; n < 26; ++n) {
            GridCell neighbour{current.cell.x + kNeighbours[n][0],
                               current.cell.y + kNeighbours[n][1],
                               current.cell.z + kNeighbours[n][2]};

            if (!grid.in_bounds(neighbour) || grid.is_occupied(neighbour)) continue;

            float tentative_g = g_current + kCosts[n];
            auto  g_it        = g_score.find(neighbour);
            if (g_it == g_score.end() || tentative_g < g_it->second) {
                g_score[neighbour]   = tentative_g;
                came_from[neighbour] = current.cell;
                float f              = tentative_g + heuristic(neighbour, goal);
                open.push({neighbour, f});
            }
        }
    }

    result.iterations = iterations;
    return result;  // path not found
}

// ─────────────────────────────────────────────────────────────
// AStarPathPlanner — GridPlannerBase implementation using A*
// ─────────────────────────────────────────────────────────────

class AStarPathPlanner final : public GridPlannerBase {
public:
    explicit AStarPathPlanner(const GridPlannerConfig& config = {}) : GridPlannerBase(config) {}

    std::string name() const override { return "AStarPathPlanner"; }

protected:
    bool do_search(const GridCell& start, const GridCell& goal,
                   std::vector<std::array<float, 3>>& out_world_path) override {
        auto result = astar_search(grid_, start, goal, config_.max_iterations,
                                   config_.max_search_time_ms);
        if (result.found && result.world_path.size() >= 2) {
            out_world_path = std::move(result.world_path);
            spdlog::debug("[A*] Path found: {} cells, cost={:.1f}, iters={}", out_world_path.size(),
                          result.path_cost, result.iterations);
            return true;
        }
        spdlog::debug("[A*] No path found (iters={}) — direct line fallback", result.iterations);
        return false;
    }
};

}  // namespace drone::planner
