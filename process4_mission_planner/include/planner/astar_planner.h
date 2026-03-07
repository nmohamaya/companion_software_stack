// process4_mission_planner/include/planner/astar_planner.h
// A* 3D Grid Path Planner — finds optimal paths through a voxel grid
// with obstacle inflation.
//
// Architecture:
//   1. Obstacle map: maintains a lightweight 3D occupancy grid built from
//      ShmDetectedObjectList.  Each detected object inflates a sphere of
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
#pragma once

#include "ipc/shm_types.h"
#include "planner/ipath_planner.h"
#include "planner/mission_fsm.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::planner {

// ─────────────────────────────────────────────────────────────
// OccupancyGrid3D — lightweight voxel grid for obstacle representation
// ─────────────────────────────────────────────────────────────

/// A cell index in the 3D grid.
struct GridCell {
    int x = 0, y = 0, z = 0;

    bool operator==(const GridCell& o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator!=(const GridCell& o) const { return !(*this == o); }
};

/// Hash for GridCell (for use in unordered containers).
struct GridCellHash {
    size_t operator()(const GridCell& c) const {
        // FNV-1a inspired mixing
        size_t h = 2166136261u;
        h ^= static_cast<size_t>(c.x);
        h *= 16777619u;
        h ^= static_cast<size_t>(c.y);
        h *= 16777619u;
        h ^= static_cast<size_t>(c.z);
        h *= 16777619u;
        return h;
    }
};

/// Configuration for the occupancy grid and A* planner.
struct AStarConfig {
    float resolution_m       = 0.5f;   // metres per grid cell
    float grid_extent_m      = 50.0f;  // half-extent of grid in each axis
    float inflation_radius_m = 1.5f;   // obstacle inflation radius
    float replan_interval_s  = 1.0f;   // how often to re-run A*
    float path_speed_mps     = 2.0f;   // cruise speed along A* path
    float smoothing_alpha    = 0.35f;  // EMA smoothing for velocity output
    int   max_iterations     = 50000;  // A* iteration limit (prevents runaway)
};

/// 3D occupancy grid backed by a hash set of occupied cells.
/// Sparse representation — only occupied cells use memory.
class OccupancyGrid3D {
public:
    explicit OccupancyGrid3D(float resolution = 0.5f, float extent = 50.0f, float inflation = 1.5f)
        : resolution_(resolution)
        , half_extent_cells_(static_cast<int>(extent / resolution))
        , inflation_cells_(std::max(1, static_cast<int>(std::ceil(inflation / resolution)))) {}

    /// Clear all obstacles.
    void clear() { occupied_.clear(); }

    /// Insert obstacles from a detected object list.
    /// Each object's 3D position is mapped to grid cells and inflated.
    void update_from_objects(const drone::ipc::ShmDetectedObjectList& objects,
                             const drone::ipc::ShmPose& /* drone_pose */) {
        clear();
        for (uint32_t i = 0; i < objects.num_objects; ++i) {
            const auto& obj = objects.objects[i];
            if (obj.confidence < 0.3f) continue;  // skip low-confidence

            // Object position in world frame → grid cell
            GridCell center = world_to_grid(obj.position_x, obj.position_y, obj.position_z);

            // Inflate a sphere of cells around the obstacle
            for (int dz = -inflation_cells_; dz <= inflation_cells_; ++dz) {
                for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
                    for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
                        if (dx * dx + dy * dy + dz * dz <= inflation_cells_ * inflation_cells_) {
                            GridCell c{center.x + dx, center.y + dy, center.z + dz};
                            if (in_bounds(c)) occupied_.insert(c);
                        }
                    }
                }
            }
        }
    }

    /// Check if a cell is occupied (obstacle or inflated zone).
    [[nodiscard]] bool is_occupied(const GridCell& c) const { return occupied_.count(c) > 0; }

    /// Check if a cell is within the grid bounds.
    [[nodiscard]] bool in_bounds(const GridCell& c) const {
        return std::abs(c.x) <= half_extent_cells_ && std::abs(c.y) <= half_extent_cells_ &&
               std::abs(c.z) <= half_extent_cells_;
    }

    /// Convert world coordinates to grid cell.
    [[nodiscard]] GridCell world_to_grid(float wx, float wy, float wz) const {
        return {static_cast<int>(std::round(wx / resolution_)),
                static_cast<int>(std::round(wy / resolution_)),
                static_cast<int>(std::round(wz / resolution_))};
    }

    /// Convert grid cell to world coordinates (cell centre).
    [[nodiscard]] std::array<float, 3> grid_to_world(const GridCell& c) const {
        return {c.x * resolution_, c.y * resolution_, c.z * resolution_};
    }

    [[nodiscard]] float  resolution() const { return resolution_; }
    [[nodiscard]] size_t occupied_count() const { return occupied_.size(); }

private:
    float                                      resolution_;
    int                                        half_extent_cells_;
    int                                        inflation_cells_;
    std::unordered_set<GridCell, GridCellHash> occupied_;
};

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
/// @return AStarResult with the path (if found).
inline AStarResult astar_search(const OccupancyGrid3D& grid, const GridCell& start,
                                const GridCell& goal, int max_iter = 50000) {
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
    if (grid.is_occupied(start) || !grid.in_bounds(start)) {
        return result;  // start is blocked
    }

    // Euclidean heuristic (admissible in 3D grid with 26-connectivity)
    auto heuristic = [](const GridCell& a, const GridCell& b) -> float {
        float dx = static_cast<float>(a.x - b.x);
        float dy = static_cast<float>(a.y - b.y);
        float dz = static_cast<float>(a.z - b.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    // 26-connected neighbours (face + edge + corner adjacent)
    static constexpr int kNeighbours[26][3] = {
        // 6 face-adjacent
        {1, 0, 0},
        {-1, 0, 0},
        {0, 1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1},
        // 12 edge-adjacent
        {1, 1, 0},
        {1, -1, 0},
        {-1, 1, 0},
        {-1, -1, 0},
        {1, 0, 1},
        {1, 0, -1},
        {-1, 0, 1},
        {-1, 0, -1},
        {0, 1, 1},
        {0, 1, -1},
        {0, -1, 1},
        {0, -1, -1},
        // 8 corner-adjacent
        {1, 1, 1},
        {1, 1, -1},
        {1, -1, 1},
        {1, -1, -1},
        {-1, 1, 1},
        {-1, 1, -1},
        {-1, -1, 1},
        {-1, -1, -1},
    };
    static constexpr float kCosts[26] = {
        1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f,                   // face
        1.414f, 1.414f, 1.414f, 1.414f, 1.414f, 1.414f,                 // edge (√2)
        1.414f, 1.414f, 1.414f, 1.414f, 1.414f, 1.414f,                 // edge
        1.732f, 1.732f, 1.732f, 1.732f, 1.732f, 1.732f, 1.732f, 1.732f  // corner (√3)
    };

    struct Node {
        GridCell cell;
        float    f;  // g + h
        bool     operator>(const Node& o) const { return f > o.f; }
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<GridCell, float, GridCellHash>                g_score;
    std::unordered_map<GridCell, GridCell, GridCellHash>             came_from;

    g_score[start] = 0.0f;
    open.push({start, heuristic(start, goal)});

    int iterations = 0;
    while (!open.empty() && iterations < max_iter) {
        ++iterations;
        auto current = open.top();
        open.pop();

        if (current.cell == goal) {
            // Reconstruct path
            result.found      = true;
            result.iterations = iterations;
            result.path_cost  = g_score[goal];

            GridCell c = goal;
            while (c != start) {
                result.path.push_back(c);
                c = came_from[c];
            }
            result.path.push_back(start);
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
// AStarPathPlanner — IPathPlanner implementation
// ─────────────────────────────────────────────────────────────

class AStarPathPlanner final : public IPathPlanner {
public:
    explicit AStarPathPlanner(const AStarConfig& config = {})
        : config_(config)
        , grid_(config.resolution_m, config.grid_extent_m, config.inflation_radius_m) {}

    /// Update the obstacle grid from the latest detected objects.
    /// Call this each planning cycle before plan().
    void update_obstacles(const drone::ipc::ShmDetectedObjectList& objects,
                          const drone::ipc::ShmPose&               pose) {
        grid_.update_from_objects(objects, pose);
    }

    drone::ipc::ShmTrajectoryCmd plan(const drone::ipc::ShmPose& pose,
                                      const Waypoint&            target) override {
        drone::ipc::ShmTrajectoryCmd cmd{};
        cmd.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        cmd.valid = true;

        float px = static_cast<float>(pose.translation[0]);
        float py = static_cast<float>(pose.translation[1]);
        float pz = static_cast<float>(pose.translation[2]);

        // ── Re-plan on interval or if no path ───────────────
        auto now         = std::chrono::steady_clock::now();
        bool need_replan = cached_path_.empty() || path_index_ >= cached_path_.size() ||
                           std::chrono::duration<float>(now - last_plan_time_).count() >
                               config_.replan_interval_s;

        if (need_replan) {
            GridCell start = grid_.world_to_grid(px, py, pz);
            GridCell goal  = grid_.world_to_grid(target.x, target.y, target.z);

            auto result = astar_search(grid_, start, goal, config_.max_iterations);
            if (result.found && result.world_path.size() >= 2) {
                cached_path_     = std::move(result.world_path);
                path_index_      = 1;  // skip start cell (we're already there)
                last_plan_time_  = now;
                direct_fallback_ = false;
                spdlog::debug("[A*] Path found: {} cells, cost={:.1f}, iters={}",
                              cached_path_.size(), result.path_cost, result.iterations);
            } else {
                // A* failed — fall back to direct line toward target
                spdlog::debug("[A*] No path found (iters={}) — direct line fallback",
                              result.iterations);
                cached_path_.clear();
                path_index_      = 0;
                direct_fallback_ = true;
            }
        }

        // ── Follow path or go direct ────────────────────────
        float goal_x = target.x, goal_y = target.y, goal_z = target.z;

        if (!cached_path_.empty() && path_index_ < cached_path_.size()) {
            auto& wp = cached_path_[path_index_];
            goal_x   = wp[0];
            goal_y   = wp[1];
            goal_z   = wp[2];

            // Advance along path when close to intermediate waypoint
            float dx   = goal_x - px;
            float dy   = goal_y - py;
            float dz   = goal_z - pz;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < config_.resolution_m * 1.5f && path_index_ + 1 < cached_path_.size()) {
                ++path_index_;
            }
        }

        // ── Velocity command toward current goal ────────────
        float dx   = goal_x - px;
        float dy   = goal_y - py;
        float dz   = goal_z - pz;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        float raw_vx = 0.0f, raw_vy = 0.0f, raw_vz = 0.0f;
        if (dist > 0.01f) {
            float speed = std::min(config_.path_speed_mps, target.speed);
            // Ramp down near final waypoint
            constexpr float ramp_dist = 3.0f;
            constexpr float min_speed = 1.0f;
            if (dist < ramp_dist && path_index_ + 1 >= cached_path_.size()) {
                speed = min_speed + (speed - min_speed) * (dist / ramp_dist);
            }
            raw_vx = (dx / dist) * speed;
            raw_vy = (dy / dist) * speed;
            raw_vz = (dz / dist) * speed;
        }

        // EMA smoothing
        smooth_vx_ = config_.smoothing_alpha * raw_vx +
                     (1.0f - config_.smoothing_alpha) * smooth_vx_;
        smooth_vy_ = config_.smoothing_alpha * raw_vy +
                     (1.0f - config_.smoothing_alpha) * smooth_vy_;
        smooth_vz_ = config_.smoothing_alpha * raw_vz +
                     (1.0f - config_.smoothing_alpha) * smooth_vz_;

        cmd.velocity_x = smooth_vx_;
        cmd.velocity_y = smooth_vy_;
        cmd.velocity_z = smooth_vz_;
        cmd.target_x   = target.x;
        cmd.target_y   = target.y;
        cmd.target_z   = target.z;
        cmd.target_yaw = target.yaw;

        return cmd;
    }

    std::string name() const override { return "AStarPathPlanner"; }

    /// Get the current obstacle grid (for diagnostics/testing).
    [[nodiscard]] const OccupancyGrid3D& grid() const { return grid_; }

    /// Get the cached A* path (for diagnostics/testing).
    [[nodiscard]] const std::vector<std::array<float, 3>>& cached_path() const {
        return cached_path_;
    }

    /// True if the last planning cycle could not find an A* path
    /// and fell back to a direct line toward the target.
    [[nodiscard]] bool using_direct_fallback() const { return direct_fallback_; }

private:
    AStarConfig     config_;
    OccupancyGrid3D grid_;

    // Cached A* path
    std::vector<std::array<float, 3>>     cached_path_;
    size_t                                path_index_      = 0;
    bool                                  direct_fallback_ = false;
    std::chrono::steady_clock::time_point last_plan_time_{};

    // Velocity smoothing state
    float smooth_vx_ = 0.0f;
    float smooth_vy_ = 0.0f;
    float smooth_vz_ = 0.0f;
};

}  // namespace drone::planner

// ── Factory ──────────────────────────────────────────────────
// Defined here (not in ipath_planner.h) so that AStarPathPlanner
// is fully visible — avoids circular-include issues.
namespace drone::planner {

inline std::unique_ptr<IPathPlanner> create_path_planner(
    const std::string& backend = "potential_field") {
    if (backend == "potential_field") {
        return std::make_unique<PotentialFieldPlanner>();
    }
    if (backend == "astar") {
        return std::make_unique<AStarPathPlanner>();
    }
    throw std::runtime_error("Unknown path planner: " + backend);
}

}  // namespace drone::planner
