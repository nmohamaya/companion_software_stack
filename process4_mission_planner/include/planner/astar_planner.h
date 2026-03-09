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

/// 3D occupancy grid backed by a hash map of occupied cells with TTL.
///
/// Key design: obstacles are NEVER cleared unless they have not been
/// observed for `cell_ttl_s` seconds (default 3 s).  This prevents a
/// single missed detection frame from wiping the grid and causing A* to
/// re-route straight through a known obstacle.
class OccupancyGrid3D {
public:
    explicit OccupancyGrid3D(float resolution = 0.5f, float extent = 50.0f, float inflation = 1.5f,
                             float cell_ttl_s = 3.0f)
        : resolution_(resolution)
        , half_extent_cells_(static_cast<int>(extent / resolution))
        , inflation_cells_(std::max(1, static_cast<int>(std::ceil(inflation / resolution))))
        , cell_ttl_ns_(static_cast<uint64_t>(cell_ttl_s * 1e9f)) {}

    /// Force-clear all obstacles (for testing / reset).
    void clear() { occupied_.clear(); }

    /// Pre-populate the grid with a known static vertical obstacle (HD-map style).
    /// These cells are permanent — no TTL — and represent the pre-loaded world map.
    /// The camera pipeline independently confirms obstacles via update_from_objects(),
    /// which writes TTL cells. is_occupied() returns true if EITHER layer flags a cell.
    /// @param wx,wy     World-frame X,Y centre of the obstacle.
    /// @param radius_m  Obstacle footprint radius in metres.
    /// @param height_m  Obstacle height above ground in metres.
    void add_static_obstacle(float wx, float wy, float radius_m, float height_m) {
        GridCell base    = world_to_grid(wx, wy, 0.0f);
        int      r_cells = static_cast<int>(std::ceil(radius_m / resolution_)) + inflation_cells_;
        int      h_cells = static_cast<int>(std::ceil(height_m / resolution_)) + inflation_cells_;
        size_t   before  = static_occupied_.size();
        for (int dz = -inflation_cells_; dz <= h_cells; ++dz)
            for (int dy = -r_cells; dy <= r_cells; ++dy)
                for (int dx = -r_cells; dx <= r_cells; ++dx) {
                    if (std::sqrt(float(dx * dx + dy * dy)) <= float(r_cells)) {
                        GridCell c{base.x + dx, base.y + dy, base.z + dz};
                        if (in_bounds(c)) static_occupied_.insert(c);
                    }
                }
        spdlog::info("[HD-map] Static obstacle at ({:.1f},{:.1f}) r={:.1f}m h={:.1f}m "
                     "→ {} new cells (total static: {})",
                     wx, wy, radius_m, height_m, static_occupied_.size() - before,
                     static_occupied_.size());
    }

    /// Insert obstacles from a detected object list.
    /// Updates timestamps for observed cells; stale cells expire after TTL.
    void update_from_objects(const drone::ipc::ShmDetectedObjectList& objects,
                             const drone::ipc::ShmPose& /* drone_pose */) {
        const auto     now    = std::chrono::steady_clock::now();
        const uint64_t now_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());

        // Stamp newly detected cells
        for (uint32_t i = 0; i < objects.num_objects; ++i) {
            const auto& obj = objects.objects[i];
            if (obj.confidence < 0.3f) continue;

            GridCell center = world_to_grid(obj.position_x, obj.position_y, obj.position_z);

            for (int dz = -inflation_cells_; dz <= inflation_cells_; ++dz) {
                for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
                    for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
                        if (dx * dx + dy * dy + dz * dz <= inflation_cells_ * inflation_cells_) {
                            GridCell c{center.x + dx, center.y + dy, center.z + dz};
                            if (in_bounds(c)) occupied_[c] = now_ns;
                        }
                    }
                }
            }
        }

        // Expire stale cells (only when we actually have new data to avoid
        // running the prune loop with no benefit on empty-detection frames)
        if (objects.num_objects > 0 || !occupied_.empty()) {
            for (auto it = occupied_.begin(); it != occupied_.end();) {
                if (now_ns - it->second > cell_ttl_ns_) {
                    it = occupied_.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    /// Check if a cell is occupied (obstacle or inflated zone).
    /// Returns true if the cell is in the permanent HD-map layer (static_occupied_)
    /// OR in the camera-confirmed TTL layer (occupied_).
    [[nodiscard]] bool is_occupied(const GridCell& c) const {
        return static_occupied_.count(c) > 0 || occupied_.count(c) > 0;
    }

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
    [[nodiscard]] size_t static_count() const { return static_occupied_.size(); }

private:
    float    resolution_;
    int      half_extent_cells_;
    int      inflation_cells_;
    uint64_t cell_ttl_ns_;
    // HD-map layer: permanent cells loaded from scenario config at startup.
    // Never expire — represent known world geometry.
    std::unordered_set<GridCell, GridCellHash> static_occupied_;
    // Camera-confirmation layer: cells observed by perception, expire after TTL.
    // Refreshed each detection cycle; also catches unexpected/dynamic obstacles.
    std::unordered_map<GridCell, uint64_t, GridCellHash> occupied_;
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

    g_score[eff_start] = 0.0f;
    open.push({eff_start, heuristic(eff_start, goal)});

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

    /// Pre-load a static obstacle from the HD map (scenario config).
    /// These cells are permanent and supplement live camera detections.
    void add_static_obstacle(float wx, float wy, float radius_m, float height_m) {
        grid_.add_static_obstacle(wx, wy, radius_m, height_m);
        snap_valid_ = false;  // invalidate cached snap since grid changed
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

            // ── Goal snapping — cached per waypoint ──────────
            // If the waypoint maps to an occupied cell we snap to the nearest
            // free cell in the horizontal plane.  CRITICALLY this snap is only
            // recomputed when the target waypoint coords change.  If we re-snap
            // every replanning cycle the drone oscillates: deflecting sideways
            // makes a different cell the "nearest free", which redirects the
            // drone back toward the obstacle, causing the reversal the user sees.
            bool target_changed = (target.x != last_target_x_ || target.y != last_target_y_ ||
                                   target.z != last_target_z_);

            if (target_changed) {
                // New waypoint — discard any previous snap.
                snap_valid_    = false;
                last_target_x_ = target.x;
                last_target_y_ = target.y;
                last_target_z_ = target.z;
            }

            if (!snap_valid_ && (grid_.is_occupied(goal) || !grid_.in_bounds(goal))) {

                const GridCell orig_goal = goal;
                bool           snapped   = false;

                // ── Lateral-preferring snap ──────────────────────────────
                // Walk perpendicular to the approach direction (drone→obstacle)
                // to find a free cell on the SIDE of the obstacle, not behind it.
                // Prefers the side the drone is already deflecting toward
                // (determined by cross-product of offset from obstacle to drone
                //  against the approach direction).
                //
                // WHY: a nearest-free scan can choose a cell on the far side of
                // the obstacle, forcing A* to route THROUGH it.  Walking
                // perpendicular always places the snap goal beside the obstacle.
                const float adx  = static_cast<float>(orig_goal.x - start.x);
                const float ady  = static_cast<float>(orig_goal.y - start.y);
                const float amag = std::sqrt(adx * adx + ady * ady);

                if (amag > 0.5f) {
                    // Normalised approach unit vector (drone → obstacle)
                    const float ax = adx / amag, ay = ady / amag;
                    // Perpendiculars: left (CCW 90°) and right (CW 90°)
                    const float lx = -ay, ly = ax;
                    const float rx = ay, ry = -ax;

                    // Cross product: (drone - obstacle) × approach_dir
                    // Positive → drone is to the LEFT of the approach line.
                    // Prefer that side first — less additional deflection needed.
                    const float cross = static_cast<float>(start.x - orig_goal.x) * ay -
                                        static_cast<float>(start.y - orig_goal.y) * ax;
                    const bool first_left = (cross >= 0.0f);

                    for (int r = 1; r <= 8 && !snapped; ++r) {
                        for (int side = 0; side < 2 && !snapped; ++side) {
                            const bool  try_left = (side == 0) ? first_left : !first_left;
                            const float fx       = try_left ? lx : rx;
                            const float fy       = try_left ? ly : ry;
                            GridCell    c{orig_goal.x + static_cast<int>(std::round(fx * r)),
                                       orig_goal.y + static_cast<int>(std::round(fy * r)),
                                       orig_goal.z};
                            if (grid_.in_bounds(c) && !grid_.is_occupied(c)) {
                                goal    = c;
                                snapped = true;
                            }
                        }
                    }

                    // Reachability check: warn if the drone may not have
                    // enough time to deflect to the snapped side before
                    // reaching the obstacle at cruise speed.
                    if (snapped) {
                        const float snap_cells = std::sqrt(
                            static_cast<float>((goal.x - orig_goal.x) * (goal.x - orig_goal.x) +
                                               (goal.y - orig_goal.y) * (goal.y - orig_goal.y)));
                        const float snap_dist_m     = snap_cells * grid_.resolution();
                        const float approach_dist_m = amag * grid_.resolution();
                        const float cruise_spd      = std::max(0.5f, config_.path_speed_mps);
                        const float time_to_obs_s   = approach_dist_m / cruise_spd;
                        // Assume 3 m/s max lateral correction (avoider cap)
                        const float time_to_deflect = snap_dist_m / 3.0f;
                        if (time_to_obs_s - time_to_deflect < 0.5f) {
                            spdlog::warn("[A*] Snap {:.1f} m lateral but only {:.1f}s to "
                                         "obstacle (need {:.1f}s) — tight margin",
                                         snap_dist_m, time_to_obs_s, time_to_deflect);
                        }
                    }
                }

                // Fallback: no lateral snap found (both sides blocked or no
                // approach direction) — fall back to nearest horizontal free cell.
                if (!snapped) {
                    float best_dist_sq = 1e9f;
                    for (int dy = -8; dy <= 8; ++dy) {
                        for (int dx = -8; dx <= 8; ++dx) {
                            const float dist_sq = static_cast<float>(dx * dx + dy * dy);
                            if (dist_sq == 0.0f || dist_sq >= best_dist_sq) continue;
                            GridCell c{orig_goal.x + dx, orig_goal.y + dy, orig_goal.z};
                            if (grid_.in_bounds(c) && !grid_.is_occupied(c)) {
                                best_dist_sq = dist_sq;
                                goal         = c;
                                snapped      = true;
                            }
                        }
                    }
                }

                if (snapped) {
                    auto wc               = grid_.grid_to_world(goal);
                    snapped_world_x_      = wc[0];
                    snapped_world_y_      = wc[1];
                    snapped_world_z_      = wc[2];
                    snap_valid_           = true;
                    const float snap_dist = std::sqrt(static_cast<float>(
                                                (goal.x - orig_goal.x) * (goal.x - orig_goal.x) +
                                                (goal.y - orig_goal.y) * (goal.y - orig_goal.y))) *
                                            grid_.resolution();
                    spdlog::info("[A*] WP({:.1f},{:.1f},{:.1f}) occupied — snapped lateral"
                                 " to ({:.1f},{:.1f},{:.1f}) offset={:.2f} m",
                                 target.x, target.y, target.z, snapped_world_x_, snapped_world_y_,
                                 snapped_world_z_, snap_dist);
                }
                // If no free cell found, A* will fail and use direct fallback.
            } else if (snap_valid_) {
                // Reuse the previously computed snap — convert cached world
                // coords back to grid for this replan.
                goal = grid_.world_to_grid(snapped_world_x_, snapped_world_y_, snapped_world_z_);
            }

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
        // When using direct fallback, prefer the cached snapped position over
        // the raw waypoint (obstacle centre) to avoid flying straight into it.
        float goal_x = snap_valid_ ? snapped_world_x_ : target.x;
        float goal_y = snap_valid_ ? snapped_world_y_ : target.y;
        float goal_z = snap_valid_ ? snapped_world_z_ : target.z;

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

    // Cached snapped goal — persists across replans for the same waypoint so the
    // drone does not oscillate as it deflects around an obstacle.
    // Only recomputed when the target waypoint changes.
    float last_target_x_ = 1e9f, last_target_y_ = 1e9f, last_target_z_ = 1e9f;
    float snapped_world_x_ = 0.0f, snapped_world_y_ = 0.0f, snapped_world_z_ = 0.0f;
    bool  snap_valid_ = false;

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
