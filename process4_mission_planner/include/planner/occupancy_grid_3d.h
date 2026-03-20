// process4_mission_planner/include/planner/occupancy_grid_3d.h
// 3D occupancy grid for obstacle representation, used by D* Lite planner.
//
// Originally extracted from astar_planner.h as part of Issue #158.
// A* removed in Issue #203 — D* Lite supersedes it.
#pragma once

#include "ipc/ipc_types.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::planner {

// ─────────────────────────────────────────────────────────────
// GridCell — a cell index in the 3D grid
// ─────────────────────────────────────────────────────────────

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

// ─────────────────────────────────────────────────────────────
// 26-connected neighbour tables (used by D* Lite grid search)
// ─────────────────────────────────────────────────────────────

/// 26-connected neighbours (face + edge + corner adjacent).
inline constexpr int kNeighbours[26][3] = {
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

/// Movement costs: 1.0 for face, sqrt(2) for edge, sqrt(3) for corner.
inline constexpr float kCosts[26] = {
    1.0f,   1.0f,   1.0f,   1.0f,   1.0f,   1.0f,                   // face
    1.414f, 1.414f, 1.414f, 1.414f, 1.414f, 1.414f,                 // edge
    1.414f, 1.414f, 1.414f, 1.414f, 1.414f, 1.414f,                 // edge
    1.732f, 1.732f, 1.732f, 1.732f, 1.732f, 1.732f, 1.732f, 1.732f  // corner
};

// ─────────────────────────────────────────────────────────────
// OccupancyGrid3D — lightweight voxel grid for obstacle representation
// ─────────────────────────────────────────────────────────────

/// 3D occupancy grid backed by a hash map of occupied cells with TTL.
///
/// Key design: obstacles are NEVER cleared unless they have not been
/// observed for `cell_ttl_s` seconds (default 3 s).  This prevents a
/// single missed detection frame from wiping the grid and causing the planner
/// to re-route straight through a known obstacle.
class OccupancyGrid3D {
public:
    explicit OccupancyGrid3D(float resolution = 0.5f, float extent = 50.0f, float inflation = 1.5f,
                             float cell_ttl_s = 3.0f)
        : resolution_(resolution)
        , half_extent_cells_(static_cast<int>(extent / resolution))
        , inflation_cells_(std::max(1, static_cast<int>(std::ceil(inflation / resolution))))
        , cell_ttl_ns_(static_cast<uint64_t>(cell_ttl_s * 1e9f)) {}

    /// Force-clear all *dynamic* (TTL-based) obstacles (for testing / reset).
    /// Static HD-map obstacles are left untouched; call clear_static() to remove those.
    void clear() { clear_dynamic(); }

    /// Clear only the dynamic / TTL-based obstacle layer.
    void clear_dynamic() { occupied_.clear(); }

    /// Clear only the static HD-map obstacle layer.
    void clear_static() { static_occupied_.clear(); }

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
                        if (in_bounds(c)) {
                            auto [it, inserted] = static_occupied_.insert(c);
                            if (inserted) {
                                changed_cells_.push_back({c, true});
                            }
                        }
                    }
                }
        spdlog::info("[HD-map] Static obstacle at ({:.1f},{:.1f}) r={:.1f}m h={:.1f}m "
                     "-> {} new cells (total static: {})",
                     wx, wy, radius_m, height_m, static_occupied_.size() - before,
                     static_occupied_.size());
    }

    /// Insert obstacles from a detected object list.
    /// Updates timestamps for observed cells; stale cells expire after TTL.
    void update_from_objects(const drone::ipc::DetectedObjectList& objects,
                             const drone::ipc::Pose& /* drone_pose */) {
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
                            if (in_bounds(c)) {
                                bool was_absent = (occupied_.count(c) == 0);
                                occupied_[c]    = now_ns;
                                if (was_absent) {
                                    changed_cells_.push_back({c, true});
                                }
                            }
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
                    changed_cells_.push_back({it->first, false});
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

    /// Return and clear the list of cells that changed since the last drain.
    /// Each entry is {cell, is_now_occupied}.
    /// D* Lite uses this to know which edges changed.
    std::vector<std::pair<GridCell, bool>> drain_changes() {
        std::vector<std::pair<GridCell, bool>> result;
        result.swap(changed_cells_);
        return result;
    }

    /// Number of pending (undrained) changes.
    [[nodiscard]] size_t pending_changes() const { return changed_cells_.size(); }

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
    // Change tracking for incremental planners (D* Lite).
    std::vector<std::pair<GridCell, bool>> changed_cells_;
};

}  // namespace drone::planner
