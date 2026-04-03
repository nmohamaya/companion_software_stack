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
                             float cell_ttl_s = 3.0f, float min_confidence = 0.3f,
                             int promotion_hits = 0, uint32_t radar_promotion_hits = 3,
                             float min_promotion_depth_confidence = 0.5f, int max_static_cells = 0,
                             bool prediction_enabled = true, float prediction_dt_s = 2.0f)
        : resolution_(resolution)
        , half_extent_cells_(static_cast<int>(extent / resolution))
        , inflation_cells_(std::max(1, static_cast<int>(std::ceil(inflation / resolution))))
        , cell_ttl_ns_(static_cast<uint64_t>(cell_ttl_s * 1e9f))
        , min_confidence_(min_confidence)
        , promotion_hits_(promotion_hits)
        , radar_promotion_hits_(radar_promotion_hits)
        , min_promotion_depth_confidence_(min_promotion_depth_confidence)
        , max_static_cells_(max_static_cells)
        , prediction_enabled_(prediction_enabled)
        , prediction_dt_s_(prediction_dt_s) {}

    /// Force-clear all *dynamic* (TTL-based) obstacles (for testing / reset).
    /// Static HD-map obstacles are left untouched; call clear_static() to remove those.
    void clear() { clear_dynamic(); }

    /// Clear only the dynamic / TTL-based obstacle layer.
    void clear_dynamic() {
        occupied_.clear();
        hit_count_.clear();
    }

    /// Clear only the static HD-map obstacle layer.
    void clear_static() {
        static_occupied_.clear();
        hd_map_static_count_ = 0;
    }

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
        const size_t added = static_occupied_.size() - before;
        hd_map_static_count_ += added;
        spdlog::info("[HD-map] Static obstacle at ({:.1f},{:.1f}) r={:.1f}m h={:.1f}m "
                     "-> {} new cells (total static: {})",
                     wx, wy, radius_m, height_m, added, static_occupied_.size());
    }

    /// Insert obstacles from a detected object list.
    /// Updates timestamps for observed cells; stale cells expire after TTL.
    void update_from_objects(const drone::ipc::DetectedObjectList& objects,
                             const drone::ipc::Pose&               drone_pose) {
        const auto     now    = std::chrono::steady_clock::now();
        const uint64_t now_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());

        // Drone grid cell — used for self-exclusion zone.
        // Objects whose inflated region would overlap the drone's cell are skipped
        // to prevent the planner start node from being blocked.
        const GridCell drone_cell = world_to_grid(static_cast<float>(drone_pose.translation[0]),
                                                  static_cast<float>(drone_pose.translation[1]),
                                                  static_cast<float>(drone_pose.translation[2]));

        // Stamp newly detected cells
        int accepted       = 0;
        int suppressed     = 0;
        int excluded_cells = 0;
        for (uint32_t i = 0; i < objects.num_objects; ++i) {
            const auto& obj = objects.objects[i];
            if (obj.confidence < min_confidence_) continue;

            GridCell center = world_to_grid(obj.position_x, obj.position_y, obj.position_z);

            // Known-obstacle suppression: split into two behaviours.
            // 1. Dynamic cells are ALWAYS created (close-range detections
            //    during navigation are more accurate than survey positions).
            // 2. Promotion is SUPPRESSED near existing static cells to
            //    prevent parallax from growing runaway walls of promoted
            //    cells that block D* Lite paths (Issue #237).
            bool skip_promotion = false;
            if (promotion_hits_ > 0 && near_static_cell_(center)) {
                ++suppressed;
                skip_promotion = true;
            }
            ++accepted;

            // Per-object inflation: use estimated radius if available (from
            // camera bbox + radar range back-projection), otherwise fall back
            // to the configured inflation radius.  This gives accurate grid
            // footprints for radar-confirmed obstacles.
            const int obj_inflation =
                (obj.estimated_radius_m > 0.0f)
                    ? std::max(1, static_cast<int>(std::ceil(
                                      (obj.estimated_radius_m + resolution_ * 0.5f) / resolution_)))
                    : inflation_cells_;

            // Per-object promotion eligibility (invariant across inflated cells).
            const bool depth_ok    = obj.depth_confidence >= min_promotion_depth_confidence_;
            const bool can_promote = !skip_promotion && depth_ok;

            // 2D disk inflation: only inflate in XY at the object's Z level.
            // The path planner runs a 2D horizontal search, so vertical
            // inflation wastes memory and floods the grid. A single-layer
            // disk at the detection Z is sufficient because the search is
            // snapped to the drone's flight altitude.
            //
            // Uses shared inflate_disk_at_cell_() helper (same logic as
            // prediction inflation) to prevent the two paths from diverging.
            inflate_disk_at_cell_(center, obj_inflation, drone_cell, now_ns, &excluded_cells, &obj,
                                  !can_promote);

            // ── Velocity-based prediction inflation (Issue #256) ────
            // For moving objects, inflate cells along the velocity vector
            // from the current position to the predicted position at
            // prediction_dt_s in the future.  This gives D* Lite early
            // awareness of where dynamic obstacles WILL BE.
            if (prediction_enabled_ && prediction_dt_s_ > 0.0f) {
                const float vx = obj.velocity_x;
                const float vy = obj.velocity_y;
                const float vz = obj.velocity_z;
                // 3D speed threshold — include all velocity components
                const float speed_sq = vx * vx + vy * vy + vz * vz;
                // Only predict for objects with meaningful 3D speed (>0.5 m/s)
                constexpr float kMinPredictionSpeedSq = 0.25f;  // 0.5^2
                if (speed_sq > kMinPredictionSpeedSq) {
                    const float pred_x = obj.position_x + vx * prediction_dt_s_;
                    const float pred_y = obj.position_y + vy * prediction_dt_s_;
                    const float pred_z = obj.position_z + vz * prediction_dt_s_;

                    // Walk from current to predicted position in grid-cell steps
                    // using Bresenham-style interpolation, inflating a disk at each.
                    // Clamp predicted cell to grid bounds before computing steps
                    // to prevent unbounded iteration for fast objects.
                    GridCell pred_cell = world_to_grid(pred_x, pred_y, pred_z);
                    pred_cell.x = std::clamp(pred_cell.x, -half_extent_cells_, half_extent_cells_);
                    pred_cell.y = std::clamp(pred_cell.y, -half_extent_cells_, half_extent_cells_);
                    pred_cell.z = std::clamp(pred_cell.z, -half_extent_cells_, half_extent_cells_);

                    // Cap interpolation steps to bound worst-case CPU cost.
                    constexpr int kMaxPredictionSteps = 200;
                    const int     raw_steps           = std::max({std::abs(pred_cell.x - center.x),
                                                                  std::abs(pred_cell.y - center.y),
                                                                  std::abs(pred_cell.z - center.z), 1});
                    const int     steps               = std::min(raw_steps, kMaxPredictionSteps);

                    for (int s = 1; s <= steps; ++s) {
                        const float t  = static_cast<float>(s) / static_cast<float>(steps);
                        const int   ix = center.x +
                                       static_cast<int>(std::round(
                                           t * static_cast<float>(pred_cell.x - center.x)));
                        const int iy = center.y +
                                       static_cast<int>(std::round(
                                           t * static_cast<float>(pred_cell.y - center.y)));
                        const int iz = center.z +
                                       static_cast<int>(std::round(
                                           t * static_cast<float>(pred_cell.z - center.z)));

                        inflate_disk_at_cell_({ix, iy, iz}, obj_inflation, drone_cell, now_ns);
                    }
                    ++total_predictions_applied_;
                }
            }
        }

        // Expire stale cells (only when we actually have new data to avoid
        // running the prune loop with no benefit on empty-detection frames)
        if (objects.num_objects > 0 || !occupied_.empty()) {
            for (auto it = occupied_.begin(); it != occupied_.end();) {
                if (now_ns - it->second > cell_ttl_ns_) {
                    changed_cells_.push_back({it->first, false});
                    hit_count_.erase(it->first);  // reset observation count
                    it = occupied_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        // Diagnostic: log grid state periodically
        if (diag_tick_++ % 100 == 0 && objects.num_objects > 0) {
            spdlog::info("[Grid] {} objs (accepted={}, suppressed={}, excluded_cells={}), "
                         "{} dynamic, {} static (promoted={}, hd_map={}, max={}, predictions={}), "
                         "drone=({},{},{})",
                         objects.num_objects, accepted, suppressed, excluded_cells,
                         occupied_.size(), static_occupied_.size(), promoted_count_,
                         hd_map_static_count_, max_static_cells_, total_predictions_applied_,
                         drone_cell.x, drone_cell.y, drone_cell.z);
            for (uint32_t i = 0; i < std::min(objects.num_objects, uint32_t{8}); ++i) {
                const auto& obj = objects.objects[i];
                if (obj.confidence >= min_confidence_) {
                    spdlog::debug("[Grid]   obj[{}] pos=({:.1f},{:.1f},{:.1f}) conf={:.2f}", i,
                                  obj.position_x, obj.position_y, obj.position_z, obj.confidence);
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
    [[nodiscard]] int    promoted_count() const { return promoted_count_; }
    [[nodiscard]] int    max_static_cells() const { return max_static_cells_; }
    [[nodiscard]] size_t hd_map_static_count() const { return hd_map_static_count_; }
    /// Cumulative count of objects that had velocity-based prediction applied
    /// across all calls to update_from_objects() (lifetime counter, never reset).
    [[nodiscard]] int  total_predictions_applied() const { return total_predictions_applied_; }
    [[nodiscard]] bool prediction_enabled() const { return prediction_enabled_; }

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
    /// Check whether a grid cell is at or adjacent (Chebyshev ≤ 1) to any
    /// already-promoted static cell.  Used for promotion suppression:
    /// once an obstacle has been promoted, further detections at the same
    /// grid location skip promotion to avoid growing the static footprint
    /// beyond the real obstacle boundary (parallax/multi-track artefacts).
    [[nodiscard]] bool near_static_cell_(const GridCell& center) const {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (static_occupied_.count({center.x + dx, center.y + dy, center.z}) > 0) {
                    return true;
                }
            }
        }
        return false;
    }

    /// Inflate a 2D disk of occupied cells at `center` with the given radius.
    /// Used by both current-position inflation and prediction inflation paths
    /// to guarantee identical logic (prevents the two paths from diverging).
    ///
    /// @param center           Grid cell at the disk centre.
    /// @param inflation_cells  Disk radius in grid cells.
    /// @param drone_cell       Drone position for self-exclusion.
    /// @param now_ns           Current timestamp for TTL.
    /// @param excluded_out     If non-null, incremented for each self-excluded cell.
    /// @param obj              If non-null, promotion logic is applied (current-position path).
    /// @param skip_promotion   Whether to skip promotion for this object (includes depth_ok gate).
    void inflate_disk_at_cell_(const GridCell& center, int inflation_cells,
                               const GridCell& drone_cell, uint64_t now_ns,
                               int*                              excluded_out   = nullptr,
                               const drone::ipc::DetectedObject* obj            = nullptr,
                               bool                              skip_promotion = true) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                if (dx * dx + dy * dy <= inflation_cells * inflation_cells) {
                    GridCell c{center.x + dx, center.y + dy, center.z};
                    // Self-exclusion: never mark the drone's own cell or any cell
                    // in its immediate 3x3 neighbourhood (Chebyshev distance <= 1)
                    // as occupied — this prevents the planner start node from being
                    // blocked while still populating the rest of the obstacle footprint.
                    if (std::abs(c.x - drone_cell.x) <= 1 && std::abs(c.y - drone_cell.y) <= 1 &&
                        c.z == drone_cell.z) {
                        if (excluded_out != nullptr) ++(*excluded_out);
                        continue;
                    }
                    if (!in_bounds(c)) continue;
                    // Already promoted — no need to track in dynamic layer
                    if (static_occupied_.count(c) > 0) continue;

                    bool was_absent = (occupied_.count(c) == 0);
                    occupied_[c]    = now_ns;
                    if (was_absent) {
                        changed_cells_.push_back({c, true});
                    }

                    // Promotion logic only for current-position observations,
                    // not for predicted cells.
                    if (obj != nullptr && !skip_promotion) {
                        const bool radar_confirmed = obj->radar_update_count >=
                                                     radar_promotion_hits_;

                        // Check if promotion cap is reached (HD-map cells excluded).
                        const std::size_t promoted_static_count =
                            static_occupied_.size() > hd_map_static_count_
                                ? static_occupied_.size() - hd_map_static_count_
                                : 0;
                        const bool cap_reached = max_static_cells_ > 0 &&
                                                 promoted_static_count >=
                                                     static_cast<std::size_t>(max_static_cells_);

                        if (radar_confirmed && !cap_reached) {
                            if (static_occupied_.count(c) == 0) {
                                static_occupied_.insert(c);
                                occupied_.erase(c);
                                hit_count_.erase(c);
                                if (!was_absent) {
                                    changed_cells_.push_back({c, true});
                                }
                                ++promoted_count_;
                            }
                        } else if (promotion_hits_ > 0 && !cap_reached) {
                            int& hits = hit_count_[c];
                            ++hits;
                            if (hits >= promotion_hits_) {
                                static_occupied_.insert(c);
                                occupied_.erase(c);
                                hit_count_.erase(c);
                                ++promoted_count_;
                            }
                        }
                    }
                }
            }
        }
    }

    float    resolution_;
    int      half_extent_cells_;
    int      inflation_cells_;
    uint64_t cell_ttl_ns_;
    float    min_confidence_;
    int      promotion_hits_{0};  // promote to static after this many observations (0 = disabled)
    uint32_t radar_promotion_hits_{3};               // radar updates for immediate static promotion
    float    min_promotion_depth_confidence_{0.5f};  // min depth confidence for promotion
    int      max_static_cells_{0};                   // cap on promoted static cells (0 = unlimited)
    int      promoted_count_{0};                     // total cells promoted (diagnostic)
    size_t   hd_map_static_count_{0};                // HD-map cells (excluded from cap)
    bool     prediction_enabled_{true};              // enable velocity-based prediction inflation
    float    prediction_dt_s_{2.0f};                 // prediction horizon in seconds
    // Cumulative count of objects with velocity-based prediction applied (never reset).
    int      total_predictions_applied_{0};
    uint64_t diag_tick_{0};  // for periodic diagnostic logging
    // HD-map layer: permanent cells loaded from scenario config at startup.
    // Never expire — represent known world geometry.
    std::unordered_set<GridCell, GridCellHash> static_occupied_;
    // Camera-confirmation layer: cells observed by perception, expire after TTL.
    // Refreshed each detection cycle; also catches unexpected/dynamic obstacles.
    std::unordered_map<GridCell, uint64_t, GridCellHash> occupied_;
    // Hit counter for promotion: tracks how many update cycles each cell has been observed.
    // Cells that reach promotion_hits_ are moved to static_occupied_ and removed from here.
    std::unordered_map<GridCell, int, GridCellHash> hit_count_;
    // Change tracking for incremental planners (D* Lite).
    std::vector<std::pair<GridCell, bool>> changed_cells_;
};

}  // namespace drone::planner
