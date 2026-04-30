// process4_mission_planner/include/planner/occupancy_grid_3d.h
// 3D occupancy grid for obstacle representation, used by D* Lite planner.
//
// Originally extracted from astar_planner.h as part of Issue #158.
// A* removed in Issue #203 — D* Lite supersedes it.
#pragma once

#include "ipc/ipc_types.h"
#include "util/ilogger.h"

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
                             float min_promotion_depth_confidence = 0.3f, int max_static_cells = 0,
                             bool prediction_enabled = true, float prediction_dt_s = 2.0f,
                             bool require_radar_for_promotion = false, int voxel_promotion_hits = 3,
                             float static_cell_ttl_s                     = 0.0f,
                             int   voxel_instance_promotion_observations = 0)
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
        , prediction_dt_s_(prediction_dt_s)
        , require_radar_for_promotion_(require_radar_for_promotion)
        , voxel_promotion_hits_(voxel_promotion_hits)
        , static_cell_ttl_ns_(static_cast<uint64_t>(std::max(0.0f, static_cell_ttl_s) * 1e9f))
        , voxel_instance_promotion_observations_(voxel_instance_promotion_observations) {}

    /// Force-clear all *dynamic* (TTL-based) obstacles (for testing / reset).
    /// Static HD-map obstacles are left untouched; call clear_static() to remove those.
    void clear() { clear_dynamic(); }

    /// Clear only the dynamic / TTL-based obstacle layer.
    void clear_dynamic() {
        occupied_.clear();
        hit_count_.clear();
    }

    /// Clear all static obstacles (both HD-map and runtime-promoted).
    void clear_static() {
        static_occupied_.clear();
        hd_map_cells_.clear();
        hd_map_static_count_ = 0;
        static_cell_timestamps_.clear();
        promoted_count_ = 0;
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
        const size_t before = static_occupied_.size();
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
                            hd_map_cells_.insert(c);
                        }
                    }
                }
        const size_t added = static_occupied_.size() - before;
        // Derive HD-map count from hd_map_cells_ (authoritative) so the
        // two can never diverge — even if overlapping HD-map obstacles share cells.
        hd_map_static_count_ = hd_map_cells_.size();
        DRONE_LOG_INFO("[HD-map] Static obstacle at ({:.1f},{:.1f}) r={:.1f}m h={:.1f}m "
                       "-> {} new cells (total static: {})",
                       wx, wy, radius_m, height_m, added, static_occupied_.size());
    }

    /// Insert world-frame voxels from the PATH A pipeline (Epic #520 / Issue #608).
    ///
    /// The voxels arrive on `/semantic_voxels`, already confidence-scored and
    /// mask-gated by MaskDepthProjector on the P2 side.  That upstream work
    /// means we intentionally **bypass** the `promotion_hits` filter used in
    /// `update_from_objects()` — those hits guard against ghost cells from
    /// the detector/depth pipeline's false positives on ground texture, which
    /// the mask-aware path has already filtered out.
    ///
    /// Position clamp: the final safety guard before writing to the grid.
    /// A malformed sender (or a numerical edge case in MaskDepthProjector)
    /// could emit positions outside the world extent, which would overflow
    /// grid indexing.  Any voxel whose |position| exceeds `clamp_m` on any
    /// axis is dropped.  Addresses the security P3 deferred from PR #609.
    ///
    /// Cells written here land in the static layer but are NOT recorded in
    /// `hd_map_cells_` — an upstream pipeline regression (e.g. a future SAM
    /// misfire) can still be cleared by `clear_static()`.
    ///
    /// @return `{inserted, clamped_dropped, low_confidence_dropped, out_of_bounds, cap_blocked, instance_skipped}`
    struct VoxelInsertStats {
        size_t inserted               = 0;
        size_t clamped_dropped        = 0;
        size_t low_confidence_dropped = 0;
        size_t out_of_bounds          = 0;
        // Issue #635 patch — voxel cells that hit the promotion threshold
        // but couldn't promote because `max_static_cells_` was reached.
        // These cells stay in the dynamic bucket and can still promote
        // later if the cap frees up via the static-TTL sweep.
        size_t cap_blocked = 0;
        // Issue #638 Phase 3 — voxels rejected by the per-instance promotion
        // gate.  Either instance_id == 0 (noise from Phase 1's min_pts gate
        // or Phase 2's tracker bypass) or the tracked instance hasn't been
        // observed in enough frames yet (`voxel_instance_promotion_observations_`).
        // Always 0 when instance promotion is disabled (default).
        size_t instance_skipped = 0;
    };
    VoxelInsertStats insert_voxels(const drone::ipc::SemanticVoxel* voxels, uint32_t n,
                                   float clamp_m, float min_confidence) {
        VoxelInsertStats s;
        if (voxels == nullptr || n == 0) return s;
        const float clamp = std::max(0.0f, clamp_m);

        // Issue #638 Phase 3 — instance-aware promotion gate.
        // When `voxel_instance_promotion_observations_ > 0`, voxels are
        // only written to the grid if they belong to a tracked instance
        // (instance_id > 0) that has been observed in enough frames.
        // Per-frame observation counting: each call to insert_voxels
        // counts as one frame; each distinct instance_id seen in this
        // batch increments its observation count by exactly 1.  After
        // `voxel_instance_promotion_observations_` distinct frames, all
        // voxels for that instance pass the gate.
        if (voxel_instance_promotion_observations_ > 0) {
            // P2-B from PR #641 review: hoisted to a member to amortise
            // the per-batch heap allocation across calls (was a fresh
            // unordered_set every call on the flight-critical voxel
            // subscriber tick).  `clear()` preserves the bucket capacity.
            batch_instances_scratch_.clear();
            for (uint32_t i = 0; i < n; ++i) {
                if (voxels[i].instance_id != 0)
                    batch_instances_scratch_.insert(voxels[i].instance_id);
            }
            for (uint32_t id : batch_instances_scratch_) {
                ++instances_[id].observation_count;
            }
        }

        // Each voxel carries a point sample of an obstacle surface.  Without
        // inflation a 5 m cube hit by 4 depth samples produces only 2-3
        // unique grid cells and the avoider's repulsive field has gaps big
        // enough for the drone to squeeze through (observed during scenario
        // 33 testing).  Inflate symmetrically in 3D — voxels are point
        // samples with no height prior.  Radius 1 cell → 3×3×3-minus-corners
        // neighbourhood (~7 cells per voxel) keeps the grid bounded while
        // still filling the gap between scattered mask probes.
        constexpr int kInflate  = 1;
        constexpr int kInflate2 = kInflate * kInflate;

        // Ground-plane reject threshold.  At 2 m grid resolution, voxels
        // with world z < 0.3 m are ground textures / shadows / floor patches
        // that EdgeContourSAM latches onto — promoting them to static would
        // make every patch of grass an obstacle.  Scenario drone cruise
        // altitude is 5 m, so no real navigation-relevant obstacle lives
        // below 0.3 m from this camera's vantage point.
        constexpr float kMinObstacleZ = 0.3f;

        for (uint32_t i = 0; i < n; ++i) {
            const auto& v = voxels[i];
            // Finite-value guard (PR #614 fault-recovery review).  `abs(NaN) >
            // clamp` is false by IEEE-754, so NaN positions used to slip
            // through the clamp below and land in `world_to_grid`, where
            // `round(NaN/res)` is implementation-defined — producing phantom
            // cells at arbitrary grid indices with no log trail.  Reject
            // non-finite values explicitly and count under the clamp bucket.
            if (!std::isfinite(v.position_x) || !std::isfinite(v.position_y) ||
                !std::isfinite(v.position_z)) {
                ++s.clamped_dropped;
                continue;
            }
            // Position clamp (PR #609 review P3 deferred to consumer boundary).
            // Reject voxels whose |x|, |y|, or |z| exceeds the world extent.
            if (clamp > 0.0f && (std::abs(v.position_x) > clamp || std::abs(v.position_y) > clamp ||
                                 std::abs(v.position_z) > clamp)) {
                ++s.clamped_dropped;
                continue;
            }
            if (v.confidence < min_confidence) {
                ++s.low_confidence_dropped;
                continue;
            }
            // Ground-plane filter — count under clamped_dropped to keep the
            // stats tuple compact; the diagnostic log groups all boundary
            // drops together anyway.
            if (v.position_z < kMinObstacleZ) {
                ++s.clamped_dropped;
                continue;
            }
            // Issue #638 Phase 3 — per-voxel instance gate.  When enabled,
            // noise (id=0) is dropped, and tracked instances must have
            // accumulated `voxel_instance_promotion_observations_` frames
            // before any of their voxels reach the grid.  Real obstacles
            // pass once they're well-observed; transient flickers never do.
            if (voxel_instance_promotion_observations_ > 0) {
                if (v.instance_id == 0) {
                    ++s.instance_skipped;
                    continue;
                }
                const auto it = instances_.find(v.instance_id);
                if (it == instances_.end() ||
                    it->second.observation_count <
                        static_cast<uint32_t>(voxel_instance_promotion_observations_)) {
                    ++s.instance_skipped;
                    continue;
                }
            }
            const GridCell center = world_to_grid(v.position_x, v.position_y, v.position_z);
            if (!in_bounds(center)) {
                ++s.out_of_bounds;
                continue;
            }
            // Issue #635 — timestamp for TTL decay.  Voxels that aren't
            // re-observed within `cell_ttl_ns_` expire via the same
            // loop that handles detector observations (see line 426).
            // Without this, PATH A voxels cemented as permanent static
            // cells — the drone's outbound flight deposited an
            // everlasting wake that walled off its return corridor
            // (scenario 33 stuck-at-(24.5, 26.1) failure mode, 2026-04-24).
            const uint64_t now_ns =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
            // Small 3D inflation — every cell within `kInflate` of the sample.
            // Most neighbouring inserts collide with cells already present
            // from nearby voxels, so this is idempotent on repeated observations.
            for (int dz = -kInflate; dz <= kInflate; ++dz) {
                for (int dy = -kInflate; dy <= kInflate; ++dy) {
                    for (int dx = -kInflate; dx <= kInflate; ++dx) {
                        if (dx * dx + dy * dy + dz * dz > kInflate2) continue;
                        const GridCell c{center.x + dx, center.y + dy, center.z + dz};
                        if (!in_bounds(c)) continue;

                        // Already-permanent cell (HD-map seed or previously-
                        // promoted voxel) — nothing more to do.  Still
                        // refreshes the dynamic timestamp below so that if
                        // clear_static() ever wipes it, the dynamic layer
                        // keeps the obstacle alive until TTL decides.
                        const bool was_absent = occupied_.count(c) == 0 &&
                                                static_occupied_.count(c) == 0;

                        // Dynamic-bucket refresh: write current timestamp.
                        // Existing TTL-sweep at occupied_.erase() / line 426
                        // handles decay when this cell stops being observed.
                        occupied_[c] = now_ns;
                        if (was_absent) {
                            changed_cells_.push_back({c, true});
                            ++s.inserted;
                        }

                        // Issue #635 — refresh the static-cell timestamp if
                        // this voxel re-observes a previously-promoted cell.
                        // HD-map cells aren't in `static_cell_timestamps_`,
                        // so they're skipped (they're permanent by design);
                        // promoted cells get their TTL extended for as long
                        // as PATH A keeps confirming the obstacle.
                        if (auto it = static_cell_timestamps_.find(c);
                            it != static_cell_timestamps_.end()) {
                            it->second = now_ns;
                        }

                        // Promotion: after `voxel_promotion_hits_` distinct
                        // observations a cell becomes permanent — real walls
                        // should cement, transient frame-artefacts should
                        // decay.  Mirrors `update_from_objects()` promotion
                        // path (line 615) except this one doesn't gate on
                        // radar-confirmed (PATH A is the sole static source
                        // in scenarios that enable voxel_input).
                        //
                        // Issue #635 patch — honour `max_static_cells_` here
                        // too.  Previously the cap was only checked on the
                        // detector/radar promotion path; voxel promotion was
                        // unbounded, producing 5000+ promoted cells in <15 s
                        // on scenario 33 (cap was 500).  Without this guard
                        // the static layer overwhelms the planner before
                        // any TTL sweep can decay the wake.
                        if (voxel_promotion_hits_ > 0 && static_occupied_.count(c) == 0) {
                            int& hits = hit_count_[c];
                            ++hits;
                            if (hits >= voxel_promotion_hits_) {
                                const std::size_t promoted_static_count =
                                    static_occupied_.size() > hd_map_static_count_
                                        ? static_occupied_.size() - hd_map_static_count_
                                        : 0;
                                const bool cap_reached =
                                    max_static_cells_ > 0 &&
                                    promoted_static_count >=
                                        static_cast<std::size_t>(max_static_cells_);
                                if (!cap_reached) {
                                    static_occupied_.insert(c);
                                    occupied_.erase(c);
                                    hit_count_.erase(c);
                                    ++promoted_count_;
                                    // Record promotion time when static-cell
                                    // TTL is enabled — the sweep keys off
                                    // this map.  HD-map cells stay timestamp-
                                    // free so they can never decay.
                                    if (static_cell_ttl_ns_ > 0) {
                                        static_cell_timestamps_[c] = now_ns;
                                    }
                                } else {
                                    // Cap reached — leave cell in dynamic
                                    // bucket (still observed, still TTL-decays
                                    // normally).  hit_count_ stays so a later
                                    // re-observation after a sweep frees a
                                    // slot can still promote.
                                    ++s.cap_blocked;
                                }
                            }
                        }
                    }
                }
            }
        }
        // Issue #635 patch — also sweep the static layer on the voxel
        // arrival cadence (~13 Hz), not just on the detector tick (~10 Hz
        // and gated on `objects.num_objects > 0`).  Faster decay response
        // means cells the drone has flown past clear out before the next
        // search cycle, instead of carrying a multi-second wake.
        const uint64_t sweep_now_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        sweep_static_cells(sweep_now_ns);
        return s;
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
            // 2. Promotion is SUPPRESSED near known obstacles to prevent
            //    runaway walls of promoted cells that block D* Lite paths.
            //
            // Two suppression checks (Issue #389):
            //  a) Near HD-map cells — always suppress (both radar and camera
            //     promotion). Radar-confirmed detections near known obstacles
            //     are redundant confirmations that saturate the grid.
            //  b) Near any static cell — suppress camera-only promotion
            //     (original Issue #237 behaviour).
            bool skip_promotion = false;
            if (!hd_map_cells_.empty() && near_hd_map_cell_(center)) {
                ++suppressed;
                skip_promotion = true;
            } else if (promotion_hits_ > 0 && near_static_cell_(center)) {
                ++suppressed;
                skip_promotion = true;
            }
            ++accepted;

            // Per-object inflation: use estimated radius only for radar-confirmed
            // objects (where range is reliable), otherwise fall back to the
            // configured inflation radius.  This gives accurate grid footprints
            // for radar-confirmed obstacles.
            //
            // Cap: camera-only objects use at most the configured inflation
            // radius.  Without radar, estimated_radius is derived from the
            // (possibly bogus) camera depth — ground features can produce
            // radii of 10-20m, inflating a single object to 300+ cells and
            // flooding the dynamic grid.  Radar-confirmed objects have
            // reliable range, so their back-projected radius is trusted.
            const bool has_radar_size = obj.radar_update_count > 0 && obj.estimated_radius_m > 0.0f;
            const int  obj_inflation =
                has_radar_size
                     ? std::max(1, static_cast<int>(std::ceil(
                                      (obj.estimated_radius_m + resolution_ * 0.5f) / resolution_)))
                     : inflation_cells_;

            // Per-object promotion eligibility (invariant across inflated cells).
            //
            // Issue #645 — radar-confirmed tracks bypass `promotion_paused_`
            // when `allow_radar_promotion_when_paused_` is enabled.  Rationale:
            // the original "pause detector promotion when voxel_input is on"
            // decision (#635 / #636) was driven by camera-only detections
            // producing ghost cells.  Radar gives accurate range from the
            // first hit (no convergence latency), so radar-confirmed tracks
            // are the safe subset to allow through.  PATH A voxels remain
            // the primary static-cell source for non-radar-detected geometry;
            // radar adds strategic-routing visibility for cubes/pillars/walls
            // that radar sees clearly (which the planner otherwise misses,
            // since it doesn't consume the avoider's DetectedObjectList).
            const bool depth_ok     = obj.depth_confidence >= min_promotion_depth_confidence_;
            const bool radar_bypass = allow_radar_promotion_when_paused_ && obj.has_radar &&
                                      obj.radar_update_count > 0;
            const bool effective_pause = promotion_paused_ && !radar_bypass;
            const bool can_promote     = !skip_promotion && depth_ok && !effective_pause;

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

        // Issue #635 — sweep promoted static cells whose TTL has expired.
        // HD-map cells (in `hd_map_cells_`, never given a timestamp) are
        // immune by construction.  Cheap when `static_cell_ttl_ns_ == 0`
        // (early-out) so legacy scenarios pay nothing.
        sweep_static_cells(now_ns);

        // Diagnostic: log grid state periodically
        if (diag_tick_++ % 100 == 0 && objects.num_objects > 0) {
            DRONE_LOG_INFO(
                "[Grid] {} objs (accepted={}, suppressed={}, excluded_cells={}), "
                "{} dynamic, {} static (promoted={}, hd_map={}, max={}, predictions={}), "
                "drone=({},{},{})",
                objects.num_objects, accepted, suppressed, excluded_cells, occupied_.size(),
                static_occupied_.size(), promoted_count_, hd_map_static_count_, max_static_cells_,
                total_predictions_applied_, drone_cell.x, drone_cell.y, drone_cell.z);
            for (uint32_t i = 0; i < std::min(objects.num_objects, uint32_t{8}); ++i) {
                const auto& obj = objects.objects[i];
                if (obj.confidence >= min_confidence_) {
                    DRONE_LOG_DEBUG("[Grid]   obj[{}] pos=({:.1f},{:.1f},{:.1f}) conf={:.2f}", i,
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
    [[nodiscard]] size_t hd_map_cell_count() const { return hd_map_cells_.size(); }
    /// Cumulative count of objects that had velocity-based prediction applied
    /// across all calls to update_from_objects() (lifetime counter, never reset).
    [[nodiscard]] int  total_predictions_applied() const { return total_predictions_applied_; }
    [[nodiscard]] bool prediction_enabled() const { return prediction_enabled_; }

    /// Pause/resume promotion to static layer.  When paused, dynamic cells
    /// are still created (reactive avoidance works) but nothing new promotes.
    /// Intended for RTL/LAND phases where the drone descends toward a known-safe
    /// location and ground-feature detections would pollute the static layer.
    void               set_promotion_paused(bool paused) { promotion_paused_ = paused; }
    [[nodiscard]] bool promotion_paused() const { return promotion_paused_; }

    /// Issue #645 — when promotion_paused_ is on (typically during voxel_input
    /// scenarios where PATH A voxels are the sole static-cell source), allow
    /// radar-confirmed detected objects to bypass the pause and promote into
    /// the static layer.  Radar gives accurate range from the first hit, so
    /// radar-confirmed tracks don't suffer the convergence-latency / ghost-
    /// cell problem that originally motivated the pause.  Lets the strategic
    /// path planner see radar-detected cubes/pillars/walls that would
    /// otherwise only reach the (reactive) ObstacleAvoider3D via the
    /// DetectedObjectList feed.  Default false = legacy behaviour.
    void set_allow_radar_promotion_when_paused(bool allow) {
        allow_radar_promotion_when_paused_ = allow;
    }
    [[nodiscard]] bool allow_radar_promotion_when_paused() const {
        return allow_radar_promotion_when_paused_;
    }

    /// Issue #638 Phase 3 — number of stable instances tracked for the
    /// instance-promotion gate.  Useful for diagnostics; 0 when the gate
    /// is disabled.
    [[nodiscard]] size_t tracked_instance_count() const { return instances_.size(); }

    /// Number of tracked instances that have crossed the promotion
    /// threshold (their voxels are currently writing to the grid).
    [[nodiscard]] size_t promoted_instance_count() const {
        if (voxel_instance_promotion_observations_ <= 0) return 0;
        const auto threshold = static_cast<uint32_t>(voxel_instance_promotion_observations_);
        size_t     n         = 0;
        for (const auto& [id, rec] : instances_) {
            if (rec.observation_count >= threshold) ++n;
        }
        return n;
    }

    /// Clear per-instance observation counters.  Call on RTL/LAND so a
    /// stale instance from earlier in the mission can't keep promoting
    /// after the perception context resets.
    void clear_instance_state() { instances_.clear(); }

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
    /// Issue #635 — sweep promoted static cells whose TTL has expired.
    ///
    /// Walks `static_cell_timestamps_` (only contains promoted cells, never
    /// HD-map cells) and evicts any entry older than `static_cell_ttl_ns_`.
    /// HD-map cells are immune by construction — they never receive a
    /// timestamp.  No-op when `static_cell_ttl_ns_ == 0` (legacy behaviour).
    ///
    /// Eviction:
    ///   1. Drop the cell from `static_occupied_` (path becomes traversable).
    ///   2. Drop its timestamp.
    ///   3. Push to `changed_cells_` so D* Lite re-plans through the cleared
    ///      space on the next tick.
    ///   4. Decrement `promoted_count_` so the diagnostic counter tracks
    ///      live promotions, not lifetime promotions.  (Lifetime cumulative
    ///      counters could be added separately if needed.)
    ///
    /// Cost: O(|static_cell_timestamps_|).  In practice the set is small
    /// (capped by `max_static_cells_` when set, else bounded by perception
    /// observation rate × TTL).  Called from BOTH `update_from_objects()`
    /// (~10 Hz, detector tick) AND `insert_voxels()` (~13 Hz, voxel batch
    /// arrival).  The voxel-side call is the responsive one — it ensures
    /// the wake decays at sensor rate even when the detector is silent
    /// (e.g. an empty `DetectedObjectList` on frames with no COCO objects).
    void sweep_static_cells(uint64_t now_ns) {
        if (static_cell_ttl_ns_ == 0) return;
        for (auto it = static_cell_timestamps_.begin(); it != static_cell_timestamps_.end();) {
            if (now_ns - it->second > static_cell_ttl_ns_) {
                const GridCell c = it->first;
                static_occupied_.erase(c);
                changed_cells_.push_back({c, false});
                if (promoted_count_ > 0) --promoted_count_;
                it = static_cell_timestamps_.erase(it);
            } else {
                ++it;
            }
        }
    }

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

    /// Check if a cell is near an HD-map (pre-loaded) static obstacle.
    /// Used to suppress radar promotion near known obstacles — those detections
    /// are redundant confirmations that would otherwise saturate the grid (Issue #389).
    [[nodiscard]] bool near_hd_map_cell_(const GridCell& center) const {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (hd_map_cells_.count({center.x + dx, center.y + dy, center.z}) > 0) {
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
                                // Issue #635 — record promotion time so
                                // detector/radar-promoted cells also decay
                                // (uniform behaviour with the voxel path).
                                if (static_cell_ttl_ns_ > 0) {
                                    static_cell_timestamps_[c] = now_ns;
                                }
                            }
                        } else if (promotion_hits_ > 0 && !cap_reached) {
                            // When require_radar_for_promotion_ is set, the hit-count
                            // path requires at least one radar update on the object.
                            // Camera-only detections stay dynamic (TTL-based) — they
                            // still provide immediate avoidance but cannot permanently
                            // pollute the grid with false ground-feature promotions.
                            const bool has_any_radar = obj->radar_update_count > 0;
                            if (!require_radar_for_promotion_ || has_any_radar) {
                                int& hits = hit_count_[c];
                                ++hits;
                                if (hits >= promotion_hits_) {
                                    static_occupied_.insert(c);
                                    occupied_.erase(c);
                                    hit_count_.erase(c);
                                    ++promoted_count_;
                                    if (static_cell_ttl_ns_ > 0) {
                                        static_cell_timestamps_[c] = now_ns;
                                    }
                                }
                            } else {
                                // Camera-only: clear any stale hit count so a
                                // later radar-confirmed object in the same cell
                                // doesn't inherit accumulated hits.
                                hit_count_.erase(c);
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
    float    min_promotion_depth_confidence_{0.3f};  // min depth confidence for promotion
    int      max_static_cells_{0};                   // cap on promoted static cells (0 = unlimited)
    int      promoted_count_{0};                     // total cells promoted (diagnostic)
    size_t   hd_map_static_count_{0};                // HD-map cells (excluded from cap)
    bool     promotion_paused_{false};               // pause promotion during RTL/LAND
    // Issue #645 — radar-confirmed bypass for promotion_paused_; see
    // `set_allow_radar_promotion_when_paused()` for rationale.
    bool  allow_radar_promotion_when_paused_{false};
    bool  prediction_enabled_{true};            // enable velocity-based prediction inflation
    float prediction_dt_s_{2.0f};               // prediction horizon in seconds
    bool  require_radar_for_promotion_{false};  // hit-count path needs ≥1 radar update
    // Issue #635 — PATH A voxel observations become permanent static cells
    // only after this many hits.  Lower values pollute the grid with
    // transient detections; higher values require multi-frame robustness
    // before a voxel "cements".  Default 3 (same magnitude as
    // promotion_hits_ for the detector path).
    int voxel_promotion_hits_{3};
    // Issue #635 — TTL for *promoted* static cells (cells that came via
    // voxel/radar promotion, not HD-map).  Cells in `static_occupied_` whose
    // entry in `static_cell_timestamps_` is older than this expire on the
    // next sweep.  HD-map cells (those in `hd_map_cells_`) are timestamp-
    // free and never decay.  0 = disabled (legacy behaviour, all promoted
    // cells live forever).  Recommended: 30-60 s for no-HD-map scenarios so
    // the outbound flight's voxel wake doesn't wall off the return corridor.
    uint64_t static_cell_ttl_ns_{0};
    // Issue #638 Phase 3 — instance-aware promotion gate for PATH A voxels.
    // When > 0, voxels carrying instance_id from Phase 2's tracker are only
    // written to the grid once their instance has been observed in this many
    // distinct frames.  Noise (instance_id == 0 from Phase 1's min_pts gate)
    // is rejected unconditionally.  0 = disabled, voxels write directly to
    // the grid as before (current default).
    int voxel_instance_promotion_observations_{0};
    // Per-instance observation tracking for the Phase 3 gate.  Keyed by the
    // stable instance_id from Phase 2's tracker.  Each entry counts the
    // number of distinct insert_voxels() batches that have referenced this
    // instance — when count >= voxel_instance_promotion_observations_, the
    // instance's voxels start writing to the grid.
    struct InstanceRecord {
        uint32_t observation_count = 0;
    };
    std::unordered_map<uint32_t, InstanceRecord> instances_;
    // Per-batch scratch for `insert_voxels()` distinct-instance counting.
    // Hoisted out of the function body to amortise allocation across calls
    // (P2-B from PR #641 review).  `clear()` preserves bucket capacity.
    std::unordered_set<uint32_t> batch_instances_scratch_;
    // Cumulative count of objects with velocity-based prediction applied (never reset).
    int      total_predictions_applied_{0};
    uint64_t diag_tick_{0};  // for periodic diagnostic logging
    // HD-map layer: permanent cells loaded from scenario config at startup.
    // Never expire — represent known world geometry.
    std::unordered_set<GridCell, GridCellHash> static_occupied_;
    // Subset of static_occupied_ that came from add_static_obstacle() (HD-map).
    // Used to suppress radar promotion near known obstacles (Issue #389).
    std::unordered_set<GridCell, GridCellHash> hd_map_cells_;
    // Issue #635 — timestamp side-table for *promoted* static cells (NOT
    // HD-map).  Lookup semantics:
    //   - cell ∈ static_occupied_ AND ∈ hd_map_cells_   → permanent (HD-map)
    //   - cell ∈ static_occupied_ AND ∈ static_cell_timestamps_
    //                                                   → decays after TTL
    //   - cell ∈ static_occupied_ AND in neither        → permanent (legacy
    //         path: created when static_cell_ttl_ns_ == 0, before this map
    //         was wired)
    // Refreshed when a voxel re-observes a promoted cell; entries are
    // dropped when the cell decays out of static_occupied_.
    std::unordered_map<GridCell, uint64_t, GridCellHash> static_cell_timestamps_;
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
