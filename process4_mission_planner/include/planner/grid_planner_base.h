// process4_mission_planner/include/planner/grid_planner_base.h
// Shared base class for grid-based path planners (A*, D* Lite).
//
// Provides IGridPlanner interface and GridPlannerBase with shared logic:
//   - OccupancyGrid3D ownership and obstacle management
//   - Goal snapping (lateral-preferring + nearest-free fallback)
//   - Path following + velocity command generation (EMA smoothing, speed ramping)
//   - Replan timing
//
// Created as part of Issue #158.
#pragma once

#include "ipc/ipc_types.h"
#include "planner/ipath_planner.h"
#include "planner/mission_fsm.h"
#include "planner/occupancy_grid_3d.h"
#include "util/ilogger.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace drone::planner {

// ─────────────────────────────────────────────────────────────
// GridPlannerConfig — configuration for grid-based planners
// ─────────────────────────────────────────────────────────────

struct GridPlannerConfig {
    float resolution_m       = 0.5f;    // metres per grid cell
    float grid_extent_m      = 50.0f;   // half-extent of grid in each axis
    float inflation_radius_m = 1.5f;    // obstacle inflation radius
    float replan_interval_s  = 1.0f;    // how often to re-run search
    float path_speed_mps     = 2.0f;    // cruise speed along path
    float smoothing_alpha    = 0.35f;   // EMA smoothing for velocity output
    int   max_iterations     = 50000;   // search iteration limit
    float max_search_time_ms = 0.0f;    // wall-clock timeout (0 = disabled)
    float ramp_dist_m        = 3.0f;    // distance to begin speed ramp-down
    float min_speed_mps      = 1.0f;    // minimum speed during ramp-down
    int   snap_search_radius = 8;       // goal snap search radius (grid cells)
    float cell_ttl_s         = 3.0f;    // dynamic obstacle TTL in occupancy grid
    float min_confidence     = 0.3f;    // minimum object confidence for grid insertion
    int   z_band_cells       = 0;       // Z-band limit: restrict search to ±N cells around
                                        // start/goal Z range (0 = unlimited, full 3D search)
    int promotion_hits = 0;             // Promote dynamic cell to static after N observations
                                        // (0 = disabled, no promotion)
    uint32_t radar_promotion_hits = 3;  // Radar update count for immediate static promotion
    float    min_promotion_depth_confidence = 0.3f;  // Min depth confidence for promotion [0-1]
                                                     // Scenario configs override to 0.8 to block
    // camera-only (0.01-0.7), allowing radar-confirmed (1.0)
    float look_ahead_m = 0.0f;                 // Pure-pursuit look-ahead distance along path
                                               // (0 = disabled, use cell-by-cell following)
    bool require_radar_for_promotion = false;  // Require ≥1 radar update for hit-count promotion
                                               // (radar-confirmed path always works regardless)
    int   max_static_cells   = 0;              // Cap on promoted static cells (0 = unlimited)
    bool  yaw_towards_travel = true;           // Face sensors toward next waypoint during NAVIGATE
    float yaw_smoothing_rate = 0.3f;  // EMA alpha for yaw transitions (0=frozen, 1=instant)
    // When `yaw_towards_velocity` is true (and `yaw_towards_travel` also true),
    // the yaw target follows the smoothed velocity command instead of the
    // bee-line to the next waypoint. Points the camera at the actual flight
    // direction during obstacle detours so PATH A can voxelise the obstacle's
    // far face before the drone clips it. Default false to preserve existing
    // behaviour; scenario 33 turns it on (Issue #612).
    bool yaw_towards_velocity = false;

    // Minimum velocity magnitude (m/s) below which `yaw_towards_velocity`
    // falls back to the bee-line-to-goal yaw.  Prevents jitter when the
    // drone is hovering or moving slower than sensor noise.  Tuned against
    // scenario 33 detour profile (nominal detour speed 1.5-2.5 m/s).
    float yaw_velocity_threshold_mps = 0.4f;
    float snap_approach_bias         = 0.5f;  // Approach-direction penalty for snap fallback
    bool  prediction_enabled         = true;  // Enable velocity-based obstacle prediction
    float prediction_dt_s            = 2.0f;  // Prediction horizon (seconds into the future)
    // Issue #635 — PATH A voxel observations become permanent static cells
    // only after this many hits.  Prevents transient voxels (misprojections,
    // one-frame artefacts) from cementing as permanent obstacles that wall
    // off return corridors.  Real walls / pillars get re-observed every
    // frame while in FOV and promote quickly; sparse detections decay via
    // TTL.  Default 3 — mirrors `promotion_hits` magnitude for the
    // detector-observation path.
    int voxel_promotion_hits = 3;
    // Issue #635 — TTL for *promoted* static cells (cells that came via
    // voxel/radar promotion, NOT HD-map).  When > 0, promoted cells decay
    // out of `static_occupied_` if they aren't re-observed for this many
    // seconds.  HD-map cells loaded via `add_static_obstacle` are never
    // affected.  0 = disabled (legacy permanent-promotion behaviour).
    // Recommended: 30-60 s for no-HD-map scenarios so the outbound voxel
    // wake doesn't wall off return corridors.
    float static_cell_ttl_s = 0.0f;
    // Issue #638 Phase 3 — instance-aware voxel promotion gate.  When > 0,
    // PATH A voxels are only written to the grid once their tracked
    // instance has accumulated this many distinct frames of observation.
    // Noise voxels (instance_id == 0) are unconditionally rejected.
    // 0 = disabled (legacy behaviour, voxels write directly to grid).
    int voxel_instance_promotion_observations = 0;
};

// ─────────────────────────────────────────────────────────────
// IGridPlanner — extended interface for grid-based planners
// ─────────────────────────────────────────────────────────────

class IGridPlanner : public IPathPlanner {
public:
    ~IGridPlanner() override = default;

    /// Update the obstacle grid from the latest detected objects.
    virtual void update_obstacles(const drone::ipc::DetectedObjectList& objects,
                                  const drone::ipc::Pose&               pose) = 0;

    /// Pre-load a static obstacle from the HD map (scenario config).
    virtual void add_static_obstacle(float wx, float wy, float radius_m, float height_m) = 0;

    /// Insert world-frame voxels from the PATH A semantic-voxels pipeline
    /// (Epic #520 / Issue #608).  See `OccupancyGrid3D::insert_voxels` for
    /// semantics; this is the IGridPlanner pass-through so the P4 voxel
    /// subscriber thread doesn't need a downcast to GridPlannerBase.
    virtual OccupancyGrid3D::VoxelInsertStats insert_voxels(const drone::ipc::SemanticVoxel* voxels,
                                                            uint32_t n, float clamp_m,
                                                            float min_confidence) = 0;

    /// True if the last planning cycle could not find a search path AND
    /// there was no cached path to keep following — in that case plan()
    /// returns a hover-in-place command (NOT a direct line through
    /// obstacles, despite the legacy method name).  See
    /// `GridPlannerBase::plan()` for the hover branch.
    [[nodiscard]] virtual bool using_direct_fallback() const = 0;

    /// Total number of plan() calls where search failed AND there was no
    /// cached path to fall back to, so the planner hovered in place
    /// (Plan A diagnostic — Issue #645 follow-up).  Persistent non-zero
    /// indicates the obstacle grid is over-occupied for the current
    /// waypoint and the drone cannot make progress without grid
    /// reconfiguration.
    [[nodiscard]] virtual uint64_t hover_fallback_count() const noexcept = 0;

    /// Invalidate the cached path, forcing a full replan on the next tick.
    /// Called after collision recovery to avoid re-using a path that led into an obstacle.
    virtual void invalidate_path() = 0;

    /// Pause/resume promotion to static layer (e.g. during RTL/LAND).
    virtual void set_promotion_paused(bool paused) = 0;

    /// Issue #638 Phase 3 — clear per-instance observation counters.
    /// Call on FSM transitions where the perception context resets
    /// (RTL/LAND, mission abort) so a stale instance_id from earlier in
    /// the mission can't keep promoting after a P2 restart re-mints
    /// fresh IDs that collide with existing counters.  Mirror of the
    /// `set_promotion_paused()` pattern; both protect the grid during
    /// state transitions that perception is unaware of.
    virtual void clear_instance_state() = 0;

    /// Grid diagnostic counters — exposed on the interface so diagnostic
    /// logging in tick_survey/tick_navigate can avoid dynamic_cast.
    [[nodiscard]] virtual size_t grid_occupied_count() const = 0;
    [[nodiscard]] virtual size_t grid_static_count() const   = 0;
    [[nodiscard]] virtual int    grid_promoted_count() const = 0;

    /// Whether the planner has snapped the current goal to an alternate position.
    [[nodiscard]] virtual bool has_snapped_goal() const = 0;

    /// Get the snapped world-frame position {x, y, z} if a snap is active.
    /// Returns std::nullopt when no snap is active (has_snapped_goal() == false).
    [[nodiscard]] virtual std::optional<std::array<float, 3>> snapped_goal_xyz() const = 0;

    /// Issue #624 — post-avoider yaw refresh needs read-only access to the
    /// yaw-towards-velocity feature flag and velocity threshold so the
    /// orchestration layer (mission_state_tick) can honour the same values
    /// the planner was configured with, without duplicating config wiring
    /// or downcasting.  `noexcept` because both are simple scalar reads of
    /// already-stored config values — the no-exceptions contract for
    /// flight-critical accessors (PR #632 review-memory-safety P2).
    [[nodiscard]] virtual bool  yaw_towards_velocity_enabled() const noexcept = 0;
    [[nodiscard]] virtual float yaw_velocity_threshold_mps() const noexcept   = 0;
};

// ─────────────────────────────────────────────────────────────
// GridPlannerBase — shared logic for grid-based planners
//
// Thread safety: all methods must be called from the planning
// loop thread only (P4 is single-threaded). The snap accessors
// has_snapped_goal() and snapped_goal_xyz() are only safe to
// call from the same thread that calls plan().
// ─────────────────────────────────────────────────────────────

class GridPlannerBase : public IGridPlanner {
public:
    /// Issue #624 — expose the yaw-towards-velocity config to the
    /// orchestration layer.  The planner already uses these values
    /// internally; the post-avoider yaw refresh needs to honour the
    /// same feature flag and threshold without downcasting from the
    /// IGridPlanner interface.
    [[nodiscard]] bool yaw_towards_velocity_enabled() const noexcept override {
        return config_.yaw_towards_velocity;
    }
    [[nodiscard]] float yaw_velocity_threshold_mps() const noexcept override {
        return config_.yaw_velocity_threshold_mps;
    }

    explicit GridPlannerBase(const GridPlannerConfig& config = {})
        : config_(config)
        , grid_(config.resolution_m, config.grid_extent_m, config.inflation_radius_m,
                config.cell_ttl_s, config.min_confidence, config.promotion_hits,
                config.radar_promotion_hits, config.min_promotion_depth_confidence,
                config.max_static_cells, config.prediction_enabled, config.prediction_dt_s,
                config.require_radar_for_promotion, config.voxel_promotion_hits,
                config.static_cell_ttl_s, config.voxel_instance_promotion_observations) {}

    void update_obstacles(const drone::ipc::DetectedObjectList& objects,
                          const drone::ipc::Pose&               pose) override {
        grid_.update_from_objects(objects, pose);
    }

    void add_static_obstacle(float wx, float wy, float radius_m, float height_m) override {
        grid_.add_static_obstacle(wx, wy, radius_m, height_m);
        snap_valid_ = false;
    }

    OccupancyGrid3D::VoxelInsertStats insert_voxels(const drone::ipc::SemanticVoxel* voxels,
                                                    uint32_t n, float clamp_m,
                                                    float min_confidence) override {
        auto stats = grid_.insert_voxels(voxels, n, clamp_m, min_confidence);
        // If any voxel landed, invalidate the cached path so D*Lite/A* replan
        // against the updated grid on the next tick.
        if (stats.inserted > 0) {
            snap_valid_ = false;
        }
        return stats;
    }

    [[nodiscard]] bool using_direct_fallback() const override { return direct_fallback_; }

    [[nodiscard]] uint64_t hover_fallback_count() const noexcept override {
        return hover_fallback_count_;
    }

    void set_promotion_paused(bool paused) override { grid_.set_promotion_paused(paused); }
    void clear_instance_state() override { grid_.clear_instance_state(); }

    [[nodiscard]] size_t grid_occupied_count() const override { return grid_.occupied_count(); }
    [[nodiscard]] size_t grid_static_count() const override { return grid_.static_count(); }
    [[nodiscard]] int    grid_promoted_count() const override { return grid_.promoted_count(); }

    void invalidate_path() override {
        cached_path_.clear();
        path_index_ = 0;
        snap_valid_ = false;
        DRONE_LOG_INFO("[Planner] Path cache invalidated — forcing full replan");
    }

    /// Whether the planner has snapped the current goal to an alternate position.
    [[nodiscard]] bool has_snapped_goal() const override { return snap_valid_; }

    /// Get the snapped world-frame position {x, y, z}.
    /// Updated when a replan occurs; remains stable between replan cycles as
    /// long as the goal is unchanged. Returns std::nullopt when no snap is
    /// active. The acceptance radius remains wp.radius.
    [[nodiscard]] std::optional<std::array<float, 3>> snapped_goal_xyz() const override {
        if (!snap_valid_) return std::nullopt;
        return snapped_xyz_;
    }

    /// Get the current obstacle grid (for diagnostics/testing).
    [[nodiscard]] const OccupancyGrid3D& grid() const { return grid_; }

    /// Get the cached path (for diagnostics/testing).
    [[nodiscard]] const std::vector<std::array<float, 3>>& cached_path() const {
        return cached_path_;
    }

    drone::ipc::TrajectoryCmd plan(const drone::ipc::Pose& pose, const Waypoint& target) override {
        drone::ipc::TrajectoryCmd cmd{};
        cmd.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        cmd.valid = true;

        float px = static_cast<float>(pose.translation[0]);
        float py = static_cast<float>(pose.translation[1]);
        float pz = static_cast<float>(pose.translation[2]);

        // ── Re-plan on interval, missing path, or target change ─────
        // target_has_changed() is the single source of truth for target-change
        // detection — also used by snap_goal() to invalidate the snap cache.
        auto now         = std::chrono::steady_clock::now();
        bool need_replan = target_has_changed(target) || cached_path_.empty() ||
                           path_index_ >= cached_path_.size() ||
                           std::chrono::duration<float>(now - last_plan_time_).count() >
                               config_.replan_interval_s;

        if (need_replan) {
            GridCell start = grid_.world_to_grid(px, py, pz);
            GridCell goal  = grid_.world_to_grid(target.x, target.y, target.z);

            // ── Goal snapping — cached per waypoint ──────────
            GridCell pre_snap = goal;
            goal              = snap_goal(start, goal, target);
            if (goal != pre_snap) {
                DRONE_LOG_INFO("[PlanBase] Goal snapped: ({},{},{}) → ({},{},{})", pre_snap.x,
                               pre_snap.y, pre_snap.z, goal.x, goal.y, goal.z);
            }

            // ── Delegate to subclass search ──────────────────
            std::vector<std::array<float, 3>> search_path;
            auto                              replan_t0 = std::chrono::steady_clock::now();
            bool                              found     = do_search(start, goal, search_path);
            auto                              replan_ms = std::chrono::duration<float, std::milli>(
                                 std::chrono::steady_clock::now() - replan_t0)
                                 .count();

            if (found && search_path.size() >= 2) {
                // DEBUG(#234): Log path direction vs goal direction
                auto& p1      = search_path[1];
                float path_dx = p1[0] - px, path_dy = p1[1] - py;
                float goal_dx = target.x - px, goal_dy = target.y - py;
                float path_mag = std::sqrt(path_dx * path_dx + path_dy * path_dy);
                float goal_mag = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);
                float dot      = 0.0f;
                if (path_mag > 0.01f && goal_mag > 0.01f) {
                    dot = (path_dx * goal_dx + path_dy * goal_dy) / (path_mag * goal_mag);
                }
                DRONE_LOG_INFO(
                    "[PlanBase] Replan: path_dir=({:.1f},{:.1f}) goal_dir=({:.1f},{:.1f}) "
                    "dot={:.2f} path_pts={} search={:.0f}ms",
                    path_dx, path_dy, goal_dx, goal_dy, dot, search_path.size(), replan_ms);

                // Reject backward paths — when the first step points away from
                // the goal (dot < 0) and we have a usable cached path, keep the
                // old path rather than flying in the wrong direction (Issue #237).
                if (dot < 0.0f && !cached_path_.empty()) {
                    DRONE_LOG_WARN("[PlanBase] Rejecting backward path (dot={:.2f}) — "
                                   "keeping cached path ({} pts, idx {})",
                                   dot, cached_path_.size(), path_index_);
                    last_plan_time_ = now;  // wait for next replan cycle
                } else {
                    cached_path_     = std::move(search_path);
                    path_index_      = 1;  // skip start cell
                    last_plan_time_  = now;
                    direct_fallback_ = false;
                }
            } else {
                if (!cached_path_.empty()) {
                    // Keep following the last good path — much safer than
                    // a direct line which flies through obstacles.
                    direct_fallback_ = false;
                    last_plan_time_  = now;  // avoid hammering failed replans
                    DRONE_LOG_WARN("[PlanBase] Search failed — keeping last good path "
                                   "({} pts, idx {}) (took {:.0f}ms)",
                                   cached_path_.size(), path_index_, replan_ms);
                } else {
                    // No cached path at all — hover in place (zero velocity)
                    // rather than flying a direct line through obstacles.
                    direct_fallback_ = true;
                    ++hover_fallback_count_;
                    DRONE_LOG_WARN("[PlanBase] Search failed, no cached path — "
                                   "hovering in place (took {:.0f}ms, total "
                                   "hover-fallback count={})",
                                   replan_ms, hover_fallback_count_);
                }
            }
        }

        // ── Hover if no path exists and search failed ────────
        if (direct_fallback_ && cached_path_.empty()) {
            // No path found and no cached path — hold XY position,
            // but still drive toward target altitude.
            smooth_vx_ *= (1.0f - config_.smoothing_alpha);
            smooth_vy_ *= (1.0f - config_.smoothing_alpha);
            float dz_hover = target.z - pz;
            float vz_max   = std::min(config_.path_speed_mps, target.speed);
            float raw_vz_h = (std::abs(dz_hover) > 0.1f) ? std::clamp(dz_hover, -vz_max, vz_max)
                                                         : 0.0f;
            smooth_vz_     = config_.smoothing_alpha * raw_vz_h +
                         (1.0f - config_.smoothing_alpha) * smooth_vz_;
            cmd.velocity_x = smooth_vx_;
            cmd.velocity_y = smooth_vy_;
            cmd.velocity_z = smooth_vz_;
            cmd.target_x   = target.x;
            cmd.target_y   = target.y;
            cmd.target_z   = target.z;
            cmd.target_yaw = target.yaw;

            if (diag_tick_++ % 50 == 0) {
                DRONE_LOG_WARN("[PlanBase] HOVERING — no valid path to "
                               "({:.1f},{:.1f},{:.1f}), vel=({:.2f},{:.2f},{:.2f})",
                               target.x, target.y, target.z, smooth_vx_, smooth_vy_, smooth_vz_);
            }
            return cmd;
        }

        // ── Follow path or go direct ────────────────────────
        float goal_x = snap_valid_ ? snapped_xyz_[0] : target.x;
        float goal_y = snap_valid_ ? snapped_xyz_[1] : target.y;

        if (!cached_path_.empty() && path_index_ < cached_path_.size()) {
            if (config_.look_ahead_m > 0.0f) {
                // Pure-pursuit mode: advance conservatively (half-cell), carrot
                // handles smooth following — no need for aggressive skipping.
                while (path_index_ + 1 < cached_path_.size()) {
                    auto& wp      = cached_path_[path_index_];
                    float dx      = wp[0] - px;
                    float dy      = wp[1] - py;
                    float dist_xy = std::sqrt(dx * dx + dy * dy);
                    if (dist_xy >= config_.resolution_m * 0.5f) break;
                    ++path_index_;
                }
                auto carrot = find_carrot(px, py, pz);
                goal_x      = carrot[0];
                goal_y      = carrot[1];
            } else {
                // Legacy cell-by-cell: advance when close, target next waypoint
                while (path_index_ + 1 < cached_path_.size()) {
                    auto& wp      = cached_path_[path_index_];
                    float dx      = wp[0] - px;
                    float dy      = wp[1] - py;
                    float dist_xy = std::sqrt(dx * dx + dy * dy);
                    if (dist_xy >= config_.resolution_m * 1.5f) break;
                    ++path_index_;
                }
                auto& wp = cached_path_[path_index_];
                goal_x   = wp[0];
                goal_y   = wp[1];
            }
        }

        // ── Velocity command toward current goal ────────────
        // XY: follow path waypoints (D* Lite routing around obstacles)
        // Z:  always drive toward target waypoint altitude — D* Lite paths
        //     use the drone's current Z which can drift/collapse, so altitude
        //     must be controlled independently.
        float dx      = goal_x - px;
        float dy      = goal_y - py;
        float dz_alt  = target.z - pz;  // altitude error toward waypoint Z
        float dist_xy = std::sqrt(dx * dx + dy * dy);

        float raw_vx = 0.0f, raw_vy = 0.0f, raw_vz = 0.0f;
        if (dist_xy > 0.01f) {
            float speed = std::min(config_.path_speed_mps, target.speed);
            if (dist_xy < config_.ramp_dist_m && path_index_ + 1 >= cached_path_.size()) {
                speed = config_.min_speed_mps +
                        (speed - config_.min_speed_mps) * (dist_xy / config_.ramp_dist_m);
            }
            raw_vx = (dx / dist_xy) * speed;
            raw_vy = (dy / dist_xy) * speed;
        }
        // Altitude: proportional correction clamped to cruise speed
        if (std::abs(dz_alt) > 0.1f) {
            float vz_max = std::min(config_.path_speed_mps, target.speed);
            raw_vz       = std::clamp(dz_alt, -vz_max, vz_max);
        }

        // EMA smoothing — XY uses the configured alpha; Z uses a faster
        // alpha (0.8) so altitude corrections propagate in 1-2 ticks rather
        // than 5+.  Altitude is a simple proportional controller, not path-
        // following, so sluggish response lets small perturbations compound.
        smooth_vx_ = config_.smoothing_alpha * raw_vx +
                     (1.0f - config_.smoothing_alpha) * smooth_vx_;
        smooth_vy_ = config_.smoothing_alpha * raw_vy +
                     (1.0f - config_.smoothing_alpha) * smooth_vy_;
        constexpr float kAltAlpha = 0.8f;
        smooth_vz_                = kAltAlpha * raw_vz + (1.0f - kAltAlpha) * smooth_vz_;

        cmd.velocity_x = smooth_vx_;
        cmd.velocity_y = smooth_vy_;
        cmd.velocity_z = smooth_vz_;
        cmd.target_x   = target.x;
        cmd.target_y   = target.y;
        cmd.target_z   = target.z;
        cmd.target_yaw = target.yaw;

        // Yaw-towards-travel: face sensors toward the next waypoint.
        // Uses waypoint direction (not path step) to avoid feedback loop
        // where camera view → grid → planner → yaw → camera causes circling.
        //
        // When `yaw_towards_velocity` is true, override the bee-line-to-goal
        // direction with the actual smoothed velocity command — the camera
        // tracks the *real* flight direction during obstacle detours so PATH A
        // can voxelise the obstacle's far face before the drone clips it
        // (Issue #612 — observed: drone routed around one cube face, then
        // clipped the unseen face on the way back to the waypoint).  Reverts
        // to bee-line direction when velocity magnitude is too small to
        // estimate a heading, so we don't yaw randomly during hover.
        if (config_.yaw_towards_travel) {
            float ytt_dx = target.x - px;
            float ytt_dy = target.y - py;
            if (config_.yaw_towards_velocity) {
                const float vmag = std::sqrt(smooth_vx_ * smooth_vx_ + smooth_vy_ * smooth_vy_);
                if (vmag > config_.yaw_velocity_threshold_mps) {
                    ytt_dx = smooth_vx_;
                    ytt_dy = smooth_vy_;
                }
            }
            const float ytt_dist = std::sqrt(ytt_dx * ytt_dx + ytt_dy * ytt_dy);
            if (ytt_dist > 0.5f) {
                constexpr float kPi     = 3.14159265358979323846f;
                const float     desired = std::atan2(ytt_dy, ytt_dx);
                if (!smooth_yaw_init_) {
                    smooth_yaw_      = desired;
                    smooth_yaw_init_ = true;
                }
                // Shortest angular difference with wraparound
                float diff = desired - smooth_yaw_;
                if (diff > kPi) diff -= 2.0f * kPi;
                if (diff < -kPi) diff += 2.0f * kPi;
                smooth_yaw_ += config_.yaw_smoothing_rate * diff;
                // Re-normalise to [-pi, pi]
                if (smooth_yaw_ > kPi) smooth_yaw_ -= 2.0f * kPi;
                if (smooth_yaw_ < -kPi) smooth_yaw_ += 2.0f * kPi;
                cmd.target_yaw = smooth_yaw_;
            }
        }

        // Periodic diagnostic: velocity, path state, position
        if (diag_tick_++ % 50 == 0) {
            DRONE_LOG_INFO("[PlanBase] pos=({:.1f},{:.1f},{:.1f}) vel=({:.2f},{:.2f},{:.2f}) "
                           "raw=({:.2f},{:.2f},{:.2f}) path={}/{} fallback={} "
                           "goal_xy=({:.1f},{:.1f})",
                           px, py, pz, smooth_vx_, smooth_vy_, smooth_vz_, raw_vx, raw_vy, raw_vz,
                           path_index_, cached_path_.size(), direct_fallback_ ? 1 : 0, goal_x,
                           goal_y);
        }

        return cmd;
    }

protected:
    /// Subclass implements the actual search algorithm.
    /// @param start     Start cell in grid coordinates.
    /// @param goal      Goal cell (already snapped).
    /// @param out_world_path  Output: world-coordinate path if found.
    /// @return true if a path was found.
    virtual bool do_search(const GridCell& start, const GridCell& goal,
                           std::vector<std::array<float, 3>>& out_world_path) = 0;

    GridPlannerConfig config_;
    OccupancyGrid3D   grid_;

private:
    /// Single source of truth for target-change detection.  Called from
    /// plan() to force a replan and from snap_goal() to invalidate the snap
    /// cache.  Compares against stored last_target_ and returns true if the
    /// waypoint position has changed.  Does NOT update the stored target —
    /// that is done in snap_goal() after the change is consumed.
    [[nodiscard]] bool target_has_changed(const Waypoint& target) const {
        return target.x != last_target_x_ || target.y != last_target_y_ ||
               target.z != last_target_z_;
    }

    /// Goal snapping — find nearest free cell when goal is occupied.
    GridCell snap_goal(const GridCell& start, const GridCell& raw_goal, const Waypoint& target) {
        if (target_has_changed(target)) {
            snap_valid_    = false;
            last_target_x_ = target.x;
            last_target_y_ = target.y;
            last_target_z_ = target.z;
        }

        // Invalidate snap cache if static cell count changed significantly (>10%).
        // No guard on last_snap_static_count_ > 0: snaps made on an empty grid
        // must also invalidate when the grid later grows.
        const size_t current_static = grid_.static_count();
        if (snap_valid_) {
            const float change = std::abs(static_cast<float>(current_static) -
                                          static_cast<float>(last_snap_static_count_));
            if (change / static_cast<float>(std::max(last_snap_static_count_, size_t{1})) > 0.1f) {
                snap_valid_ = false;
            }
        }

        GridCell goal = raw_goal;

        if (!snap_valid_ && (grid_.is_occupied(goal) || !grid_.in_bounds(goal))) {
            const GridCell orig_goal = goal;
            bool           snapped   = false;

            // Lateral-preferring snap
            const float adx  = static_cast<float>(orig_goal.x - start.x);
            const float ady  = static_cast<float>(orig_goal.y - start.y);
            const float amag = std::sqrt(adx * adx + ady * ady);

            if (amag > 0.5f) {
                const float ax = adx / amag, ay = ady / amag;
                const float lx = -ay, ly = ax;
                const float rx = ay, ry = -ax;

                const float cross = static_cast<float>(start.x - orig_goal.x) * ay -
                                    static_cast<float>(start.y - orig_goal.y) * ax;
                const bool first_left = (cross >= 0.0f);

                for (int r = 1; r <= config_.snap_search_radius && !snapped; ++r) {
                    for (int side = 0; side < 2 && !snapped; ++side) {
                        const bool  try_left = (side == 0) ? first_left : !first_left;
                        const float fx       = try_left ? lx : rx;
                        const float fy       = try_left ? ly : ry;
                        GridCell    c{orig_goal.x + static_cast<int>(std::round(fx * r)),
                                   orig_goal.y + static_cast<int>(std::round(fy * r)), orig_goal.z};
                        if (grid_.in_bounds(c) && !grid_.is_occupied(c)) {
                            goal    = c;
                            snapped = true;
                        }
                    }
                }

                if (snapped) {
                    const float snap_cells = std::sqrt(
                        static_cast<float>((goal.x - orig_goal.x) * (goal.x - orig_goal.x) +
                                           (goal.y - orig_goal.y) * (goal.y - orig_goal.y)));
                    const float snap_dist_m     = snap_cells * grid_.resolution();
                    const float approach_dist_m = amag * grid_.resolution();
                    const float cruise_spd      = std::max(0.5f, config_.path_speed_mps);
                    const float time_to_obs_s   = approach_dist_m / cruise_spd;
                    const float time_to_deflect = snap_dist_m / 3.0f;
                    if (time_to_obs_s - time_to_deflect < 0.5f) {
                        DRONE_LOG_WARN("[Planner] Snap {:.1f} m lateral but only {:.1f}s to "
                                       "obstacle (need {:.1f}s) — tight margin",
                                       snap_dist_m, time_to_obs_s, time_to_deflect);
                    }
                }
            }

            // Fallback: nearest free cell with approach-direction bias.
            // Penalises cells on the far side of the goal (away from drone)
            // so the snapped position is navigable, not just geometrically close.
            if (!snapped) {
                float     best_score = 1e9f;
                const int radius     = std::clamp(config_.snap_search_radius, 0, 100);
                // Approach vector: drone (start) → goal, normalised
                const float anx = (amag > 0.5f) ? adx / amag : 0.0f;
                const float any = (amag > 0.5f) ? ady / amag : 0.0f;

                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        const float dist_sq = static_cast<float>(dx * dx + dy * dy);
                        if (dist_sq == 0.0f) continue;
                        GridCell c{orig_goal.x + dx, orig_goal.y + dy, orig_goal.z};
                        if (!grid_.in_bounds(c) || grid_.is_occupied(c)) continue;

                        const float dist = std::sqrt(dist_sq);
                        // Dot of (candidate offset from goal) with approach direction.
                        // Positive = candidate is past the goal (away from drone) — penalise.
                        // Negative = candidate is between drone and goal — preferred.
                        const float dot = static_cast<float>(dx) * anx +
                                          static_cast<float>(dy) * any;
                        const float penalty = (dot > 0.0f) ? dot * config_.snap_approach_bias
                                                           : 0.0f;
                        const float score   = dist + penalty;

                        if (score < best_score) {
                            best_score = score;
                            goal       = c;
                            snapped    = true;
                        }
                    }
                }
            }

            if (snapped) {
                snapped_xyz_ = grid_.grid_to_world(goal);
                // NaN/Inf guard — reject snaps that produce non-finite coordinates.
                // This can happen if grid_to_world produces degenerate values from
                // extreme cell indices near grid boundaries.
                if (!std::isfinite(snapped_xyz_[0]) || !std::isfinite(snapped_xyz_[1]) ||
                    !std::isfinite(snapped_xyz_[2])) {
                    snap_valid_ = false;
                    DRONE_LOG_WARN("[Planner] Snapped position contains NaN/Inf — "
                                   "rejecting snap, using original goal");
                    return raw_goal;
                }
                snap_valid_             = true;
                last_snap_static_count_ = current_static;
                const float snap_dist =
                    std::sqrt(static_cast<float>((goal.x - orig_goal.x) * (goal.x - orig_goal.x) +
                                                 (goal.y - orig_goal.y) * (goal.y - orig_goal.y))) *
                    grid_.resolution();
                DRONE_LOG_INFO("[Planner] WP({:.1f},{:.1f},{:.1f}) occupied — snapped lateral"
                               " to ({:.1f},{:.1f},{:.1f}) offset={:.2f} m",
                               target.x, target.y, target.z, snapped_xyz_[0], snapped_xyz_[1],
                               snapped_xyz_[2], snap_dist);
            }
        } else if (snap_valid_) {
            goal = grid_.world_to_grid(snapped_xyz_[0], snapped_xyz_[1], snapped_xyz_[2]);
        }

        return goal;
    }

    /// Pure-pursuit carrot: walk forward along cached_path_ from path_index_
    /// by look_ahead_m and return the interpolated point.
    [[nodiscard]] std::array<float, 3> find_carrot(float px, float py, float pz) const {
        // Start from the current waypoint on the path
        size_t idx      = path_index_;
        float  remain_m = config_.look_ahead_m;

        // Consume distance from drone to first path waypoint
        {
            auto& wp   = cached_path_[idx];
            float dx   = wp[0] - px;
            float dy   = wp[1] - py;
            float dz   = wp[2] - pz;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist >= remain_m) {
                // Carrot is between drone and first waypoint
                float t = remain_m / dist;
                return {px + dx * t, py + dy * t, pz + dz * t};
            }
            remain_m -= dist;
        }

        // Walk along subsequent path segments
        while (idx + 1 < cached_path_.size()) {
            auto& a    = cached_path_[idx];
            auto& b    = cached_path_[idx + 1];
            float dx   = b[0] - a[0];
            float dy   = b[1] - a[1];
            float dz   = b[2] - a[2];
            float slen = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (slen >= remain_m) {
                // Interpolate within this segment
                float t = remain_m / slen;
                return {a[0] + dx * t, a[1] + dy * t, a[2] + dz * t};
            }
            remain_m -= slen;
            ++idx;
        }

        // Exhausted path — return last waypoint
        return cached_path_.back();
    }

    // Cached path
    std::vector<std::array<float, 3>> cached_path_;
    size_t                            path_index_ = 0;
    // Despite the legacy name, direct_fallback_=true means the planner is
    // HOVERING in place because search failed and there's no cached path to
    // keep following.  No code path actually flies a direct line through
    // obstacles — the hover branch in plan() at the `direct_fallback_ &&
    // cached_path_.empty()` guard catches it.  Renaming the flag is a wider
    // refactor (touches IGridPlanner, derived classes, tests, log lines);
    // deferred to a follow-up — Plan A's observability fix corrects the
    // user-facing message text without that blast radius.
    bool direct_fallback_ = false;
    // Plan A diagnostic — count of plan() calls where search returned no
    // path AND there was no cached path to fall back to, so the planner
    // hovered in place.  Persistent non-zero indicates the obstacle grid is
    // over-occupied for the current waypoint.  Surfaced by
    // `hover_fallback_count()` for run-report telemetry.
    uint64_t                              hover_fallback_count_ = 0;
    std::chrono::steady_clock::time_point last_plan_time_{};

    // Cached snapped goal — single source of truth for snapped world position.
    // Valid when snap_valid_ is true.  Written in snap_goal(), read by plan()
    // path-following and externally via snapped_goal_xyz().
    float                last_target_x_ = 1e9f, last_target_y_ = 1e9f, last_target_z_ = 1e9f;
    std::array<float, 3> snapped_xyz_ = {0.0f, 0.0f, 0.0f};
    bool                 snap_valid_  = false;
    size_t last_snap_static_count_    = 0;  // for snap cache invalidation on grid changes

    // Velocity smoothing state
    float smooth_vx_ = 0.0f;
    float smooth_vy_ = 0.0f;
    float smooth_vz_ = 0.0f;

    // Yaw-towards-travel state
    float smooth_yaw_      = 0.0f;
    bool  smooth_yaw_init_ = false;

    // Diagnostic tick counter
    uint64_t diag_tick_ = 0;
};

}  // namespace drone::planner
