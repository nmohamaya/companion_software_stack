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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::planner {

// ─────────────────────────────────────────────────────────────
// GridPlannerConfig — configuration for grid-based planners
// ─────────────────────────────────────────────────────────────

struct GridPlannerConfig {
    float resolution_m       = 0.5f;   // metres per grid cell
    float grid_extent_m      = 50.0f;  // half-extent of grid in each axis
    float inflation_radius_m = 1.5f;   // obstacle inflation radius
    float replan_interval_s  = 1.0f;   // how often to re-run search
    float path_speed_mps     = 2.0f;   // cruise speed along path
    float smoothing_alpha    = 0.35f;  // EMA smoothing for velocity output
    int   max_iterations     = 50000;  // search iteration limit
    float max_search_time_ms = 0.0f;   // wall-clock timeout (0 = disabled)
    float ramp_dist_m        = 3.0f;   // distance to begin speed ramp-down
    float min_speed_mps      = 1.0f;   // minimum speed during ramp-down
    int   snap_search_radius = 8;      // goal snap search radius (grid cells)
    float cell_ttl_s         = 3.0f;   // dynamic obstacle TTL in occupancy grid
    float min_confidence     = 0.3f;   // minimum object confidence for grid insertion
    int   z_band_cells       = 0;      // Z-band limit: restrict search to ±N cells around
                                       // start/goal Z range (0 = unlimited, full 3D search)
    int promotion_hits = 0;            // Promote dynamic cell to static after N observations
                                       // (0 = disabled, no promotion)
    float look_ahead_m = 0.0f;         // Pure-pursuit look-ahead distance along path
                                       // (0 = disabled, use cell-by-cell following)
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

    /// True if the last planning cycle could not find a search path
    /// and fell back to a direct line toward the target.
    [[nodiscard]] virtual bool using_direct_fallback() const = 0;
};

// ─────────────────────────────────────────────────────────────
// GridPlannerBase — shared logic for grid-based planners
// ─────────────────────────────────────────────────────────────

class GridPlannerBase : public IGridPlanner {
public:
    explicit GridPlannerBase(const GridPlannerConfig& config = {})
        : config_(config)
        , grid_(config.resolution_m, config.grid_extent_m, config.inflation_radius_m,
                config.cell_ttl_s, config.min_confidence, config.promotion_hits) {}

    void update_obstacles(const drone::ipc::DetectedObjectList& objects,
                          const drone::ipc::Pose&               pose) override {
        grid_.update_from_objects(objects, pose);
    }

    void add_static_obstacle(float wx, float wy, float radius_m, float height_m) override {
        grid_.add_static_obstacle(wx, wy, radius_m, height_m);
        snap_valid_ = false;
    }

    [[nodiscard]] bool using_direct_fallback() const override { return direct_fallback_; }

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

        // ── Re-plan on interval or if no path ───────────────
        auto now         = std::chrono::steady_clock::now();
        bool need_replan = cached_path_.empty() || path_index_ >= cached_path_.size() ||
                           std::chrono::duration<float>(now - last_plan_time_).count() >
                               config_.replan_interval_s;

        if (need_replan) {
            GridCell start = grid_.world_to_grid(px, py, pz);
            GridCell goal  = grid_.world_to_grid(target.x, target.y, target.z);

            // ── Goal snapping — cached per waypoint ──────────
            GridCell pre_snap = goal;
            goal              = snap_goal(start, goal, target);
            if (goal != pre_snap) {
                spdlog::info("[PlanBase] Goal snapped: ({},{},{}) → ({},{},{})", pre_snap.x,
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
                spdlog::info("[PlanBase] Replan: path_dir=({:.1f},{:.1f}) goal_dir=({:.1f},{:.1f}) "
                             "dot={:.2f} path_pts={} search={:.0f}ms",
                             path_dx, path_dy, goal_dx, goal_dy, dot, search_path.size(),
                             replan_ms);

                // Reject backward paths — when the first step points away from
                // the goal (dot < 0) and we have a usable cached path, keep the
                // old path rather than flying in the wrong direction (Issue #237).
                if (dot < 0.0f && !cached_path_.empty()) {
                    spdlog::warn("[PlanBase] Rejecting backward path (dot={:.2f}) — "
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
                    spdlog::warn("[PlanBase] Search failed — keeping last good path "
                                 "({} pts, idx {}) (took {:.0f}ms)",
                                 cached_path_.size(), path_index_, replan_ms);
                } else {
                    // No cached path at all — hover in place (zero velocity)
                    // rather than flying a direct line through obstacles.
                    direct_fallback_ = true;
                    spdlog::warn("[PlanBase] Search failed, no cached path — "
                                 "hovering in place (took {:.0f}ms)",
                                 replan_ms);
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
                spdlog::warn("[PlanBase] HOVERING — no valid path to "
                             "({:.1f},{:.1f},{:.1f}), vel=({:.2f},{:.2f},{:.2f})",
                             target.x, target.y, target.z, smooth_vx_, smooth_vy_, smooth_vz_);
            }
            return cmd;
        }

        // ── Follow path or go direct ────────────────────────
        float goal_x = snap_valid_ ? snapped_world_x_ : target.x;
        float goal_y = snap_valid_ ? snapped_world_y_ : target.y;

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

        // Periodic diagnostic: velocity, path state, position
        if (diag_tick_++ % 50 == 0) {
            spdlog::info("[PlanBase] pos=({:.1f},{:.1f},{:.1f}) vel=({:.2f},{:.2f},{:.2f}) "
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
    /// Goal snapping — find nearest free cell when goal is occupied.
    GridCell snap_goal(const GridCell& start, const GridCell& raw_goal, const Waypoint& target) {
        bool target_changed = (target.x != last_target_x_ || target.y != last_target_y_ ||
                               target.z != last_target_z_);

        if (target_changed) {
            snap_valid_    = false;
            last_target_x_ = target.x;
            last_target_y_ = target.y;
            last_target_z_ = target.z;
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
                        spdlog::warn("[Planner] Snap {:.1f} m lateral but only {:.1f}s to "
                                     "obstacle (need {:.1f}s) — tight margin",
                                     snap_dist_m, time_to_obs_s, time_to_deflect);
                    }
                }
            }

            // Fallback: nearest horizontal free cell
            if (!snapped) {
                float     best_dist_sq = 1e9f;
                const int radius       = std::clamp(config_.snap_search_radius, 0, 100);
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
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
                auto wc          = grid_.grid_to_world(goal);
                snapped_world_x_ = wc[0];
                snapped_world_y_ = wc[1];
                snapped_world_z_ = wc[2];
                snap_valid_      = true;
                const float snap_dist =
                    std::sqrt(static_cast<float>((goal.x - orig_goal.x) * (goal.x - orig_goal.x) +
                                                 (goal.y - orig_goal.y) * (goal.y - orig_goal.y))) *
                    grid_.resolution();
                spdlog::info("[Planner] WP({:.1f},{:.1f},{:.1f}) occupied — snapped lateral"
                             " to ({:.1f},{:.1f},{:.1f}) offset={:.2f} m",
                             target.x, target.y, target.z, snapped_world_x_, snapped_world_y_,
                             snapped_world_z_, snap_dist);
            }
        } else if (snap_valid_) {
            goal = grid_.world_to_grid(snapped_world_x_, snapped_world_y_, snapped_world_z_);
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
    std::vector<std::array<float, 3>>     cached_path_;
    size_t                                path_index_      = 0;
    bool                                  direct_fallback_ = false;
    std::chrono::steady_clock::time_point last_plan_time_{};

    // Cached snapped goal
    float last_target_x_ = 1e9f, last_target_y_ = 1e9f, last_target_z_ = 1e9f;
    float snapped_world_x_ = 0.0f, snapped_world_y_ = 0.0f, snapped_world_z_ = 0.0f;
    bool  snap_valid_ = false;

    // Velocity smoothing state
    float smooth_vx_ = 0.0f;
    float smooth_vy_ = 0.0f;
    float smooth_vz_ = 0.0f;

    // Diagnostic tick counter
    uint64_t diag_tick_ = 0;
};

}  // namespace drone::planner
