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
        , grid_(config.resolution_m, config.grid_extent_m, config.inflation_radius_m) {}

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
            goal = snap_goal(start, goal, target);

            // ── Delegate to subclass search ──────────────────
            std::vector<std::array<float, 3>> search_path;
            bool                              found = do_search(start, goal, search_path);

            if (found && search_path.size() >= 2) {
                cached_path_     = std::move(search_path);
                path_index_      = 1;  // skip start cell
                last_plan_time_  = now;
                direct_fallback_ = false;
            } else {
                cached_path_.clear();
                path_index_      = 0;
                direct_fallback_ = true;
            }
        }

        // ── Follow path or go direct ────────────────────────
        float goal_x = snap_valid_ ? snapped_world_x_ : target.x;
        float goal_y = snap_valid_ ? snapped_world_y_ : target.y;
        float goal_z = snap_valid_ ? snapped_world_z_ : target.z;

        if (!cached_path_.empty() && path_index_ < cached_path_.size()) {
            auto& wp = cached_path_[path_index_];
            goal_x   = wp[0];
            goal_y   = wp[1];
            goal_z   = wp[2];

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
            if (dist < config_.ramp_dist_m && path_index_ + 1 >= cached_path_.size()) {
                speed = config_.min_speed_mps +
                        (speed - config_.min_speed_mps) * (dist / config_.ramp_dist_m);
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
};

}  // namespace drone::planner
