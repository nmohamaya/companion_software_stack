// process4_mission_planner/include/planner/static_obstacle_layer.h
// Manages HD-map static obstacles: loading, camera cross-check confirmation,
// collision detection, and unconfirmed approach warnings.
//
// Extracted from main.cpp as part of Issue #154.
// Updated in Issue #158: AStarPathPlanner* → IGridPlanner*.
#pragma once

#include "planner/grid_planner_base.h"
#include "util/ilogger.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace drone::planner {

/// A single HD-map static obstacle with camera confirmation state.
struct StaticObstacleRecord {
    float    x, y, radius_m, height_m;
    bool     confirmed{false};       // true once camera has seen this obstacle
    int      confirm_count{0};       // raw detection hit count (need >=2 to confirm)
    uint64_t first_confirmed_ns{0};  // loop timestamp of first confirmation
};

/// Manages static obstacle loading, camera cross-checking, collision detection,
/// and unconfirmed approach warnings.
class StaticObstacleLayer {
public:
    /// Load static obstacles from config JSON and populate planner grid if available.
    template<typename Config>
    void load(const Config& cfg, IGridPlanner* grid_planner) {
        obstacles_.clear();
        auto obs_json = cfg.section("mission_planner.static_obstacles");
        if (obs_json.is_array() && !obs_json.empty()) {
            for (const auto& o : obs_json) {
                StaticObstacleRecord rec{o.value("x", 0.0f), o.value("y", 0.0f),
                                         o.value("radius_m", 0.75f), o.value("height_m", 3.0f)};
                obstacles_.push_back(rec);
                if (grid_planner)
                    grid_planner->add_static_obstacle(rec.x, rec.y, rec.radius_m, rec.height_m);
            }
            DRONE_LOG_INFO("[HD-map] Loaded {} static obstacles into planner grid",
                           obs_json.size());
        }
    }

    /// Load directly from a vector (for tests or programmatic use).
    void load(const std::vector<StaticObstacleRecord>& obstacles,
              IGridPlanner*                            grid_planner = nullptr) {
        obstacles_ = obstacles;
        if (grid_planner) {
            for (const auto& obs : obstacles_) {
                grid_planner->add_static_obstacle(obs.x, obs.y, obs.radius_m, obs.height_m);
            }
        }
    }

    /// Camera cross-check of HD-map static obstacles.
    /// If a detection falls within (radius_m + 1 m) of a known HD-map obstacle
    /// it counts as a hit.  After 2 independent hits the obstacle is marked
    /// confirmed and logged.
    void cross_check(const drone::ipc::DetectedObjectList& objects, int pose_quality,
                     uint64_t now_ns) {
        if (obstacles_.empty() || objects.num_objects == 0 || pose_quality < 1) return;

        for (uint32_t i = 0; i < objects.num_objects; ++i) {
            const auto& det = objects.objects[i];
            const float wx  = det.position_x;
            const float wy  = det.position_y;
            const float wz  = det.position_z;
            for (auto& obs : obstacles_) {
                if (obs.confirmed) continue;
                const float dx   = wx - obs.x;
                const float dy   = wy - obs.y;
                const float dist = std::sqrt(dx * dx + dy * dy);
                if (dist < obs.radius_m + 1.0f && wz <= obs.height_m + 0.5f) {
                    obs.confirm_count++;
                    if (obs.confirm_count >= 2) {
                        obs.confirmed          = true;
                        obs.first_confirmed_ns = now_ns;
                        DRONE_LOG_INFO("[HD-map] Obstacle at ({:.1f},{:.1f}) CONFIRMED by camera "
                                       "(world det ({:.2f},{:.2f},{:.2f}), dist={:.2f}m)",
                                       obs.x, obs.y, wx, wy, wz, dist);
                    }
                }
            }
        }
    }

    /// Proximity collision detection using HD-map obstacle positions.
    /// Returns true if the drone is within (radius_m + margin) of any known obstacle.
    /// Throttled to once per cooldown period.
    bool check_collision(float px, float py, float pz, std::chrono::steady_clock::time_point now) {
        constexpr float kCollisionMarginM = 0.5f;
        constexpr float kCooldownSeconds  = 2.0f;

        if (std::chrono::duration<float>(now - last_collision_warn_time_).count() <=
            kCooldownSeconds)
            return false;

        for (const auto& obs : obstacles_) {
            const float dx   = px - obs.x;
            const float dy   = py - obs.y;
            const float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < obs.radius_m + kCollisionMarginM && pz <= obs.height_m + kCollisionMarginM) {
                DRONE_LOG_WARN("[Planner] OBSTACLE COLLISION detected — drone {:.2f}m"
                               " from obstacle at ({:.1f},{:.1f})",
                               dist, obs.x, obs.y);
                last_collision_warn_time_ = now;
                return true;
            }
        }
        return false;
    }

    /// Warn if approaching an HD-map obstacle that the camera has not yet confirmed.
    /// Throttled to once per 10 s.
    bool check_unconfirmed_approach(float px, float py, std::chrono::steady_clock::time_point now) {
        constexpr float kApproachWarnM     = 3.0f;
        constexpr float kApproachCooldownS = 10.0f;

        if (std::chrono::duration<float>(now - last_unconfirmed_approach_warn_).count() <=
            kApproachCooldownS)
            return false;

        for (const auto& obs : obstacles_) {
            if (obs.confirmed) continue;
            const float dx   = px - obs.x;
            const float dy   = py - obs.y;
            const float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < obs.radius_m + kApproachWarnM) {
                DRONE_LOG_WARN("[HD-map] Approaching UNCONFIRMED obstacle at ({:.1f},{:.1f}) "
                               "dist={:.2f}m — camera has not verified this obstacle yet",
                               obs.x, obs.y, dist);
                last_unconfirmed_approach_warn_ = now;
                return true;
            }
        }
        return false;
    }

    /// Access obstacles for inspection/tests.
    [[nodiscard]] const std::vector<StaticObstacleRecord>& obstacles() const { return obstacles_; }

    /// Mutable access (for tests).
    [[nodiscard]] std::vector<StaticObstacleRecord>& obstacles() { return obstacles_; }

    /// Whether any obstacles are loaded.
    [[nodiscard]] bool empty() const { return obstacles_.empty(); }

private:
    std::vector<StaticObstacleRecord>     obstacles_;
    std::chrono::steady_clock::time_point last_collision_warn_time_ =
        std::chrono::steady_clock::now() - std::chrono::seconds(300);
    std::chrono::steady_clock::time_point last_unconfirmed_approach_warn_ =
        std::chrono::steady_clock::now() - std::chrono::seconds(300);
};

}  // namespace drone::planner
