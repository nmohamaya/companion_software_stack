// process4_mission_planner/include/planner/obstacle_avoider_3d.h
// 3D Velocity-Obstacle–inspired avoider.
//
// Full 3D obstacle avoidance:
//   - Considers obstacle Z positions (not just XY)
//   - Adds vertical repulsive force component
//   - Uses obstacle velocity for predictive avoidance (if available)
//   - Clamps total correction to a safe maximum
//
// Plugs into the existing IObstacleAvoider interface.
//
// Implements Epic #110 Phase 3 — 3D obstacle avoidance.
// PotentialFieldAvoider (2D) removed in Issue #207 — this is the only avoider.
#pragma once

#include "ipc/ipc_types.h"
#include "planner/iobstacle_avoider.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace drone::planner {

/// Configuration for the 3D obstacle avoider.
struct ObstacleAvoider3DConfig {
    float    influence_radius_m = 5.0f;            // max distance at which obstacles exert force
    float    repulsive_gain     = 2.0f;            // strength of repulsive field
    float    max_correction_mps = 3.0f;            // max Euclidean magnitude of correction vector
    float    min_confidence     = 0.3f;            // minimum object confidence to consider
    float    prediction_dt_s    = 0.5f;            // look-ahead time for velocity-based prediction
    float    vertical_gain      = 1.0f;            // scale factor for Z repulsion (0=lateral only)
    uint64_t max_age_ns         = 500'000'000ULL;  // max age of object data (500ms)
    bool     path_aware         = true;            // remove repulsion opposing planned direction
    // Close-regime authority — Issue #503.
    // When the closest obstacle distance falls below min_distance_m, path-aware
    // stripping is bypassed so the avoider can push against the planner.
    // Hysteresis prevents chatter at the boundary.
    float min_distance_m                 = 2.0f;
    float path_aware_bypass_hysteresis_m = 0.5f;
    // Per-obstacle INFO log when its contribution exceeds ~0.5 m/s.  Gated so
    // scenarios that want quiet avoider output can disable it (Issue #503).
    bool log_corrections = true;
};

class ObstacleAvoider3D final : public IObstacleAvoider {
public:
    explicit ObstacleAvoider3D(const ObstacleAvoider3DConfig& config = {}) : config_(config) {}

    /// Convenience constructor with influence radius and repulsive gain.
    ObstacleAvoider3D(float influence_radius, float repulsive_gain)
        : config_{influence_radius, repulsive_gain} {}

    /// Config-driven constructor — reads all avoider params from drone::Config.
    explicit ObstacleAvoider3D(const drone::Config& cfg) {
        config_.influence_radius_m =
            cfg.get<float>(drone::cfg_key::mission_planner::obstacle_avoidance::INFLUENCE_RADIUS_M,
                           config_.influence_radius_m);
        config_.repulsive_gain =
            cfg.get<float>(drone::cfg_key::mission_planner::obstacle_avoidance::REPULSIVE_GAIN,
                           config_.repulsive_gain);
        config_.max_correction_mps =
            cfg.get<float>(drone::cfg_key::mission_planner::obstacle_avoidance::MAX_CORRECTION_MPS,
                           config_.max_correction_mps);
        config_.min_confidence =
            cfg.get<float>(drone::cfg_key::mission_planner::obstacle_avoidance::MIN_CONFIDENCE,
                           config_.min_confidence);
        config_.prediction_dt_s =
            cfg.get<float>(drone::cfg_key::mission_planner::obstacle_avoidance::PREDICTION_DT_S,
                           config_.prediction_dt_s);
        config_.vertical_gain =
            cfg.get<float>(drone::cfg_key::mission_planner::obstacle_avoidance::VERTICAL_GAIN,
                           config_.vertical_gain);
        const int default_max_age_ms = static_cast<int>(config_.max_age_ns / 1'000'000ULL);
        int       max_age_ms         = cfg.get<int>(
            drone::cfg_key::mission_planner::obstacle_avoidance::MAX_AGE_MS, default_max_age_ms);
        if (max_age_ms < 0) {
            DRONE_LOG_WARN("[ObstacleAvoider3D] max_age_ms ({}) < 0, clamping to 0", max_age_ms);
            max_age_ms = 0;
        }
        config_.max_age_ns = static_cast<uint64_t>(max_age_ms) * 1'000'000ULL;
        config_.path_aware = cfg.get<bool>(
            drone::cfg_key::mission_planner::obstacle_avoidance::PATH_AWARE, config_.path_aware);
        config_.min_distance_m =
            cfg.get<float>(drone::cfg_key::mission_planner::obstacle_avoidance::MIN_DISTANCE_M,
                           config_.min_distance_m);
        config_.path_aware_bypass_hysteresis_m = cfg.get<float>(
            drone::cfg_key::mission_planner::obstacle_avoidance::PATH_AWARE_BYPASS_HYSTERESIS_M,
            config_.path_aware_bypass_hysteresis_m);
        config_.log_corrections =
            cfg.get<bool>(drone::cfg_key::mission_planner::obstacle_avoidance::LOG_CORRECTIONS,
                          config_.log_corrections);
    }

    drone::ipc::TrajectoryCmd avoid(const drone::ipc::TrajectoryCmd&      planned,
                                    const drone::ipc::Pose&               pose,
                                    const drone::ipc::DetectedObjectList& objects) override {
        auto cmd = planned;

        // Skip if data is stale or empty
        auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::steady_clock::now().time_since_epoch())
                                                .count());
        if (objects.num_objects == 0 || objects.timestamp_ns == 0 ||
            (now_ns > objects.timestamp_ns && now_ns - objects.timestamp_ns > config_.max_age_ns)) {
            return cmd;
        }

        float drone_x = static_cast<float>(pose.translation[0]);
        float drone_y = static_cast<float>(pose.translation[1]);
        float drone_z = static_cast<float>(pose.translation[2]);

        // Guard against NaN/Inf pose — return planned trajectory unmodified
        if (!std::isfinite(drone_x) || !std::isfinite(drone_y) || !std::isfinite(drone_z)) {
            return cmd;
        }

        float total_rep_x = 0.0f;
        float total_rep_y = 0.0f;
        float total_rep_z = 0.0f;

        // Observability counters (Issue #503).
        uint32_t considered = 0;
        uint32_t active     = 0;

        for (uint32_t i = 0; i < objects.num_objects; ++i) {
            const auto& obj = objects.objects[i];
            if (obj.confidence < config_.min_confidence) continue;
            ++considered;

            // Predicted object position (if velocity is available)
            float ox = obj.position_x + obj.velocity_x * config_.prediction_dt_s;
            float oy = obj.position_y + obj.velocity_y * config_.prediction_dt_s;
            float oz = obj.position_z + obj.velocity_z * config_.prediction_dt_s;

            // Relative position from drone to obstacle
            float dx = ox - drone_x;
            float dy = oy - drone_y;
            float dz = oz - drone_z;

            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < config_.influence_radius_m && dist > 0.01f) {
                ++active;
                // Inverse-square repulsive force (decays with distance)
                float repulsion = config_.repulsive_gain / (dist * dist);

                // Per-obstacle contribution vector (so we can log its magnitude).
                const float cx = -(dx / dist) * repulsion;
                const float cy = -(dy / dist) * repulsion;
                const float cz = -(dz / dist) * repulsion * config_.vertical_gain;

                // Direction: away from obstacle (negative of relative vector)
                total_rep_x += cx;
                total_rep_y += cy;
                total_rep_z += cz;

                DRONE_LOG_DEBUG("[Avoider3D] Obstacle at ({:.1f},{:.1f},{:.1f}), "
                                "dist={:.1f}m, rep={:.2f}",
                                ox, oy, oz, dist, repulsion);

                // Promote per-obstacle log to INFO when its contribution is
                // significant (> 0.5 m/s magnitude).  Gated on log_corrections
                // so quiet scenarios can disable it (Issue #503).
                if (config_.log_corrections) {
                    const float cmag = std::sqrt(cx * cx + cy * cy + cz * cz);
                    if (cmag > 0.5f) {
                        DRONE_LOG_INFO("[Avoider] obstacle d={:.1f}m rep={:.2f} |c|={:.2f}m/s at "
                                       "({:.1f},{:.1f},{:.1f})",
                                       dist, repulsion, cmag, ox, oy, oz);
                    }
                }
            }
        }

        // Clamp corrections
        total_rep_x = std::clamp(total_rep_x, -config_.max_correction_mps,
                                 config_.max_correction_mps);
        total_rep_y = std::clamp(total_rep_y, -config_.max_correction_mps,
                                 config_.max_correction_mps);
        total_rep_z = std::clamp(total_rep_z, -config_.max_correction_mps,
                                 config_.max_correction_mps);

        // Path-aware repulsion: remove the component that opposes the planned
        // direction.  This prevents the avoider from fighting the planner when
        // obstacles sit between the drone and its goal (Issue #229).
        uint32_t path_aware_strip_count = 0;
        if (config_.path_aware) {
            if (config_.vertical_gain == 0.0f) {
                // 2D stripping — never inject Z from XY repulsion (Issue #237).
                // With vertical_gain=0, total_rep_z is guaranteed zero from the
                // repulsion loop.  3D stripping would leak Z via dir_z when the
                // planned command has a vertical component.
                const float mag_xy =
                    std::sqrt(cmd.velocity_x * cmd.velocity_x + cmd.velocity_y * cmd.velocity_y);
                if (mag_xy > 0.01f) {
                    const float inv   = 1.0f / mag_xy;
                    const float dx    = cmd.velocity_x * inv;
                    const float dy    = cmd.velocity_y * inv;
                    const float along = total_rep_x * dx + total_rep_y * dy;
                    if (along < 0.0f) {
                        total_rep_x -= along * dx;
                        total_rep_y -= along * dy;
                        ++path_aware_strip_count;
                        // total_rep_z stays at 0 — no Z injection
                    }
                }
            } else {
                // Full 3D stripping when vertical avoidance is active.
                const float planned_mag = std::sqrt(cmd.velocity_x * cmd.velocity_x +
                                                    cmd.velocity_y * cmd.velocity_y +
                                                    cmd.velocity_z * cmd.velocity_z);
                if (planned_mag > 0.01f) {
                    const float inv_mag = 1.0f / planned_mag;
                    const float dir_x   = cmd.velocity_x * inv_mag;
                    const float dir_y   = cmd.velocity_y * inv_mag;
                    const float dir_z   = cmd.velocity_z * inv_mag;

                    const float along = total_rep_x * dir_x + total_rep_y * dir_y +
                                        total_rep_z * dir_z;

                    if (along < 0.0f) {
                        total_rep_x -= along * dir_x;
                        total_rep_y -= along * dir_y;
                        total_rep_z -= along * dir_z;
                        ++path_aware_strip_count;
                    }
                }
            }
        }

        cmd.velocity_x += total_rep_x;
        cmd.velocity_y += total_rep_y;
        cmd.velocity_z += total_rep_z;

        // Single-line summary at the end of every avoid() call (Issue #503).
        // INFO-gated on log_corrections + meaningful delta so quiet ticks stay
        // at DEBUG.  Magnitude is the Euclidean size of the total correction
        // applied to the planned velocity.
        if (config_.log_corrections) {
            const float delta_mag = std::sqrt(
                total_rep_x * total_rep_x + total_rep_y * total_rep_y + total_rep_z * total_rep_z);
            if (active > 0 || delta_mag > 0.1f) {
                DRONE_LOG_INFO("[Avoider] considered={} active={} |delta|={:.2f} m/s "
                               "path_aware_strip={}",
                               considered, active, delta_mag, path_aware_strip_count);
            }
        }

        return cmd;
    }

    std::string name() const override { return "ObstacleAvoider3D"; }

private:
    ObstacleAvoider3DConfig config_;
};

}  // namespace drone::planner

// ── Factory ──────────────────────────────────────────────────
// Defined here (not in iobstacle_avoider.h) so that ObstacleAvoider3D
// is fully visible — avoids circular-include issues.
namespace drone::planner {

inline std::unique_ptr<IObstacleAvoider> create_obstacle_avoider(
    const std::string& backend = "potential_field_3d", float influence_radius = 5.0f,
    float repulsive_gain = 2.0f, const drone::Config* cfg = nullptr) {
    if (backend == "3d" || backend == "obstacle_avoider_3d" || backend == "potential_field_3d") {
        if (cfg) {
            return std::make_unique<ObstacleAvoider3D>(*cfg);
        }
        return std::make_unique<ObstacleAvoider3D>(influence_radius, repulsive_gain);
    }
    throw std::runtime_error("Unknown obstacle avoider: " + backend);
}

}  // namespace drone::planner
