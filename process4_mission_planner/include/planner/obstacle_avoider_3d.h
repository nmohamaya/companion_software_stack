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
#include "util/per_class_config.h"
#include "util/result.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

namespace drone::planner {

static_assert(drone::util::kPerClassCount ==
                  static_cast<uint8_t>(drone::ipc::ObjectClass::GEOMETRIC_OBSTACLE) + 1,
              "kPerClassCount must match ObjectClass enum size");

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
    // Brake arbitration in close-regime — Issue #513.
    // When close_regime_active_ fires, cancel the component of planned
    // velocity pointing at the nearest obstacle and scale total commanded
    // magnitude by proximity.  Fixes the additive-correction gap where a
    // modest lateral deflection (e.g. |delta|=0.98 m/s) was insufficient
    // against 2 m/s cruise pointed at an obstacle.  Default true.
    bool brake_in_close_regime = true;
    // Floor on the proximity-based brake scale (PR #617 fault-recovery P2).
    // Without this floor, a single spurious detection at 0.05 m (radar
    // noise) against min_distance_m=2.0 produces brake_scale=0.025 — a
    // near-stop command for one full planner tick.  Capping the worst
    // single-tick output preserves the safety property (persistent
    // obstacles still drive the scale to the floor over multiple ticks)
    // while rejecting one-tick noise events.  Default 0.1 (= 10 % of
    // cruise).
    float min_brake_scale = 0.1f;

    // Per-class overrides (Epic #519).  Indexed by ObjectClass enum value (0-7).
    // Zero-initialized here; the ObstacleAvoider3D constructor fills them
    // from the global scalars above, then the Config-driven constructor
    // overwrites with per-class values from JSON.
    std::array<float, drone::util::kPerClassCount> influence_radius_per_class{};
    std::array<float, drone::util::kPerClassCount> repulsive_gain_per_class{};
    std::array<float, drone::util::kPerClassCount> min_distance_per_class{};
    std::array<float, drone::util::kPerClassCount> prediction_dt_per_class{};
    std::array<float, drone::util::kPerClassCount> min_confidence_per_class{};
};

// ── Named constants (PR #617 code-quality review) ──────────────────────────
// The 0.01f dist gate and 0.01f velocity-magnitude gate are both "minimum
// non-zero scalar to prevent div-by-zero on degenerate geometry", but they
// guard different quantities — distance vs velocity — so they get distinct
// names.  The log thresholds live here because they govern when [Avoider]
// lines are promoted to INFO; unlike `min_distance_m`, they are not physics
// values and don't belong in per-scenario config.
namespace avoider_constants {
inline constexpr float kMinDistGateM               = 0.01f;  // geometry dead-zone
inline constexpr float kMinVelMagnitude            = 0.01f;  // velocity dead-zone (normalisation)
inline constexpr float kLogPerObstacleThresholdMps = 0.5f;   // promote per-obstacle log to INFO
inline constexpr float kLogDeltaThresholdMps       = 0.1f;   // promote summary log to INFO
}  // namespace avoider_constants

class ObstacleAvoider3D final : public IObstacleAvoider {
public:
    explicit ObstacleAvoider3D(const ObstacleAvoider3DConfig& config = {}) : config_(config) {
        sync_per_class_from_globals();
    }

    /// Convenience constructor with influence radius and repulsive gain.
    ObstacleAvoider3D(float influence_radius, float repulsive_gain)
        : config_{influence_radius, repulsive_gain} {
        sync_per_class_from_globals();
    }

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
        // Clamp BEFORE casting: a config value > INT_MAX ms would wrap to
        // a negative int and (pre-clamp-guard below) silently disable
        // staleness filtering.  Cap at INT_MAX so the cast is well-defined.
        const uint64_t default_ms_u64     = config_.max_age_ns / 1'000'000ULL;
        const int      default_max_age_ms = static_cast<int>(std::min<uint64_t>(
            default_ms_u64, static_cast<uint64_t>(std::numeric_limits<int>::max())));
        int            max_age_ms         = cfg.get<int>(
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
        config_.brake_in_close_regime = cfg.get<bool>(
            drone::cfg_key::mission_planner::obstacle_avoidance::BRAKE_IN_CLOSE_REGIME,
            config_.brake_in_close_regime);
        config_.min_brake_scale =
            cfg.get<float>(drone::cfg_key::mission_planner::obstacle_avoidance::MIN_BRAKE_SCALE,
                           config_.min_brake_scale);
        // PR #617 review: a config mistake of min_distance_m=0 silently disables
        // BOTH the brake path (guarded by `min_distance_m > 0` inside avoid())
        // AND the close_regime_active_ hysteresis (entry condition is
        // `dist < min_distance_m`, which can never be true when it's 0).
        // Warn loudly so operators notice before scenario time.
        if (config_.brake_in_close_regime && config_.min_distance_m <= 0.0f) {
            DRONE_LOG_WARN("[ObstacleAvoider3D] brake_in_close_regime enabled but "
                           "min_distance_m={:.3f} <= 0 — brake path will be skipped and "
                           "close_regime_active_ hysteresis will also be suppressed.",
                           config_.min_distance_m);
        }

        // Per-class overrides — fall back to the global scalar loaded above.
        namespace oa                       = drone::cfg_key::mission_planner::obstacle_avoidance;
        config_.influence_radius_per_class = drone::util::load_per_class<float>(
            cfg, oa::PER_CLASS_INFLUENCE_RADIUS_M, config_.influence_radius_m);
        config_.repulsive_gain_per_class = drone::util::load_per_class<float>(
            cfg, oa::PER_CLASS_REPULSIVE_GAIN, config_.repulsive_gain);
        config_.min_distance_per_class = drone::util::load_per_class<float>(
            cfg, oa::PER_CLASS_MIN_DISTANCE_M, config_.min_distance_m);
        config_.prediction_dt_per_class = drone::util::load_per_class<float>(
            cfg, oa::PER_CLASS_PREDICTION_DT_S, config_.prediction_dt_s);
        config_.min_confidence_per_class = drone::util::load_per_class<float>(
            cfg, oa::PER_CLASS_MIN_CONFIDENCE, config_.min_confidence);
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
        // Track nearest active obstacle for the path-aware bypass hysteresis
        // (Issue #503).  Only updated when an obstacle is within influence.
        float min_active_dist = std::numeric_limits<float>::infinity();
        // Direction vector from drone to the nearest active obstacle —
        // captured for Issue #513's brake arbitration.  Unit-less; normalised
        // only when we actually use it.
        float nearest_dx = 0.0f;
        float nearest_dy = 0.0f;
        float nearest_dz = 0.0f;

        for (uint32_t i = 0; i < objects.num_objects; ++i) {
            const auto& obj = objects.objects[i];
            const auto  ci  = static_cast<uint8_t>(obj.class_id);
            if (ci >= drone::util::kPerClassCount) continue;

            if (obj.confidence < config_.min_confidence_per_class[ci]) continue;
            ++considered;

            // Predicted object position (if velocity is available)
            const float pred_dt = config_.prediction_dt_per_class[ci];
            float       ox      = obj.position_x + obj.velocity_x * pred_dt;
            float       oy      = obj.position_y + obj.velocity_y * pred_dt;
            float       oz      = obj.position_z + obj.velocity_z * pred_dt;

            // Relative position from drone to obstacle
            float dx = ox - drone_x;
            float dy = oy - drone_y;
            float dz = oz - drone_z;

            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < config_.influence_radius_per_class[ci] &&
                dist > avoider_constants::kMinDistGateM) {
                ++active;
                if (dist < min_active_dist) {
                    min_active_dist = dist;
                    nearest_dx      = dx;
                    nearest_dy      = dy;
                    nearest_dz      = dz;
                }
                // Inverse-square repulsive force (decays with distance)
                float repulsion = config_.repulsive_gain_per_class[ci] / (dist * dist);

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
                    if (cmag > avoider_constants::kLogPerObstacleThresholdMps) {
                        DRONE_LOG_DEBUG("[Avoider] obstacle d={:.1f}m rep={:.2f} |c|={:.2f}m/s "
                                        "at ({:.1f},{:.1f},{:.1f})",
                                        dist, repulsion, cmag, ox, oy, oz);
                    }
                }
            }
        }

        // Clamp Euclidean magnitude of the correction vector (Issue #503).
        // The old per-axis clamp allowed the 3-vector magnitude to exceed
        // max_correction_mps by sqrt(N_axes) — e.g. (1.0, 1.0, 0) has magnitude
        // 1.41 under a 1.0 cap.  Clamping the magnitude preserves direction
        // and honours the configured authority limit exactly.
        {
            const float mag_sq = total_rep_x * total_rep_x + total_rep_y * total_rep_y +
                                 total_rep_z * total_rep_z;
            const float max_sq = config_.max_correction_mps * config_.max_correction_mps;
            if (mag_sq > max_sq && mag_sq > 0.0f) {
                const float scale = config_.max_correction_mps / std::sqrt(mag_sq);
                total_rep_x *= scale;
                total_rep_y *= scale;
                total_rep_z *= scale;
            }
        }

        // Path-aware bypass hysteresis (Issue #503).
        // When the closest active obstacle falls below min_distance_m, path-aware
        // stripping is wrong — in the close regime we want the avoider to push
        // back against the planner.  Hysteresis: enter the bypass below
        // min_distance_m, exit only once the distance rises above
        // min_distance_m + hysteresis_m.  This matches the contract: path-aware
        // stripping is disabled in the close regime and re-enabled when clearance
        // is comfortably re-established.
        if (std::isfinite(min_active_dist)) {
            if (!close_regime_active_ && min_active_dist < config_.min_distance_m) {
                close_regime_active_ = true;
            } else if (close_regime_active_ &&
                       min_active_dist >
                           config_.min_distance_m + config_.path_aware_bypass_hysteresis_m) {
                close_regime_active_ = false;
            }
        }
        const bool apply_path_aware = config_.path_aware && !close_regime_active_;

        // Path-aware repulsion: remove the component that opposes the planned
        // direction.  This prevents the avoider from fighting the planner when
        // obstacles sit between the drone and its goal (Issue #229).
        uint32_t path_aware_strip_count = 0;
        if (apply_path_aware) {
            if (config_.vertical_gain == 0.0f) {
                // 2D stripping — never inject Z from XY repulsion (Issue #237).
                // With vertical_gain=0, total_rep_z is guaranteed zero from the
                // repulsion loop.  3D stripping would leak Z via dir_z when the
                // planned command has a vertical component.
                const float mag_xy =
                    std::sqrt(cmd.velocity_x * cmd.velocity_x + cmd.velocity_y * cmd.velocity_y);
                if (mag_xy > avoider_constants::kMinVelMagnitude) {
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
                if (planned_mag > avoider_constants::kMinVelMagnitude) {
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

        // ── Brake arbitration in close regime (Issue #513) ─────────────
        // Additive lateral correction alone is insufficient when the planner's
        // cruise vector points at an obstacle.  In the close regime:
        //   (1) cancel the component of cmd.velocity pointing INTO the nearest
        //       obstacle (drone's own forward momentum stops driving it in), and
        //   (2) scale total commanded magnitude by proximity so the drone
        //       decelerates linearly to a stop at the obstacle surface.
        // Then fall through to the existing repulsion-add, which contributes
        // the lateral push.  Gated by `brake_in_close_regime` (default true);
        // scenarios that want pure-deflection semantics set it to false.
        bool  brake_applied = false;
        float brake_scale   = 1.0f;
        float v_toward      = 0.0f;
        if (close_regime_active_ && config_.brake_in_close_regime &&
            std::isfinite(min_active_dist) && min_active_dist > avoider_constants::kMinDistGateM &&
            config_.min_distance_m > 0.0f) {
            const float inv = 1.0f / min_active_dist;
            const float nx  = nearest_dx * inv;
            const float ny  = nearest_dy * inv;
            const float nz  = nearest_dz * inv;
            // (1) Cancel component heading INTO the obstacle.
            v_toward = cmd.velocity_x * nx + cmd.velocity_y * ny + cmd.velocity_z * nz;
            if (v_toward > 0.0f) {
                cmd.velocity_x -= v_toward * nx;
                cmd.velocity_y -= v_toward * ny;
                cmd.velocity_z -= v_toward * nz;
                brake_applied = true;
            }
            // (2) Scale magnitude by proximity — linear ramp 0→1 over 0→min_distance_m.
            //     At d = min_distance_m, scale = 1 (full commanded speed).
            //     At d = 0, scale = 0 (stop at the obstacle surface).
            //     Clamp BELOW by `min_brake_scale` (PR #617 fault-recovery P2) so a
            //     single-frame spurious short-range detection can't zero the
            //     command for a full tick.  Persistent obstacles still pin the
            //     scale to this floor over consecutive ticks.
            //     NOTE: `min_distance_m` here is the global scalar — the same
            //     value that gates close_regime_active_ hysteresis above.  If
            //     per-class min_distance_per_class overrides are set (Epic #519)
            //     they affect the per-class repulsion loop upstream, not the
            //     brake scale denominator.  Keeping the denominator global keeps
            //     the brake semantics independent of which class happened to be
            //     the nearest obstacle this tick.
            brake_scale = std::min(1.0f, min_active_dist / config_.min_distance_m);
            brake_scale = std::max(config_.min_brake_scale, brake_scale);
            cmd.velocity_x *= brake_scale;
            cmd.velocity_y *= brake_scale;
            cmd.velocity_z *= brake_scale;
        }

        cmd.velocity_x += total_rep_x;
        cmd.velocity_y += total_rep_y;
        cmd.velocity_z += total_rep_z;

        // Single-line summary at the end of every avoid() call (Issue #503).
        // INFO-gated on log_corrections + meaningful delta so quiet ticks stay
        // at DEBUG.  Magnitude is the Euclidean size of the total correction
        // applied to the planned velocity.
        //
        // Log-field reference (for operators parsing [Avoider] lines):
        //   considered      = objects passing min_confidence filter
        //   active          = objects within influence_radius_m
        //   |delta|         = Euclidean magnitude of repulsion correction (m/s)
        //   path_aware_strip= times path-aware stripping removed an opposing component
        //   close_regime    = 1 if nearest obstacle < min_distance_m (hysteresis)
        //   brake           = 1 if PR #617 brake arbitration cancelled forward motion
        //   v_toward        = forward-velocity component that was cancelled (m/s)
        //   scale           = proximity-based magnitude scaler applied after cancel
        if (config_.log_corrections) {
            const float delta_mag = std::sqrt(
                total_rep_x * total_rep_x + total_rep_y * total_rep_y + total_rep_z * total_rep_z);
            if (active > 0 || delta_mag > avoider_constants::kLogDeltaThresholdMps) {
                // Emit brake fields only when the brake path was entered — on quiet
                // ticks the three defaults (0, 0.0, 1.0) aren't informative.  This
                // saves one fmt::format + alloc per active tick when the brake is
                // idle (PR #617 perf review).
                if (brake_applied || brake_scale < 1.0f) {
                    DRONE_LOG_INFO("[Avoider] considered={} active={} |delta|={:.2f} m/s "
                                   "path_aware_strip={} close_regime={} brake={} v_toward={:.2f} "
                                   "scale={:.2f}",
                                   considered, active, delta_mag, path_aware_strip_count,
                                   close_regime_active_ ? 1 : 0, brake_applied ? 1 : 0, v_toward,
                                   brake_scale);
                } else {
                    DRONE_LOG_INFO("[Avoider] considered={} active={} |delta|={:.2f} m/s "
                                   "path_aware_strip={} close_regime={}",
                                   considered, active, delta_mag, path_aware_strip_count,
                                   close_regime_active_ ? 1 : 0);
                }
            }
        }

        return cmd;
    }

    std::string name() const override { return "ObstacleAvoider3D"; }

    /// Whether the avoider is currently in close-regime (path-aware bypassed).
    /// Exposed for tests and diagnostics.
    [[nodiscard]] bool close_regime_active() const { return close_regime_active_; }

    /// Mutable config access — for tests that need per-class overrides
    /// after construction (the constructor fills per-class arrays from
    /// the global scalars, so tests must override after).
    ObstacleAvoider3DConfig& mutable_config() { return config_; }

private:
    // Fill all per-class arrays from their corresponding global scalar.
    // Called once at construction so legacy configs that only set globals
    // propagate correctly to the per-class arrays.
    void sync_per_class_from_globals() {
        config_.influence_radius_per_class.fill(config_.influence_radius_m);
        config_.repulsive_gain_per_class.fill(config_.repulsive_gain);
        config_.min_distance_per_class.fill(config_.min_distance_m);
        config_.prediction_dt_per_class.fill(config_.prediction_dt_s);
        config_.min_confidence_per_class.fill(config_.min_confidence);
    }

    ObstacleAvoider3DConfig config_;
    // Path-aware bypass hysteresis state (Issue #503).
    // Enters when closest obstacle < min_distance_m; exits when it rises
    // above min_distance_m + path_aware_bypass_hysteresis_m.
    bool close_regime_active_ = false;
};

}  // namespace drone::planner

// ── Factory ──────────────────────────────────────────────────
// Defined here (not in iobstacle_avoider.h) so that ObstacleAvoider3D
// is fully visible — avoids circular-include issues.
namespace drone::planner {

[[nodiscard]] inline drone::util::Result<std::unique_ptr<IObstacleAvoider>> create_obstacle_avoider(
    const std::string& backend = "potential_field_3d", float influence_radius = 5.0f,
    float repulsive_gain = 2.0f, const drone::Config* cfg = nullptr) {
    using R = drone::util::Result<std::unique_ptr<IObstacleAvoider>>;
    if (backend == "3d" || backend == "obstacle_avoider_3d" || backend == "potential_field_3d") {
        if (cfg) {
            return R::ok(std::make_unique<ObstacleAvoider3D>(*cfg));
        }
        return R::ok(std::make_unique<ObstacleAvoider3D>(influence_radius, repulsive_gain));
    }
    return R::err(drone::util::Error(drone::util::ErrorCode::InvalidValue,
                                     "Unknown obstacle avoider: " + backend));
}

}  // namespace drone::planner
