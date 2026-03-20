// process4_mission_planner/include/planner/fault_manager.h
// Graceful degradation engine — evaluates system health and sensor state
// each loop tick and returns the highest-priority response action.
//
// Design:
//   - Stateful evaluator: evaluate() inspects current sensor/health snapshots
//     and updates internal escalation and timing state (e.g. high-water mark,
//     reason string, loiter timer) before returning a FaultState.
//   - Escalation-only: once an action is raised, it can only be superseded
//     by a higher-severity action (NONE < WARN < LOITER < RTL < EMERGENCY_LAND).
//   - Config-driven: all thresholds loaded from the "fault_manager" JSON section.
//
// Issue: #61
#pragma once

#include "ipc/ipc_types.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>

#include <spdlog/spdlog.h>

namespace drone::planner {

// Re-export fault types from ipc layer for planner convenience.
// Canonical definitions live in ipc/ipc_types.h so any process can
// decode MissionStatus::active_faults / fault_action.
using drone::ipc::FaultAction;
using drone::ipc::fault_action_name;
using drone::ipc::FaultType;
inline constexpr auto FAULT_NONE             = FaultType::FAULT_NONE;
inline constexpr auto FAULT_CRITICAL_PROCESS = FaultType::FAULT_CRITICAL_PROCESS;
inline constexpr auto FAULT_POSE_STALE       = FaultType::FAULT_POSE_STALE;
inline constexpr auto FAULT_BATTERY_LOW      = FaultType::FAULT_BATTERY_LOW;
inline constexpr auto FAULT_BATTERY_CRITICAL = FaultType::FAULT_BATTERY_CRITICAL;
inline constexpr auto FAULT_BATTERY_RTL      = FaultType::FAULT_BATTERY_RTL;
inline constexpr auto FAULT_THERMAL_WARNING  = FaultType::FAULT_THERMAL_WARNING;
inline constexpr auto FAULT_THERMAL_CRITICAL = FaultType::FAULT_THERMAL_CRITICAL;
inline constexpr auto FAULT_PERCEPTION_DEAD  = FaultType::FAULT_PERCEPTION_DEAD;
inline constexpr auto FAULT_FC_LINK_LOST     = FaultType::FAULT_FC_LINK_LOST;
inline constexpr auto FAULT_GEOFENCE_BREACH  = FaultType::FAULT_GEOFENCE_BREACH;
inline constexpr auto FAULT_VIO_DEGRADED     = FaultType::FAULT_VIO_DEGRADED;
inline constexpr auto FAULT_VIO_LOST         = FaultType::FAULT_VIO_LOST;
using drone::ipc::to_uint;

// ═══════════════════════════════════════════════════════════
// FaultState — output of evaluate()
// ═══════════════════════════════════════════════════════════
struct FaultState {
    FaultAction recommended_action = FaultAction::NONE;
    uint32_t    active_faults      = 0;          // bitmask of FaultType
    const char* reason             = "nominal";  // human-readable
};

// ═══════════════════════════════════════════════════════════
// FaultConfig — thresholds loaded from config JSON
// ═══════════════════════════════════════════════════════════
struct FaultConfig {
    // Pose staleness
    uint64_t pose_stale_timeout_ns = 500'000'000ULL;  // 500 ms

    // Battery — three-tier progressive escalation
    float battery_warn_percent = 30.0f;  // → WARN (alert GCS, continue)
    float battery_rtl_percent  = 20.0f;  // → RTL (return to launch)
    float battery_crit_percent = 10.0f;  // → EMERGENCY_LAND (land now)

    // FC link
    uint64_t fc_link_lost_timeout_ns = 3'000'000'000ULL;  // 3 s → LOITER
    // FC link-loss contingency: LOITER for this long, then auto-RTL
    uint64_t fc_link_rtl_timeout_ns = 15'000'000'000ULL;  // 15 s → RTL

    // Loiter → RTL escalation delay (general, not FC-specific)
    uint64_t loiter_escalation_timeout_ns = 30'000'000'000ULL;  // 30 s

    // VIO quality thresholds (Pose.quality: 0=lost, 1=degraded, 2=good, 3=excellent)
    uint32_t vio_quality_loiter_threshold = 1;  // quality <= this → LOITER
    uint32_t vio_quality_rtl_threshold    = 0;  // quality <= this → RTL
};

// ═══════════════════════════════════════════════════════════
// FaultManager — graceful degradation engine
// ═══════════════════════════════════════════════════════════
class FaultManager {
public:
    /// Construct with config-driven thresholds.
    /// @param cfg  Drone config — reads "fault_manager.*" keys.
    template<typename Config>
    explicit FaultManager(const Config& cfg) {
        config_.pose_stale_timeout_ns = static_cast<uint64_t>(cfg.template get<int>(
                                            "fault_manager.pose_stale_timeout_ms", 500)) *
                                        1'000'000ULL;

        config_.battery_warn_percent = cfg.template get<float>("fault_manager.battery_warn_percent",
                                                               30.0f);
        config_.battery_rtl_percent  = cfg.template get<float>("fault_manager.battery_rtl_percent",
                                                               20.0f);
        config_.battery_crit_percent = cfg.template get<float>("fault_manager.battery_crit_percent",
                                                               10.0f);

        config_.fc_link_lost_timeout_ns = static_cast<uint64_t>(cfg.template get<int>(
                                              "fault_manager.fc_link_lost_timeout_ms", 3000)) *
                                          1'000'000ULL;

        config_.fc_link_rtl_timeout_ns = static_cast<uint64_t>(cfg.template get<int>(
                                             "fault_manager.fc_link_rtl_timeout_ms", 15000)) *
                                         1'000'000ULL;

        config_.loiter_escalation_timeout_ns =
            static_cast<uint64_t>(
                cfg.template get<int>("fault_manager.loiter_escalation_timeout_s", 30)) *
            1'000'000'000ULL;

        {
            int loiter_q = cfg.template get<int>("fault_manager.vio_quality_loiter_threshold", 1);
            int rtl_q    = cfg.template get<int>("fault_manager.vio_quality_rtl_threshold", 0);
            config_.vio_quality_loiter_threshold =
                static_cast<uint32_t>(std::clamp(loiter_q, 0, 3));
            config_.vio_quality_rtl_threshold = static_cast<uint32_t>(std::clamp(rtl_q, 0, 3));
        }

        spdlog::info("[FaultMgr] Thresholds: pose_stale={}ms, "
                     "batt_warn={}%, batt_rtl={}%, batt_crit={}%, "
                     "fc_lost={}ms, fc_rtl={}ms, loiter_esc={}s, "
                     "vio_loiter_q<={}, vio_rtl_q<={}",
                     config_.pose_stale_timeout_ns / 1'000'000, config_.battery_warn_percent,
                     config_.battery_rtl_percent, config_.battery_crit_percent,
                     config_.fc_link_lost_timeout_ns / 1'000'000,
                     config_.fc_link_rtl_timeout_ns / 1'000'000,
                     config_.loiter_escalation_timeout_ns / 1'000'000'000,
                     config_.vio_quality_loiter_threshold, config_.vio_quality_rtl_threshold);
    }

    /// Construct with explicit config (for unit tests).
    explicit FaultManager(const FaultConfig& cfg) : config_(cfg) {}

    /// Evaluate all fault conditions and return the highest-priority action.
    ///
    /// @param health           Latest SystemHealth from Process 7.
    /// @param fc_state         Latest FCState from Process 5.
    /// @param pose_timestamp_ns  Timestamp from the last received Pose.
    /// @param now_ns           Current monotonic time (steady_clock).
    /// @return FaultState with recommended action and active fault bitmask.
    [[nodiscard]] FaultState evaluate(const drone::ipc::SystemHealth& health,
                                      const drone::ipc::FCState&      fc_state,
                                      uint64_t pose_timestamp_ns, uint64_t now_ns,
                                      uint32_t pose_quality = 2) {
        FaultState result;

        // ── 1. Critical process death (comms / SLAM) ────────
        if (health.critical_failure) {
            result.active_faults |= FAULT_CRITICAL_PROCESS;
            escalate(result, FaultAction::LOITER, "critical process died");
        }

        // ── 2. Pose staleness ───────────────────────────────
        if (pose_timestamp_ns > 0 && now_ns > pose_timestamp_ns) {
            uint64_t age = now_ns - pose_timestamp_ns;
            if (age > config_.pose_stale_timeout_ns) {
                result.active_faults |= FAULT_POSE_STALE;
                escalate(result, FaultAction::LOITER, "pose data stale");
            }
        }

        // ── 3. VIO health degradation ────────────────────────
        // Only evaluate VIO quality after we've received at least one valid pose.
        // Debounce: require kVioDebounceCount consecutive low-quality readings
        // before firing, to prevent a single transient glitch (e.g. a zero-init
        // FaultOverrides warmup) from triggering irreversible RTL.
        if (pose_timestamp_ns > 0) {
            if (pose_quality <= config_.vio_quality_rtl_threshold) {
                if (++vio_low_quality_count_ >= kVioDebounceCount) {
                    result.active_faults |= FAULT_VIO_LOST;
                    escalate(result, FaultAction::RTL, "VIO tracking lost");
                }
            } else if (pose_quality <= config_.vio_quality_loiter_threshold) {
                if (++vio_low_quality_count_ >= kVioDebounceCount) {
                    result.active_faults |= FAULT_VIO_DEGRADED;
                    escalate(result, FaultAction::LOITER, "VIO quality degraded");
                }
            } else {
                vio_low_quality_count_ = 0;  // reset on good reading
            }
        }

        // ── 4. Battery — three-tier progressive escalation ──
        if (fc_state.connected && fc_state.battery_remaining > 0.0f) {
            if (fc_state.battery_remaining < config_.battery_crit_percent) {
                result.active_faults |= FAULT_BATTERY_CRITICAL;
                escalate(result, FaultAction::EMERGENCY_LAND, "battery critical");
            } else if (fc_state.battery_remaining < config_.battery_rtl_percent) {
                result.active_faults |= FAULT_BATTERY_RTL;
                escalate(result, FaultAction::RTL, "battery low — RTL");
            } else if (fc_state.battery_remaining < config_.battery_warn_percent) {
                result.active_faults |= FAULT_BATTERY_LOW;
                escalate(result, FaultAction::WARN, "battery warning");
            }
        }

        // ── 5. Thermal ──────────────────────────────────────
        if (health.thermal_zone >= 3) {
            result.active_faults |= FAULT_THERMAL_CRITICAL;
            escalate(result, FaultAction::RTL, "thermal critical");
        } else if (health.thermal_zone == 2) {
            result.active_faults |= FAULT_THERMAL_WARNING;
            escalate(result, FaultAction::WARN, "thermal warning");
        }

        // ── 6. Perception process death ─────────────────────
        if (is_process_dead(health, "perception")) {
            result.active_faults |= FAULT_PERCEPTION_DEAD;
            escalate(result, FaultAction::WARN, "perception died — no obstacle avoidance");
        }

        // ── 7. FC link lost — LOITER then escalate to RTL ─
        if (!fc_state.connected && fc_state.timestamp_ns > 0 && now_ns > fc_state.timestamp_ns) {
            uint64_t fc_age = now_ns - fc_state.timestamp_ns;
            if (fc_age > config_.fc_link_rtl_timeout_ns) {
                // Contingency: been disconnected too long → RTL
                result.active_faults |= FAULT_FC_LINK_LOST;
                escalate(result, FaultAction::RTL, "FC link lost — contingency RTL");
            } else if (fc_age > config_.fc_link_lost_timeout_ns) {
                // Initial: loiter and wait for reconnect
                result.active_faults |= FAULT_FC_LINK_LOST;
                escalate(result, FaultAction::LOITER, "FC link lost — loitering");
            }
        }

        // ── 8. Geofence breach ──────────────────────────────
        if (geofence_violated_) {
            result.active_faults |= FAULT_GEOFENCE_BREACH;
            escalate(result, FaultAction::RTL, "geofence breach");
        }

        // ── 9. Enforce escalation-only policy ───────────────
        // Once we've escalated to a higher action, never downgrade.
        // Applied BEFORE the loiter timer so that a cleared LOITER
        // cause still counts toward the escalation timeout.
        if (result.recommended_action < high_water_mark_) {
            result.recommended_action = high_water_mark_;
            result.reason             = high_water_reason_;
        } else if (result.recommended_action > high_water_mark_) {
            high_water_mark_   = result.recommended_action;
            high_water_reason_ = result.reason;
        }

        // ── 10. Loiter escalation to RTL ────────────────────
        // If the effective action (post high-water mark) has been
        // LOITER for too long, escalate to RTL.
        if (result.recommended_action == FaultAction::LOITER) {
            if (loiter_start_ns_ == 0) {
                loiter_start_ns_ = now_ns;  // start the timer
            } else if (now_ns - loiter_start_ns_ > config_.loiter_escalation_timeout_ns) {
                result.recommended_action = FaultAction::RTL;
                result.reason             = "loiter timeout — escalating to RTL";
                high_water_mark_          = FaultAction::RTL;
                high_water_reason_        = result.reason;
            }
        } else if (result.recommended_action > FaultAction::LOITER) {
            loiter_start_ns_ = 0;  // reset — already above LOITER
        } else {
            loiter_start_ns_ = 0;  // reset — below LOITER (nominal/warn)
        }

        return result;
    }

    /// Reset the escalation state (e.g. after landing / new mission).
    void reset() {
        high_water_mark_       = FaultAction::NONE;
        high_water_reason_     = "nominal";
        loiter_start_ns_       = 0;
        geofence_violated_     = false;
        vio_low_quality_count_ = 0;
    }

    /// Current high-water mark (highest action ever returned).
    [[nodiscard]] FaultAction high_water_mark() const { return high_water_mark_; }

    /// Get the config (for inspection / logging).
    [[nodiscard]] const FaultConfig& config() const { return config_; }

    /// Set geofence violation state (called from the planning loop).
    void set_geofence_violation(bool violated) { geofence_violated_ = violated; }

private:
    static constexpr int kVioDebounceCount = 3;

    FaultConfig config_;
    FaultAction high_water_mark_       = FaultAction::NONE;
    const char* high_water_reason_     = "nominal";
    uint64_t    loiter_start_ns_       = 0;
    bool        geofence_violated_     = false;
    int         vio_low_quality_count_ = 0;

    /// Escalate to a higher action (no-op if target is lower/equal).
    static void escalate(FaultState& state, FaultAction target, const char* reason) {
        if (target > state.recommended_action) {
            state.recommended_action = target;
            state.reason             = reason;
        }
    }

    /// Check if a specific process is reported dead in system health.
    static bool is_process_dead(const drone::ipc::SystemHealth& health, const char* name) {
        for (uint8_t i = 0; i < health.num_processes; ++i) {
            if (std::strncmp(health.processes[i].name, name, sizeof(health.processes[i].name)) ==
                0) {
                return !health.processes[i].alive;
            }
        }
        return false;  // not tracked → assume alive
    }
};

}  // namespace drone::planner
