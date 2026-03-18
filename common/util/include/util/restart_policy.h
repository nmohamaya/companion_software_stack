// common/util/include/util/restart_policy.h
// Phase 4 (#92) — Configurable restart policy with thermal-aware suppression.
//
// This struct is used by ProcessManager (P7) to decide whether and when to
// restart a dead child process.  Each managed process gets its own policy,
// loaded from config/default.json under "watchdog.processes.<name>".
//
// Key fields:
//   - max_restarts      — give up after N consecutive failures
//   - cooldown_window_s — reset counter after N seconds stable
//   - initial_backoff_ms / max_backoff_ms — exponential backoff
//   - is_critical       — if true, stack enters CRITICAL on give-up
//   - thermal_gate      — defer restart when thermal_zone >= this value
//
// Backoff formula:
//   delay = min(initial_backoff_ms * 2^(attempt - 1), max_backoff_ms)
//
// Thermal gate values:
//   0 = always block (restarts never proceed — effectively disabled)
//   1 = block at WARM or hotter
//   2 = block at HOT or hotter
//   3 = block at CRITICAL only (default — only extreme heat blocks)
//   4 = never block (restarts always proceed regardless of temperature)
#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace drone::util {

// ── Stack-level status model ────────────────────────────────
// Published in SystemHealth::stack_status.
// Transitions: NOMINAL → DEGRADED (non-critical died/restarting)
//              DEGRADED → NOMINAL (all processes recovered)
//              any → CRITICAL (critical process exhausted restarts)
enum class StackStatus : uint8_t {
    NOMINAL  = 0,  // All processes alive, all critical threads healthy
    DEGRADED = 1,  // Non-critical process died or restarting
    CRITICAL = 2,  // Critical process failed (exhausted restarts)
};

inline const char* to_string(StackStatus s) {
    switch (s) {
        case StackStatus::NOMINAL: return "NOMINAL";
        case StackStatus::DEGRADED: return "DEGRADED";
        case StackStatus::CRITICAL: return "CRITICAL";
    }
    return "UNKNOWN";
}

// ── Restart policy ──────────────────────────────────────────
struct RestartPolicy {
    uint32_t max_restarts       = 5;      // Before giving up
    uint32_t cooldown_window_s  = 60;     // Reset counter after stable run
    uint32_t initial_backoff_ms = 500;    // Doubles each attempt
    uint32_t max_backoff_ms     = 30000;  // Cap at 30 s
    bool     is_critical        = false;  // Stack enters CRITICAL on give-up
    uint8_t  thermal_gate       = 3;      // Block restart when thermal_zone >= this
                                          // 0=always block, 3=critical only, 4=never block
    uint32_t max_thermal_defer_s = 300;   // Force restart after this many seconds of
                                          // thermal deferral (0 = no limit) (#183)

    /// Compute backoff delay in milliseconds for a given attempt number.
    /// attempt is 0-based (first restart = attempt 0).
    [[nodiscard]] uint32_t backoff_ms(uint32_t attempt) const {
        uint32_t delay = initial_backoff_ms;
        for (uint32_t i = 0; i < attempt && delay < max_backoff_ms; ++i) {
            delay = std::min(delay * 2, max_backoff_ms);
        }
        return delay;
    }

    /// Check if restart should be blocked due to thermal conditions.
    /// thermal_zone: current system thermal zone (0=normal, 1=warm, 2=hot, 3=critical).
    [[nodiscard]] bool is_thermal_blocked(uint8_t thermal_zone) const {
        if (thermal_gate >= 4) return false;  // Never block
        return thermal_zone >= thermal_gate;
    }
};

// ── Config-driven process descriptor ────────────────────────
// Holds the per-process configuration loaded from JSON.
struct ProcessConfig {
    std::string              name;
    std::string              binary;  // e.g. "build/bin/comms"
    RestartPolicy            policy;
    std::vector<std::string> launch_after;     // Launch dependency names
    std::vector<std::string> restart_cascade;  // Cascade restart targets

    /// Load a ProcessConfig from a JSON object (the per-process block).
    /// Missing fields fall back to defaults.
    static ProcessConfig from_json(const std::string& proc_name, const nlohmann::json& j) {
        ProcessConfig cfg;
        cfg.name = proc_name;

        cfg.binary = j.value("binary", std::string{});

        // Restart policy fields — clamp to valid ranges to prevent
        // negative-to-unsigned wrapping.
        cfg.policy.is_critical  = j.value("critical", false);
        cfg.policy.max_restarts = static_cast<uint32_t>(std::max(0, j.value("max_restarts", 5)));
        cfg.policy.cooldown_window_s =
            static_cast<uint32_t>(std::max(0, j.value("cooldown_s", 60)));
        cfg.policy.initial_backoff_ms =
            static_cast<uint32_t>(std::max(0, j.value("backoff_ms", 500)));
        cfg.policy.max_backoff_ms =
            static_cast<uint32_t>(std::max(0, j.value("max_backoff_ms", 30000)));
        cfg.policy.thermal_gate =
            static_cast<uint8_t>(std::clamp(j.value("thermal_gate", 3), 0, 4));
        cfg.policy.max_thermal_defer_s =
            static_cast<uint32_t>(std::max(0, j.value("max_thermal_defer_s", 300)));

        // Dependency edges
        if (j.contains("launch_after") && j["launch_after"].is_array()) {
            for (const auto& dep : j["launch_after"]) {
                if (dep.is_string()) {
                    cfg.launch_after.push_back(dep.get<std::string>());
                }
            }
        }
        if (j.contains("restart_cascade") && j["restart_cascade"].is_array()) {
            for (const auto& dep : j["restart_cascade"]) {
                if (dep.is_string()) {
                    cfg.restart_cascade.push_back(dep.get<std::string>());
                }
            }
        }

        return cfg;
    }
};

}  // namespace drone::util
