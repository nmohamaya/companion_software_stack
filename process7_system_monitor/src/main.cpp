// process7_system_monitor/src/main.cpp
// Process 7 — System Monitor: CPU / memory / thermal monitoring, process
// watchdog.  Publishes ShmSystemHealth.

#include "ipc/message_bus_factory.h"
#include "ipc/shm_types.h"
#include "ipc/zenoh_liveliness.h"
#include "monitor/iprocess_monitor.h"
#include "monitor/sys_info.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/log_config.h"
#include "util/signal_handler.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <spdlog/spdlog.h>
#include <unistd.h>

static std::atomic<bool> g_running{true};

int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "system_monitor");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("system_monitor", LogConfig::resolve_log_dir(), args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== System Monitor starting (PID {}) ===", getpid());

    // ── Create message bus (config-driven: shm or zenoh) ───
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("system_monitor");

    auto health_pub = drone::ipc::bus_advertise<drone::ipc::ShmSystemHealth>(
        bus, drone::ipc::shm_names::SYSTEM_HEALTH);
    if (!health_pub->is_ready()) {
        spdlog::error("Failed to create system health publisher");
        return 1;
    }

    // ── Optional: subscribe to FC state for battery info ────
    auto fc_sub = drone::ipc::bus_subscribe_optional<drone::ipc::ShmFCState>(
        bus, drone::ipc::shm_names::FC_STATE);

    // ── Process health monitoring via liveliness tokens ─────
    // Critical processes: if these die, flag critical_failure.
    static const std::vector<std::string> critical_processes = {"comms", "slam_vio_nav"};

    // Shared state for liveliness events (updated from callbacks)
    struct ProcessLiveness {
        std::mutex mutex;
        struct Event {
            std::string name;
            bool        alive;
            uint64_t    timestamp_ns;
        };
        std::vector<Event> events;
    };
    auto liveness = std::make_shared<ProcessLiveness>();

    drone::ipc::LivelinessMonitor liveliness_monitor(
        [liveness](const std::string& proc) {
            spdlog::info("[SysMon] Process ALIVE: {}", proc);
            auto now_ns =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
            std::lock_guard<std::mutex> lock(liveness->mutex);
            liveness->events.push_back({proc, true, now_ns});
        },
        [liveness](const std::string& proc) {
            spdlog::error("[SysMon] Process DIED: {}", proc);
            auto now_ns =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
            std::lock_guard<std::mutex> lock(liveness->mutex);
            liveness->events.push_back({proc, false, now_ns});
        });

    // Local map: process name → (alive, last_seen_ns)
    struct ProcessState {
        bool     alive        = false;
        uint64_t last_seen_ns = 0;
    };
    std::unordered_map<std::string, ProcessState> process_alive_map;

    spdlog::info("System Monitor READY");

    // Config-driven thresholds
    const float cpu_warn  = cfg.get<float>("system_monitor.thresholds.cpu_warn_percent", 90.0f);
    const float mem_warn  = cfg.get<float>("system_monitor.thresholds.mem_warn_percent", 90.0f);
    const float temp_warn = cfg.get<float>("system_monitor.thresholds.temp_warn_c", 80.0f);
    const float temp_crit = cfg.get<float>("system_monitor.thresholds.temp_crit_c", 95.0f);
    const float batt_warn = cfg.get<float>("system_monitor.thresholds.battery_warn_percent", 20.0f);
    const float batt_crit = cfg.get<float>("system_monitor.thresholds.battery_crit_percent", 10.0f);
    const float disk_crit = cfg.get<float>("system_monitor.thresholds.disk_crit_percent", 98.0f);
    const int   disk_check_s  = cfg.get<int>("system_monitor.disk_check_interval_s", 10);
    const int   update_rate   = cfg.get<int>("system_monitor.update_rate_hz", 1);
    const int   loop_sleep_ms = update_rate > 0 ? 1000 / update_rate : 1000;

    // Convert disk check interval from seconds to ticks (calls)
    const int disk_interval_ticks = std::max(1, disk_check_s * (update_rate > 0 ? update_rate : 1));

    // Create process monitor via strategy factory (backend from config)
    const std::string monitor_backend = cfg.get<std::string>("system_monitor.backend", "linux");
    auto              monitor         = drone::monitor::create_process_monitor(
        monitor_backend, cpu_warn, mem_warn, temp_warn, temp_crit, disk_crit, batt_warn, batt_crit,
        disk_interval_ticks);
    spdlog::info("Process monitor: {}", monitor->name());

    uint32_t tick = 0;

    // ── Main loop (1 Hz) ────────────────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        tick++;

        // Incorporate battery from FC if available
        float                  battery = 100.0f;
        drone::ipc::ShmFCState fc{};
        if (fc_sub->is_connected() && fc_sub->receive(fc) && fc.connected) {
            battery = fc.battery_remaining;
        }
        monitor->set_battery_percent(battery);

        // Collect health via IProcessMonitor strategy
        // (battery thresholds are applied inside the strategy)
        auto health = monitor->collect();

        // ── Merge liveliness events into health struct ──────
        {
            std::lock_guard<std::mutex> lock(liveness->mutex);
            for (auto& evt : liveness->events) {
                process_alive_map[evt.name] = {evt.alive, evt.timestamp_ns};
            }
            liveness->events.clear();
        }

        // Populate ProcessHealthEntry array
        health.num_processes    = 0;
        health.critical_failure = false;
        for (auto& [name, state] : process_alive_map) {
            if (health.num_processes < drone::ipc::kMaxTrackedProcesses) {
                auto& entry = health.processes[health.num_processes];
                std::memset(entry.name, 0, sizeof(entry.name));
                std::strncpy(entry.name, name.c_str(), sizeof(entry.name) - 1);
                entry.alive        = state.alive;
                entry.last_seen_ns = state.last_seen_ns;
                health.num_processes++;
            }
            if (!state.alive) {
                auto it = std::find(critical_processes.begin(), critical_processes.end(), name);
                if (it != critical_processes.end()) {
                    health.critical_failure = true;
                }
            }
        }

        health_pub->publish(health);

        // Log summary
        if (tick % 5 == 0) {
            const char* status_str = "NOMINAL";
            if (health.thermal_zone == 2)
                status_str = "WARNING";
            else if (health.thermal_zone == 3)
                status_str = "CRITICAL";

            spdlog::info("[SysMon] CPU={:.1f}% MEM={:.1f}% TEMP={:.1f}°C "
                         "DISK={:.0f}% BATT={:.0f}% => {}",
                         health.cpu_usage_percent, health.memory_usage_percent, health.cpu_temp_c,
                         health.disk_usage_percent, battery, status_str);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    spdlog::info("=== System Monitor stopped ===");
    return 0;
}
