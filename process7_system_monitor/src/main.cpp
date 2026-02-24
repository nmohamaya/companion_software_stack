// process7_system_monitor/src/main.cpp
// Process 7 — System Monitor: CPU / memory / thermal monitoring, process
// watchdog.  Publishes ShmSystemHealth.

#include "monitor/sys_info.h"
#include "monitor/iprocess_monitor.h"
#include "ipc/shm_message_bus.h"
#include "ipc/shm_types.h"
#include "util/signal_handler.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/log_config.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>
#include <spdlog/spdlog.h>

static std::atomic<bool> g_running{true};

int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "system_monitor");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("system_monitor", "/tmp/drone_logs", args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== System Monitor starting (PID {}) ===", getpid());

    // ── Create message bus ──────────────────────────────────
    drone::ipc::ShmMessageBus bus;

    auto health_pub = bus.advertise<drone::ipc::ShmSystemHealth>(
        drone::ipc::shm_names::SYSTEM_HEALTH);
    if (!health_pub->is_ready()) {
        spdlog::error("Failed to create system health publisher");
        return 1;
    }

    // ── Optional: subscribe to FC state for battery info ────
    auto fc_sub = bus.subscribe_lazy<drone::ipc::ShmFCState>();
    fc_sub->connect(drone::ipc::shm_names::FC_STATE);

    spdlog::info("System Monitor READY");

    // Config-driven thresholds
    const float cpu_warn   = cfg.get<float>("system_monitor.thresholds.cpu_warn_percent", 90.0f);
    const float mem_warn   = cfg.get<float>("system_monitor.thresholds.mem_warn_percent", 90.0f);
    const float temp_warn  = cfg.get<float>("system_monitor.thresholds.temp_warn_c", 80.0f);
    const float temp_crit  = cfg.get<float>("system_monitor.thresholds.temp_crit_c", 95.0f);
    const float batt_warn  = cfg.get<float>("system_monitor.thresholds.battery_warn_percent", 20.0f);
    const float batt_crit  = cfg.get<float>("system_monitor.thresholds.battery_crit_percent", 10.0f);
    const float disk_crit  = cfg.get<float>("system_monitor.thresholds.disk_crit_percent", 98.0f);
    const int disk_interval = cfg.get<int>("system_monitor.disk_check_interval_s", 10);
    const int update_rate   = cfg.get<int>("system_monitor.update_rate_hz", 1);
    const int loop_sleep_ms = update_rate > 0 ? 1000 / update_rate : 1000;

    // Create process monitor via strategy factory
    auto monitor = drone::monitor::create_process_monitor(
        "linux", cpu_warn, mem_warn, temp_warn, temp_crit,
        disk_crit, batt_warn, batt_crit, disk_interval);
    spdlog::info("Process monitor: {}", monitor->name());

    uint32_t tick = 0;

    // ── Main loop (1 Hz) ────────────────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        tick++;

        // Collect health via IProcessMonitor strategy
        auto health = monitor->collect();

        // Incorporate battery from FC if available
        float battery = 100.0f;
        drone::ipc::ShmFCState fc{};
        if (fc_sub->is_connected() && fc_sub->receive(fc) && fc.connected) {
            battery = fc.battery_remaining;
        }
        health.power_watts = battery * 0.16f;  // rough estimate

        // Apply battery thresholds
        if (battery < batt_warn) {
            health.thermal_zone = std::max(health.thermal_zone, static_cast<uint8_t>(2));
        }
        if (battery < batt_crit) {
            health.thermal_zone = 3;
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
                         health.cpu_usage_percent, health.memory_usage_percent,
                         health.cpu_temp_c,
                         health.disk_usage_percent, battery, status_str);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    spdlog::info("=== System Monitor stopped ===");
    return 0;
}
