// process7_system_monitor/src/main.cpp
// Process 7 — System Monitor: CPU / memory / thermal monitoring, process
// watchdog.  Publishes ShmSystemHealth.

#include "monitor/sys_info.h"
#include "ipc/shm_writer.h"
#include "ipc/shm_reader.h"
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

    // ── SHM output ──────────────────────────────────────────
    ShmWriter<drone::ipc::ShmSystemHealth> health_writer;
    if (!health_writer.create(drone::ipc::shm_names::SYSTEM_HEALTH)) {
        spdlog::error("Failed to create system health SHM");
        return 1;
    }

    // ── Optional: read FC state for battery info ────────────
    ShmReader<drone::ipc::ShmFCState> fc_reader;
    fc_reader.open(drone::ipc::shm_names::FC_STATE);

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

    auto prev_cpu = drone::monitor::read_cpu_times();
    uint32_t tick = 0;

    // ── Main loop (1 Hz) ────────────────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        tick++;

        // CPU
        auto now_cpu = drone::monitor::read_cpu_times();
        float cpu_usage = drone::monitor::compute_cpu_usage(prev_cpu, now_cpu);
        prev_cpu = now_cpu;

        // Memory
        auto mem = drone::monitor::read_meminfo();

        // Temperature
        float temp = drone::monitor::read_cpu_temp();

        // Disk (every N ticks to reduce overhead)
        static drone::monitor::DiskInfo disk{};
        if (tick % disk_interval == 1) {
            disk = drone::monitor::read_disk_usage();
        }

        // Battery (from FC)
        float battery = 100.0f;
        drone::ipc::ShmFCState fc{};
        if (fc_reader.is_open() && fc_reader.read(fc) && fc.connected) {
            battery = fc.battery_remaining;
        }

        // Build health status
        drone::ipc::ShmSystemHealth health{};
        health.timestamp_ns = std::chrono::duration_cast<
            std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        health.cpu_usage_percent    = cpu_usage;
        health.memory_usage_percent = mem.usage_percent;
        health.cpu_temp_c           = temp;
        health.max_temp_c           = temp;
        health.disk_usage_percent   = disk.usage_percent;
        health.power_watts          = battery * 0.16f;  // rough estimate

        // Determine overall status via thermal_zone
        health.thermal_zone = 0;  // normal
        if (cpu_usage > cpu_warn || mem.usage_percent > mem_warn ||
            temp > temp_warn || battery < batt_warn) {
            health.thermal_zone = 2;  // hot
        }
        if (temp > temp_crit || battery < batt_crit || disk.usage_percent > disk_crit) {
            health.thermal_zone = 3;  // critical
        }

        health_writer.write(health);

        // Log summary
        if (tick % 5 == 0) {
            const char* status_str = "NOMINAL";
            if (health.thermal_zone == 2)
                status_str = "WARNING";
            else if (health.thermal_zone == 3)
                status_str = "CRITICAL";

            spdlog::info("[SysMon] CPU={:.1f}% MEM={:.1f}% TEMP={:.1f}°C "
                         "DISK={:.0f}% BATT={:.0f}% => {}",
                         cpu_usage, mem.usage_percent, temp,
                         disk.usage_percent, battery, status_str);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    spdlog::info("=== System Monitor stopped ===");
    return 0;
}
