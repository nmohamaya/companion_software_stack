// process7_system_monitor/include/monitor/iprocess_monitor.h
// Strategy interface for system health monitoring.
//
// The process monitor collects system metrics (CPU, memory, temperature,
// disk, battery) and produces a health summary.  Swapping the
// implementation lets us move from Linux /proc to Jetson-specific APIs,
// DBUS, or a remote health aggregator.
#pragma once

#include "ipc/shm_types.h"
#include "monitor/sys_info.h"

#include <string>
#include <memory>
#include <chrono>

namespace drone::monitor {

/// Abstract system health monitor.
class IProcessMonitor {
public:
    virtual ~IProcessMonitor() = default;

    /// Collect current system health metrics.
    /// @return A filled ShmSystemHealth struct.
    virtual drone::ipc::ShmSystemHealth collect() = 0;

    /// Human-readable name for logging.
    virtual std::string name() const = 0;
};

// ─────────────────────────────────────────────────────────────
// Linux process monitor — reads from /proc and /sys.
// This is the current behaviour extracted into a strategy object.
// ─────────────────────────────────────────────────────────────

class LinuxProcessMonitor final : public IProcessMonitor {
public:
    /// @param cpu_warn    CPU % threshold for WARNING status.
    /// @param mem_warn    Memory % threshold for WARNING status.
    /// @param temp_warn   Temperature (C) threshold for WARNING status.
    /// @param temp_crit   Temperature (C) threshold for CRITICAL status.
    /// @param disk_crit   Disk % threshold for CRITICAL status.
    /// @param batt_warn   Battery % threshold for WARNING status.
    /// @param batt_crit   Battery % threshold for CRITICAL status.
    /// @param disk_interval  Check disk every N calls to reduce popen overhead.
    LinuxProcessMonitor(float cpu_warn = 90.0f,
                        float mem_warn = 90.0f,
                        float temp_warn = 80.0f,
                        float temp_crit = 95.0f,
                        float disk_crit = 98.0f,
                        float batt_warn = 20.0f,
                        float batt_crit = 10.0f,
                        int disk_interval = 10)
        : cpu_warn_(cpu_warn), mem_warn_(mem_warn)
        , temp_warn_(temp_warn), temp_crit_(temp_crit)
        , disk_crit_(disk_crit), batt_warn_(batt_warn)
        , batt_crit_(batt_crit), disk_interval_(disk_interval)
    {
        prev_cpu_ = read_cpu_times();
    }

    drone::ipc::ShmSystemHealth collect() override {
        ++tick_;

        // CPU
        auto now_cpu = read_cpu_times();
        float cpu_usage = compute_cpu_usage(prev_cpu_, now_cpu);
        prev_cpu_ = now_cpu;

        // Memory
        auto mem = read_meminfo();

        // Temperature
        float temp = read_cpu_temp();

        // Disk (periodically)
        if (tick_ % disk_interval_ == 1) {
            disk_ = read_disk_usage();
        }

        // Build health struct
        drone::ipc::ShmSystemHealth health{};
        health.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        health.cpu_usage_percent    = cpu_usage;
        health.memory_usage_percent = mem.usage_percent;
        health.cpu_temp_c           = temp;
        health.max_temp_c           = temp;
        health.disk_usage_percent   = disk_.usage_percent;

        // Thermal zone (overall status)
        health.thermal_zone = 0;  // normal
        if (cpu_usage > cpu_warn_ || mem.usage_percent > mem_warn_ ||
            temp > temp_warn_) {
            health.thermal_zone = 2;  // hot
        }
        if (temp > temp_crit_ || disk_.usage_percent > disk_crit_) {
            health.thermal_zone = 3;  // critical
        }

        return health;
    }

    std::string name() const override { return "LinuxProcessMonitor"; }

    /// Incorporate battery level into the health reading.
    void set_battery_percent(float battery) { battery_ = battery; }

private:
    float cpu_warn_, mem_warn_, temp_warn_, temp_crit_;
    float disk_crit_, batt_warn_, batt_crit_;
    int disk_interval_;

    CpuTimes prev_cpu_;
    DiskInfo disk_{};
    float battery_ = 100.0f;
    uint32_t tick_ = 0;
};

/// Factory — creates the appropriate monitor based on config.
inline std::unique_ptr<IProcessMonitor> create_process_monitor(
    const std::string& backend = "linux",
    float cpu_warn = 90.0f, float mem_warn = 90.0f,
    float temp_warn = 80.0f, float temp_crit = 95.0f,
    float disk_crit = 98.0f, float batt_warn = 20.0f,
    float batt_crit = 10.0f, int disk_interval = 10)
{
    if (backend == "linux") {
        return std::make_unique<LinuxProcessMonitor>(
            cpu_warn, mem_warn, temp_warn, temp_crit,
            disk_crit, batt_warn, batt_crit, disk_interval);
    }
    throw std::runtime_error("Unknown process monitor: " + backend);
}

}  // namespace drone::monitor
