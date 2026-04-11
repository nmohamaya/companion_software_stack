// process7_system_monitor/include/monitor/iprocess_monitor.h
// Strategy interface for system health monitoring.
//
// The process monitor collects system metrics (CPU, memory, temperature,
// disk, battery) and produces a health summary.  Swapping the
// implementation lets us move from Linux /proc to Jetson-specific APIs,
// DBUS, or a remote health aggregator.
//
// thermal_zone is a composite system status field (not purely thermal):
//   0 = normal, 2 = warning (CPU/mem/temp/battery), 3 = critical (temp/disk/battery).
//   Zone 1 is reserved for future "warm" status and currently unused.
#pragma once

#include "ipc/ipc_types.h"
#include "util/ilogger.h"
#include "util/isys_info.h"

#include <chrono>
#include <memory>
#include <string>

namespace drone::monitor {

/// Abstract system health monitor.
class IProcessMonitor {
public:
    virtual ~IProcessMonitor() = default;

    /// Collect current system health metrics.
    /// @return A filled SystemHealth struct.
    [[nodiscard]] virtual drone::ipc::SystemHealth collect() = 0;

    /// Incorporate external battery level into the next health reading.
    virtual void set_battery_percent(float /*battery*/) {}

    /// Human-readable name for logging.
    [[nodiscard]] virtual std::string name() const = 0;
};

// ─────────────────────────────────────────────────────────────
// Linux process monitor — reads from /proc and /sys via ISysInfo.
// This is the current behaviour extracted into a strategy object.
// ─────────────────────────────────────────────────────────────

class LinuxProcessMonitor final : public IProcessMonitor {
public:
    /// @param sys_info    Platform abstraction for system metrics.
    /// @param cpu_warn    CPU % threshold for WARNING status.
    /// @param mem_warn    Memory % threshold for WARNING status.
    /// @param temp_warn   Temperature (C) threshold for WARNING status.
    /// @param temp_crit   Temperature (C) threshold for CRITICAL status.
    /// @param disk_crit   Disk % threshold for CRITICAL status.
    /// @param batt_warn   Battery % threshold for WARNING status.
    /// @param batt_crit   Battery % threshold for CRITICAL status.
    /// @param disk_interval  Check disk every N calls to reduce overhead.
    LinuxProcessMonitor(drone::util::ISysInfo& sys_info, float cpu_warn = 90.0f,
                        float mem_warn = 90.0f, float temp_warn = 80.0f, float temp_crit = 95.0f,
                        float disk_crit = 98.0f, float batt_warn = 20.0f, float batt_crit = 10.0f,
                        int disk_interval = 10)
        : sys_info_(sys_info)
        , cpu_warn_(cpu_warn)
        , mem_warn_(mem_warn)
        , temp_warn_(temp_warn)
        , temp_crit_(temp_crit)
        , disk_crit_(disk_crit)
        , batt_warn_(batt_warn)
        , batt_crit_(batt_crit)
        , disk_interval_(disk_interval) {
        prev_cpu_ = sys_info_.read_cpu_times();
    }

    [[nodiscard]] drone::ipc::SystemHealth collect() override {
        ++tick_;

        // CPU — use shared free function with underflow guard
        auto  now_cpu   = sys_info_.read_cpu_times();
        float cpu_usage = drone::util::compute_cpu_usage(prev_cpu_, now_cpu);
        prev_cpu_       = now_cpu;

        // Memory
        auto mem = sys_info_.read_meminfo();

        // Temperature
        float temp = sys_info_.read_cpu_temp();

        // Disk (periodically)
        if (tick_ % disk_interval_ == 1) {
            disk_ = sys_info_.read_disk_usage();
        }

        // Build health struct
        drone::ipc::SystemHealth health{};
        health.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        health.cpu_usage_percent    = cpu_usage;
        health.memory_usage_percent = mem.usage_percent;
        health.cpu_temp_c           = temp;
        health.max_temp_c           = temp;
        health.disk_usage_percent   = disk_.usage_percent;

        static constexpr float kBatteryToPowerCoeff = 0.16f;
        health.power_watts                          = battery_ * kBatteryToPowerCoeff;

        // Composite status (thermal_zone):
        //   0 = normal, 2 = warning, 3 = critical
        health.thermal_zone = 0;
        if (cpu_usage > cpu_warn_ || mem.usage_percent > mem_warn_ || temp > temp_warn_) {
            health.thermal_zone = 2;
        }
        if (temp > temp_crit_ || disk_.usage_percent > disk_crit_) {
            health.thermal_zone = 3;
        }
        // Battery thresholds
        if (battery_ < batt_warn_) {
            health.thermal_zone = std::max(health.thermal_zone, static_cast<uint8_t>(2));
        }
        if (battery_ < batt_crit_) {
            health.thermal_zone = 3;
        }

        return health;
    }

    [[nodiscard]] std::string name() const override { return "LinuxProcessMonitor"; }

    void set_battery_percent(float battery) override { battery_ = battery; }

private:
    drone::util::ISysInfo& sys_info_;
    float                  cpu_warn_;
    float                  mem_warn_;
    float                  temp_warn_;
    float                  temp_crit_;
    float                  disk_crit_;
    float                  batt_warn_;
    float                  batt_crit_;
    int                    disk_interval_;

    drone::util::CpuTimes prev_cpu_;
    drone::util::DiskInfo disk_{};
    float                 battery_ = 100.0f;
    uint32_t              tick_    = 0;
};

/// Factory — creates the appropriate monitor based on config.
/// @param sys_info  Platform abstraction (caller owns lifetime).
/// Unknown backends log a warning and fall back to LinuxProcessMonitor.
[[nodiscard]] inline std::unique_ptr<IProcessMonitor> create_process_monitor(
    drone::util::ISysInfo& sys_info, const std::string& backend = "linux", float cpu_warn = 90.0f,
    float mem_warn = 90.0f, float temp_warn = 80.0f, float temp_crit = 95.0f,
    float disk_crit = 98.0f, float batt_warn = 20.0f, float batt_crit = 10.0f,
    int disk_interval = 10) {
    if (backend != "linux" && backend != "jetson") {
        DRONE_LOG_WARN("[ProcessMonitorFactory] Unknown backend '{}' — falling back to "
                       "LinuxProcessMonitor",
                       backend);
    }
    return std::make_unique<LinuxProcessMonitor>(sys_info, cpu_warn, mem_warn, temp_warn, temp_crit,
                                                 disk_crit, batt_warn, batt_crit, disk_interval);
}

}  // namespace drone::monitor
