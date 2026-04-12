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
#include "util/iclock.h"
#include "util/ilogger.h"
#include "util/isys_info.h"

#include <memory>
#include <string>

namespace drone::monitor {

// ─────────────────────────────────────────────────────────────
// MonitorThresholds — named struct to avoid positional float confusion.
// All thresholds are configurable via drone::Config.
// ─────────────────────────────────────────────────────────────

struct MonitorThresholds {
    float cpu_warn{90.0f};     ///< CPU % threshold for WARNING status.
    float mem_warn{90.0f};     ///< Memory % threshold for WARNING status.
    float temp_warn{80.0f};    ///< Temperature (C) threshold for WARNING status.
    float temp_crit{95.0f};    ///< Temperature (C) threshold for CRITICAL status.
    float disk_crit{98.0f};    ///< Disk % threshold for CRITICAL status.
    float batt_warn{20.0f};    ///< Battery % threshold for WARNING status.
    float batt_crit{10.0f};    ///< Battery % threshold for CRITICAL status.
    float power_coeff{0.16f};  ///< Battery-to-power linear coefficient (watts per %).
    int   disk_interval{
        10};  ///< Check disk every N ticks (= disk_check_interval_s * update_rate_hz).
};

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
    /// @param thresholds  Warning/critical thresholds (all configurable via Config).
    LinuxProcessMonitor(const drone::util::ISysInfo& sys_info, MonitorThresholds thresholds = {})
        : sys_info_(sys_info), thresholds_(thresholds) {
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

        // Disk (periodically — read on first tick, then every disk_interval ticks)
        if (tick_ == 1 || (tick_ % thresholds_.disk_interval) == 0) {
            disk_ = sys_info_.read_disk_usage();
        }

        // Build health struct
        drone::ipc::SystemHealth health{};
        health.timestamp_ns         = drone::util::get_clock().now_ns();
        health.cpu_usage_percent    = cpu_usage;
        health.memory_usage_percent = mem.usage_percent;
        health.cpu_temp_c           = temp;
        health.max_temp_c           = temp;
        health.disk_usage_percent   = disk_.usage_percent;

        // Battery-to-power estimate (linear approximation).
        health.power_watts = battery_ * thresholds_.power_coeff;

        // Composite status (thermal_zone):
        //   0 = normal, 2 = warning, 3 = critical (zone 1 reserved, never assigned)
        health.thermal_zone = 0;
        if (cpu_usage > thresholds_.cpu_warn || mem.usage_percent > thresholds_.mem_warn ||
            temp > thresholds_.temp_warn) {
            health.thermal_zone = 2;
        }
        if (temp > thresholds_.temp_crit || disk_.usage_percent > thresholds_.disk_crit) {
            health.thermal_zone = 3;
        }
        // Battery thresholds
        if (battery_ < thresholds_.batt_warn) {
            health.thermal_zone = std::max(health.thermal_zone, static_cast<uint8_t>(2));
        }
        if (battery_ < thresholds_.batt_crit) {
            health.thermal_zone = 3;
        }

        return health;
    }

    [[nodiscard]] std::string name() const override { return "LinuxProcessMonitor"; }

    void set_battery_percent(float battery) override { battery_ = battery; }

private:
    const drone::util::ISysInfo& sys_info_;
    MonitorThresholds            thresholds_;

    drone::util::CpuTimes prev_cpu_;
    drone::util::DiskInfo disk_{};
    float                 battery_ = 100.0f;
    uint32_t              tick_    = 0;
};

/// Factory — creates a LinuxProcessMonitor with ISysInfo injection.
/// Platform differences (Linux vs Jetson) are handled by the ISysInfo
/// implementation, not by the monitor itself.
/// @param sys_info    Platform abstraction (caller owns lifetime).
/// @param thresholds  Warning/critical thresholds.
[[nodiscard]] inline std::unique_ptr<IProcessMonitor> create_process_monitor(
    const drone::util::ISysInfo& sys_info, MonitorThresholds thresholds = {}) {
    return std::make_unique<LinuxProcessMonitor>(sys_info, thresholds);
}

}  // namespace drone::monitor
