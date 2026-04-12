// common/util/include/util/jetson_sys_info.h
// Jetson-specific ISysInfo implementation.
//
// Extends LinuxSysInfo — inherits all /proc-based readers and only
// overrides read_cpu_temp() to prefer the Tegra-specific thermal zone
// (the CPU zone reported by tegra_tsensor).
//
// TODO(hardware): When real Jetson Orin hardware is available, extend
// with jetson_clocks, power rail (INA3221), and GPU utilization reads.
#pragma once

#include "util/linux_sys_info.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

namespace drone::util {

/// Jetson-specific ISysInfo — inherits LinuxSysInfo, overrides CPU thermal zone.
/// Thread safety: NOT thread-safe — single-thread use only (see ISysInfo).
/// The atomic cached_cpu_zone_idx_ is safe for concurrent reads after first
/// write, but the mutable ifstreams are not protected.
class JetsonSysInfo final : public LinuxSysInfo {
public:
    /// Jetson-specific: scan thermal zones for the CPU sensor (tegra_tsensor).
    /// Caches the zone index after first discovery to avoid 16+ open() calls per tick.
    /// Also caches "not found" (kZoneNotFound) so we don't rescan every tick on
    /// non-Jetson hardware where the CPU zone name doesn't match.
    [[nodiscard]] float read_cpu_temp() const override {
        // Use cached result: either a found zone index or "not found" sentinel
        const int cached = cached_cpu_zone_idx_.load(std::memory_order_acquire);
        if (cached != kZoneUncached) {
            if (cached == kZoneNotFound) {
                return LinuxSysInfo::read_cpu_temp();
            }
            return read_thermal_zone(cached);
        }

        // First call: scan up to 16 thermal zones for the CPU-specific sensor
        for (int i = 0; i < 16; ++i) {
            char type_path[128];
            std::snprintf(type_path, sizeof(type_path),
                          "/sys/devices/virtual/thermal/thermal_zone%d/type", i);
            std::ifstream type_f(type_path);
            if (!type_f.is_open()) break;

            std::string zone_type;
            std::getline(type_f, zone_type);

            // Jetson Orin CPU thermal zone names
            if (zone_type == "CPU-therm" || zone_type == "cpu-therm" ||
                zone_type == "Tdiode_tegra") {
                cached_cpu_zone_idx_.store(i, std::memory_order_release);
                return read_thermal_zone(i);
            }
        }

        // Cache "not found" to avoid rescan on every tick
        cached_cpu_zone_idx_.store(kZoneNotFound, std::memory_order_release);
        return LinuxSysInfo::read_cpu_temp();
    }

    [[nodiscard]] std::string name() const override { return "JetsonSysInfo"; }

private:
    static constexpr int     kZoneUncached = -1;
    static constexpr int     kZoneNotFound = -2;
    mutable std::atomic<int> cached_cpu_zone_idx_{kZoneUncached};

    /// Cached file handle for the discovered Jetson CPU thermal zone.
    mutable std::ifstream cached_jetson_thermal_;
    mutable char          cached_jetson_thermal_path_[128]{};

    [[nodiscard]] float read_thermal_zone(int zone_idx) const {
        // Build the expected path (only changes if zone_idx changes, which
        // it doesn't after the initial scan — but we handle it for safety).
        char temp_path[128];
        std::snprintf(temp_path, sizeof(temp_path),
                      "/sys/devices/virtual/thermal/thermal_zone%d/temp", zone_idx);

        // If cached path doesn't match (first call or zone change), reopen
        if (std::strcmp(cached_jetson_thermal_path_, temp_path) != 0) {
            if (cached_jetson_thermal_.is_open()) cached_jetson_thermal_.close();
            std::snprintf(cached_jetson_thermal_path_, sizeof(cached_jetson_thermal_path_), "%s",
                          temp_path);
        }

        if (rewind_or_open(cached_jetson_thermal_, cached_jetson_thermal_path_)) {
            int millideg = 0;
            cached_jetson_thermal_ >> millideg;
            return static_cast<float>(millideg) / 1000.0f;
        }
        return kFallbackCpuTempC;
    }
};

}  // namespace drone::util
