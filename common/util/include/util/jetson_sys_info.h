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
#include <fstream>
#include <string>

namespace drone::util {

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

    [[nodiscard]] static float read_thermal_zone(int zone_idx) {
        char temp_path[128];
        std::snprintf(temp_path, sizeof(temp_path),
                      "/sys/devices/virtual/thermal/thermal_zone%d/temp", zone_idx);
        std::ifstream temp_f(temp_path);
        if (temp_f.is_open()) {
            int millideg = 0;
            temp_f >> millideg;
            return static_cast<float>(millideg) / 1000.0f;
        }
        return kFallbackCpuTempC;
    }
};

}  // namespace drone::util
