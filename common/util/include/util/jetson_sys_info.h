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

#include <cstdio>
#include <fstream>
#include <string>

namespace drone::util {

class JetsonSysInfo final : public LinuxSysInfo {
public:
    /// Jetson-specific: scan thermal zones for the CPU sensor (tegra_tsensor).
    /// Caches the zone index after first discovery to avoid 16+ open() calls per tick.
    /// Falls back to generic zone0 if the CPU zone is not found.
    [[nodiscard]] float read_cpu_temp() override {
        // Use cached zone index if already discovered
        if (cached_cpu_zone_idx_ >= 0) {
            return read_thermal_zone(cached_cpu_zone_idx_);
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
                cached_cpu_zone_idx_ = i;
                return read_thermal_zone(i);
            }
        }

        // Fallback: use generic zone0 (same as LinuxSysInfo)
        return LinuxSysInfo::read_cpu_temp();
    }

    [[nodiscard]] std::string name() const override { return "JetsonSysInfo"; }

private:
    mutable int cached_cpu_zone_idx_ = -1;

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
