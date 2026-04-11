// common/util/include/util/linux_sys_info.h
// Standard Linux ISysInfo implementation — reads from /proc and /sys.
#pragma once

#include "util/isys_info.h"

#include <cinttypes>
#include <cstdio>
#include <fstream>
#include <string>

#include <sys/statvfs.h>

namespace drone::util {

class LinuxSysInfo : public ISysInfo {
public:
    [[nodiscard]] CpuTimes read_cpu_times() const override {
        CpuTimes      t{};
        std::ifstream f("/proc/stat");
        if (!f.is_open()) return t;
        std::string line;
        if (std::getline(f, line)) {
            // NOLINTNEXTLINE(cert-err34-c) — /proc/stat has a fixed format
            std::sscanf(line.c_str(),
                        "cpu %" SCNu64 " %" SCNu64 " %" SCNu64 " %" SCNu64 " %" SCNu64 " %" SCNu64
                        " %" SCNu64 " %" SCNu64,
                        &t.user, &t.nice, &t.system, &t.idle, &t.iowait, &t.irq, &t.softirq,
                        &t.steal);
        }
        return t;
    }

    [[nodiscard]] MemInfo read_meminfo() const override {
        MemInfo       m{};
        std::ifstream f("/proc/meminfo");
        if (!f.is_open()) return m;
        std::string line;
        while (std::getline(f, line)) {
            // NOLINTNEXTLINE(cert-err34-c) — /proc/meminfo has a fixed format
            if (line.rfind("MemTotal:", 0) == 0)
                std::sscanf(line.c_str(), "MemTotal: %" SCNu64 " kB", &m.total_kb);
            else if (line.rfind("MemAvailable:", 0) == 0)
                std::sscanf(line.c_str(), "MemAvailable: %" SCNu64 " kB", &m.available_kb);
            // Early exit once both fields are found
            if (m.total_kb > 0 && m.available_kb > 0) break;
        }
        if (m.total_kb > 0) {
            // Guard against corrupted /proc/meminfo where available > total
            if (m.available_kb > m.total_kb) m.available_kb = m.total_kb;
            m.usage_percent = 100.0f * static_cast<float>(m.total_kb - m.available_kb) /
                              static_cast<float>(m.total_kb);
        }
        return m;
    }

    [[nodiscard]] float read_cpu_temp() const override {
        const char* paths[] = {
            "/sys/devices/virtual/thermal/thermal_zone0/temp",
            "/sys/class/thermal/thermal_zone0/temp",
        };
        for (auto path : paths) {
            std::ifstream f(path);
            if (f.is_open()) {
                int millideg = 0;
                f >> millideg;
                return static_cast<float>(millideg) / 1000.0f;
            }
        }
        return kFallbackCpuTempC;
    }

    [[nodiscard]] DiskInfo read_disk_usage() const override {
        DiskInfo       d{};
        struct statvfs sv {};
        if (statvfs("/", &sv) == 0) {
            const auto block_size = static_cast<uint64_t>(sv.f_frsize);
            d.total_mb = (static_cast<uint64_t>(sv.f_blocks) * block_size) / (1024ULL * 1024ULL);
            d.free_mb  = (static_cast<uint64_t>(sv.f_bavail) * block_size) / (1024ULL * 1024ULL);
            if (d.total_mb > 0) {
                // Guard against statvfs inconsistency where free > total
                if (d.free_mb > d.total_mb) d.free_mb = d.total_mb;
                d.usage_percent = 100.0f * static_cast<float>(d.total_mb - d.free_mb) /
                                  static_cast<float>(d.total_mb);
            }
        }
        return d;
    }

    [[nodiscard]] bool is_process_alive(pid_t pid) const override {
        if (pid <= 0) return false;
        char path[64];
        std::snprintf(path, sizeof(path), "/proc/%d/stat", pid);
        std::ifstream f(path);
        return f.is_open();
    }

    [[nodiscard]] std::string name() const override { return "LinuxSysInfo"; }
};

}  // namespace drone::util
