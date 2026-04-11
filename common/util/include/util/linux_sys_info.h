// common/util/include/util/linux_sys_info.h
// Standard Linux ISysInfo implementation — reads from /proc and /sys.
#pragma once

#include "util/isys_info.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

namespace drone::util {

class LinuxSysInfo final : public ISysInfo {
public:
    CpuTimes read_cpu_times() override {
        CpuTimes      t{};
        std::ifstream f("/proc/stat");
        if (!f.is_open()) return t;
        std::string line;
        if (std::getline(f, line)) {
            // "cpu  user nice system idle iowait irq softirq steal ..."
            std::sscanf(line.c_str(), "cpu %lu %lu %lu %lu %lu %lu %lu %lu", &t.user, &t.nice,
                        &t.system, &t.idle, &t.iowait, &t.irq, &t.softirq, &t.steal);
        }
        return t;
    }

    float compute_cpu_usage(const CpuTimes& prev, const CpuTimes& now) override {
        const uint64_t dt = now.total() - prev.total();
        const uint64_t da = now.active() - prev.active();
        if (dt == 0) return 0.0f;
        return 100.0f * static_cast<float>(da) / static_cast<float>(dt);
    }

    MemInfo read_meminfo() override {
        MemInfo       m{};
        std::ifstream f("/proc/meminfo");
        if (!f.is_open()) return m;
        std::string line;
        while (std::getline(f, line)) {
            if (line.rfind("MemTotal:", 0) == 0)
                std::sscanf(line.c_str(), "MemTotal: %lu kB", &m.total_kb);
            else if (line.rfind("MemAvailable:", 0) == 0)
                std::sscanf(line.c_str(), "MemAvailable: %lu kB", &m.available_kb);
        }
        if (m.total_kb > 0) {
            m.usage_percent = 100.0f * static_cast<float>(m.total_kb - m.available_kb) /
                              static_cast<float>(m.total_kb);
        }
        return m;
    }

    float read_cpu_temp() override {
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
        return 42.0f;  // simulated fallback
    }

    DiskInfo read_disk_usage() override {
        DiskInfo d{};
        FILE*    fp = popen("df -m / 2>/dev/null | tail -1", "r");
        if (fp) {
            char buf[256];
            if (fgets(buf, sizeof(buf), fp)) {
                uint64_t total = 0, used = 0, avail = 0;
                char     dev[128], mount[128];
                int      pct = 0;
                if (std::sscanf(buf, "%s %lu %lu %lu %d%% %s", dev, &total, &used, &avail, &pct,
                                mount) >= 5) {
                    d.total_mb      = total;
                    d.free_mb       = avail;
                    d.usage_percent = static_cast<float>(pct);
                }
            }
            pclose(fp);
        }
        return d;
    }

    bool is_process_alive(pid_t pid) override {
        if (pid <= 0) return false;
        char path[64];
        std::snprintf(path, sizeof(path), "/proc/%d/stat", pid);
        std::ifstream f(path);
        return f.is_open();
    }

    std::string name() const override { return "LinuxSysInfo"; }
};

}  // namespace drone::util
