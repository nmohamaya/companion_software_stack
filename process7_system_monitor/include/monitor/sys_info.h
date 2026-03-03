// process7_system_monitor/include/monitor/sys_info.h
// Linux system information gathering from /proc and /sys.

#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include <spdlog/spdlog.h>

namespace drone::monitor {

struct CpuTimes {
    uint64_t user{0}, nice{0}, system{0}, idle{0};
    uint64_t iowait{0}, irq{0}, softirq{0}, steal{0};

    uint64_t total() const { return user + nice + system + idle + iowait + irq + softirq + steal; }
    uint64_t active() const { return total() - idle - iowait; }
};

// ── CPU usage (two-sample delta) ────────────────────────────
inline CpuTimes read_cpu_times() {
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

inline float compute_cpu_usage(const CpuTimes& prev, const CpuTimes& now) {
    uint64_t dt = now.total() - prev.total();
    uint64_t da = now.active() - prev.active();
    if (dt == 0) return 0.0f;
    return 100.0f * static_cast<float>(da) / static_cast<float>(dt);
}

// ── Memory ──────────────────────────────────────────────────
struct MemInfo {
    uint64_t total_kb{0};
    uint64_t available_kb{0};
    float    usage_percent{0.0f};
};

inline MemInfo read_meminfo() {
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

// ── Temperature ─────────────────────────────────────────────
inline float read_cpu_temp() {
    // Try Jetson thermal zone first, then generic
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

// ── Disk usage (simple statfs wrapper) ──────────────────────
struct DiskInfo {
    uint64_t total_mb{0};
    uint64_t free_mb{0};
    float    usage_percent{0.0f};
};

inline DiskInfo read_disk_usage() {
    DiskInfo d{};
    // Use /proc/mounts approach — simple simulation
    FILE* fp = popen("df -m / 2>/dev/null | tail -1", "r");
    if (fp) {
        char buf[256];
        if (fgets(buf, sizeof(buf), fp)) {
            uint64_t total, used, avail;
            char     dev[128], mount[128];
            int      pct;
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

// ── Process watchdog (check if PID is alive) ────────────────
inline bool is_process_alive(pid_t pid) {
    if (pid <= 0) return false;
    char path[64];
    std::snprintf(path, sizeof(path), "/proc/%d/stat", pid);
    std::ifstream f(path);
    return f.is_open();
}

}  // namespace drone::monitor
