// common/util/include/util/linux_sys_info.h
// Standard Linux ISysInfo implementation — reads from /proc and /sys.
//
// Performance: file handles for /proc/stat, /proc/meminfo, and the thermal
// zone are cached as mutable members.  On each read we seekg(0) + clear()
// instead of opening a new fd — saves 3+ open/close syscalls per P7 tick.
#pragma once

#include "util/isys_info.h"

#include <cinttypes>
#include <cstdio>
#include <fstream>
#include <string>

#include <sys/statvfs.h>

namespace drone::util {

/// Standard Linux ISysInfo — reads /proc and /sys via cached file handles.
/// Thread safety: NOT thread-safe — single-thread use only (see ISysInfo).
class LinuxSysInfo : public ISysInfo {
public:
    [[nodiscard]] CpuTimes read_cpu_times() const override {
        CpuTimes t{};
        if (!rewind_or_open(cached_proc_stat_, "/proc/stat")) return t;
        std::string line;
        if (std::getline(cached_proc_stat_, line)) {
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
        MemInfo m{};
        if (!rewind_or_open(cached_proc_meminfo_, "/proc/meminfo")) return m;
        std::string line;
        while (std::getline(cached_proc_meminfo_, line)) {
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
        // Reuse cached handle if a path was previously discovered
        if (cached_thermal_path_ != nullptr) {
            if (rewind_or_open(cached_thermal_, cached_thermal_path_)) {
                return read_millideg(cached_thermal_);
            }
        }

        // First call or cached handle failed — probe both paths
        // (string literals have static storage duration — pointer stays valid)
        const char* paths[] = {
            "/sys/devices/virtual/thermal/thermal_zone0/temp",
            "/sys/class/thermal/thermal_zone0/temp",
        };
        for (const auto* path : paths) {
            if (rewind_or_open(cached_thermal_, path)) {
                cached_thermal_path_ = path;
                return read_millideg(cached_thermal_);
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

protected:
    /// Read a millidegree integer from a thermal zone file and convert to Celsius.
    static float read_millideg(std::ifstream& stream) {
        int millideg = 0;
        stream >> millideg;
        return static_cast<float>(millideg) / 1000.0f;
    }

    /// Rewind a cached ifstream to the beginning, or (re)open it on failure.
    /// Returns true if the stream is ready for reading.
    /// @note Not thread-safe — caller must ensure single-threaded access.
    bool rewind_or_open(std::ifstream& stream, const char* path) const {
        if (stream.is_open()) {
            stream.clear();
            stream.seekg(0);
            if (stream.good()) return true;
            // Seek failed — close and fall through to reopen
            stream.close();
        }
        stream.open(path);
        return stream.is_open();
    }

private:
    // Cached file handles — mutable because caching is an implementation
    // detail invisible to callers (all read methods are const).
    mutable std::ifstream cached_proc_stat_;
    mutable std::ifstream cached_proc_meminfo_;
    mutable std::ifstream cached_thermal_;
    mutable const char*   cached_thermal_path_{nullptr};
};

}  // namespace drone::util
