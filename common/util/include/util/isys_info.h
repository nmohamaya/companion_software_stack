// common/util/include/util/isys_info.h
// Platform abstraction interface for system information gathering.
//
// POD structs (CpuTimes, MemInfo, DiskInfo) live here so they can be
// shared across implementations without pulling in platform headers.
// Implementations: LinuxSysInfo (proc/sys), JetsonSysInfo (tegra thermals),
// MockSysInfo (deterministic testing).
#pragma once

#include <cstdint>
#include <string>

#include <sys/types.h>  // pid_t

namespace drone::util {

// ═══════════════════════════════════════════════════════════
// POD structs — shared by all ISysInfo implementations
// ═══════════════════════════════════════════════════════════

struct CpuTimes {
    uint64_t user{0};
    uint64_t nice{0};
    uint64_t system{0};
    uint64_t idle{0};
    uint64_t iowait{0};
    uint64_t irq{0};
    uint64_t softirq{0};
    uint64_t steal{0};

    [[nodiscard]] uint64_t total() const {
        return user + nice + system + idle + iowait + irq + softirq + steal;
    }
    [[nodiscard]] uint64_t active() const { return total() - idle - iowait; }
};

struct MemInfo {
    uint64_t total_kb{0};
    uint64_t available_kb{0};
    float    usage_percent{0.0f};
};

struct DiskInfo {
    uint64_t total_mb{0};
    uint64_t free_mb{0};
    float    usage_percent{0.0f};
};

// ═══════════════════════════════════════════════════════════
// Constants
// ═══════════════════════════════════════════════════════════

/// Fallback CPU temperature when no thermal zone is readable (e.g. containers).
inline constexpr float kFallbackCpuTempC = 42.0f;

// ═══════════════════════════════════════════════════════════
// Free function — shared CPU usage computation with underflow guard
// ═══════════════════════════════════════════════════════════

/// Compute CPU usage percentage from two CpuTimes samples.
/// Returns 0.0f if samples are identical or if prev > now (counter wrap).
[[nodiscard]] inline float compute_cpu_usage(const CpuTimes& prev, const CpuTimes& now) {
    if (now.total() <= prev.total()) return 0.0f;
    const uint64_t dt = now.total() - prev.total();
    const uint64_t da = (now.active() >= prev.active()) ? (now.active() - prev.active()) : 0;
    return 100.0f * static_cast<float>(da) / static_cast<float>(dt);
}

// ═══════════════════════════════════════════════════════════
// ISysInfo — platform abstraction interface
// ═══════════════════════════════════════════════════════════

class ISysInfo {
public:
    virtual ~ISysInfo() = default;

    ISysInfo()                           = default;
    ISysInfo(const ISysInfo&)            = delete;
    ISysInfo& operator=(const ISysInfo&) = delete;
    ISysInfo(ISysInfo&&)                 = default;
    ISysInfo& operator=(ISysInfo&&)      = default;

    /// Read aggregate CPU times from the platform.
    [[nodiscard]] virtual CpuTimes read_cpu_times() = 0;

    /// Read memory information from the platform.
    [[nodiscard]] virtual MemInfo read_meminfo() = 0;

    /// Read CPU temperature in degrees Celsius.
    [[nodiscard]] virtual float read_cpu_temp() = 0;

    /// Read root filesystem disk usage.
    [[nodiscard]] virtual DiskInfo read_disk_usage() = 0;

    /// Check whether a given PID is alive.
    [[nodiscard]] virtual bool is_process_alive(pid_t pid) = 0;

    /// Human-readable name for logging.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::util
