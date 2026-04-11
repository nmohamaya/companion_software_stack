// common/util/include/util/isys_info.h
// Platform abstraction interface for system information gathering.
//
// POD structs (CpuTimes, MemInfo, DiskInfo) live here so they can be
// shared across implementations without pulling in platform headers.
// Implementations: LinuxSysInfo (proc/sys), JetsonSysInfo (tegra thermals),
// MockSysInfo (deterministic testing).
#pragma once

#include <cstdint>
#include <memory>
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

    uint64_t total() const { return user + nice + system + idle + iowait + irq + softirq + steal; }
    uint64_t active() const { return total() - idle - iowait; }
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
    virtual CpuTimes read_cpu_times() = 0;

    /// Compute CPU usage percentage from two samples.
    virtual float compute_cpu_usage(const CpuTimes& prev, const CpuTimes& now) = 0;

    /// Read memory information from the platform.
    virtual MemInfo read_meminfo() = 0;

    /// Read CPU temperature in degrees Celsius.
    virtual float read_cpu_temp() = 0;

    /// Read root filesystem disk usage.
    virtual DiskInfo read_disk_usage() = 0;

    /// Check whether a given PID is alive.
    virtual bool is_process_alive(pid_t pid) = 0;

    /// Human-readable name for logging.
    virtual std::string name() const = 0;
};

// ═══════════════════════════════════════════════════════════
// Factory — creates the appropriate ISysInfo based on platform string.
// Declared here, defined in isys_info_factory.h (after all impls).
// ═══════════════════════════════════════════════════════════

/// Create a platform-specific ISysInfo implementation.
/// @param platform  One of "linux", "jetson", or "mock".
/// @return Owning pointer to ISysInfo implementation.
std::unique_ptr<ISysInfo> create_sys_info(const std::string& platform);

}  // namespace drone::util
