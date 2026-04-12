// process7_system_monitor/include/monitor/sys_info.h
// Linux system information gathering from /proc and /sys.
//
// This header now delegates to the ISysInfo abstraction in common/util.
// It preserves backward-compatible free functions and re-exports POD types
// into the drone::monitor namespace so existing callers need no changes.
//
// DEPRECATED: These free functions always use real /proc reads and bypass
// ISysInfo dependency injection.  New code should inject ISysInfo directly.
// Tests should use MockSysInfo rather than calling these free functions.

#pragma once

#include "util/isys_info.h"
#include "util/linux_sys_info.h"

namespace drone::monitor {

// Re-export POD types into drone::monitor for backward compatibility.
using drone::util::CpuTimes;
using drone::util::DiskInfo;
using drone::util::MemInfo;

// Re-export LinuxSysInfo so monitor code can refer to it.
using drone::util::LinuxSysInfo;

// ─────────────────────────────────────────────────────────────
// Backward-compatible free functions — delegate to LinuxSysInfo.
// Existing code that calls drone::monitor::read_cpu_times() still works.
// ─────────────────────────────────────────────────────────────

/// @note Function-local static keeps cached file handles alive across calls.
/// Safe because P7 is single-threaded for these reads.
inline drone::util::CpuTimes read_cpu_times() {
    static LinuxSysInfo sys;
    return sys.read_cpu_times();
}

// compute_cpu_usage: use drone::util::compute_cpu_usage() directly.
// The free function in isys_info.h includes the underflow guard.
using drone::util::compute_cpu_usage;

inline drone::util::MemInfo read_meminfo() {
    static LinuxSysInfo sys;
    return sys.read_meminfo();
}

inline float read_cpu_temp() {
    static LinuxSysInfo sys;
    return sys.read_cpu_temp();
}

inline drone::util::DiskInfo read_disk_usage() {
    static LinuxSysInfo sys;
    return sys.read_disk_usage();
}

inline bool is_process_alive(pid_t pid) {
    static LinuxSysInfo sys;
    return sys.is_process_alive(pid);
}

}  // namespace drone::monitor
