// common/util/include/util/mock_sys_info.h
// Mock ISysInfo implementation for deterministic unit testing.
// All values are settable via public members.
#pragma once

#include "util/isys_info.h"

namespace drone::util {

class MockSysInfo final : public ISysInfo {
public:
    // ── Injectable state ───────────────────────────────────────
    CpuTimes injected_cpu_times{};
    MemInfo  injected_meminfo{};
    DiskInfo injected_disk_info{};
    float    injected_cpu_temp{kFallbackCpuTempC};
    bool     injected_process_alive{true};

    // ── ISysInfo interface ─────────────────────────────────────

    [[nodiscard]] CpuTimes read_cpu_times() const override { return injected_cpu_times; }

    [[nodiscard]] MemInfo read_meminfo() const override { return injected_meminfo; }

    [[nodiscard]] float read_cpu_temp() const override { return injected_cpu_temp; }

    [[nodiscard]] DiskInfo read_disk_usage() const override { return injected_disk_info; }

    [[nodiscard]] bool is_process_alive(pid_t /*pid*/) const override {
        return injected_process_alive;
    }

    [[nodiscard]] std::string name() const override { return "MockSysInfo"; }
};

}  // namespace drone::util
