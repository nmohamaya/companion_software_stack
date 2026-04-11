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
    float    injected_cpu_temp{42.0f};
    bool     injected_process_alive{true};

    // ── ISysInfo interface ─────────────────────────────────────

    CpuTimes read_cpu_times() override { return injected_cpu_times; }

    float compute_cpu_usage(const CpuTimes& prev, const CpuTimes& now) override {
        const uint64_t dt = now.total() - prev.total();
        const uint64_t da = now.active() - prev.active();
        if (dt == 0) return 0.0f;
        return 100.0f * static_cast<float>(da) / static_cast<float>(dt);
    }

    MemInfo read_meminfo() override { return injected_meminfo; }

    float read_cpu_temp() override { return injected_cpu_temp; }

    DiskInfo read_disk_usage() override { return injected_disk_info; }

    bool is_process_alive(pid_t /*pid*/) override { return injected_process_alive; }

    std::string name() const override { return "MockSysInfo"; }
};

}  // namespace drone::util
