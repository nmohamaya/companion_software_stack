// tests/test_system_monitor.cpp — Tests for sys_info.h functions
#include "monitor/sys_info.h"

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::monitor;

// ═══════════════════════════════════════════════════════════
// CpuTimes
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, CpuTimesDefaultsZero) {
    CpuTimes t{};
    EXPECT_EQ(t.total(), 0u);
    EXPECT_EQ(t.active(), 0u);
}

TEST(SysInfo, CpuTimesTotalAndActive) {
    CpuTimes t{};
    t.user    = 100;
    t.nice    = 10;
    t.system  = 50;
    t.idle    = 200;
    t.iowait  = 20;
    t.irq     = 5;
    t.softirq = 3;
    t.steal   = 2;

    EXPECT_EQ(t.total(), 390u);
    EXPECT_EQ(t.active(), 170u);  // total - idle - iowait = 390 - 200 - 20
}

TEST(SysInfo, ReadCpuTimesReturnsNonZero) {
    // This test reads from /proc/stat which should exist on Linux
    auto t = read_cpu_times();
    EXPECT_GT(t.total(), 0u);
}

TEST(SysInfo, ComputeCpuUsageZeroDelta) {
    CpuTimes prev{}, now{};
    float    usage = compute_cpu_usage(prev, now);
    EXPECT_FLOAT_EQ(usage, 0.0f);
}

TEST(SysInfo, ComputeCpuUsageKnownValues) {
    CpuTimes prev{};
    prev.user   = 100;
    prev.system = 50;
    prev.idle   = 200;
    prev.iowait = 50;

    CpuTimes now{};
    now.user   = 200;
    now.system = 100;
    now.idle   = 250;
    now.iowait = 50;

    float usage = compute_cpu_usage(prev, now);
    // delta_total = (200+100+250+50) - (100+50+200+50) = 600 - 400 = 200
    // delta_active = delta_total - delta_idle - delta_iowait = 200 - 50 - 0 = 150
    // usage = 150/200 = 75%
    EXPECT_NEAR(usage, 75.0f, 0.1f);
}

// ═══════════════════════════════════════════════════════════
// MemInfo
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, ReadMeminfoReturnsNonZero) {
    auto m = read_meminfo();
    EXPECT_GT(m.total_kb, 0u);
    EXPECT_GT(m.available_kb, 0u);
    EXPECT_GE(m.usage_percent, 0.0f);
    EXPECT_LE(m.usage_percent, 100.0f);
}

// ═══════════════════════════════════════════════════════════
// Temperature
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, ReadCpuTempReasonable) {
    float temp = read_cpu_temp();
    // Should be in [-20, 120] range or simulated 42
    EXPECT_GE(temp, -20.0f);
    EXPECT_LE(temp, 120.0f);
}

// ═══════════════════════════════════════════════════════════
// Disk
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, ReadDiskUsage) {
    auto d = read_disk_usage();
    // On any real system, root should have non-zero total
    EXPECT_GT(d.total_mb, 0u);
    EXPECT_GE(d.usage_percent, 0.0f);
    EXPECT_LE(d.usage_percent, 100.0f);
}

// ═══════════════════════════════════════════════════════════
// Process watchdog
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, CurrentProcessIsAlive) {
    EXPECT_TRUE(is_process_alive(getpid()));
}

TEST(SysInfo, InvalidPidNotAlive) {
    EXPECT_FALSE(is_process_alive(0));
    EXPECT_FALSE(is_process_alive(-1));
}

TEST(SysInfo, NonexistentPidNotAlive) {
    // PID 99999999 almost certainly doesn't exist
    EXPECT_FALSE(is_process_alive(99999999));
}
