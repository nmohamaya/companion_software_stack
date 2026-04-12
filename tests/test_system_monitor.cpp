// tests/test_system_monitor.cpp — Tests for sys_info.h functions and ISysInfo abstraction
#include "monitor/iprocess_monitor.h"
#include "monitor/sys_info.h"
#include "util/isys_info.h"
#include "util/linux_sys_info.h"
#include "util/mock_sys_info.h"
#include "util/sys_info_factory.h"

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::monitor;

// ═══════════════════════════════════════════════════════════
// CpuTimes (backward-compatible free function tests)
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, CpuTimesDefaultsZero) {
    drone::util::CpuTimes t{};
    EXPECT_EQ(t.total(), 0u);
    EXPECT_EQ(t.active(), 0u);
}

TEST(SysInfo, CpuTimesTotalAndActive) {
    drone::util::CpuTimes t{};
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
    drone::util::CpuTimes prev{}, now{};
    float                 usage = compute_cpu_usage(prev, now);
    EXPECT_FLOAT_EQ(usage, 0.0f);
}

TEST(SysInfo, ComputeCpuUsageKnownValues) {
    drone::util::CpuTimes prev{};
    prev.user   = 100;
    prev.system = 50;
    prev.idle   = 200;
    prev.iowait = 50;

    drone::util::CpuTimes now{};
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
// MemInfo (backward-compatible free function tests)
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, ReadMeminfoReturnsNonZero) {
    auto m = read_meminfo();
    EXPECT_GT(m.total_kb, 0u);
    EXPECT_GT(m.available_kb, 0u);
    EXPECT_GE(m.usage_percent, 0.0f);
    EXPECT_LE(m.usage_percent, 100.0f);
}

// ═══════════════════════════════════════════════════════════
// Temperature (backward-compatible free function tests)
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, ReadCpuTempReasonable) {
    float temp = read_cpu_temp();
    // Should be in [-20, 120] range or simulated 42
    EXPECT_GE(temp, -20.0f);
    EXPECT_LE(temp, 120.0f);
}

// ═══════════════════════════════════════════════════════════
// Disk (backward-compatible free function tests)
// ═══════════════════════════════════════════════════════════

TEST(SysInfo, ReadDiskUsage) {
    auto d = read_disk_usage();
    // On any real system, root should have non-zero total
    EXPECT_GT(d.total_mb, 0u);
    EXPECT_GE(d.usage_percent, 0.0f);
    EXPECT_LE(d.usage_percent, 100.0f);
}

// ═══════════════════════════════════════════════════════════
// Process watchdog (backward-compatible free function tests)
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

// ═══════════════════════════════════════════════════════════
// ISysInfo interface — LinuxSysInfo
// ═══════════════════════════════════════════════════════════

TEST(ISysInfo, LinuxSysInfoName) {
    drone::util::LinuxSysInfo sys;
    EXPECT_EQ(sys.name(), "LinuxSysInfo");
}

TEST(ISysInfo, LinuxSysInfoReadCpuTimesNonZero) {
    drone::util::LinuxSysInfo sys;
    auto                      t = sys.read_cpu_times();
    EXPECT_GT(t.total(), 0u);
}

TEST(ISysInfo, LinuxSysInfoReadMeminfoNonZero) {
    drone::util::LinuxSysInfo sys;
    auto                      m = sys.read_meminfo();
    EXPECT_GT(m.total_kb, 0u);
}

TEST(ISysInfo, LinuxSysInfoReadCpuTempReasonable) {
    drone::util::LinuxSysInfo sys;
    float                     temp = sys.read_cpu_temp();
    EXPECT_GE(temp, -20.0f);
    EXPECT_LE(temp, 120.0f);
}

TEST(ISysInfo, LinuxSysInfoIsProcessAlive) {
    drone::util::LinuxSysInfo sys;
    EXPECT_TRUE(sys.is_process_alive(getpid()));
    EXPECT_FALSE(sys.is_process_alive(0));
}

// ═══════════════════════════════════════════════════════════
// ISysInfo interface — MockSysInfo
// ═══════════════════════════════════════════════════════════

TEST(ISysInfo, MockSysInfoName) {
    drone::util::MockSysInfo mock;
    EXPECT_EQ(mock.name(), "MockSysInfo");
}

TEST(ISysInfo, MockSysInfoReturnsInjectedValues) {
    drone::util::MockSysInfo mock;

    // Inject CPU times
    mock.injected_cpu_times.user   = 500;
    mock.injected_cpu_times.system = 200;
    mock.injected_cpu_times.idle   = 300;
    auto cpu                       = mock.read_cpu_times();
    EXPECT_EQ(cpu.user, 500u);
    EXPECT_EQ(cpu.system, 200u);
    EXPECT_EQ(cpu.idle, 300u);

    // Inject memory info
    mock.injected_meminfo.total_kb      = 16'000'000;
    mock.injected_meminfo.available_kb  = 8'000'000;
    mock.injected_meminfo.usage_percent = 50.0f;
    auto mem                            = mock.read_meminfo();
    EXPECT_EQ(mem.total_kb, 16'000'000u);
    EXPECT_FLOAT_EQ(mem.usage_percent, 50.0f);

    // Inject temperature
    mock.injected_cpu_temp = 65.5f;
    EXPECT_FLOAT_EQ(mock.read_cpu_temp(), 65.5f);

    // Inject disk info
    mock.injected_disk_info.total_mb      = 500'000;
    mock.injected_disk_info.free_mb       = 250'000;
    mock.injected_disk_info.usage_percent = 50.0f;
    auto disk                             = mock.read_disk_usage();
    EXPECT_EQ(disk.total_mb, 500'000u);
    EXPECT_FLOAT_EQ(disk.usage_percent, 50.0f);

    // Inject process alive
    mock.injected_process_alive = false;
    EXPECT_FALSE(mock.is_process_alive(1234));
}

TEST(ISysInfo, ComputeCpuUsageFreeFunction) {
    drone::util::CpuTimes prev{};
    prev.user = 100;
    prev.idle = 100;

    drone::util::CpuTimes now{};
    now.user = 200;
    now.idle = 100;

    // prev.total()=200, now.total()=300, delta_total=100
    // All delta is active (idle unchanged), so usage=100%
    float usage = drone::util::compute_cpu_usage(prev, now);
    EXPECT_NEAR(usage, 100.0f, 0.1f);
}

TEST(ISysInfo, ComputeCpuUsageUnderflowGuard) {
    // Simulate counter wrap: prev > now
    drone::util::CpuTimes prev{};
    prev.user = 500;
    prev.idle = 500;

    drone::util::CpuTimes now{};
    now.user = 100;
    now.idle = 100;

    // prev.total() > now.total() — must clamp to 0, not underflow
    float usage = drone::util::compute_cpu_usage(prev, now);
    EXPECT_FLOAT_EQ(usage, 0.0f);
}

TEST(ISysInfo, ComputeCpuUsageIdenticalSamples) {
    drone::util::CpuTimes t{};
    t.user = 100;
    t.idle = 200;

    float usage = drone::util::compute_cpu_usage(t, t);
    EXPECT_FLOAT_EQ(usage, 0.0f);
}

// ═══════════════════════════════════════════════════════════
// ISysInfo factory
// ═══════════════════════════════════════════════════════════

TEST(ISysInfo, FactoryCreatesLinux) {
    auto sys = drone::util::create_sys_info("linux");
    ASSERT_NE(sys, nullptr);
    EXPECT_EQ(sys->name(), "LinuxSysInfo");
}

TEST(ISysInfo, FactoryCreatesJetson) {
    auto sys = drone::util::create_sys_info("jetson");
    ASSERT_NE(sys, nullptr);
    EXPECT_EQ(sys->name(), "JetsonSysInfo");
}

TEST(ISysInfo, FactoryCreatesMock) {
    auto sys = drone::util::create_sys_info("mock");
    ASSERT_NE(sys, nullptr);
    EXPECT_EQ(sys->name(), "MockSysInfo");
}

TEST(ISysInfo, FactoryFallbackToLinux) {
    auto sys = drone::util::create_sys_info("unknown_platform");
    ASSERT_NE(sys, nullptr);
    EXPECT_EQ(sys->name(), "LinuxSysInfo");
}

// ═══════════════════════════════════════════════════════════
// LinuxProcessMonitor with MockSysInfo — deterministic health
// ═══════════════════════════════════════════════════════════

TEST(ProcessMonitor, MockSysInfoDeterministicHealth) {
    drone::util::MockSysInfo mock;

    // Set up initial CPU times (constructor reads once for prev_cpu_)
    mock.injected_cpu_times.user   = 100;
    mock.injected_cpu_times.system = 50;
    mock.injected_cpu_times.idle   = 200;

    drone::monitor::LinuxProcessMonitor monitor(mock);

    // Now inject "after" CPU times for the first collect()
    mock.injected_cpu_times.user   = 200;
    mock.injected_cpu_times.system = 100;
    mock.injected_cpu_times.idle   = 250;

    // Inject memory: 50% usage
    mock.injected_meminfo.total_kb      = 16'000'000;
    mock.injected_meminfo.available_kb  = 8'000'000;
    mock.injected_meminfo.usage_percent = 50.0f;

    // Inject temperature: normal
    mock.injected_cpu_temp = 55.0f;

    // Inject disk: 30% usage
    mock.injected_disk_info.total_mb      = 500'000;
    mock.injected_disk_info.free_mb       = 350'000;
    mock.injected_disk_info.usage_percent = 30.0f;

    auto health = monitor.collect();

    // CPU usage: delta_total = (200+100+250) - (100+50+200) = 200
    //            delta_active = 200 - 50 - 0 = 150 => 75%
    EXPECT_NEAR(health.cpu_usage_percent, 75.0f, 0.5f);
    EXPECT_FLOAT_EQ(health.memory_usage_percent, 50.0f);
    EXPECT_FLOAT_EQ(health.cpu_temp_c, 55.0f);
    EXPECT_FLOAT_EQ(health.disk_usage_percent, 30.0f);
    EXPECT_EQ(health.thermal_zone, 0u);  // all normal
}

TEST(ProcessMonitor, MockSysInfoWarningThreshold) {
    drone::util::MockSysInfo mock;

    // CPU times: 0 initially
    mock.injected_cpu_times = {};

    // cpu_warn=90, mem_warn=90, temp_warn=80, temp_crit=95
    drone::monitor::MonitorThresholds th;
    th.cpu_warn  = 90.0f;
    th.mem_warn  = 90.0f;
    th.temp_warn = 80.0f;
    th.temp_crit = 95.0f;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    // Now inject high temperature (above warn but below crit)
    mock.injected_cpu_temp                = 85.0f;
    mock.injected_meminfo.usage_percent   = 50.0f;
    mock.injected_disk_info.usage_percent = 10.0f;

    auto health = monitor.collect();
    EXPECT_EQ(health.thermal_zone, 2u);  // WARNING due to temp > 80
}

TEST(ProcessMonitor, MockSysInfoCriticalThreshold) {
    drone::util::MockSysInfo mock;
    mock.injected_cpu_times = {};

    drone::monitor::MonitorThresholds th;
    th.cpu_warn  = 90.0f;
    th.mem_warn  = 90.0f;
    th.temp_warn = 80.0f;
    th.temp_crit = 95.0f;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    // Inject critical temperature
    mock.injected_cpu_temp                = 100.0f;
    mock.injected_meminfo.usage_percent   = 50.0f;
    mock.injected_disk_info.usage_percent = 10.0f;

    auto health = monitor.collect();
    EXPECT_EQ(health.thermal_zone, 3u);  // CRITICAL due to temp > 95
}

TEST(ProcessMonitor, MockSysInfoBatteryWarning) {
    drone::util::MockSysInfo mock;
    mock.injected_cpu_times = {};

    // batt_warn=20, batt_crit=10
    drone::monitor::MonitorThresholds th;
    th.batt_warn = 20.0f;
    th.batt_crit = 10.0f;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    mock.injected_cpu_temp                = 40.0f;
    mock.injected_meminfo.usage_percent   = 30.0f;
    mock.injected_disk_info.usage_percent = 10.0f;

    // Set low battery (below warn but above crit)
    monitor.set_battery_percent(15.0f);

    auto health = monitor.collect();
    EXPECT_EQ(health.thermal_zone, 2u);  // WARNING due to battery < 20
}

TEST(ProcessMonitor, MockSysInfoBatteryCritical) {
    drone::util::MockSysInfo mock;
    mock.injected_cpu_times = {};

    drone::monitor::MonitorThresholds th;
    th.batt_warn = 20.0f;
    th.batt_crit = 10.0f;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    mock.injected_cpu_temp                = 40.0f;
    mock.injected_meminfo.usage_percent   = 30.0f;
    mock.injected_disk_info.usage_percent = 10.0f;

    // Set critically low battery (below batt_crit)
    monitor.set_battery_percent(5.0f);

    auto health = monitor.collect();
    EXPECT_EQ(health.thermal_zone, 3u);  // CRITICAL due to battery < 10
}

TEST(ProcessMonitor, MockSysInfoCpuWarningThreshold) {
    drone::util::MockSysInfo mock;

    // Initial CPU: all zeros
    mock.injected_cpu_times = {};
    // cpu_warn=90
    drone::monitor::MonitorThresholds th;
    th.cpu_warn = 90.0f;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    // Inject CPU that will produce >90% usage (95% active out of total)
    mock.injected_cpu_times.user          = 950;
    mock.injected_cpu_times.idle          = 50;
    mock.injected_cpu_temp                = 40.0f;  // normal temp
    mock.injected_meminfo.usage_percent   = 30.0f;  // normal mem
    mock.injected_disk_info.usage_percent = 10.0f;

    auto health = monitor.collect();
    EXPECT_EQ(health.thermal_zone, 2u);  // WARNING due to CPU > 90%
}

TEST(ProcessMonitor, MockSysInfoMemWarningThreshold) {
    drone::util::MockSysInfo mock;
    mock.injected_cpu_times = {};
    // mem_warn=90
    drone::monitor::MonitorThresholds th;
    th.mem_warn = 90.0f;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    mock.injected_meminfo.usage_percent   = 95.0f;  // above warn
    mock.injected_cpu_temp                = 40.0f;
    mock.injected_disk_info.usage_percent = 10.0f;

    auto health = monitor.collect();
    EXPECT_EQ(health.thermal_zone, 2u);  // WARNING due to mem > 90%
}

TEST(ProcessMonitor, MockSysInfoDiskCriticalThreshold) {
    drone::util::MockSysInfo mock;
    mock.injected_cpu_times = {};
    // disk_crit=98, disk_interval=1 to ensure disk is always read
    drone::monitor::MonitorThresholds th;
    th.disk_crit     = 98.0f;
    th.disk_interval = 1;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    mock.injected_cpu_temp                = 40.0f;
    mock.injected_meminfo.usage_percent   = 30.0f;
    mock.injected_disk_info.usage_percent = 99.0f;  // above crit

    auto health = monitor.collect();
    EXPECT_EQ(health.thermal_zone, 3u);  // CRITICAL due to disk > 98%
}

TEST(ProcessMonitor, MockSysInfoDiskUsageAsserted) {
    drone::util::MockSysInfo mock;
    mock.injected_cpu_times = {};

    // Explicit disk_interval=1 to guarantee disk is read every tick
    drone::monitor::MonitorThresholds th;
    th.disk_interval = 1;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    mock.injected_cpu_temp                = 40.0f;
    mock.injected_meminfo.usage_percent   = 30.0f;
    mock.injected_disk_info.total_mb      = 500'000;
    mock.injected_disk_info.free_mb       = 350'000;
    mock.injected_disk_info.usage_percent = 30.0f;

    auto health = monitor.collect();
    EXPECT_FLOAT_EQ(health.disk_usage_percent, 30.0f);
}

TEST(ProcessMonitor, OverlappingThresholdsPickHighestSeverity) {
    drone::util::MockSysInfo mock;
    mock.injected_cpu_times = {};

    // CPU warning AND temperature critical — result should be zone=3
    drone::monitor::MonitorThresholds th;
    th.cpu_warn  = 90.0f;
    th.temp_warn = 80.0f;
    th.temp_crit = 95.0f;
    drone::monitor::LinuxProcessMonitor monitor(mock, th);

    // Inject CPU >90% usage and temp >95 crit
    mock.injected_cpu_times.user          = 950;
    mock.injected_cpu_times.idle          = 50;
    mock.injected_cpu_temp                = 100.0f;  // above crit
    mock.injected_meminfo.usage_percent   = 30.0f;
    mock.injected_disk_info.usage_percent = 10.0f;

    auto health = monitor.collect();
    // Both CPU warning (zone=2) and temp critical (zone=3) triggered;
    // sequential logic ensures the higher severity wins.
    EXPECT_EQ(health.thermal_zone, 3u);
}
