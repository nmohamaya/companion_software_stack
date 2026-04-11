// tests/test_process_interfaces.cpp
// Tests for internal process strategy interfaces:
//   IProcessMonitor
//
// IVisualFrontend removed in Issue #255 (superseded by IVIOBackend).
// Path planner tests moved to test_dstar_lite_planner.cpp (Issue #207).
// Obstacle avoider tests moved to test_obstacle_avoider_3d.cpp (Issue #207).
#include "monitor/iprocess_monitor.h"
#include "util/linux_sys_info.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::monitor;

// ═══════════════════════════════════════════════════════════
// IProcessMonitor / LinuxProcessMonitor
// ═══════════════════════════════════════════════════════════

TEST(ProcessMonitorTest, FactoryCreatesLinux) {
    drone::util::LinuxSysInfo sys;
    auto                      pm = create_process_monitor(sys);
    EXPECT_NE(pm, nullptr);
}

TEST(ProcessMonitorTest, FactoryCreatesWithDefaultThresholds) {
    drone::util::LinuxSysInfo sys;
    auto                      pm = create_process_monitor(sys);
    EXPECT_NE(pm, nullptr);
    EXPECT_EQ(pm->name(), "LinuxProcessMonitor");
}

TEST(ProcessMonitorTest, FactoryCreatesWithCustomThresholds) {
    drone::util::LinuxSysInfo         sys;
    drone::monitor::MonitorThresholds th;
    th.cpu_warn = 80.0f;
    auto pm     = create_process_monitor(sys, th);
    EXPECT_NE(pm, nullptr);
    EXPECT_EQ(pm->name(), "LinuxProcessMonitor");
}

TEST(ProcessMonitorTest, CollectReturnsValidHealth) {
    drone::util::LinuxSysInfo sys;
    auto                      pm = create_process_monitor(sys);

    auto health = pm->collect();

    // On any Linux machine, some reasonable values
    EXPECT_GE(health.cpu_usage_percent, 0.0f);
    EXPECT_LE(health.cpu_usage_percent, 100.0f);

    EXPECT_GE(health.memory_usage_percent, 0.0f);
    EXPECT_LE(health.memory_usage_percent, 100.0f);

    // Temperature can be 0 if /sys/class/thermal doesn't exist (containers)
    EXPECT_GE(health.cpu_temp_c, 0.0f);

    EXPECT_GE(health.disk_usage_percent, 0.0f);
    EXPECT_LE(health.disk_usage_percent, 100.0f);
}

TEST(ProcessMonitorTest, ThermalZonePopulated) {
    drone::util::LinuxSysInfo sys;
    auto                      pm     = create_process_monitor(sys);
    auto                      health = pm->collect();

    // thermal_zone: 0=normal, 2=warning, 3=critical (1 is reserved/unused)
    EXPECT_LE(health.thermal_zone, 3u);
}

TEST(ProcessMonitorTest, ImplementsInterface) {
    drone::util::LinuxSysInfo sys;
    auto                      pm    = create_process_monitor(sys);
    IProcessMonitor*          iface = pm.get();
    EXPECT_NE(iface, nullptr);

    auto health = iface->collect();
    EXPECT_GE(health.cpu_usage_percent, 0.0f);
}

TEST(ProcessMonitorTest, MultipleSamples) {
    drone::util::LinuxSysInfo sys;
    auto                      pm = create_process_monitor(sys);
    for (int i = 0; i < 3; ++i) {
        auto health = pm->collect();
        EXPECT_GE(health.cpu_usage_percent, 0.0f);
        EXPECT_GE(health.memory_usage_percent, 0.0f);
    }
}
