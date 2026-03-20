// tests/test_process_interfaces.cpp
// Tests for internal process strategy interfaces:
//   IVisualFrontend, IProcessMonitor
//
// Path planner tests moved to test_dstar_lite_planner.cpp (Issue #207).
// Obstacle avoider tests moved to test_obstacle_avoider_3d.cpp (Issue #207).
#include "monitor/iprocess_monitor.h"
#include "slam/ivisual_frontend.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::ipc;
using namespace drone::slam;
using namespace drone::monitor;

// ═══════════════════════════════════════════════════════════
// IVisualFrontend / SimulatedVisualFrontend
// ═══════════════════════════════════════════════════════════

TEST(VisualFrontendTest, FactoryCreatesSimulated) {
    auto fe = create_visual_frontend("simulated");
    EXPECT_NE(fe, nullptr);
}

TEST(VisualFrontendTest, FactoryThrowsOnUnknown) {
    EXPECT_THROW(create_visual_frontend("nonexistent"), std::runtime_error);
}

TEST(VisualFrontendTest, ProcessFrameReturnsPose) {
    auto fe = create_visual_frontend("simulated");

    StereoFrame frame{};
    frame.timestamp_ns = 1000;

    // Run 10 frames — should produce a pose with some nonzero position
    drone::slam::Pose pose;
    for (int i = 0; i < 10; ++i) {
        frame.timestamp_ns += 100000000;  // 100ms increments
        pose = fe->process_frame(frame);
    }

    EXPECT_GT(pose.timestamp, 0.0);
    EXPECT_EQ(pose.quality, 2u);  // simulated always returns quality=2
    // At least one translation component should be nonzero (circular path)
    double dist = pose.position.norm();
    EXPECT_GT(dist, 0.0);
}

TEST(VisualFrontendTest, NameReturnsString) {
    auto fe = create_visual_frontend("simulated");
    EXPECT_FALSE(fe->name().empty());
}

TEST(VisualFrontendTest, ImplementsInterface) {
    auto             fe    = create_visual_frontend("simulated");
    IVisualFrontend* iface = fe.get();
    EXPECT_NE(iface, nullptr);

    StereoFrame frame{};
    frame.timestamp_ns = 500;
    auto pose          = iface->process_frame(frame);
    EXPECT_GT(pose.timestamp, 0.0);
}

// ═══════════════════════════════════════════════════════════
// IProcessMonitor / LinuxProcessMonitor
// ═══════════════════════════════════════════════════════════

TEST(ProcessMonitorTest, FactoryCreatesLinux) {
    auto pm = create_process_monitor("linux");
    EXPECT_NE(pm, nullptr);
}

TEST(ProcessMonitorTest, FactoryThrowsOnUnknown) {
    EXPECT_THROW(create_process_monitor("windows_nt"), std::runtime_error);
}

TEST(ProcessMonitorTest, CollectReturnsValidHealth) {
    auto pm = create_process_monitor("linux");

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
    auto pm     = create_process_monitor("linux");
    auto health = pm->collect();

    // thermal_zone should be one of 0=normal, 1=warm, 2=hot, 3=critical
    EXPECT_LE(health.thermal_zone, 3u);
}

TEST(ProcessMonitorTest, ImplementsInterface) {
    auto             pm    = create_process_monitor("linux");
    IProcessMonitor* iface = pm.get();
    EXPECT_NE(iface, nullptr);

    auto health = iface->collect();
    EXPECT_GE(health.cpu_usage_percent, 0.0f);
}

TEST(ProcessMonitorTest, MultipleSamples) {
    auto pm = create_process_monitor("linux");
    for (int i = 0; i < 3; ++i) {
        auto health = pm->collect();
        EXPECT_GE(health.cpu_usage_percent, 0.0f);
        EXPECT_GE(health.memory_usage_percent, 0.0f);
    }
}
