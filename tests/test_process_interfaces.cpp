// tests/test_process_interfaces.cpp
// Tests for internal process strategy interfaces:
//   IVisualFrontend, IPathPlanner, IObstacleAvoider, IProcessMonitor
#include <gtest/gtest.h>

#include "slam/ivisual_frontend.h"
#include "planner/ipath_planner.h"
#include "planner/iobstacle_avoider.h"
#include "monitor/iprocess_monitor.h"

#include <cmath>
#include <memory>
#include <vector>

using namespace drone::ipc;
using namespace drone::slam;
using namespace drone::planner;
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

    ShmStereoFrame frame{};
    frame.timestamp_ns = 1000;

    // Run 10 frames — should produce a pose with some nonzero position
    Pose pose;
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
    auto fe = create_visual_frontend("simulated");
    IVisualFrontend* iface = fe.get();
    EXPECT_NE(iface, nullptr);

    ShmStereoFrame frame{};
    frame.timestamp_ns = 500;
    auto pose = iface->process_frame(frame);
    EXPECT_GT(pose.timestamp, 0.0);
}

// ═══════════════════════════════════════════════════════════
// IPathPlanner / PotentialFieldPlanner
// ═══════════════════════════════════════════════════════════

TEST(PathPlannerTest, FactoryCreatesDefault) {
    auto pp = create_path_planner("potential_field");
    EXPECT_NE(pp, nullptr);
}

TEST(PathPlannerTest, FactoryThrowsOnUnknown) {
    EXPECT_THROW(create_path_planner("astar_3d_doesnt_exist"), std::runtime_error);
}

TEST(PathPlannerTest, PlanReturnsWaypoints) {
    auto pp = create_path_planner("potential_field");

    ShmPose pose{};
    pose.translation[0] = 0; pose.translation[1] = 0; pose.translation[2] = 0;

    Waypoint target{10.0f, 0.0f, 0.0f, 0.0f, 2.0f, 5.0f, false};

    auto cmd = pp->plan(pose, target);
    EXPECT_TRUE(cmd.valid);
    EXPECT_GT(cmd.timestamp_ns, 0u);
    // Velocity should point toward the target (positive X)
    EXPECT_GT(cmd.velocity_x, 0.0f);
    EXPECT_FLOAT_EQ(cmd.target_x, 10.0f);
}

TEST(PathPlannerTest, WithObstacles) {
    // Path planner itself doesn't handle obstacles — that's the avoider.
    // But plan should still work even when drone is close to target.
    auto pp = create_path_planner("potential_field");

    ShmPose pose{};
    pose.translation[0] = 9.5;

    Waypoint target{10.0f, 0.0f, 0.0f, 0.0f, 2.0f, 5.0f, false};
    auto cmd = pp->plan(pose, target);
    EXPECT_TRUE(cmd.valid);
    // Close to target — speed ramps down but stays above min_speed (1 m/s)
    EXPECT_GT(std::abs(cmd.velocity_x), 0.5f);
    EXPECT_LT(std::abs(cmd.velocity_x), 5.0f);
}

TEST(PathPlannerTest, AtGoalZeroVelocity) {
    auto pp = create_path_planner("potential_field");

    ShmPose pose{};
    pose.translation[0] = 5; pose.translation[1] = 5; pose.translation[2] = 5;

    Waypoint target{5.0f, 5.0f, 5.0f, 0.0f, 2.0f, 5.0f, false};
    auto cmd = pp->plan(pose, target);
    // At target — velocity should be ~0
    EXPECT_NEAR(cmd.velocity_x, 0.0f, 0.1f);
    EXPECT_NEAR(cmd.velocity_y, 0.0f, 0.1f);
    EXPECT_NEAR(cmd.velocity_z, 0.0f, 0.1f);
}

// ═══════════════════════════════════════════════════════════
// IObstacleAvoider / PotentialFieldAvoider
// ═══════════════════════════════════════════════════════════

TEST(ObstacleAvoiderTest, FactoryCreatesDefault) {
    auto oa = create_obstacle_avoider("potential_field");
    EXPECT_NE(oa, nullptr);
}

TEST(ObstacleAvoiderTest, FactoryThrowsOnUnknown) {
    EXPECT_THROW(create_obstacle_avoider("does_not_exist"), std::runtime_error);
}

TEST(ObstacleAvoiderTest, NoObstacleNoChange) {
    auto oa = create_obstacle_avoider("potential_field");

    ShmTrajectoryCmd planned{};
    planned.valid = true;
    planned.velocity_x = 2.0f;
    planned.velocity_y = 0.0f;
    planned.velocity_z = 0.0f;

    ShmPose pose{};
    ShmDetectedObjectList objects{};
    objects.num_objects = 0;

    auto result = oa->avoid(planned, pose, objects);
    EXPECT_FLOAT_EQ(result.velocity_x, 2.0f);
    EXPECT_FLOAT_EQ(result.velocity_y, 0.0f);
    EXPECT_FLOAT_EQ(result.velocity_z, 0.0f);
}

TEST(ObstacleAvoiderTest, ObstacleDeflectsTrajectory) {
    auto oa = create_obstacle_avoider("potential_field", 5.0f, 1.0f);

    ShmTrajectoryCmd planned{};
    planned.valid = true;
    planned.velocity_x = 2.0f;
    planned.velocity_y = 0.0f;

    ShmPose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;

    // Place obstacle 2m ahead
    ShmDetectedObjectList objects{};
    objects.num_objects = 1;
    objects.objects[0].position_x = 2.0f;
    objects.objects[0].position_y = 0.0f;

    auto result = oa->avoid(planned, pose, objects);
    // Repulsive force should reduce forward velocity
    EXPECT_LT(result.velocity_x, planned.velocity_x);
}

TEST(ObstacleAvoiderTest, FarObstacleNoEffect) {
    auto oa = create_obstacle_avoider("potential_field", 2.0f, 1.0f);

    ShmTrajectoryCmd planned{};
    planned.valid = true;
    planned.velocity_x = 1.0f;

    ShmPose pose{};

    // Obstacle very far away
    ShmDetectedObjectList objects{};
    objects.num_objects = 1;
    objects.objects[0].position_x = 100.0f;
    objects.objects[0].position_y = 100.0f;

    auto result = oa->avoid(planned, pose, objects);
    EXPECT_FLOAT_EQ(result.velocity_x, planned.velocity_x);
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
    auto pm = create_process_monitor("linux");
    auto health = pm->collect();

    // thermal_zone should be one of 0=normal, 1=warm, 2=hot, 3=critical
    EXPECT_LE(health.thermal_zone, 3u);
}

TEST(ProcessMonitorTest, ImplementsInterface) {
    auto pm = create_process_monitor("linux");
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
