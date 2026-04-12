// tests/integration/test_ipc_roundtrip.cpp
// Integration tests: IPC message round-trip through shared bus.
//
// Verifies that multiple "processes" can communicate via the shared
// IntegrationTestHarness MessageBus — the foundational capability for
// in-process integration testing.
//
// See: Issue #292 (Epic #284 — Platform Modularity)
#include "integration/integration_harness.h"
#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "ipc/isubscriber.h"

#include <chrono>
#include <cstring>
#include <thread>

#include <gtest/gtest.h>

using drone::test::IntegrationTestHarness;
namespace ipc = drone::ipc;

// ═══════════════════════════════════════════════════════════
// Fixture: shared harness with Zenoh resource lock
// ═══════════════════════════════════════════════════════════
class IpcRoundTripTest : public ::testing::Test {
protected:
    IntegrationTestHarness harness_;

    /// Helper: wait for Zenoh delivery (in-process still needs a small delay
    /// for the async subscriber callback to fire).
    static void wait_delivery() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
};

// ─── Basic: single message, same bus ──────────────────────

TEST_F(IpcRoundTripTest, DetectedObjectsRoundTrip) {
    auto topic = IntegrationTestHarness::unique_topic("/detected_objects");
    auto pub   = harness_.bus().advertise<ipc::DetectedObjectList>(topic);
    auto sub   = harness_.bus().subscribe<ipc::DetectedObjectList>(topic);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);

    // Simulate P2 publishing detections
    ipc::DetectedObjectList sent{};
    sent.timestamp_ns                  = harness_.clock().now_ns();
    sent.frame_sequence                = 42;
    sent.num_objects                   = 1;
    sent.objects[0].track_id           = 1;
    sent.objects[0].class_id           = ipc::ObjectClass::PERSON;
    sent.objects[0].confidence         = 0.95f;
    sent.objects[0].position_x         = 5.0f;
    sent.objects[0].position_y         = 0.0f;
    sent.objects[0].position_z         = -1.5f;
    sent.objects[0].velocity_x         = 0.0f;
    sent.objects[0].velocity_y         = 0.0f;
    sent.objects[0].velocity_z         = 0.0f;
    sent.objects[0].heading            = 0.0f;
    sent.objects[0].bbox_x             = 100.0f;
    sent.objects[0].bbox_y             = 200.0f;
    sent.objects[0].bbox_w             = 50.0f;
    sent.objects[0].bbox_h             = 80.0f;
    sent.objects[0].has_camera         = true;
    sent.objects[0].has_radar          = false;
    sent.objects[0].estimated_radius_m = 0.3f;
    sent.objects[0].estimated_height_m = 1.8f;
    sent.objects[0].radar_update_count = 0;
    sent.objects[0].depth_confidence   = 0.5f;
    ASSERT_TRUE(sent.validate());

    pub->publish(sent);
    wait_delivery();

    // Simulate P4 receiving detections
    ipc::DetectedObjectList received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.frame_sequence, 42u);
    EXPECT_EQ(received.num_objects, 1u);
    EXPECT_EQ(received.objects[0].track_id, 1u);
    EXPECT_EQ(received.objects[0].class_id, ipc::ObjectClass::PERSON);
    EXPECT_FLOAT_EQ(received.objects[0].confidence, 0.95f);
    EXPECT_FLOAT_EQ(received.objects[0].position_x, 5.0f);
    EXPECT_TRUE(received.validate());
}

TEST_F(IpcRoundTripTest, FCStateRoundTrip) {
    auto topic = IntegrationTestHarness::unique_topic("/fc_state");
    auto pub   = harness_.bus().advertise<ipc::FCState>(topic);
    auto sub   = harness_.bus().subscribe<ipc::FCState>(topic);

    ipc::FCState sent{};
    sent.timestamp_ns       = harness_.clock().now_ns();
    sent.gps_lat            = 47.3977f;
    sent.gps_lon            = 8.5456f;
    sent.gps_alt            = 420.0f;
    sent.rel_alt            = 15.0f;
    sent.roll               = 0.0f;
    sent.pitch              = 0.0f;
    sent.yaw                = 1.57f;
    sent.vx                 = 2.0f;
    sent.vy                 = 0.0f;
    sent.vz                 = 0.0f;
    sent.battery_voltage    = 22.4f;
    sent.battery_remaining  = 75.0f;
    sent.flight_mode        = 4;
    sent.armed              = true;
    sent.connected          = true;
    sent.gps_fix_type       = 3;
    sent.satellites_visible = 12;
    ASSERT_TRUE(sent.validate());

    pub->publish(sent);
    wait_delivery();

    ipc::FCState received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_TRUE(received.connected);
    EXPECT_TRUE(received.armed);
    EXPECT_FLOAT_EQ(received.battery_remaining, 75.0f);
    EXPECT_FLOAT_EQ(received.gps_lat, 47.3977f);
}

TEST_F(IpcRoundTripTest, TrajectoryCommandRoundTrip) {
    auto topic = IntegrationTestHarness::unique_topic("/trajectory_cmd");
    auto pub   = harness_.bus().advertise<ipc::TrajectoryCmd>(topic);
    auto sub   = harness_.bus().subscribe<ipc::TrajectoryCmd>(topic);

    ipc::TrajectoryCmd sent{};
    sent.timestamp_ns     = harness_.clock().now_ns();
    sent.correlation_id   = 99;
    sent.target_x         = 10.0f;
    sent.target_y         = 20.0f;
    sent.target_z         = -5.0f;
    sent.target_yaw       = 0.0f;
    sent.velocity_x       = 1.0f;
    sent.velocity_y       = 0.5f;
    sent.velocity_z       = 0.0f;
    sent.yaw_rate         = 0.0f;
    sent.coordinate_frame = 1;
    sent.valid            = true;
    ASSERT_TRUE(sent.validate());

    pub->publish(sent);
    wait_delivery();

    ipc::TrajectoryCmd received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_TRUE(received.valid);
    EXPECT_EQ(received.correlation_id, 99u);
    EXPECT_FLOAT_EQ(received.target_x, 10.0f);
    EXPECT_FLOAT_EQ(received.target_y, 20.0f);
    EXPECT_FLOAT_EQ(received.target_z, -5.0f);
}

// ─── Multi-hop: P2 → P4 → P5 message flow ───────────────

TEST_F(IpcRoundTripTest, MultiHopP2ToP4ToP5) {
    // Simulates the detection → command pipeline:
    //   P2 publishes DetectedObjectList on /detected_objects
    //   P4 "processes" it and publishes FCCommand on /fc_commands
    //   P5 receives the FCCommand

    auto det_topic = IntegrationTestHarness::unique_topic("/detected_objects");
    auto cmd_topic = IntegrationTestHarness::unique_topic("/fc_commands");

    auto det_pub = harness_.bus().advertise<ipc::DetectedObjectList>(det_topic);
    auto det_sub = harness_.bus().subscribe<ipc::DetectedObjectList>(det_topic);
    auto cmd_pub = harness_.bus().advertise<ipc::FCCommand>(cmd_topic);
    auto cmd_sub = harness_.bus().subscribe<ipc::FCCommand>(cmd_topic);

    // Step 1: P2 publishes detections
    ipc::DetectedObjectList dets{};
    dets.timestamp_ns                  = harness_.clock().now_ns();
    dets.frame_sequence                = 100;
    dets.num_objects                   = 1;
    dets.objects[0].track_id           = 5;
    dets.objects[0].class_id           = ipc::ObjectClass::DRONE;
    dets.objects[0].confidence         = 0.9f;
    dets.objects[0].position_x         = 3.0f;
    dets.objects[0].position_y         = 0.0f;
    dets.objects[0].position_z         = 0.0f;
    dets.objects[0].velocity_x         = 0.0f;
    dets.objects[0].velocity_y         = 0.0f;
    dets.objects[0].velocity_z         = 0.0f;
    dets.objects[0].heading            = 0.0f;
    dets.objects[0].bbox_x             = 0.0f;
    dets.objects[0].bbox_y             = 0.0f;
    dets.objects[0].bbox_w             = 0.0f;
    dets.objects[0].bbox_h             = 0.0f;
    dets.objects[0].has_camera         = true;
    dets.objects[0].has_radar          = false;
    dets.objects[0].estimated_radius_m = 0.5f;
    dets.objects[0].estimated_height_m = 0.3f;
    dets.objects[0].radar_update_count = 0;
    dets.objects[0].depth_confidence   = 0.4f;

    det_pub->publish(dets);
    wait_delivery();

    // Step 2: "P4 logic" reads detections and decides to RTL
    ipc::DetectedObjectList received_dets{};
    ASSERT_TRUE(det_sub->receive(received_dets));
    EXPECT_EQ(received_dets.num_objects, 1u);

    // P4 decides: obstacle too close, issue RTL command
    ipc::FCCommand cmd{};
    cmd.timestamp_ns   = harness_.clock().now_ns();
    cmd.correlation_id = 200;
    cmd.command        = ipc::FCCommandType::RTL;
    cmd.param1         = 0.0f;
    cmd.sequence_id    = 1;
    cmd.valid          = true;

    cmd_pub->publish(cmd);
    wait_delivery();

    // Step 3: P5 receives the command
    ipc::FCCommand received_cmd{};
    ASSERT_TRUE(cmd_sub->receive(received_cmd));
    EXPECT_EQ(received_cmd.command, ipc::FCCommandType::RTL);
    EXPECT_EQ(received_cmd.correlation_id, 200u);
    EXPECT_TRUE(received_cmd.valid);
}

// ─── Mock clock integration ──────────────────────────────

TEST_F(IpcRoundTripTest, MockClockTimestampsAreConsistent) {
    auto topic = IntegrationTestHarness::unique_topic("/slam_pose");
    auto pub   = harness_.bus().advertise<ipc::Pose>(topic);
    auto sub   = harness_.bus().subscribe<ipc::Pose>(topic);

    // Publish at t=1s (MockClock default start)
    const uint64_t t0 = harness_.clock().now_ns();

    ipc::Pose pose1{};
    pose1.timestamp_ns   = t0;
    pose1.translation[0] = 0.0;
    pose1.translation[1] = 0.0;
    pose1.translation[2] = 0.0;
    pose1.quaternion[0]  = 1.0;
    pose1.quaternion[1]  = 0.0;
    pose1.quaternion[2]  = 0.0;
    pose1.quaternion[3]  = 0.0;
    pose1.velocity[0]    = 0.0;
    pose1.velocity[1]    = 0.0;
    pose1.velocity[2]    = 0.0;
    pose1.quality        = 2;
    pub->publish(pose1);

    // Advance time by 500ms
    harness_.advance_time_ms(500);
    const uint64_t t1 = harness_.clock().now_ns();

    // Verify time advanced correctly
    EXPECT_EQ(t1 - t0, 500'000'000ULL);  // 500ms in nanoseconds

    // Publish second pose at new time
    ipc::Pose pose2{};
    pose2.timestamp_ns   = t1;
    pose2.translation[0] = 1.0;
    pose2.translation[1] = 0.0;
    pose2.translation[2] = 0.0;
    pose2.quaternion[0]  = 1.0;
    pose2.quaternion[1]  = 0.0;
    pose2.quaternion[2]  = 0.0;
    pose2.quaternion[3]  = 0.0;
    pose2.velocity[0]    = 2.0;
    pose2.velocity[1]    = 0.0;
    pose2.velocity[2]    = 0.0;
    pose2.quality        = 2;
    pub->publish(pose2);
    wait_delivery();

    // Read latest — should be the second pose
    ipc::Pose received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.timestamp_ns, t1);
    EXPECT_DOUBLE_EQ(received.translation[0], 1.0);
}

// ─── Capturing logger integration ────────────────────────

TEST_F(IpcRoundTripTest, CapturingLoggerCapturesMessages) {
    // The harness constructor already installed the capturing logger.
    // Any DRONE_LOG_* calls should be captured.
    DRONE_LOG_INFO("Integration test log message");
    ASSERT_NE(harness_.logger(), nullptr);
    EXPECT_TRUE(harness_.logger()->contains("Integration test log message"));
}

// ─── System health round-trip ────────────────────────────

TEST_F(IpcRoundTripTest, SystemHealthRoundTrip) {
    auto topic = IntegrationTestHarness::unique_topic("/system_health");
    auto pub   = harness_.bus().advertise<ipc::SystemHealth>(topic);
    auto sub   = harness_.bus().subscribe<ipc::SystemHealth>(topic);

    ipc::SystemHealth sent{};
    sent.timestamp_ns         = harness_.clock().now_ns();
    sent.cpu_usage_percent    = 45.0f;
    sent.memory_usage_percent = 60.0f;
    sent.disk_usage_percent   = 30.0f;
    sent.max_temp_c           = 55.0f;
    sent.gpu_temp_c           = 50.0f;
    sent.cpu_temp_c           = 52.0f;
    sent.total_healthy        = 7;
    sent.total_degraded       = 0;
    sent.total_dead           = 0;
    sent.power_watts          = 12.5f;
    sent.thermal_zone         = 0;
    sent.stack_status         = 0;
    sent.total_restarts       = 0;
    sent.num_processes        = 0;
    sent.critical_failure     = false;
    ASSERT_TRUE(sent.validate());

    pub->publish(sent);
    wait_delivery();

    ipc::SystemHealth received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_FLOAT_EQ(received.cpu_usage_percent, 45.0f);
    EXPECT_EQ(received.total_healthy, 7u);
    EXPECT_FALSE(received.critical_failure);
}

// ─── Reset clears logger ────────────────────────────────

TEST_F(IpcRoundTripTest, ResetClearsLoggerAndClock) {
    DRONE_LOG_WARN("pre-reset warning");
    ASSERT_NE(harness_.logger(), nullptr);
    EXPECT_TRUE(harness_.logger()->contains("pre-reset warning"));

    harness_.reset();

    // Logger should be cleared
    EXPECT_FALSE(harness_.logger()->contains("pre-reset warning"));
    EXPECT_EQ(harness_.logger()->count(), 0u);

    // Clock should be reset to default 1 second
    EXPECT_EQ(harness_.clock().now_ns(), 1'000'000'000ULL);
}
