// tests/test_comms.cpp — Tests for MavlinkSim and GCSLink
#include <gtest/gtest.h>
#include "comms/mavlink_sim.h"
#include "comms/gcs_link.h"

using namespace drone::comms;

// ═══════════════════════════════════════════════════════════
// MavlinkSim tests
// ═══════════════════════════════════════════════════════════

TEST(MavlinkSim, InitiallyDisconnected) {
    MavlinkSim mav;
    EXPECT_FALSE(mav.is_connected());
}

TEST(MavlinkSim, OpenConnects) {
    MavlinkSim mav;
    EXPECT_TRUE(mav.open("/dev/ttyTHS1", 921600));
    EXPECT_TRUE(mav.is_connected());
}

TEST(MavlinkSim, SendTrajectoryRequiresConnection) {
    MavlinkSim mav;
    EXPECT_FALSE(mav.send_trajectory(1.0f, 0.0f, 0.0f, 0.0f));

    mav.open("/dev/ttyTHS1", 921600);
    EXPECT_TRUE(mav.send_trajectory(1.0f, 2.0f, -0.5f, 1.57f));
}

TEST(MavlinkSim, HeartbeatBatteryDrains) {
    MavlinkSim mav;
    mav.open("/dev/ttyTHS1", 921600);

    auto hb1 = mav.receive_heartbeat();
    EXPECT_GT(hb1.battery_percent, 0.0f);
    EXPECT_LE(hb1.battery_percent, 100.0f);
    EXPECT_TRUE(hb1.timestamp_ns > 0);
    EXPECT_GE(hb1.satellites, 0);
}

TEST(MavlinkSim, ArmDisarm) {
    MavlinkSim mav;
    mav.open("/dev/ttyTHS1", 921600);

    EXPECT_TRUE(mav.send_arm(true));
    auto hb = mav.receive_heartbeat();
    EXPECT_TRUE(hb.armed);

    EXPECT_TRUE(mav.send_arm(false));
    hb = mav.receive_heartbeat();
    EXPECT_FALSE(hb.armed);
}

TEST(MavlinkSim, ModeChange) {
    MavlinkSim mav;
    mav.open("/dev/ttyTHS1", 921600);

    EXPECT_TRUE(mav.send_mode(2));  // AUTO
    auto hb = mav.receive_heartbeat();
    EXPECT_EQ(hb.flight_mode, 2);
}

TEST(MavlinkSim, GroundSpeedReflectsTrajectory) {
    MavlinkSim mav;
    mav.open("/dev/ttyTHS1", 921600);

    mav.send_trajectory(3.0f, 4.0f, 0.0f, 0.0f);
    auto hb = mav.receive_heartbeat();
    float expected_speed = 5.0f;  // sqrt(3^2 + 4^2)
    EXPECT_NEAR(hb.ground_speed, expected_speed, 0.01f);
}

// ═══════════════════════════════════════════════════════════
// GCSLink tests
// ═══════════════════════════════════════════════════════════

TEST(GCSLink, InitiallyDisconnected) {
    GCSLink gcs;
    EXPECT_FALSE(gcs.is_connected());
}

TEST(GCSLink, OpenConnects) {
    GCSLink gcs;
    EXPECT_TRUE(gcs.open("0.0.0.0", 14550));
    EXPECT_TRUE(gcs.is_connected());
}

TEST(GCSLink, SendTelemetryRequiresConnection) {
    GCSLink gcs;
    EXPECT_FALSE(gcs.send_telemetry(0.0f, 0.0f, 0.0f, 100.0f, 0));

    gcs.open("0.0.0.0", 14550);
    EXPECT_TRUE(gcs.send_telemetry(37.7749f, -122.4194f, 50.0f, 85.0f, 3));
}

TEST(GCSLink, PollCommandInitiallyEmpty) {
    GCSLink gcs;
    gcs.open("0.0.0.0", 14550);
    auto msg = gcs.poll_command();
    // Should not produce RTL immediately (only after 120s)
    EXPECT_FALSE(msg.valid);
}

TEST(GCSLink, MessageTypes) {
    // Verify enum values are correct
    EXPECT_EQ(static_cast<uint8_t>(GCSMessageType::HEARTBEAT), 0);
    EXPECT_EQ(static_cast<uint8_t>(GCSMessageType::MISSION_CMD), 1);
    EXPECT_EQ(static_cast<uint8_t>(GCSMessageType::RTL_CMD), 2);
    EXPECT_EQ(static_cast<uint8_t>(GCSMessageType::LAND_CMD), 3);
    EXPECT_EQ(static_cast<uint8_t>(GCSMessageType::PARAM_SET), 4);
}

TEST(GCSLink, GCSMessageDefaults) {
    GCSMessage msg{};
    EXPECT_EQ(msg.type, GCSMessageType::HEARTBEAT);
    EXPECT_FLOAT_EQ(msg.param1, 0.0f);
    EXPECT_FLOAT_EQ(msg.param2, 0.0f);
    EXPECT_EQ(msg.timestamp_ns, 0u);
    EXPECT_FALSE(msg.valid);
}
