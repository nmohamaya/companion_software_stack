// tests/test_payload_manager.cpp — Tests for GimbalController
#include "payload/gimbal_controller.h"

#include <gtest/gtest.h>

using namespace drone::payload;

TEST(GimbalController, InitialState) {
    GimbalController gimbal;
    EXPECT_FALSE(gimbal.is_initialised());
    EXPECT_FALSE(gimbal.is_recording());
    EXPECT_FLOAT_EQ(gimbal.state().pitch, 0.0f);
    EXPECT_FLOAT_EQ(gimbal.state().yaw, 0.0f);
    EXPECT_FLOAT_EQ(gimbal.state().roll, 0.0f);
    EXPECT_TRUE(gimbal.state().stabilised);
}

TEST(GimbalController, Init) {
    GimbalController gimbal;
    EXPECT_TRUE(gimbal.init());
    EXPECT_TRUE(gimbal.is_initialised());
}

TEST(GimbalController, SetTargetClamping) {
    GimbalController gimbal;
    gimbal.init();

    // Pitch should clamp to [-90, 30]
    gimbal.set_target(-120.0f, 0.0f);
    // Run update long enough to reach target
    for (int i = 0; i < 200; ++i) gimbal.update(0.02f);
    EXPECT_NEAR(gimbal.state().pitch, -90.0f, 0.5f);

    gimbal.set_target(60.0f, 0.0f);
    for (int i = 0; i < 200; ++i) gimbal.update(0.02f);
    EXPECT_NEAR(gimbal.state().pitch, 30.0f, 0.5f);

    // Yaw should clamp to [-180, 180]
    gimbal.set_target(0.0f, 200.0f);
    for (int i = 0; i < 300; ++i) gimbal.update(0.02f);
    EXPECT_NEAR(gimbal.state().yaw, 180.0f, 0.5f);
}

TEST(GimbalController, SmoothMotion) {
    GimbalController gimbal;
    gimbal.init();

    gimbal.set_target(-45.0f, 90.0f);
    gimbal.update(0.02f);

    // After one step (RATE=60 deg/s, dt=0.02), max_step = 1.2 deg
    // Should not jump to target immediately
    EXPECT_NE(gimbal.state().pitch, -45.0f);
    EXPECT_NE(gimbal.state().yaw, 90.0f);

    // But should be moving in the right direction
    EXPECT_LT(gimbal.state().pitch, 0.0f);  // heading negative
    EXPECT_GT(gimbal.state().yaw, 0.0f);    // heading positive
}

TEST(GimbalController, ConvergesToTarget) {
    GimbalController gimbal;
    gimbal.init();

    gimbal.set_target(-45.0f, 90.0f);
    // Run enough steps to converge (90 deg / 1.2 deg/step = ~75 steps)
    for (int i = 0; i < 200; ++i) {
        gimbal.update(0.02f);
    }
    EXPECT_NEAR(gimbal.state().pitch, -45.0f, 0.01f);
    EXPECT_NEAR(gimbal.state().yaw, 90.0f, 0.01f);
}

TEST(GimbalController, CaptureImage) {
    GimbalController gimbal;
    gimbal.init();

    uint64_t ts = gimbal.capture_image();
    EXPECT_GT(ts, 0u);
}

TEST(GimbalController, Recording) {
    GimbalController gimbal;
    gimbal.init();

    EXPECT_FALSE(gimbal.is_recording());

    gimbal.start_recording();
    EXPECT_TRUE(gimbal.is_recording());

    gimbal.stop_recording();
    EXPECT_FALSE(gimbal.is_recording());
}

TEST(GimbalController, MultipleCapturesIncrement) {
    GimbalController gimbal;
    gimbal.init();

    uint64_t ts1 = gimbal.capture_image();
    uint64_t ts2 = gimbal.capture_image();
    // Second capture should have later or equal timestamp
    EXPECT_GE(ts2, ts1);
}

TEST(GimbalController, ZeroDtNoMovement) {
    GimbalController gimbal;
    gimbal.init();

    gimbal.set_target(-30.0f, 60.0f);
    float pitch_before = gimbal.state().pitch;
    float yaw_before   = gimbal.state().yaw;

    gimbal.update(0.0f);  // zero dt

    EXPECT_FLOAT_EQ(gimbal.state().pitch, pitch_before);
    EXPECT_FLOAT_EQ(gimbal.state().yaw, yaw_before);
}
