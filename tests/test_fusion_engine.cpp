// tests/test_fusion_engine.cpp
// Unit tests for FusionEngine camera-only fusion.
// LiDAR/radar tests removed (Phase 1A, Issue #112).
#include "perception/fusion_engine.h"
#include "perception/kalman_tracker.h"

#include <gtest/gtest.h>

using namespace drone::perception;

static CalibrationData make_test_calib() {
    CalibrationData calib;
    calib.camera_intrinsics       = Eigen::Matrix3f::Identity();
    calib.camera_intrinsics(0, 0) = 500.0f;  // fx
    calib.camera_intrinsics(1, 1) = 500.0f;  // fy
    calib.camera_intrinsics(0, 2) = 320.0f;  // cx
    calib.camera_intrinsics(1, 2) = 240.0f;  // cy
    calib.camera_height_m         = 1.5f;
    return calib;
}

TEST(FusionEngineTest, EmptyInputsProduceEmptyOutput) {
    FusionEngine      engine(make_test_calib());
    TrackedObjectList empty_tracked;

    auto result = engine.fuse(empty_tracked);
    EXPECT_TRUE(result.objects.empty());
}

TEST(FusionEngineTest, CameraOnlyFusion) {
    FusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;

    TrackedObject obj;
    obj.track_id     = 1;
    obj.class_id     = ObjectClass::PERSON;
    obj.confidence   = 0.8f;
    obj.position_2d  = {320.0f, 240.0f};
    obj.velocity_2d  = {1.0f, 0.5f};
    obj.timestamp_ns = 1000;
    tracked.objects.push_back(obj);

    auto result = engine.fuse(tracked);

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_EQ(result.objects[0].track_id, 1u);
    EXPECT_EQ(result.objects[0].class_id, ObjectClass::PERSON);
    EXPECT_TRUE(result.objects[0].has_camera);
    EXPECT_FALSE(result.objects[0].has_lidar);
    EXPECT_FALSE(result.objects[0].has_radar);
}

TEST(FusionEngineTest, DepthEstimationFromBboxHeight) {
    auto         calib = make_test_calib();
    FusionEngine engine(calib);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 2000;
    tracked.frame_sequence = 2;

    TrackedObject obj;
    obj.track_id     = 1;
    obj.class_id     = ObjectClass::PERSON;
    obj.confidence   = 0.9f;
    obj.position_2d  = {320.0f, 240.0f};  // center of image
    obj.velocity_2d  = Eigen::Vector2f::Zero();
    obj.timestamp_ns = 2000;
    tracked.objects.push_back(obj);

    auto result = engine.fuse(tracked);

    ASSERT_EQ(result.objects.size(), 1u);
    // Expected depth = camera_height_m * 500 / max(10, y)
    // = 1.5 * 500 / 240 = 3.125
    float expected_depth = calib.camera_height_m * 500.0f / 240.0f;
    EXPECT_FLOAT_EQ(result.objects[0].position_3d.x(), expected_depth);
    EXPECT_FLOAT_EQ(result.objects[0].position_3d.y(), 0.0f);
    EXPECT_FLOAT_EQ(result.objects[0].position_3d.z(), 0.0f);
}

TEST(FusionEngineTest, MultipleTrackedObjectsProduceMultipleFused) {
    FusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 3000;
    tracked.frame_sequence = 3;

    for (uint32_t i = 0; i < 5; ++i) {
        TrackedObject obj;
        obj.track_id     = i + 1;
        obj.class_id     = ObjectClass::PERSON;
        obj.confidence   = 0.7f + static_cast<float>(i) * 0.05f;
        obj.position_2d  = {100.0f * static_cast<float>(i + 1), 200.0f};
        obj.velocity_2d  = Eigen::Vector2f::Zero();
        obj.timestamp_ns = 3000;
        tracked.objects.push_back(obj);
    }

    auto result = engine.fuse(tracked);

    ASSERT_EQ(result.objects.size(), 5u);
    for (uint32_t i = 0; i < 5; ++i) {
        EXPECT_EQ(result.objects[i].track_id, i + 1);
        EXPECT_TRUE(result.objects[i].has_camera);
        EXPECT_FALSE(result.objects[i].has_lidar);
        EXPECT_FALSE(result.objects[i].has_radar);
    }
}
