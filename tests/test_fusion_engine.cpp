// tests/test_fusion_engine.cpp
// Unit tests for FusionEngine multi-sensor fusion.
#include <gtest/gtest.h>
#include "perception/fusion_engine.h"
#include "perception/kalman_tracker.h"

using namespace drone::perception;

static CalibrationData make_test_calib() {
    CalibrationData calib;
    calib.camera_intrinsics = Eigen::Matrix3f::Identity();
    calib.camera_intrinsics(0, 0) = 500.0f;  // fx
    calib.camera_intrinsics(1, 1) = 500.0f;  // fy
    calib.camera_intrinsics(0, 2) = 320.0f;  // cx
    calib.camera_intrinsics(1, 2) = 240.0f;  // cy
    calib.T_cam_lidar = Eigen::Matrix4f::Identity();
    calib.T_cam_radar = Eigen::Matrix4f::Identity();
    calib.camera_height_m = 1.5f;
    return calib;
}

TEST(FusionEngineTest, EmptyInputsProduceEmptyOutput) {
    FusionEngine engine(make_test_calib());
    TrackedObjectList empty_tracked;
    std::vector<LiDARCluster> empty_lidar;
    RadarDetectionList empty_radar;

    auto result = engine.fuse(empty_tracked, empty_lidar, empty_radar);
    EXPECT_TRUE(result.objects.empty());
}

TEST(FusionEngineTest, CameraOnlyFusion) {
    FusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    tracked.timestamp_ns = 1000;
    tracked.frame_sequence = 1;

    TrackedObject obj;
    obj.track_id = 1;
    obj.class_id = ObjectClass::PERSON;
    obj.confidence = 0.8f;
    obj.position_2d = {320.0f, 240.0f};
    obj.velocity_2d = {1.0f, 0.5f};
    obj.timestamp_ns = 1000;
    tracked.objects.push_back(obj);

    auto result = engine.fuse(tracked, {}, {});

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_EQ(result.objects[0].track_id, 1u);
    EXPECT_EQ(result.objects[0].class_id, ObjectClass::PERSON);
    EXPECT_TRUE(result.objects[0].has_camera);
    EXPECT_FALSE(result.objects[0].has_lidar);
    EXPECT_FALSE(result.objects[0].has_radar);
}

TEST(FusionEngineTest, LiDARBoostsConfidence) {
    FusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    TrackedObject obj;
    obj.track_id = 1;
    obj.class_id = ObjectClass::VEHICLE_CAR;
    obj.confidence = 0.7f;
    obj.position_2d = {320.0f, 240.0f};
    obj.velocity_2d = Eigen::Vector2f::Zero();
    obj.timestamp_ns = 1000;
    tracked.objects.push_back(obj);

    // LiDAR cluster near the estimated camera position
    LiDARCluster cluster;
    cluster.centroid = {3.0f, 0.0f, 0.0f};
    cluster.bbox_min = {2.5f, -0.5f, -0.5f};
    cluster.bbox_max = {3.5f, 0.5f, 0.5f};
    cluster.distance = 3.0f;

    auto result = engine.fuse(tracked, {cluster}, {});

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_TRUE(result.objects[0].has_lidar);
    EXPECT_GT(result.objects[0].confidence, 0.7f);  // boosted
}

TEST(FusionEngineTest, UnmatchedLiDARBecomeUnknown) {
    FusionEngine engine(make_test_calib());

    TrackedObjectList empty_tracked;

    // A lone LiDAR cluster with no camera track match
    LiDARCluster cluster;
    cluster.centroid = {10.0f, 5.0f, 0.0f};
    cluster.bbox_min = {9.5f, 4.5f, -0.5f};
    cluster.bbox_max = {10.5f, 5.5f, 0.5f};
    cluster.distance = 11.18f;  // < 50m so included

    auto result = engine.fuse(empty_tracked, {cluster}, {});

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_EQ(result.objects[0].class_id, ObjectClass::UNKNOWN);
    EXPECT_FALSE(result.objects[0].has_camera);
    EXPECT_TRUE(result.objects[0].has_lidar);
    EXPECT_FLOAT_EQ(result.objects[0].confidence, 0.4f);
}

TEST(FusionEngineTest, RadarProvidesVelocity) {
    FusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    TrackedObject obj;
    obj.track_id = 1;
    obj.class_id = ObjectClass::VEHICLE_CAR;
    obj.confidence = 0.8f;
    obj.position_2d = {320.0f, 240.0f};
    obj.velocity_2d = Eigen::Vector2f::Zero();
    obj.timestamp_ns = 1000;
    tracked.objects.push_back(obj);

    // Radar detection close to estimated position
    RadarDetectionList radar;
    RadarDetection rdet;
    rdet.range = 3.0f;
    rdet.azimuth = 0.0f;
    rdet.elevation = 0.0f;
    rdet.velocity_radial = 5.0f;
    rdet.rcs = 10.0f;
    rdet.timestamp_ns = 1000;
    radar.detections.push_back(rdet);

    auto result = engine.fuse(tracked, {}, radar);

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_TRUE(result.objects[0].has_radar);
    EXPECT_GT(result.objects[0].confidence, 0.8f);
}
