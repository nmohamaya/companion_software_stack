// tests/test_fusion_engine.cpp
// Unit tests for CameraOnlyFusionEngine, UKFFusionEngine, IFusionEngine factory.
// LiDAR/radar tests removed (Phase 1A, Issue #112).
// UKF + thermal + factory tests added (Phase 1C, Issue #114).
#include "perception/fusion_engine.h"
#include "perception/ifusion_engine.h"
#include "perception/kalman_tracker.h"
#include "perception/ukf_fusion_engine.h"

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

TEST(FusionEngineTest, DepthEstimationFromImageY) {
    // Bug fix #129: fusion now uses pinhole unproject (not raw pixel-Y depth formula).
    // camera frame: X=forward, Y=right, Z=down.
    // calib: fx=fy=500, cx=320, cy=240, camera_height=1.5m.
    auto         calib = make_test_calib();
    FusionEngine engine(calib);

    const float fx = calib.camera_intrinsics(0, 0);
    const float fy = calib.camera_intrinsics(1, 1);
    const float cx = calib.camera_intrinsics(0, 2);
    const float cy = calib.camera_intrinsics(1, 2);

    // ── Case 1: pixel below the horizon (v > cy) ─────────────
    // position_2d = {320, 340} → ray_down = (340-240)/500 = 0.2 > 0
    // depth = camera_height_m / ray_down = 1.5 / 0.2 = 7.5m
    // position_3d = {depth, 0, depth*ray_down} = {7.5, 0.0, 1.5}
    {
        TrackedObjectList tracked;
        tracked.timestamp_ns   = 2000;
        tracked.frame_sequence = 2;
        TrackedObject obj;
        obj.track_id     = 1;
        obj.class_id     = ObjectClass::PERSON;
        obj.confidence   = 0.9f;
        obj.position_2d  = {320.0f, 340.0f};  // below horizon, on boresight column
        obj.velocity_2d  = Eigen::Vector2f::Zero();
        obj.timestamp_ns = 2000;
        tracked.objects.push_back(obj);

        auto result = engine.fuse(tracked);
        ASSERT_EQ(result.objects.size(), 1u);

        const float ray_down     = (340.0f - cy) / fy;                // = 0.2
        const float expected_d   = calib.camera_height_m / ray_down;  // = 7.5
        const float expected_fwd = expected_d * 1.0f;
        const float expected_rgt = expected_d * ((320.0f - cx) / fx);  // = 0 (on-axis)
        const float expected_dwn = expected_d * ray_down;              // = 1.5

        EXPECT_NEAR(result.objects[0].position_3d.x(), expected_fwd, 0.01f);
        EXPECT_NEAR(result.objects[0].position_3d.y(), expected_rgt, 0.01f);
        EXPECT_NEAR(result.objects[0].position_3d.z(), expected_dwn, 0.01f);
    }

    // ── Case 2: pixel at/above horizon (v <= cy) — fallback depth = 20m ─
    {
        TrackedObjectList tracked;
        tracked.timestamp_ns   = 3000;
        tracked.frame_sequence = 3;
        TrackedObject obj;
        obj.track_id     = 2;
        obj.class_id     = ObjectClass::PERSON;
        obj.confidence   = 0.7f;
        obj.position_2d  = {320.0f, 240.0f};  // exactly at horizon
        obj.velocity_2d  = Eigen::Vector2f::Zero();
        obj.timestamp_ns = 3000;
        tracked.objects.push_back(obj);

        auto result = engine.fuse(tracked);
        ASSERT_EQ(result.objects.size(), 1u);
        // ray_down = 0 → else branch → near-horizon conservative depth = 8m
        // (changed from 20m so obstacles fall inside the 5m influence radius)
        EXPECT_NEAR(result.objects[0].position_3d.x(), 8.0f, 0.01f);
        EXPECT_NEAR(result.objects[0].position_3d.y(), 0.0f, 0.01f);
    }
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

// ═══════════════════════════════════════════════════════════
// IFusionEngine factory tests
// ═══════════════════════════════════════════════════════════

TEST(FusionFactoryTest, CreateCameraOnly) {
    auto engine = create_fusion_engine("camera_only", make_test_calib());
    ASSERT_NE(engine, nullptr);
    EXPECT_EQ(engine->name(), "camera_only");
}

TEST(FusionFactoryTest, CreateUKF) {
    auto engine = create_fusion_engine("ukf", make_test_calib());
    ASSERT_NE(engine, nullptr);
    EXPECT_EQ(engine->name(), "ukf");
}

TEST(FusionFactoryTest, UnknownBackendThrows) {
    EXPECT_THROW(create_fusion_engine("nonexistent", make_test_calib()), std::invalid_argument);
}

// ═══════════════════════════════════════════════════════════
// UKFFusionEngine tests
// ═══════════════════════════════════════════════════════════

TEST(UKFFusionEngineTest, EmptyInputsProduceEmptyOutput) {
    UKFFusionEngine   engine(make_test_calib());
    TrackedObjectList empty;

    auto result = engine.fuse(empty);
    EXPECT_TRUE(result.objects.empty());
}

TEST(UKFFusionEngineTest, SingleObjectProduces3DEstimate) {
    UKFFusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;

    TrackedObject obj;
    obj.track_id     = 1;
    obj.class_id     = ObjectClass::PERSON;
    obj.confidence   = 0.9f;
    obj.position_2d  = {320.0f, 240.0f};
    obj.velocity_2d  = Eigen::Vector2f::Zero();
    obj.timestamp_ns = 1000;
    tracked.objects.push_back(obj);

    auto result = engine.fuse(tracked);

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_EQ(result.objects[0].track_id, 1u);
    EXPECT_TRUE(result.objects[0].has_camera);
    EXPECT_FALSE(result.objects[0].has_thermal);
    // Position should be non-zero (depth estimated)
    EXPECT_GT(result.objects[0].position_3d.x(), 0.0f);
}

TEST(UKFFusionEngineTest, CovarianceReducesWithRepeatedMeasurements) {
    UKFFusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 0;
    tracked.frame_sequence = 0;

    TrackedObject obj;
    obj.track_id     = 1;
    obj.class_id     = ObjectClass::PERSON;
    obj.confidence   = 0.9f;
    obj.position_2d  = {320.0f, 240.0f};
    obj.velocity_2d  = Eigen::Vector2f::Zero();
    obj.timestamp_ns = 0;
    tracked.objects.push_back(obj);

    // First frame — high covariance
    auto result1 = engine.fuse(tracked);
    ASSERT_EQ(result1.objects.size(), 1u);
    float cov1 = result1.objects[0].position_covariance.trace();

    // Feed same measurement 10 more times — covariance should decrease
    for (int i = 1; i <= 10; ++i) {
        tracked.timestamp_ns   = static_cast<uint64_t>(i) * 33000000;
        tracked.frame_sequence = static_cast<uint64_t>(i);
        obj.timestamp_ns       = tracked.timestamp_ns;
        tracked.objects[0]     = obj;
        (void)engine.fuse(tracked);
    }

    tracked.timestamp_ns   = 11 * 33000000;
    tracked.frame_sequence = 11;
    obj.timestamp_ns       = tracked.timestamp_ns;
    tracked.objects[0]     = obj;
    auto  result2          = engine.fuse(tracked);
    float cov2             = result2.objects[0].position_covariance.trace();

    EXPECT_LT(cov2, cov1);  // covariance should decrease
}

TEST(UKFFusionEngineTest, ThermalConfirmationSetsFlag) {
    UKFFusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;

    TrackedObject obj;
    obj.track_id     = 1;
    obj.class_id     = ObjectClass::PERSON;
    obj.confidence   = 0.9f;
    obj.position_2d  = {320.0f, 240.0f};
    obj.velocity_2d  = Eigen::Vector2f::Zero();
    obj.timestamp_ns = 1000;
    tracked.objects.push_back(obj);

    // Provide thermal detection overlapping with the tracked object
    Detection2DList thermal;
    thermal.timestamp_ns   = 1000;
    thermal.frame_sequence = 1;
    thermal.detections.push_back({300, 220, 40, 40, 0.85f, ObjectClass::PERSON, 1000, 1});

    engine.set_thermal_detections(thermal);
    auto result = engine.fuse(tracked);

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_TRUE(result.objects[0].has_thermal);
    EXPECT_TRUE(result.objects[0].has_camera);
}

TEST(UKFFusionEngineTest, ResetClearsFilters) {
    UKFFusionEngine engine(make_test_calib());

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;

    TrackedObject obj;
    obj.track_id     = 1;
    obj.class_id     = ObjectClass::PERSON;
    obj.confidence   = 0.9f;
    obj.position_2d  = {320.0f, 240.0f};
    obj.velocity_2d  = Eigen::Vector2f::Zero();
    obj.timestamp_ns = 1000;
    tracked.objects.push_back(obj);

    (void)engine.fuse(tracked);

    engine.reset();

    // After reset, fusing the same object should create a fresh filter
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    // Covariance should be high again (fresh filter)
    float cov = result.objects[0].position_covariance.trace();
    EXPECT_GT(cov, 1.0f);
}

TEST(UKFFusionEngineTest, NameReturnsUKF) {
    UKFFusionEngine engine(make_test_calib());
    EXPECT_EQ(engine.name(), "ukf");
}

TEST(CameraOnlyFusionEngineTest, NameReturnsCameraOnly) {
    CameraOnlyFusionEngine engine(make_test_calib());
    EXPECT_EQ(engine.name(), "camera_only");
}
