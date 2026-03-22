// tests/test_fusion_engine.cpp
// Unit tests for CameraOnlyFusionEngine, UKFFusionEngine, IFusionEngine factory.
// UKF + factory tests added (Phase 1C, Issue #114).
// Radar fusion tests added (Issue #210).
#include "ipc/ipc_types.h"
#include "perception/fusion_engine.h"
#include "perception/ifusion_engine.h"
#include "perception/kalman_tracker.h"
#include "perception/ukf_fusion_engine.h"

#include <algorithm>
#include <cmath>

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
}

TEST(FusionEngineTest, DepthEstimationFromImageY) {
    // Fusion uses pinhole unproject (not raw pixel-Y depth formula).
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

// ═══════════════════════════════════════════════════════════
// Radar fusion tests (Issue #210)
// ═══════════════════════════════════════════════════════════

// Helper: create a TrackedObject at a known position for radar tests.
static TrackedObject make_test_tracked(uint32_t id = 1, float px = 320.0f, float py = 340.0f) {
    TrackedObject obj;
    obj.track_id     = id;
    obj.class_id     = ObjectClass::PERSON;
    obj.confidence   = 0.9f;
    obj.position_2d  = {px, py};
    obj.velocity_2d  = Eigen::Vector2f::Zero();
    obj.timestamp_ns = 1000;
    return obj;
}

// Helper: create a RadarDetection from range/azimuth/elevation/velocity.
static drone::ipc::RadarDetection make_radar_det(float range, float azimuth, float elevation,
                                                 float velocity) {
    drone::ipc::RadarDetection det{};
    det.timestamp_ns        = 1000;
    det.range_m             = range;
    det.azimuth_rad         = azimuth;
    det.elevation_rad       = elevation;
    det.radial_velocity_mps = velocity;
    det.confidence          = 0.95f;
    det.rcs_dbsm            = 10.0f;
    det.snr_db              = 20.0f;
    return det;
}

// Helper: compute depth the same way UKFFusionEngine::estimate_depth does.
static float test_estimate_depth(const CalibrationData& calib, const TrackedObject& trk) {
    const float fy = calib.camera_intrinsics(1, 1);
    return calib.camera_height_m * fy / std::max(10.0f, trk.position_2d.y());
}

TEST(RadarFusionTest, RadarMeasurementModel) {
    // ObjectUKF constructor with initial_depth=10 sets:
    //   x_ = [10, 320*0.001*10=3.2, 0, 0, 0, 0]
    // So the object is ~10m forward, ~3.2m lateral, on the horizontal plane.
    TrackedObject trk = make_test_tracked();  // position_2d = {320, 340}
    ObjectUKF     ukf(trk, 10.0f);

    auto z_pred = ukf.predicted_radar_measurement();

    // range = sqrt(10² + 3.2² + 0²) ≈ 10.5
    const float expected_range = std::sqrt(10.0f * 10.0f + 3.2f * 3.2f);
    EXPECT_NEAR(z_pred(0), expected_range, 0.1f);

    // azimuth = atan2(3.2, 10) ≈ 0.309 rad
    const float expected_azimuth = std::atan2(3.2f, 10.0f);
    EXPECT_NEAR(z_pred(1), expected_azimuth, 0.01f);

    // elevation ≈ 0 (z=0, on horizontal plane)
    EXPECT_NEAR(z_pred(2), 0.0f, 0.01f);

    // radial velocity ≈ 0 (velocity is zero)
    EXPECT_NEAR(z_pred(3), 0.0f, 0.01f);
}

TEST(RadarFusionTest, RadarUpdateReducesCovariance) {
    // A single radar update should reduce position covariance.
    TrackedObject trk = make_test_tracked();
    ObjectUKF     ukf(trk, 10.0f);

    ukf.predict();
    float cov_before = ukf.position_covariance().trace();

    // Create a radar detection that matches the predicted measurement
    auto z_pred = ukf.predicted_radar_measurement();
    auto det    = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));
    ukf.update_radar(det);

    float cov_after = ukf.position_covariance().trace();
    EXPECT_LT(cov_after, cov_before);
}

TEST(RadarFusionTest, CameraRadarFusionTighterThanEither) {
    // Camera+radar should produce tighter covariance than camera-only.
    auto  calib = make_test_calib();
    auto  trk   = make_test_tracked();
    float depth = test_estimate_depth(calib, trk);

    // Camera-only UKF
    UKFFusionEngine   cam_engine(calib);
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto cam_result = cam_engine.fuse(tracked);
    ASSERT_EQ(cam_result.objects.size(), 1u);
    float cam_cov = cam_result.objects[0].position_covariance.trace();

    // Camera+radar UKF — use matching depth to create a valid radar detection
    UKFFusionEngine both_engine(calib, RadarNoiseConfig{}, true);

    ObjectUKF temp_ukf(trk, depth);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 1000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));

    both_engine.set_radar_detections(radar_list);
    auto both_result = both_engine.fuse(tracked);
    ASSERT_EQ(both_result.objects.size(), 1u);
    float both_cov = both_result.objects[0].position_covariance.trace();

    EXPECT_LT(both_cov, cam_cov);
}

TEST(RadarFusionTest, RadarGateRejectsOutlier) {
    // A radar detection far from the track should not be associated.
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked());

    // Radar detection far away (range=500, opposite azimuth)
    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 1000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(500.0f, -1.5f, 1.0f, 50.0f);

    engine.set_radar_detections(radar_list);
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_FALSE(result.objects[0].has_radar);  // outlier should be rejected
}

TEST(RadarFusionTest, RadarNoiseConfig) {
    // Verify custom noise config propagates to the R_radar matrix.
    RadarNoiseConfig cfg;
    cfg.range_std_m       = 1.0f;
    cfg.azimuth_std_rad   = 0.1f;
    cfg.elevation_std_rad = 0.2f;
    cfg.velocity_std_mps  = 0.5f;

    TrackedObject trk = make_test_tracked();
    ObjectUKF     ukf(trk, 10.0f, cfg);

    const auto& R = ukf.radar_noise();
    EXPECT_NEAR(R(0, 0), 1.0f, 1e-6f);   // range_std² = 1.0
    EXPECT_NEAR(R(1, 1), 0.01f, 1e-6f);  // azimuth_std² = 0.01
    EXPECT_NEAR(R(2, 2), 0.04f, 1e-6f);  // elevation_std² = 0.04
    EXPECT_NEAR(R(3, 3), 0.25f, 1e-6f);  // velocity_std² = 0.25
    // Off-diagonals should be zero
    EXPECT_NEAR(R(0, 1), 0.0f, 1e-6f);
    EXPECT_NEAR(R(1, 2), 0.0f, 1e-6f);
}

TEST(RadarFusionTest, RadarDisabledByDefault) {
    // Without radar data, output should have has_radar=false and match camera-only.
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked());

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_TRUE(result.objects[0].has_camera);
    EXPECT_FALSE(result.objects[0].has_radar);
}

TEST(RadarFusionTest, SetRadarDetectionsAndFuse) {
    // Verify set_radar_detections + fuse uses radar data.
    auto  calib = make_test_calib();
    auto  trk   = make_test_tracked();
    float depth = test_estimate_depth(calib, trk);

    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    // First fuse without radar to establish the track
    auto result1 = engine.fuse(tracked);
    ASSERT_EQ(result1.objects.size(), 1u);

    // Simulate the same UKF steps the engine performed to get matching radar coords
    ObjectUKF temp_ukf(trk, depth);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    // Now fuse with matching radar data
    tracked.timestamp_ns            = 2000;
    tracked.frame_sequence          = 2;
    tracked.objects[0].timestamp_ns = 2000;

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 2000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));

    engine.set_radar_detections(radar_list);
    auto result2 = engine.fuse(tracked);
    ASSERT_EQ(result2.objects.size(), 1u);
    EXPECT_TRUE(result2.objects[0].has_radar);
}

TEST(RadarFusionTest, HasRadarFlagOnlyWhenMatched) {
    // has_radar should be true only for tracks that matched a radar detection.
    auto  calib  = make_test_calib();
    auto  trk1   = make_test_tracked(1, 320.0f, 340.0f);
    auto  trk2   = make_test_tracked(2, 100.0f, 400.0f);
    float depth1 = test_estimate_depth(calib, trk1);

    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk1);
    tracked.objects.push_back(trk2);

    // First fuse to establish tracks
    (void)engine.fuse(tracked);

    // Create radar detection that matches only track 1.
    // Use matching depth so the predicted radar measurement aligns with the engine's UKF state.
    ObjectUKF temp_ukf(trk1, depth1);
    temp_ukf.predict();
    temp_ukf.update_camera(trk1, depth1);
    temp_ukf.predict();
    temp_ukf.update_camera(trk1, depth1);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 2000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));

    tracked.timestamp_ns            = 2000;
    tracked.frame_sequence          = 2;
    tracked.objects[0].timestamp_ns = 2000;
    tracked.objects[1].timestamp_ns = 2000;

    engine.set_radar_detections(radar_list);
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 2u);

    // Exactly one should have radar — the detection should match only the closer track
    int radar_count = 0;
    for (const auto& obj : result.objects) {
        if (obj.has_radar) ++radar_count;
    }
    EXPECT_EQ(radar_count, 1);
}

// ═══════════════════════════════════════════════════════════════
// Radar ground-plane filter tests (Issue #225)
// ═══════════════════════════════════════════════════════════════

TEST(RadarFusionTest, GroundFilterRejectsLowAltitude) {
    // Radar detection at negative elevation resolving below 0.3m AGL should be rejected.
    // drone_altitude=5.0m, range=10m, elevation=-0.5rad
    // → object_alt = 5.0 + 10*sin(-0.5) ≈ 5.0 - 4.79 ≈ 0.21m → below 0.3m → rejected
    auto             calib = make_test_calib();
    RadarNoiseConfig radar_cfg;
    radar_cfg.ground_filter_enabled = true;
    radar_cfg.min_object_altitude_m = 0.3f;

    UKFFusionEngine engine(calib, radar_cfg, true);
    engine.set_drone_altitude(5.0f);

    auto              trk = make_test_tracked();
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    // First fuse to create the UKF filter for this track
    auto first_result = engine.fuse(tracked);
    ASSERT_EQ(first_result.objects.size(), 1u);

    // Now provide a ground-return radar detection
    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 2000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(10.0f, 0.0f, -0.5f, 0.0f);

    engine.set_radar_detections(radar_list);
    tracked.timestamp_ns   = 2000;
    tracked.frame_sequence = 2;
    auto result            = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // The ground detection should have been filtered — no radar match
    EXPECT_FALSE(result.objects[0].has_radar);
}

TEST(RadarFusionTest, GroundFilterPassesHighAltitude) {
    // Radar detection at zero elevation (horizontal) resolves at drone altitude.
    // drone_altitude=5.0m, elevation≈0 → object_alt ≈ 5.0m → above 0.3m → accepted.
    // Provide radar on the FIRST fuse() call so the engine's internal UKF matches.
    auto             calib = make_test_calib();
    RadarNoiseConfig radar_cfg;
    radar_cfg.ground_filter_enabled = true;
    radar_cfg.min_object_altitude_m = 0.3f;

    UKFFusionEngine engine(calib, radar_cfg, true);
    engine.set_drone_altitude(5.0f);

    auto              trk   = make_test_tracked();
    float             depth = test_estimate_depth(calib, trk);
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    // Build a matching radar detection from a temp UKF (same as CameraRadarFusion test)
    ObjectUKF temp_ukf(trk, depth, radar_cfg);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    // Ensure elevation resolves above threshold
    ASSERT_GT(5.0f + z_pred(0) * std::sin(z_pred(2)), radar_cfg.min_object_altitude_m);

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 1000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));

    engine.set_radar_detections(radar_list);
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // The high-altitude detection should have passed the filter and matched
    EXPECT_TRUE(result.objects[0].has_radar);
}

TEST(RadarFusionTest, GroundFilterDisabledPassesAll) {
    // With ground_filter_enabled=false, detections that would normally be filtered
    // should still be considered for association.  Provide radar on first fuse().
    auto             calib = make_test_calib();
    RadarNoiseConfig radar_cfg;
    radar_cfg.ground_filter_enabled = false;
    radar_cfg.min_object_altitude_m = 0.3f;

    UKFFusionEngine engine(calib, radar_cfg, true);
    engine.set_drone_altitude(5.0f);

    auto              trk   = make_test_tracked();
    float             depth = test_estimate_depth(calib, trk);
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    // Build a matching radar detection
    ObjectUKF temp_ukf(trk, depth, radar_cfg);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 1000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));

    engine.set_radar_detections(radar_list);
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // Filter is disabled — should match
    EXPECT_TRUE(result.objects[0].has_radar);
}
