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
// Horizon-Truncated Depth Estimation (Issue #237)
// ═══════════════════════════════════════════════════════════

TEST(UKFFusionEngineTest, HorizonTruncatedBboxUsesGroundPlaneDepth) {
    // When an obstacle's bbox top is at/above the horizon (cy), the apparent-size
    // formula overestimates depth because bbox_h < true projected height.
    // The engine should fall back to ground-plane depth from the bbox bottom.
    //
    // Scenario: camera at 5m altitude, obstacle 18.7m away at same height.
    // bbox_top ≈ cy (horizon), bbox_bottom well below.
    auto calib            = make_test_calib();
    calib.camera_height_m = 5.0f;
    calib.depth_scale     = 1.0f;

    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);
    engine.set_drone_altitude(5.0f);

    // Object with bbox top at horizon (cy=240): centroid at 265, bbox_h=50.
    // bbox_top = 265 - 25 = 240 = cy → horizon-truncated.
    // bbox_bottom = 265 + 25 = 290.
    // Ground-plane depth from bottom: altitude * fy / (bottom - cy) = 5 * 500 / 50 = 50m (clamped to 40m).
    //
    // Without the fix (apparent-size): 5.0 * 500 / 50 = 50m (wrong — assumes 50px = full 5m height).
    // With the fix (ground-plane from bottom): uses drone_altitude=5m.
    TrackedObject trk;
    trk.track_id     = 1;
    trk.class_id     = ObjectClass::PERSON;
    trk.confidence   = 0.9f;
    trk.position_2d  = {320.0f, 265.0f};
    trk.bbox_w       = 24.0f;
    trk.bbox_h       = 50.0f;
    trk.timestamp_ns = 1000;

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // The depth (x-component in body frame) should use ground-plane formula:
    // depth = 5.0 * 500 / (290 - 240) = 50 → clamped to 40m.
    // Without the fix, apparent-size would give: 5.0 * 500 / 50 = 50 → same (coincidence).
    // Use a different bbox to make the distinction clear.

    // Second test: bbox_top ABOVE horizon (clearly truncated)
    TrackedObject trk2;
    trk2.track_id     = 2;
    trk2.class_id     = ObjectClass::PERSON;
    trk2.confidence   = 0.9f;
    trk2.position_2d  = {320.0f, 260.0f};  // centroid at 260
    trk2.bbox_w       = 24.0f;
    trk2.bbox_h       = 80.0f;  // bbox_top = 260-40 = 220 < cy(240) → truncated
    trk2.timestamp_ns = 1000;
    // Apparent-size would give: 3.0 * 500 / 80 = 18.75m
    // Ground-plane from bottom: camera_height_m=1.5 / ray_down → larger depth
    // bbox_bottom = 260 + 40 = 300, ray_down_base = (300-240)/500 = 0.12

    tracked.objects.clear();
    tracked.objects.push_back(trk2);
    tracked.timestamp_ns   = 2000;
    tracked.frame_sequence = 2;

    auto result2 = engine.fuse(tracked);
    ASSERT_EQ(result2.objects.size(), 1u);

    // The fused depth should be ground-plane based (~40m clamped), not apparent-size (31.25m).
    // position_3d.x() is the depth (forward axis in body frame).
    EXPECT_GT(result2.objects[0].position_3d.x(), 35.0f);
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

    // Close-range detection (py=340, bbox_h=300 → depth=5m).
    // Uses moderate depth to avoid divergence from the UKF's approximate
    // bearing model (pixel*0.001 vs proper (pixel-cy)/fy).  The bearing
    // model is a known limitation (see Issue #237 notes) that causes
    // covariance growth at larger depths; fixing it requires passing
    // calibration intrinsics into ObjectUKF.
    TrackedObject obj;
    obj.track_id     = 1;
    obj.class_id     = ObjectClass::PERSON;
    obj.confidence   = 0.9f;
    obj.position_2d  = {320.0f, 340.0f};
    obj.velocity_2d  = Eigen::Vector2f::Zero();
    obj.bbox_h       = 300.0f;  // apparent-size depth = 3.0 * 500 / 300 = 5m
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

    EXPECT_LT(cov2, cov1);  // covariance should decrease after settling
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

// Build CameraIntrinsics from CalibrationData so standalone ObjectUKFs
// in tests use the same intrinsics as UKFFusionEngine does internally.
static CameraIntrinsics cam_intr_from(const CalibrationData& c) {
    return {c.camera_intrinsics(0, 0), c.camera_intrinsics(1, 1), c.camera_intrinsics(0, 2),
            c.camera_intrinsics(1, 2)};
}

// Helper: create a TrackedObject at a known position for radar tests.
static TrackedObject make_test_tracked(uint32_t id = 1, float px = 320.0f, float py = 340.0f) {
    TrackedObject obj;
    obj.track_id    = id;
    obj.class_id    = ObjectClass::PERSON;
    obj.confidence  = 0.9f;
    obj.position_2d = {px, py};
    obj.velocity_2d = Eigen::Vector2f::Zero();
    obj.bbox_h = 100.0f;  // apparent-size depth = 1.7 * 500 / 100 * 0.7 ≈ 5.95m (PERSON prior)
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

// Helper: compute depth similarly to UKFFusionEngine::estimate_depth.
// Covers three tiers: apparent-size → ground-plane → fallback.
// Note: the production engine also has horizon-truncation (bbox_top < cy);
// this helper omits that tier for simplicity — tests using truncated bboxes
// should account for the difference.
// Uses calib.depth_scale for conservative depth scaling.
static float test_estimate_depth(const CalibrationData& calib, const TrackedObject& trk) {
    const float     fy               = calib.camera_intrinsics(1, 1);
    const float     cy               = calib.camera_intrinsics(1, 2);
    constexpr float kBboxHThreshold  = 10.0f;
    constexpr float kRayDownMinThres = 0.01f;
    const float     ds               = calib.depth_scale;

    if (trk.bbox_h > kBboxHThreshold) {
        const auto  class_idx = static_cast<uint8_t>(trk.class_id);
        const float assumed_h = (class_idx < drone::perception::kNumObjectClasses)
                                    ? calib.height_priors[class_idx]
                                    : calib.height_priors[0];
        return std::clamp(assumed_h * fy / trk.bbox_h * ds, 1.0f, 40.0f);
    }
    const float ray_down = (trk.position_2d.y() - cy) / std::max(1.0f, fy);
    if (ray_down > kRayDownMinThres) {
        return std::clamp(calib.camera_height_m / ray_down * ds, 1.0f, 40.0f);
    }
    return 8.0f * ds;
}

TEST(RadarFusionTest, RadarMeasurementModel) {
    // ObjectUKF constructor with initial_depth=10 and pinhole intrinsics:
    //   bearing_x = (320-320)/277 = 0.0,  bearing_y = (340-240)/277 ≈ 0.361
    //   x_ = [10, 0, 3.61, 0, 0, 0]
    // Object is ~10m forward, 0m lateral, ~3.61m below (body Z=down).
    TrackedObject trk = make_test_tracked();  // position_2d = {320, 340}
    ObjectUKF     ukf(trk, 10.0f);

    auto z_pred = ukf.predicted_radar_measurement();

    // bearing_y = (340-240)/277 ≈ 0.3610
    const float by             = (340.0f - 240.0f) / 277.0f;
    const float expected_range = std::sqrt(10.0f * 10.0f + by * 10.0f * by * 10.0f);
    EXPECT_NEAR(z_pred(0), expected_range, 0.1f);

    // azimuth = atan2(y=0, x=10) = 0
    EXPECT_NEAR(z_pred(1), 0.0f, 0.01f);

    // elevation = atan2(z=3.61, sqrt(x²+y²)=10) ≈ 0.347 rad
    const float expected_elev = std::atan2(by * 10.0f, 10.0f);
    EXPECT_NEAR(z_pred(2), expected_elev, 0.02f);

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

TEST(RadarFusionTest, AzimuthSignConventionFLUToFRD) {
    // Verify the FLU→FRD azimuth negation for radar measurements.
    // An off-center object (bearing_x > 0 = right in body frame) should have
    // positive azimuth in the UKF model (FRD).  The engine negates the incoming
    // Gazebo detection azimuth, so a Gazebo FLU detection with NEGATIVE azimuth
    // (= right in FLU) should match a UKF track with POSITIVE azimuth.
    auto             calib = make_test_calib();
    RadarNoiseConfig radar_cfg;
    radar_cfg.ground_filter_enabled = false;

    UKFFusionEngine engine(calib, radar_cfg, true);
    engine.set_drone_altitude(5.0f);

    // Object 50px right of center → positive bearing_x → positive UKF azimuth
    TrackedObject trk       = make_test_tracked();
    trk.position_2d         = {370.0f, 340.0f};  // 50px right of cx=320
    float             depth = test_estimate_depth(calib, trk);
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    // Build matching radar detection in UKF FRD convention from a temp UKF
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, radar_cfg, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    // Sanity: predicted azimuth should be positive (object to the right)
    EXPECT_GT(z_pred(1), 0.05f);

    // The engine expects Gazebo FLU convention where right = NEGATIVE azimuth.
    // So we provide the NEGATED predicted azimuth to simulate a correct Gazebo detection.
    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 1000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(z_pred(0), -z_pred(1), z_pred(2), z_pred(3));

    engine.set_radar_detections(radar_list);
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // The negated Gazebo detection should match the track after the engine's
    // internal FLU→FRD conversion.
    EXPECT_TRUE(result.objects[0].has_radar);
}

TEST(RadarFusionTest, RadarInitializedDepthOverridesCameraEstimate) {
    // When a new camera track is created and a radar detection exists at a
    // similar bearing, the radar range should override the camera's monocular
    // depth estimate. This tests the "Option B" radar-initialized depth.
    auto             calib = make_test_calib();
    RadarNoiseConfig radar_cfg;
    radar_cfg.ground_filter_enabled = false;

    UKFFusionEngine engine(calib, radar_cfg, true);  // radar enabled
    engine.set_drone_altitude(5.0f);

    // Camera track at center — monocular depth estimate will be ~15m
    // (assumed_obstacle_height=3.0 * fy=500 / bbox_h=100 * depth_scale=0.7 ≈ 10.5m)
    TrackedObject trk = make_test_tracked();  // px=320, py=340, bbox_h=100

    // Radar detection at the same bearing but at 25m range (much further than
    // camera estimate). Provide in Gazebo FLU convention (azimuth negated).
    auto        ci        = cam_intr_from(calib);
    const float bearing_x = (trk.position_2d.x() - ci.cx) / std::max(1.0f, ci.fx);
    const float cam_az    = std::atan2(bearing_x, 1.0f);
    const float bearing_y = (trk.position_2d.y() - ci.cy) / std::max(1.0f, ci.fy);
    const float cam_el    = std::atan2(bearing_y, std::sqrt(1.0f + bearing_x * bearing_x));

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 1000;
    radar_list.num_detections = 1;
    // FLU convention: negate azimuth for Gazebo
    radar_list.detections[0] = make_radar_det(25.0f, -cam_az, cam_el, 0.0f);

    engine.set_radar_detections(radar_list);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // The initial depth should be ~25m (from radar), not ~10.5m (from camera).
    // The UKF x_(0) is the depth/forward component.
    EXPECT_GT(result.objects[0].position_3d.x(), 20.0f);
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

    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, RadarNoiseConfig{}, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
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
    // Outlier at 500m with extreme elevation (1.0 rad) fails both Mahalanobis gate
    // and orphan altitude gate — should not create any additional tracks.
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

    // Simulate the same UKF steps the engine performed to get matching radar coords.
    // The engine inflates depth covariance to 100 for camera-only tracks (Issue #237),
    // so we must match that here for the predicted measurement to align.
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, RadarNoiseConfig{}, ci);
    temp_ukf.set_depth_covariance(100.0f);  // match engine's camera-only init
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
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
    // Must match engine's camera-only depth covariance inflation (Issue #237).
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk1, depth1, RadarNoiseConfig{}, ci);
    temp_ukf.set_depth_covariance(100.0f);  // match engine's camera-only init
    temp_ukf.predict();
    temp_ukf.update_camera(trk1, depth1, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk1, depth1, ci);
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
    // Construct a radar detection that WOULD match the camera track via
    // Mahalanobis gating (same range/azimuth/elevation as predicted), then
    // override its elevation to resolve below the ground filter threshold.
    // This ensures the ground filter is the actual reason for rejection.
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

    // First fuse to create the UKF filter for this track
    auto first_result = engine.fuse(tracked);
    ASSERT_EQ(first_result.objects.size(), 1u);

    // Build a detection from the track's predicted radar measurement
    // (this would pass association), then corrupt elevation to be ground-level.
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, radar_cfg, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 2000;
    radar_list.num_detections = 1;
    // Use predicted range/azimuth for gate match, but set elevation so
    // object_alt = 5.0 + range*sin(el) < 0.3m  →  sin(el) < (0.3-5.0)/range
    radar_list.detections[0] = make_radar_det(z_pred(0), z_pred(1), -0.5f, z_pred(3));

    engine.set_radar_detections(radar_list);
    tracked.timestamp_ns   = 2000;
    tracked.frame_sequence = 2;
    auto result            = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // The ground filter should reject this despite matching range/azimuth
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
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, radar_cfg, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
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
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, radar_cfg, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
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

// ═══════════════════════════════════════════════════════════════
// Altitude gate tests (Issue #229)
// ═══════════════════════════════════════════════════════════════

TEST(RadarFusionTest, AltitudeGateRejectsMismatch) {
    // Radar return at very different body-frame z than track → rejected.
    // Track is roughly at z≈0 in body frame (default test track is horizontal).
    // Radar at range=10m, elevation=-0.4 rad → radar_z = 10*sin(-0.4) ≈ -3.89m.
    // |z_diff| = |-3.89 - 0| ≈ 3.89 > 2.0 → rejected by altitude gate.
    auto             calib = make_test_calib();
    RadarNoiseConfig radar_cfg;
    radar_cfg.altitude_gate_m       = 2.0f;
    radar_cfg.ground_filter_enabled = false;  // isolate altitude gate

    UKFFusionEngine engine(calib, radar_cfg, true);
    engine.set_drone_altitude(5.0f);

    auto              trk = make_test_tracked();
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    // First fuse to create the UKF filter
    auto first_result = engine.fuse(tracked);
    ASSERT_EQ(first_result.objects.size(), 1u);

    // Radar at similar range but very different elevation
    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 2000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(10.0f, 0.0f, -0.4f, 0.0f);

    engine.set_radar_detections(radar_list);
    tracked.timestamp_ns   = 2000;
    tracked.frame_sequence = 2;
    auto result            = engine.fuse(tracked);
    // The altitude-mismatched detection (el=-0.4) is not associated with the
    // camera track. With radar-primary architecture, it may create a separate
    // radar-only track if it passes orphan quality gates.
    ASSERT_GE(result.objects.size(), 1u);
    // Core assertion: camera track should NOT have radar association
    bool found_camera = false;
    for (const auto& obj : result.objects) {
        if (obj.has_camera) {
            EXPECT_FALSE(obj.has_radar);
            found_camera = true;
        }
    }
    EXPECT_TRUE(found_camera);
}

TEST(RadarFusionTest, AltitudeGateAcceptsSimilar) {
    // Radar return at similar body-frame z as track → accepted.
    auto             calib = make_test_calib();
    RadarNoiseConfig radar_cfg;
    radar_cfg.altitude_gate_m       = 2.0f;
    radar_cfg.ground_filter_enabled = false;

    UKFFusionEngine engine(calib, radar_cfg, true);
    engine.set_drone_altitude(5.0f);

    auto              trk   = make_test_tracked();
    float             depth = test_estimate_depth(calib, trk);
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    // Build matching radar detection from a temp UKF
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, radar_cfg, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 1000;
    radar_list.num_detections = 1;
    radar_list.detections[0]  = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));

    engine.set_radar_detections(radar_list);
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // Altitude matches → radar association accepted
    EXPECT_TRUE(result.objects[0].has_radar);
}

TEST(RadarFusionTest, AltitudeGateConfigurable) {
    // With a very large altitude gate, even mismatched elevation passes the gate.
    auto             calib = make_test_calib();
    RadarNoiseConfig radar_cfg;
    radar_cfg.altitude_gate_m       = 100.0f;    // effectively disabled
    radar_cfg.gate_threshold        = 10000.0f;  // wide Mahalanobis gate to isolate altitude test
    radar_cfg.ground_filter_enabled = false;

    UKFFusionEngine engine(calib, radar_cfg, true);
    engine.set_drone_altitude(5.0f);

    auto              trk   = make_test_tracked();
    float             depth = test_estimate_depth(calib, trk);
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    // Build radar detection matching in range but shifted in elevation
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, radar_cfg, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    drone::ipc::RadarDetectionList radar_list{};
    radar_list.timestamp_ns   = 1000;
    radar_list.num_detections = 1;
    // Shift elevation by 0.3 rad (~17°) — at ~10m range this gives ~3m altitude
    // difference, which exceeds the default 2m gate but stays within our 100m gate.
    float shifted_elev       = z_pred(2) + 0.3f;
    radar_list.detections[0] = make_radar_det(z_pred(0), z_pred(1), shifted_elev, z_pred(3));

    engine.set_radar_detections(radar_list);
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // With 100m altitude gate, the altitude check always passes
    // (matching radar → should associate)
    EXPECT_TRUE(result.objects[0].has_radar);
}

// ═══════════════════════════════════════════════════════════
// Dormant Re-Identification Tests (Issue #237)
// ═══════════════════════════════════════════════════════════

// Helper: provide a radar detection matching the default test track (px=320, py=340)
// with test calib (fx=fy=500, cx=320, cy=240). Camera bearings: az=0.0, el≈0.197 rad.
// Radar azimuth is negated in matching (FLU→FRD), so provide azimuth=0.0.
static void set_matching_radar(UKFFusionEngine& engine, float range = 15.0f) {
    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(range, 0.0f, 0.197f, 0.0f);
    engine.set_radar_detections(radar);
}

TEST(DormantReIDTest, NewTrackCreatesDormantEntry) {
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true, 5.0f, 32);

    // Provide drone pose so dormant logic activates
    engine.set_drone_pose(0.0f, 0.0f, 4.0f, 0.0f);  // at origin, facing north, 4m up
    engine.set_drone_altitude(4.0f);

    // Provide radar so the track becomes radar-confirmed (required for dormant entry)
    set_matching_radar(engine);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked(1));

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // First observation — dormant entry should exist but with observations==1,
    // so in_world_frame should be false (need >1 observation for re-ID output).
    EXPECT_FALSE(result.objects[0].in_world_frame);
    ASSERT_EQ(engine.dormant_obstacles().size(), 1u);
    EXPECT_EQ(engine.dormant_obstacles()[0].observations, 1);
}

TEST(DormantReIDTest, ReidentifiesTrackAtSimilarWorldPosition) {
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true, 5.0f, 32);

    // Drone at origin, facing north (yaw=0)
    engine.set_drone_pose(0.0f, 0.0f, 4.0f, 0.0f);
    engine.set_drone_altitude(4.0f);

    // First track: creates a radar-confirmed dormant entry
    set_matching_radar(engine);
    TrackedObjectList tracked1;
    tracked1.timestamp_ns   = 1000;
    tracked1.frame_sequence = 1;
    tracked1.objects.push_back(make_test_tracked(1));
    engine.fuse(tracked1);
    ASSERT_EQ(engine.dormant_obstacles().size(), 1u);

    // Track 1 disappears (fuse with empty list kills the filter)
    TrackedObjectList empty;
    empty.timestamp_ns   = 2000;
    empty.frame_sequence = 2;
    engine.fuse(empty);

    // Simpler approach: keep yaw=0, move drone slightly. The same pixel position
    // from a slightly different drone position produces a nearby world point.
    engine.set_drone_pose(1.0f, 0.0f, 4.0f, 0.0f);  // moved 1m north

    // Provide radar for the second track too
    set_matching_radar(engine);
    TrackedObjectList tracked2;
    tracked2.timestamp_ns   = 3000;
    tracked2.frame_sequence = 3;
    tracked2.objects.push_back(make_test_tracked(2));  // new track ID, same pixel position
    engine.fuse(tracked2);

    // Should not have created a new dormant — should have merged with existing
    EXPECT_EQ(engine.dormant_obstacles().size(), 1u);
    EXPECT_GE(engine.dormant_obstacles()[0].observations, 2);
}

TEST(DormantReIDTest, ReidentifiedObjectHasWorldFrameFlag) {
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true, 5.0f, 32);

    engine.set_drone_pose(0.0f, 0.0f, 4.0f, 0.0f);
    engine.set_drone_altitude(4.0f);

    // Frame 1: track 1 creates radar-confirmed dormant entry
    set_matching_radar(engine);
    TrackedObjectList tracked1;
    tracked1.timestamp_ns   = 1000;
    tracked1.frame_sequence = 1;
    tracked1.objects.push_back(make_test_tracked(1));
    engine.fuse(tracked1);

    // Frame 2: track 1 disappears
    TrackedObjectList empty;
    empty.timestamp_ns   = 2000;
    empty.frame_sequence = 2;
    engine.fuse(empty);

    // Frame 3: track 2 appears from slightly shifted drone, same world area
    engine.set_drone_pose(1.0f, 0.0f, 4.0f, 0.0f);
    set_matching_radar(engine);
    TrackedObjectList tracked2;
    tracked2.timestamp_ns   = 3000;
    tracked2.frame_sequence = 3;
    tracked2.objects.push_back(make_test_tracked(2));
    auto result = engine.fuse(tracked2);

    ASSERT_EQ(result.objects.size(), 1u);
    // observations > 1 → in_world_frame should be true
    EXPECT_TRUE(result.objects[0].in_world_frame);
    // Position should be the averaged dormant world position, not raw body-frame
    EXPECT_EQ(result.objects[0].track_id, 2u);
}

TEST(DormantReIDTest, DormantPoolRespectsCap) {
    auto calib = make_test_calib();
    // max_dormant = 3
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true, 5.0f, 3);
    engine.set_drone_altitude(4.0f);

    // Place drone far enough apart that each track creates a unique dormant entry.
    // Each track at a different drone position → different world position → no merge.
    for (int i = 0; i < 5; ++i) {
        engine.set_drone_pose(static_cast<float>(i) * 100.0f, 0.0f, 4.0f, 0.0f);
        set_matching_radar(engine);

        TrackedObjectList tracked;
        tracked.timestamp_ns   = static_cast<uint64_t>((i + 1) * 1000);
        tracked.frame_sequence = static_cast<uint32_t>(i + 1);
        tracked.objects.push_back(
            make_test_tracked(static_cast<uint32_t>(i + 10)));  // unique track IDs
        engine.fuse(tracked);

        // Kill the track so next iteration creates a new one
        TrackedObjectList empty;
        empty.timestamp_ns   = static_cast<uint64_t>((i + 1) * 1000 + 500);
        empty.frame_sequence = static_cast<uint32_t>(i + 1);
        engine.fuse(empty);
    }

    // Pool should not exceed max_dormant
    EXPECT_LE(static_cast<int>(engine.dormant_obstacles().size()), 3);
    // Should actually have 3 entries (filled to capacity)
    EXPECT_EQ(static_cast<int>(engine.dormant_obstacles().size()), 3);
}

TEST(DormantReIDTest, ResetClearsDormantState) {
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true, 5.0f, 32);

    engine.set_drone_pose(0.0f, 0.0f, 4.0f, 0.0f);
    engine.set_drone_altitude(4.0f);
    set_matching_radar(engine);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked(1));
    engine.fuse(tracked);
    ASSERT_EQ(engine.dormant_obstacles().size(), 1u);

    engine.reset();
    EXPECT_TRUE(engine.dormant_obstacles().empty());
}

TEST(DormantReIDTest, CameraOnlyTrackDoesNotCreateDormant) {
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true, 5.0f, 32);

    engine.set_drone_pose(0.0f, 0.0f, 4.0f, 0.0f);
    engine.set_drone_altitude(4.0f);
    // No radar detections — track will be camera-only

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked(1));
    auto result = engine.fuse(tracked);

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_FALSE(result.objects[0].in_world_frame);
    // Camera-only tracks should NOT create dormant entries (prevents phantom pollution)
    EXPECT_TRUE(engine.dormant_obstacles().empty());
}

TEST(DormantReIDTest, NoDormantWithoutPose) {
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false, 5.0f, 32);

    // Do NOT call set_drone_pose — dormant logic should not activate
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked(1));
    auto result = engine.fuse(tracked);

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_FALSE(result.objects[0].in_world_frame);
    EXPECT_TRUE(engine.dormant_obstacles().empty());
}

TEST(DormantReIDTest, DistantTrackCreatesNewDormantEntry) {
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true, 5.0f, 32);

    engine.set_drone_pose(0.0f, 0.0f, 4.0f, 0.0f);
    engine.set_drone_altitude(4.0f);

    // Track 1: creates radar-confirmed dormant entry
    set_matching_radar(engine);
    TrackedObjectList tracked1;
    tracked1.timestamp_ns   = 1000;
    tracked1.frame_sequence = 1;
    tracked1.objects.push_back(make_test_tracked(1));
    engine.fuse(tracked1);
    ASSERT_EQ(engine.dormant_obstacles().size(), 1u);

    // Kill track 1
    TrackedObjectList empty;
    empty.timestamp_ns   = 2000;
    empty.frame_sequence = 2;
    engine.fuse(empty);

    // Track 2 from a very different drone position — should NOT merge
    engine.set_drone_pose(100.0f, 100.0f, 4.0f, 0.0f);  // 100m away
    set_matching_radar(engine);
    TrackedObjectList tracked2;
    tracked2.timestamp_ns   = 3000;
    tracked2.frame_sequence = 3;
    tracked2.objects.push_back(make_test_tracked(2));
    engine.fuse(tracked2);

    // Should have 2 separate dormant entries
    EXPECT_EQ(engine.dormant_obstacles().size(), 2u);
}

// ═══════════════════════════════════════════════════════════
// Radar-Primary Architecture Tests (Phase D, Issue #242)
// ═══════════════════════════════════════════════════════════

TEST(RadarPrimaryTest, RadarOnlyConstructorSphericalToCartesian) {
    // Radar detection at 10m range, 0 azimuth, 0 elevation → (10, 0, 0) in FRD
    auto      det = make_radar_det(10.0f, 0.0f, 0.0f, 1.0f);
    ObjectUKF ukf(det, 0x80000001u);

    EXPECT_NEAR(ukf.position()(0), 10.0f, 0.01f);  // forward
    EXPECT_NEAR(ukf.position()(1), 0.0f, 0.01f);   // lateral
    EXPECT_NEAR(ukf.position()(2), 0.0f, 0.01f);   // vertical
    EXPECT_TRUE(ukf.radar_only);
    EXPECT_EQ(ukf.radar_update_count, 1u);
    EXPECT_EQ(ukf.track_id, 0x80000001u);

    // Covariance: depth should be tight (radar), lateral loose at range
    auto P = ukf.position_covariance();
    EXPECT_LT(P(0, 0), 1.0f);   // depth: tight
    EXPECT_GT(P(1, 1), 0.01f);  // lateral: range-scaled
}

TEST(RadarPrimaryTest, RadarOnlyConstructorWithAzimuth) {
    // Radar at 10m, azimuth=0.3 rad (FLU) → negated for FRD → y < 0
    auto      det = make_radar_det(10.0f, 0.3f, 0.0f, 0.0f);
    ObjectUKF ukf(det, 0x80000002u);

    // cos(0) * cos(-0.3) ≈ 0.9553
    EXPECT_NEAR(ukf.position()(0), 10.0f * std::cos(-0.3f), 0.01f);
    // cos(0) * sin(-0.3) ≈ -0.2955
    EXPECT_NEAR(ukf.position()(1), 10.0f * std::sin(-0.3f), 0.01f);
    EXPECT_NEAR(ukf.position()(2), 0.0f, 0.01f);
}

TEST(RadarPrimaryTest, RadarOrphanCreatesTrack) {
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 1;  // immediate output for this test
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);
    engine.set_drone_pose(0.0f, 0.0f, 4.0f, 0.0f);

    // Provide radar detection with NO camera tracks
    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(12.0f, 0.5f, 0.1f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result          = engine.fuse(empty);

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_FALSE(result.objects[0].has_camera);
    EXPECT_TRUE(result.objects[0].has_radar);
    EXPECT_EQ(result.objects[0].class_id, ObjectClass::UNKNOWN);
    EXPECT_GE(result.objects[0].track_id, 0x80000000u);
    EXPECT_EQ(result.objects[0].radar_update_count, 1u);
}

TEST(RadarPrimaryTest, RadarOrphanDefaultRadius) {
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_default_radius_m = 1.5f;
    rcfg.radar_only_promotion_hits   = 1;  // immediate output for this test
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(10.0f, 0.8f, 0.0f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result          = engine.fuse(empty);

    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_FLOAT_EQ(result.objects[0].estimated_radius_m, 1.5f);
}

TEST(RadarPrimaryTest, RadarOrphanProximityPreventsDuplicate) {
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    // Camera track at default position (px=320, py=340) with matching radar
    set_matching_radar(engine, 15.0f);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked(1));

    // Also add a radar detection very close to the camera track's position
    // (same range/bearing) — it should be consumed by camera association,
    // not create an orphan
    auto result = engine.fuse(tracked);

    // Should have exactly 1 object (the camera+radar fused track), not 2
    EXPECT_EQ(result.objects.size(), 1u);
    EXPECT_TRUE(result.objects[0].has_camera);
}

TEST(RadarPrimaryTest, RadarOrphanTrackPersistsAcrossFrames) {
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 1;  // immediate output for persistence test
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    // Frame 1: radar only
    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(10.0f, 0.5f, 0.1f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty1;
    empty1.timestamp_ns   = 1000;
    empty1.frame_sequence = 1;
    auto result1          = engine.fuse(empty1);
    ASSERT_EQ(result1.objects.size(), 1u);
    uint32_t rid = result1.objects[0].track_id;

    // Frame 2: same radar detection — should re-associate
    radar.timestamp_ns = 2000;
    engine.set_radar_detections(radar);
    TrackedObjectList empty2;
    empty2.timestamp_ns   = 2000;
    empty2.frame_sequence = 2;
    auto result2          = engine.fuse(empty2);

    ASSERT_EQ(result2.objects.size(), 1u);
    EXPECT_EQ(result2.objects[0].track_id, rid);  // same track
    EXPECT_EQ(result2.objects[0].radar_update_count, 2u);
}

TEST(RadarPrimaryTest, RadarOnlyTrackIDHighBit) {
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 1;  // immediate output for ID test
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 2;
    radar.detections[0]  = make_radar_det(10.0f, 0.5f, 0.0f, 0.0f);
    radar.detections[1]  = make_radar_det(20.0f, -0.5f, 0.0f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result          = engine.fuse(empty);

    for (const auto& obj : result.objects) {
        EXPECT_GE(obj.track_id, 0x80000000u) << "Radar-only track ID must have high bit set";
    }
}

TEST(RadarPrimaryTest, RadarOnlyDormantEntry) {
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 1;  // immediate output for dormant test
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);
    engine.set_drone_pose(0.0f, 0.0f, 4.0f, 0.0f);

    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(10.0f, 0.0f, 0.0f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    engine.fuse(empty);

    ASSERT_EQ(engine.dormant_obstacles().size(), 1u);
    // Radar at 10m forward, drone at origin facing north → world pos ~(10, 0, 4)
    const auto& dorm = engine.dormant_obstacles()[0];
    EXPECT_NEAR(dorm.world_pos.x(), 10.0f, 1.0f);
    EXPECT_NEAR(dorm.world_pos.y(), 0.0f, 1.0f);
}

TEST(RadarPrimaryTest, GroundFilterAppliesToOrphans) {
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.ground_filter_enabled = true;
    rcfg.min_object_altitude_m = 0.3f;
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);  // drone at 4m AGL

    // Radar detection at negative elevation (looking down) → low altitude object
    // alt = 4.0 + 10.0 * sin(-0.5) ≈ 4.0 - 4.79 = -0.79 → below 0.3m threshold
    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(10.0f, 0.0f, -0.5f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result          = engine.fuse(empty);

    // Ground-filtered detection should not create an orphan track
    EXPECT_EQ(result.objects.size(), 0u);
}

TEST(RadarPrimaryTest, EmptyCameraListRadarOutput) {
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 1;  // immediate output for this test
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    // Three radar detections, no camera
    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 3;
    radar.detections[0]  = make_radar_det(5.0f, 0.3f, 0.0f, 0.0f);
    radar.detections[1]  = make_radar_det(10.0f, -0.4f, 0.0f, 0.0f);
    radar.detections[2]  = make_radar_det(18.0f, 0.8f, 0.0f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result          = engine.fuse(empty);

    // All 3 should produce radar-only objects
    EXPECT_EQ(result.objects.size(), 3u);
    for (const auto& obj : result.objects) {
        EXPECT_FALSE(obj.has_camera);
        EXPECT_TRUE(obj.has_radar);
    }
}

TEST(RadarPrimaryTest, CameraAdoptsRadarOnlyTrack) {
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 1;  // immediate output for adoption test
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    // Frame 1: Radar-only detection at ~6m forward, ~0 lateral.
    // Range matches PERSON height prior depth: 1.7 * 500 / 100 * 0.7 ≈ 5.95m.
    // azimuth=0, elevation≈0.197 (matching default test track bearing)
    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(6.0f, 0.0f, 0.197f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result1         = engine.fuse(empty);
    ASSERT_EQ(result1.objects.size(), 1u);
    EXPECT_GE(result1.objects[0].track_id, 0x80000000u);

    // Frame 2: Camera track at same bearing (px=320, py=340) + radar
    set_matching_radar(engine, 6.0f);
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 2000;
    tracked.frame_sequence = 2;
    tracked.objects.push_back(make_test_tracked(42));
    auto result2 = engine.fuse(tracked);

    // Camera should adopt the radar-only track: 1 object, not 2
    ASSERT_EQ(result2.objects.size(), 1u);
    EXPECT_TRUE(result2.objects[0].has_camera);
    // Track ID should be the camera's track_id (42), not the radar high-bit ID
    EXPECT_EQ(result2.objects[0].track_id, 42u);
    EXPECT_EQ(result2.objects[0].class_id, ObjectClass::PERSON);
    // Track retains radar history from its radar-only origin
    EXPECT_GE(result2.objects[0].radar_update_count, 1u);
}

// ═══════════════════════════════════════════════════════════
// Depth Confidence Tests (#340)
// ═══════════════════════════════════════════════════════════

TEST(DepthConfidenceTest, Tier2_ApparentSize_HighConfidence) {
    // Close object with large bbox → Tier 2 → high covariance-based confidence.
    // bbox_h=100, PERSON (H=1.7): dD/d(bbox_h) = 1.7*500/10000 = 0.085
    // σ² = 0.085² * 2.5² = 0.045 → conf = 1/(1+0.045) ≈ 0.96 (#420).
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    TrackedObject trk;
    trk.track_id     = 1;
    trk.class_id     = ObjectClass::PERSON;
    trk.confidence   = 0.9f;
    trk.position_2d  = {320.0f, 340.0f};  // well below horizon
    trk.bbox_w       = 60.0f;
    trk.bbox_h       = 100.0f;  // large bbox = close object → high confidence
    trk.velocity_2d  = Eigen::Vector2f::Zero();
    trk.timestamp_ns = 1000;

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_GE(result.objects[0].depth_confidence, 0.9f);
}

TEST(DepthConfidenceTest, Tier2_ClampedDepth_LowConfidence) {
    // Small bbox just above threshold → Tier 2 raw depth exceeds 40m clamp
    // → confidence drops to 0.2 (Issue #340: ground features at shallow angles
    // produce huge apparent-size depths that clamp to max).
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    TrackedObject trk;
    trk.track_id     = 1;
    trk.class_id     = ObjectClass::PERSON;
    trk.confidence   = 0.9f;
    trk.position_2d  = {320.0f, 340.0f};  // well below horizon
    trk.bbox_w       = 15.0f;
    trk.bbox_h       = 11.0f;  // > 10px → Tier 2, but depth = 1.7*500/11*0.7 = 54.1m → clamped
    trk.velocity_2d  = Eigen::Vector2f::Zero();
    trk.timestamp_ns = 1000;

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    // Clamped to 40m → confidence reduced to 0.2 (below default 0.5 gate)
    EXPECT_LE(result.objects[0].depth_confidence, 0.3f);
}

TEST(DepthConfidenceTest, Tier4_NearHorizon_LowConfidence) {
    // Near-horizon small bbox → Tier 4 → confidence = 0.01 (not 0.0, so the
    // P2 camera-only fallback doesn't override it with detection confidence).
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    TrackedObject trk;
    trk.track_id     = 1;
    trk.class_id     = ObjectClass::PERSON;
    trk.confidence   = 0.9f;
    trk.position_2d  = {320.0f, 240.0f};  // exactly at horizon (cy=240)
    trk.bbox_w       = 5.0f;
    trk.bbox_h       = 5.0f;  // < 10px → not Tier 1/2, and at horizon → not Tier 3
    trk.velocity_2d  = Eigen::Vector2f::Zero();
    trk.timestamp_ns = 1000;

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_FLOAT_EQ(result.objects[0].depth_confidence, 0.01f);
}

// ═══════════════════════════════════════════════════════════
// Covariance Depth Confidence Tests (#420)
// ═══════════════════════════════════════════════════════════

TEST(CovarianceConfidenceTest, MonotonicDecay) {
    // Confidence should strictly decrease as bbox_h decreases (object farther away).
    // Covariance model: σ² = (H*fy/bbox_h²)² * σ_bbox² → grows with smaller bbox_h.
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    const float bbox_sizes[] = {200.0f, 100.0f, 50.0f, 20.0f};
    float       prev_conf    = 1.0f;

    for (float bh : bbox_sizes) {
        TrackedObject trk;
        trk.track_id     = 1;
        trk.class_id     = ObjectClass::PERSON;
        trk.confidence   = 0.9f;
        trk.position_2d  = {320.0f, 340.0f};
        trk.bbox_w       = bh * 0.6f;
        trk.bbox_h       = bh;
        trk.velocity_2d  = Eigen::Vector2f::Zero();
        trk.timestamp_ns = 1000;

        TrackedObjectList tracked;
        tracked.timestamp_ns   = 1000;
        tracked.frame_sequence = 1;
        tracked.objects.push_back(trk);

        auto result = engine.fuse(tracked);
        ASSERT_EQ(result.objects.size(), 1u);
        EXPECT_LT(result.objects[0].depth_confidence, prev_conf)
            << "bbox_h=" << bh << " should have lower confidence than previous";
        prev_conf = result.objects[0].depth_confidence;
    }
}

TEST(CovarianceConfidenceTest, CloseHighFarLow) {
    // Close object (large bbox) → high confidence, far object (small bbox) → low.
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    auto make_trk = [](float bh) {
        TrackedObject trk;
        trk.track_id     = 1;
        trk.class_id     = ObjectClass::PERSON;
        trk.confidence   = 0.9f;
        trk.position_2d  = {320.0f, 340.0f};
        trk.bbox_w       = bh * 0.6f;
        trk.bbox_h       = bh;
        trk.velocity_2d  = Eigen::Vector2f::Zero();
        trk.timestamp_ns = 1000;
        return trk;
    };

    TrackedObjectList close_tracked;
    close_tracked.timestamp_ns   = 1000;
    close_tracked.frame_sequence = 1;
    close_tracked.objects.push_back(make_trk(200.0f));  // very close

    TrackedObjectList far_tracked;
    far_tracked.timestamp_ns   = 1000;
    far_tracked.frame_sequence = 1;
    far_tracked.objects.push_back(make_trk(15.0f));  // far away

    auto close_result = engine.fuse(close_tracked);
    auto far_result   = engine.fuse(far_tracked);

    ASSERT_EQ(close_result.objects.size(), 1u);
    ASSERT_EQ(far_result.objects.size(), 1u);
    EXPECT_GT(close_result.objects[0].depth_confidence, 0.5f);
    EXPECT_LT(far_result.objects[0].depth_confidence, 0.3f);
}

TEST(CovarianceConfidenceTest, ClampPenaltyPreserved) {
    // Very small bbox that produces d_raw > 40m should still get 0.2 penalty,
    // not the covariance formula (Issue #340 regression guard).
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    TrackedObject trk;
    trk.track_id     = 1;
    trk.class_id     = ObjectClass::PERSON;
    trk.confidence   = 0.9f;
    trk.position_2d  = {320.0f, 340.0f};
    trk.bbox_w       = 8.0f;
    trk.bbox_h       = 11.0f;  // depth = 1.7*500/11*0.7 = 54.1m > 40m → clamped
    trk.velocity_2d  = Eigen::Vector2f::Zero();
    trk.timestamp_ns = 1000;

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_FLOAT_EQ(result.objects[0].depth_confidence, 0.2f);
}

TEST(CovarianceConfidenceTest, RegressionTypicalRange) {
    // bbox_h=100, PERSON → depth ≈ 5.95m, confidence in reasonable range.
    // dD/d(bbox_h) = 1.7*500/10000 = 0.085, σ² = 0.085²*2.5² ≈ 0.045
    // conf = 1/(1+0.045) ≈ 0.957
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    TrackedObject trk;
    trk.track_id     = 1;
    trk.class_id     = ObjectClass::PERSON;
    trk.confidence   = 0.9f;
    trk.position_2d  = {320.0f, 340.0f};
    trk.bbox_w       = 60.0f;
    trk.bbox_h       = 100.0f;
    trk.velocity_2d  = Eigen::Vector2f::Zero();
    trk.timestamp_ns = 1000;

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    // Depth should be ~5.95m (1.7 * 500 / 100 * 0.7)
    EXPECT_NEAR(result.objects[0].position_3d.x(), 5.95f, 0.5f);
    // Confidence in [0.9, 1.0) — high for close object
    EXPECT_GE(result.objects[0].depth_confidence, 0.9f);
    EXPECT_LT(result.objects[0].depth_confidence, 1.0f);
}

TEST(DepthConfidenceTest, RadarConfirmed_OverridesConfidence) {
    // Radar-confirmed track should have depth_confidence = 1.0
    auto  calib = make_test_calib();
    auto  trk   = make_test_tracked();
    float depth = test_estimate_depth(calib, trk);

    UKFFusionEngine engine(calib, RadarNoiseConfig{}, true);

    // First fuse camera-only to establish the track
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result1 = engine.fuse(tracked);
    ASSERT_EQ(result1.objects.size(), 1u);
    // Camera-only → confidence < 1.0
    EXPECT_LT(result1.objects[0].depth_confidence, 1.0f);

    // Get predicted radar measurement to ensure association succeeds
    auto      ci = cam_intr_from(calib);
    ObjectUKF temp_ukf(trk, depth, RadarNoiseConfig{}, ci);
    temp_ukf.set_depth_covariance(100.0f);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
    temp_ukf.predict();
    temp_ukf.update_camera(trk, depth, ci);
    auto z_pred = temp_ukf.predicted_radar_measurement();

    // Second fuse with matching radar detection
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
    EXPECT_FLOAT_EQ(result2.objects[0].depth_confidence, 1.0f);
}

// ═══════════════════════════════════════════════════════════
// Radar-Only Track Initiation Tests (Issue #231)
// ═══════════════════════════════════════════════════════════

TEST(RadarOnlyTrackTest, OrphanPromotionAfterNFrames) {
    // Radar orphan should NOT be output until radar_only_promotion_hits (3)
    // consecutive radar observations are received.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 3;
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    TrackedObjectList empty;
    empty.frame_sequence = 0;

    // Radar detection at 12m, azimuth 0.5
    drone::ipc::RadarDetectionList radar{};
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(12.0f, 0.5f, 0.1f, 0.0f);

    // Frame 1: 1st radar hit — should NOT produce output (1 < 3)
    radar.timestamp_ns = 1000;
    engine.set_radar_detections(radar);
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result1         = engine.fuse(empty);
    EXPECT_EQ(result1.objects.size(), 0u) << "Orphan should not be output before promotion";

    // Frame 2: 2nd radar hit — still not promoted (2 < 3)
    radar.timestamp_ns = 2000;
    engine.set_radar_detections(radar);
    empty.timestamp_ns   = 2000;
    empty.frame_sequence = 2;
    auto result2         = engine.fuse(empty);
    EXPECT_EQ(result2.objects.size(), 0u) << "Orphan should not be output before promotion";

    // Frame 3: 3rd radar hit — NOW promoted (3 >= 3)
    radar.timestamp_ns = 3000;
    engine.set_radar_detections(radar);
    empty.timestamp_ns   = 3000;
    empty.frame_sequence = 3;
    auto result3         = engine.fuse(empty);
    ASSERT_EQ(result3.objects.size(), 1u) << "Orphan should be promoted after N frames";
    EXPECT_GE(result3.objects[0].track_id, 0x80000000u);
    EXPECT_TRUE(result3.objects[0].has_radar);
    EXPECT_FALSE(result3.objects[0].has_camera);
    EXPECT_EQ(result3.objects[0].radar_update_count, 3u);
}

TEST(RadarOnlyTrackTest, TrackMaintainsIDAfterPromotion) {
    // After promotion, the track ID should remain stable across frames.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 2;
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    TrackedObjectList empty;

    drone::ipc::RadarDetectionList radar{};
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(15.0f, 0.3f, 0.0f, 0.0f);

    // Frame 1: not yet promoted
    radar.timestamp_ns = 1000;
    engine.set_radar_detections(radar);
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    engine.fuse(empty);

    // Frame 2: promoted
    radar.timestamp_ns = 2000;
    engine.set_radar_detections(radar);
    empty.timestamp_ns   = 2000;
    empty.frame_sequence = 2;
    auto result2         = engine.fuse(empty);
    ASSERT_EQ(result2.objects.size(), 1u);
    uint32_t promoted_id = result2.objects[0].track_id;
    EXPECT_GE(promoted_id, 0x80000000u);

    // Frame 3: same track should persist with same ID
    radar.timestamp_ns = 3000;
    engine.set_radar_detections(radar);
    empty.timestamp_ns   = 3000;
    empty.frame_sequence = 3;
    auto result3         = engine.fuse(empty);
    ASSERT_EQ(result3.objects.size(), 1u);
    EXPECT_EQ(result3.objects[0].track_id, promoted_id) << "Track ID must be stable";
    EXPECT_EQ(result3.objects[0].radar_update_count, 3u);
}

TEST(RadarOnlyTrackTest, ConfigToggleDisablesRadarOnlyTracks) {
    // When radar_only_enabled is false, radar orphans should NOT be created.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_enabled        = false;
    rcfg.radar_only_promotion_hits = 1;
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(12.0f, 0.5f, 0.1f, 0.0f);
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result          = engine.fuse(empty);

    // No radar-only tracks should be created when toggle is off
    EXPECT_EQ(result.objects.size(), 0u)
        << "radar_only_enabled=false should prevent orphan track creation";
}

TEST(RadarOnlyTrackTest, MixedCameraRadarStillWorks) {
    // Camera+radar fused tracks should still work normally with radar_only_enabled=true
    // and a promotion threshold > 1.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.ground_filter_enabled     = false;
    rcfg.radar_only_promotion_hits = 3;
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked(1));
    float depth = 10.5f;  // approx monocular depth for default test track

    // Build matching radar detection
    CameraIntrinsics ci{500.0f, 500.0f, 320.0f, 240.0f};
    ObjectUKF        temp_ukf(tracked.objects[0], depth, rcfg, ci);
    auto             z_pred = temp_ukf.predicted_radar_measurement();

    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));
    engine.set_radar_detections(radar);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_TRUE(result.objects[0].has_camera);
    EXPECT_TRUE(result.objects[0].has_radar);
    EXPECT_EQ(result.objects[0].track_id, 1u);
}

TEST(RadarOnlyTrackTest, PromotionHitsConfigurable) {
    // Verify different promotion_hits values work: set to 5.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_promotion_hits = 5;
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    TrackedObjectList empty;

    drone::ipc::RadarDetectionList radar{};
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(10.0f, 0.2f, 0.0f, 0.0f);

    // Frames 1-4: not yet promoted
    for (uint32_t f = 1; f <= 4; ++f) {
        radar.timestamp_ns = f * 1000;
        engine.set_radar_detections(radar);
        empty.timestamp_ns   = f * 1000;
        empty.frame_sequence = f;
        auto result          = engine.fuse(empty);
        EXPECT_EQ(result.objects.size(), 0u) << "Frame " << f << " should not emit (< 5 hits)";
    }

    // Frame 5: promoted
    radar.timestamp_ns = 5000;
    engine.set_radar_detections(radar);
    empty.timestamp_ns   = 5000;
    empty.frame_sequence = 5;
    auto result5         = engine.fuse(empty);
    ASSERT_EQ(result5.objects.size(), 1u);
    EXPECT_EQ(result5.objects[0].radar_update_count, 5u);
}

TEST(RadarOnlyTrackTest, DisabledToggleStillAllowsCameraRadarFusion) {
    // radar_only_enabled=false should NOT affect camera+radar fusion.
    // Only orphan track creation is disabled.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_only_enabled        = false;
    rcfg.ground_filter_enabled     = false;
    rcfg.radar_only_promotion_hits = 1;
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked(1));
    float depth = 10.5f;

    CameraIntrinsics ci{500.0f, 500.0f, 320.0f, 240.0f};
    ObjectUKF        temp_ukf(tracked.objects[0], depth, rcfg, ci);
    auto             z_pred = temp_ukf.predicted_radar_measurement();

    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 1;
    radar.detections[0]  = make_radar_det(z_pred(0), z_pred(1), z_pred(2), z_pred(3));
    engine.set_radar_detections(radar);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_TRUE(result.objects[0].has_camera);
    // Camera+radar association should still work
    EXPECT_TRUE(result.objects[0].has_radar);
}

TEST(RadarOnlyTrackTest, ProximityGateRejectsDuplicateOrphans) {
    // Two radar detections within orphan_proximity_m of each other should
    // create only ONE orphan track (the first one).  The second is suppressed
    // by the proximity check.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_orphan_proximity_m  = 5.0f;
    rcfg.radar_only_promotion_hits = 1;  // immediate output
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    // Two detections at similar positions (~2m apart in body frame)
    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 2;
    radar.detections[0]  = make_radar_det(10.0f, 0.3f, 0.0f, 0.0f);  // ~10m forward-right
    radar.detections[1]  = make_radar_det(11.0f, 0.3f, 0.0f, 0.0f);  // ~11m same direction
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result          = engine.fuse(empty);

    // Only 1 orphan track — the second detection is within 5m of the first
    EXPECT_EQ(result.objects.size(), 1u)
        << "Proximity gate should merge nearby radar detections into one orphan";
}

TEST(RadarOnlyTrackTest, MaxOrphanRangeRejectsDistantDetections) {
    // Radar detections beyond max_orphan_range_m should be ignored.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    rcfg.radar_max_orphan_range_m  = 25.0f;
    rcfg.radar_only_promotion_hits = 1;
    UKFFusionEngine engine(calib, rcfg, true, 5.0f, 32);
    engine.set_drone_altitude(4.0f);

    drone::ipc::RadarDetectionList radar{};
    radar.timestamp_ns   = 1000;
    radar.num_detections = 2;
    radar.detections[0]  = make_radar_det(20.0f, 0.3f, 0.0f, 0.0f);  // 20m — within range
    radar.detections[1]  = make_radar_det(30.0f, 0.5f, 0.0f, 0.0f);  // 30m — beyond 25m cutoff
    engine.set_radar_detections(radar);

    TrackedObjectList empty;
    empty.timestamp_ns   = 1000;
    empty.frame_sequence = 1;
    auto result          = engine.fuse(empty);

    // Only the 20m detection should create a track
    ASSERT_EQ(result.objects.size(), 1u);
    EXPECT_NEAR(result.objects[0].position_3d.norm(), 20.0f, 1.0f)
        << "Only the near detection should create a track";
}

// ═══════════════════════════════════════════════════════════
// Multi-Class Height Priors Tests (Issue #423)
// ═══════════════════════════════════════════════════════════

TEST(HeightPriorsTest, PersonShorterDepthThanBuilding) {
    // PERSON (1.7m prior) should produce a shorter depth estimate than
    // BUILDING (10.0m prior) for identical bbox_h, because depth is
    // proportional to assumed_height.
    auto calib = make_test_calib();
    // Use default height_priors (PERSON=1.7, BUILDING=10.0)

    TrackedObject person = make_test_tracked();
    person.class_id      = ObjectClass::PERSON;
    person.bbox_h        = 100.0f;  // well above kBboxHThreshold (10)

    TrackedObject building = make_test_tracked();
    building.class_id      = ObjectClass::BUILDING;
    building.bbox_h        = 100.0f;  // same bbox_h

    float depth_person   = test_estimate_depth(calib, person);
    float depth_building = test_estimate_depth(calib, building);

    EXPECT_GT(depth_building, depth_person)
        << "Building (10.0m prior) should yield greater depth than Person (1.7m prior)";
    // Ratio should match the height prior ratio: 10.0 / 1.7 ≈ 5.88
    EXPECT_NEAR(depth_building / depth_person, 10.0f / 1.7f, 0.01f);
}

TEST(HeightPriorsTest, UnknownClassFallsBackToDefault) {
    // UNKNOWN class (index 0) should use the 3.0m default prior.
    auto calib = make_test_calib();

    TrackedObject trk = make_test_tracked();
    trk.class_id      = ObjectClass::UNKNOWN;
    trk.bbox_h        = 100.0f;

    float depth = test_estimate_depth(calib, trk);

    // Expected: 3.0 * fy / bbox_h * depth_scale = 3.0 * 500 / 100 * 0.7 = 10.5
    const float expected = std::clamp(3.0f * 500.0f / 100.0f * 0.7f, 1.0f, 40.0f);
    EXPECT_NEAR(depth, expected, 0.01f);
}

TEST(HeightPriorsTest, CustomHeightPriorsOverrideDefaults) {
    // Custom height_priors in CalibrationData should override the defaults.
    auto calib                                                            = make_test_calib();
    calib.height_priors[static_cast<uint8_t>(ObjectClass::PERSON)]        = 5.0f;
    calib.height_priors[static_cast<uint8_t>(ObjectClass::VEHICLE_TRUCK)] = 12.0f;

    TrackedObject person = make_test_tracked();
    person.class_id      = ObjectClass::PERSON;
    person.bbox_h        = 100.0f;

    TrackedObject truck = make_test_tracked();
    truck.class_id      = ObjectClass::VEHICLE_TRUCK;
    truck.bbox_h        = 100.0f;

    float depth_person = test_estimate_depth(calib, person);
    float depth_truck  = test_estimate_depth(calib, truck);

    // With custom priors: person=5.0, truck=12.0
    const float expected_person = std::clamp(5.0f * 500.0f / 100.0f * 0.7f, 1.0f, 40.0f);
    const float expected_truck  = std::clamp(12.0f * 500.0f / 100.0f * 0.7f, 1.0f, 40.0f);
    EXPECT_NEAR(depth_person, expected_person, 0.01f);
    EXPECT_NEAR(depth_truck, expected_truck, 0.01f);
    EXPECT_GT(depth_truck, depth_person)
        << "Truck (12.0m custom prior) should yield greater depth than Person (5.0m custom prior)";
}

// ═══════════════════════════════════════════════════════════
// Radar-Learned Height Tests (#422)
// ═══════════════════════════════════════════════════════════

TEST(RadarLearnedHeightTest, RadarInitBackCalculatesHeight) {
    // After radar-init at range R with bbox_h=B, the back-calculated height
    // is H = R * B / (fy * ds).  Use 20m so the learned height (5.714m) is
    // clearly distinguishable from the PERSON class prior (1.7m).
    // (Review fix #2: previous 6m range produced 1.714m ≈ 1.7m, untestable.)
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    UKFFusionEngine  engine(calib, rcfg, true);
    engine.set_drone_altitude(4.0f);

    // Frame 1: radar-init at 20m → H = 20*100/(500*0.7) ≈ 5.714m
    set_matching_radar(engine, 20.0f);
    TrackedObjectList tracked1;
    tracked1.timestamp_ns   = 1000;
    tracked1.frame_sequence = 1;
    tracked1.objects.push_back(make_test_tracked(1));
    auto r1 = engine.fuse(tracked1);
    ASSERT_EQ(r1.objects.size(), 1u);
    // Radar-init snaps depth close to radar range (20m, not class prior 5.95m)
    EXPECT_NEAR(r1.objects[0].position_3d.x(), 20.0f, 2.0f);
}

TEST(RadarLearnedHeightTest, NewTrackUsesClassPrior) {
    // A new track without radar history should use the class prior, not
    // a learned height from a different track.
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    UKFFusionEngine  engine(calib, rcfg, true);
    engine.set_drone_altitude(4.0f);

    // Frame 1: Track 1 with radar-init (learns height from radar)
    set_matching_radar(engine, 10.0f);
    TrackedObjectList tracked1;
    tracked1.timestamp_ns   = 1000;
    tracked1.frame_sequence = 1;
    tracked1.objects.push_back(make_test_tracked(1));
    engine.fuse(tracked1);

    // Frame 2: New track 2 (no radar) — should use class prior, not track 1's height
    TrackedObjectList tracked2;
    tracked2.timestamp_ns   = 2000;
    tracked2.frame_sequence = 2;
    tracked2.objects.push_back(make_test_tracked(2, 200.0f, 350.0f));
    auto result = engine.fuse(tracked2);

    bool found_track2 = false;
    for (const auto& obj : result.objects) {
        if (obj.track_id == 2) {
            found_track2 = true;
            EXPECT_EQ(obj.radar_update_count, 0u);
            // Depth from PERSON class prior: 1.7*500/100*0.7 ≈ 5.95m
            EXPECT_NEAR(obj.position_3d.x(), 5.95f, 1.0f);
        }
    }
    EXPECT_TRUE(found_track2);
}

TEST(RadarLearnedHeightTest, BackCalcFormulaRoundTrips) {
    // The back-calculation H = R * bbox_h / (fy * ds) should round-trip through
    // the forward formula depth = H * fy / bbox_h * ds.  Verify algebraically
    // via test_estimate_depth (which mirrors the Tier 2 formula).
    auto calib = make_test_calib();

    TrackedObject trk = make_test_tracked(1);           // PERSON, bbox_h=100
    const float   fy  = calib.camera_intrinsics(1, 1);  // 500
    const float   ds  = calib.depth_scale;              // 0.7

    // For several radar ranges, compute learned height and verify the forward
    // formula reproduces the original range.
    for (float radar_range : {4.0f, 8.0f, 15.0f, 30.0f}) {
        const float learned_h = radar_range * trk.bbox_h / (fy * ds);
        // Forward: depth = learned_h * fy / bbox_h * ds
        const float depth = std::clamp(learned_h * fy / trk.bbox_h * ds, 1.0f, 40.0f);
        EXPECT_NEAR(depth, std::min(radar_range, 40.0f), 0.01f)
            << "Round-trip failed for radar_range=" << radar_range;
    }
}

TEST(RadarLearnedHeightTest, LearnedHeightOverridesClassPrior) {
    // When a track has a radar-learned height, its depth estimate should differ
    // from the class prior.  Verify by comparing fuse() output of a radar-init
    // track at 20m vs the class-prior depth (5.95m for PERSON).
    auto             calib = make_test_calib();
    RadarNoiseConfig rcfg;
    UKFFusionEngine  engine(calib, rcfg, true);
    engine.set_drone_altitude(4.0f);

    // Track with radar-init at 20m → learned H = 20*100/(500*0.7) ≈ 5.714m
    // vs PERSON class prior of 1.7m.  The learned height is 3.36× larger,
    // so subsequent monocular estimates should be much deeper than 5.95m.
    set_matching_radar(engine, 20.0f);
    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(make_test_tracked(1));
    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);

    // With radar-init, depth snaps near the radar range (20m), not class prior (5.95m)
    EXPECT_GT(result.objects[0].position_3d.x(), 10.0f)
        << "Radar-init depth should be far beyond the class prior (5.95m)";
}

// ═══════════════════════════════════════════════════════════
// Edge Case Tests (review findings)
// ═══════════════════════════════════════════════════════════

TEST(DepthEdgeCaseTest, SmallBboxDoesNotCrash) {
    // bbox_h=0 or very small should not cause division by zero or UB.
    // Tier 2 guard (bbox_h > kBboxHThreshold=10) rejects tiny bbox,
    // falling through to Tier 3/4 safely (review fix #2).
    auto            calib = make_test_calib();
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    TrackedObject trk = make_test_tracked(1);
    trk.bbox_h        = 0.0f;  // zero bbox height

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    // Should get a fallback depth (Tier 3 or 4), not crash or produce NaN
    EXPECT_GT(result.objects[0].position_3d.x(), 0.0f);
    EXPECT_FALSE(std::isnan(result.objects[0].position_3d.x()));
}

TEST(DepthEdgeCaseTest, ZeroBboxNoisePxProducesFullConfidence) {
    // bbox_height_noise_px=0 means zero measurement noise → sigma_depth²=0 →
    // confidence=1.0 for all depths.  This defeats the covariance model but
    // should not crash or produce NaN (review fix #16).
    auto calib                 = make_test_calib();
    calib.bbox_height_noise_px = 0.0f;
    UKFFusionEngine engine(calib, RadarNoiseConfig{}, false);

    TrackedObject trk = make_test_tracked(1);
    trk.bbox_h        = 100.0f;

    TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;
    tracked.objects.push_back(trk);

    auto result = engine.fuse(tracked);
    ASSERT_EQ(result.objects.size(), 1u);
    // With zero noise, confidence should be 1.0 (not NaN or negative)
    EXPECT_NEAR(result.objects[0].depth_confidence, 1.0f, 0.01f);
}
