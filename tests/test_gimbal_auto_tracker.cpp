// tests/test_gimbal_auto_tracker.cpp — Unit tests for gimbal auto-tracking logic
#include "payload/auto_tracker.h"

#include <cmath>

#include <gtest/gtest.h>

using namespace drone::payload;
using namespace drone::ipc;

// ── Helper: build a DetectedObjectList with one object ──────────
static DetectedObjectList make_single_object(float px, float py, float pz, float conf,
                                             uint32_t track_id = 1) {
    DetectedObjectList list{};
    list.timestamp_ns   = 1000;
    list.frame_sequence = 1;
    list.num_objects    = 1;

    auto& obj              = list.objects[0];
    obj.track_id           = track_id;
    obj.class_id           = ObjectClass::UNKNOWN;
    obj.confidence         = conf;
    obj.position_x         = px;
    obj.position_y         = py;
    obj.position_z         = pz;
    obj.velocity_x         = 0.0f;
    obj.velocity_y         = 0.0f;
    obj.velocity_z         = 0.0f;
    obj.heading            = 0.0f;
    obj.bbox_x             = 0.0f;
    obj.bbox_y             = 0.0f;
    obj.bbox_w             = 0.0f;
    obj.bbox_h             = 0.0f;
    obj.has_camera         = true;
    obj.has_radar          = false;
    obj.estimated_radius_m = 0.5f;
    obj.estimated_height_m = 1.0f;
    obj.radar_update_count = 0;

    return list;
}

/// Helper: build a Pose at origin with yaw=0.
static Pose make_origin_pose() {
    Pose p{};
    p.timestamp_ns   = 1000;
    p.translation[0] = 0.0;
    p.translation[1] = 0.0;
    p.translation[2] = 0.0;
    // Identity quaternion (w=1, x=0, y=0, z=0) → yaw=0
    p.quaternion[0] = 1.0;
    p.quaternion[1] = 0.0;
    p.quaternion[2] = 0.0;
    p.quaternion[3] = 0.0;
    p.velocity[0]   = 0.0;
    p.velocity[1]   = 0.0;
    p.velocity[2]   = 0.0;
    p.quality       = 2;  // good
    return p;
}

/// Helper: build a Pose at given position with given yaw (radians).
static Pose make_pose(double x, double y, double z, double yaw_rad) {
    Pose p{};
    p.timestamp_ns   = 1000;
    p.translation[0] = x;
    p.translation[1] = y;
    p.translation[2] = z;
    // Quaternion for pure yaw rotation about Z: w=cos(yaw/2), z=sin(yaw/2)
    p.quaternion[0] = std::cos(yaw_rad / 2.0);
    p.quaternion[1] = 0.0;
    p.quaternion[2] = 0.0;
    p.quaternion[3] = std::sin(yaw_rad / 2.0);
    p.velocity[0]   = 0.0;
    p.velocity[1]   = 0.0;
    p.velocity[2]   = 0.0;
    p.quality       = 2;  // good
    return p;
}

// ═══════════════════════════════════════════════════════════
// compute_bearing tests
// ═══════════════════════════════════════════════════════════

TEST(GimbalAutoTracker, BearingDirectlyAhead) {
    // Object straight ahead, same altitude
    auto [pitch, yaw] = compute_bearing(10.0f, 0.0f, 0.0f);
    EXPECT_NEAR(pitch, 0.0f, 0.1f);  // level
    EXPECT_NEAR(yaw, 0.0f, 0.1f);    // straight ahead
}

TEST(GimbalAutoTracker, BearingBelow) {
    // Object ahead and below
    auto [pitch, yaw] = compute_bearing(10.0f, 0.0f, -10.0f);
    EXPECT_NEAR(pitch, -45.0f, 0.1f);  // 45 degrees below horizon
    EXPECT_NEAR(yaw, 0.0f, 0.1f);      // straight ahead
}

TEST(GimbalAutoTracker, BearingAbove) {
    // Object ahead and above
    auto [pitch, yaw] = compute_bearing(10.0f, 0.0f, 10.0f);
    EXPECT_NEAR(pitch, 45.0f, 0.1f);  // 45 degrees above horizon
    EXPECT_NEAR(yaw, 0.0f, 0.1f);     // straight ahead
}

TEST(GimbalAutoTracker, BearingLeft) {
    // Object to the left
    auto [pitch, yaw] = compute_bearing(0.0f, 10.0f, 0.0f);
    EXPECT_NEAR(pitch, 0.0f, 0.1f);
    EXPECT_NEAR(yaw, 90.0f, 0.1f);  // 90 degrees left
}

TEST(GimbalAutoTracker, BearingRight) {
    // Object to the right
    auto [pitch, yaw] = compute_bearing(0.0f, -10.0f, 0.0f);
    EXPECT_NEAR(pitch, 0.0f, 0.1f);
    EXPECT_NEAR(yaw, -90.0f, 0.1f);  // 90 degrees right
}

TEST(GimbalAutoTracker, BearingBehind) {
    // Object directly behind
    auto [pitch, yaw] = compute_bearing(-10.0f, 0.0f, 0.0f);
    EXPECT_NEAR(pitch, 0.0f, 0.1f);
    EXPECT_NEAR(std::fabs(yaw), 180.0f, 0.1f);  // 180 degrees (behind)
}

TEST(GimbalAutoTracker, BearingDirectlyBelow) {
    // Object directly below (nadir)
    auto [pitch, yaw] = compute_bearing(0.0f, 0.0f, -10.0f);
    EXPECT_NEAR(pitch, -90.0f, 0.1f);  // straight down
}

TEST(GimbalAutoTracker, BearingZeroVector) {
    // Zero displacement — degenerate case
    auto [pitch, yaw] = compute_bearing(0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(pitch, 0.0f);
    EXPECT_FLOAT_EQ(yaw, 0.0f);
}

TEST(GimbalAutoTracker, BearingDiagonal) {
    // 45 degrees forward-left, same altitude
    auto [pitch, yaw] = compute_bearing(10.0f, 10.0f, 0.0f);
    EXPECT_NEAR(pitch, 0.0f, 0.1f);
    EXPECT_NEAR(yaw, 45.0f, 0.1f);
}

// ═══════════════════════════════════════════════════════════
// compute_auto_track tests (origin pose, yaw=0)
// ═══════════════════════════════════════════════════════════

TEST(GimbalAutoTracker, DisabledReturnsNoTarget) {
    AutoTrackConfig config{};
    config.enabled = false;

    auto objects = make_single_object(10.0f, 0.0f, -5.0f, 0.9f);
    auto pose    = make_origin_pose();
    auto result  = compute_auto_track(objects, pose, config);

    EXPECT_FALSE(result.has_target);
}

TEST(GimbalAutoTracker, EnabledWithHighConfidence) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.5f;

    // Object at world (10, 0, -5), drone at origin yaw=0 → body (10, 0, -5)
    auto objects = make_single_object(10.0f, 0.0f, -5.0f, 0.9f);
    auto pose    = make_origin_pose();
    auto result  = compute_auto_track(objects, pose, config);

    EXPECT_TRUE(result.has_target);
    EXPECT_FLOAT_EQ(result.target_confidence, 0.9f);
    EXPECT_EQ(result.target_track_id, 1u);
    // Object ahead and below — pitch should be negative
    EXPECT_LT(result.pitch_deg, 0.0f);
    EXPECT_NEAR(result.yaw_deg, 0.0f, 0.1f);
}

TEST(GimbalAutoTracker, MinConfidenceFiltering) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.7f;

    // Object with confidence below threshold
    auto objects = make_single_object(10.0f, 0.0f, 0.0f, 0.3f);
    auto pose    = make_origin_pose();
    auto result  = compute_auto_track(objects, pose, config);

    EXPECT_FALSE(result.has_target);  // Filtered out
}

TEST(GimbalAutoTracker, MinConfidenceAtExactThreshold) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.5f;

    // Confidence exactly at threshold — should pass
    auto objects = make_single_object(10.0f, 0.0f, 0.0f, 0.5f);
    auto pose    = make_origin_pose();
    auto result  = compute_auto_track(objects, pose, config);

    EXPECT_TRUE(result.has_target);
}

TEST(GimbalAutoTracker, HighestConfidenceSelected) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.3f;

    DetectedObjectList list{};
    list.timestamp_ns   = 1000;
    list.frame_sequence = 1;
    list.num_objects    = 3;

    // Object 0: moderate confidence, ahead
    auto& obj0              = list.objects[0];
    obj0.track_id           = 10;
    obj0.class_id           = ObjectClass::PERSON;
    obj0.confidence         = 0.6f;
    obj0.position_x         = 10.0f;
    obj0.position_y         = 0.0f;
    obj0.position_z         = 0.0f;
    obj0.velocity_x         = 0.0f;
    obj0.velocity_y         = 0.0f;
    obj0.velocity_z         = 0.0f;
    obj0.heading            = 0.0f;
    obj0.bbox_x             = 0.0f;
    obj0.bbox_y             = 0.0f;
    obj0.bbox_w             = 0.0f;
    obj0.bbox_h             = 0.0f;
    obj0.has_camera         = true;
    obj0.has_radar          = false;
    obj0.estimated_radius_m = 0.5f;
    obj0.estimated_height_m = 1.0f;
    obj0.radar_update_count = 0;

    // Object 1: highest confidence, to the left (world-frame east = +y)
    auto& obj1              = list.objects[1];
    obj1.track_id           = 20;
    obj1.class_id           = ObjectClass::VEHICLE_CAR;
    obj1.confidence         = 0.95f;
    obj1.position_x         = 5.0f;
    obj1.position_y         = 5.0f;
    obj1.position_z         = -2.0f;
    obj1.velocity_x         = 0.0f;
    obj1.velocity_y         = 0.0f;
    obj1.velocity_z         = 0.0f;
    obj1.heading            = 0.0f;
    obj1.bbox_x             = 0.0f;
    obj1.bbox_y             = 0.0f;
    obj1.bbox_w             = 0.0f;
    obj1.bbox_h             = 0.0f;
    obj1.has_camera         = true;
    obj1.has_radar          = true;
    obj1.estimated_radius_m = 2.0f;
    obj1.estimated_height_m = 1.5f;
    obj1.radar_update_count = 3;

    // Object 2: low confidence, behind
    auto& obj2              = list.objects[2];
    obj2.track_id           = 30;
    obj2.class_id           = ObjectClass::UNKNOWN;
    obj2.confidence         = 0.4f;
    obj2.position_x         = -10.0f;
    obj2.position_y         = 0.0f;
    obj2.position_z         = 0.0f;
    obj2.velocity_x         = 0.0f;
    obj2.velocity_y         = 0.0f;
    obj2.velocity_z         = 0.0f;
    obj2.heading            = 0.0f;
    obj2.bbox_x             = 0.0f;
    obj2.bbox_y             = 0.0f;
    obj2.bbox_w             = 0.0f;
    obj2.bbox_h             = 0.0f;
    obj2.has_camera         = true;
    obj2.has_radar          = false;
    obj2.estimated_radius_m = 0.3f;
    obj2.estimated_height_m = 0.5f;
    obj2.radar_update_count = 0;

    auto pose   = make_origin_pose();
    auto result = compute_auto_track(list, pose, config);

    EXPECT_TRUE(result.has_target);
    EXPECT_EQ(result.target_track_id, 20u);  // Highest confidence
    EXPECT_FLOAT_EQ(result.target_confidence, 0.95f);
    // Object is forward-left and below — yaw positive, pitch negative
    EXPECT_GT(result.yaw_deg, 0.0f);
    EXPECT_LT(result.pitch_deg, 0.0f);
}

TEST(GimbalAutoTracker, NoObjectsHoldsPosition) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.5f;

    DetectedObjectList empty_list{};
    empty_list.timestamp_ns   = 1000;
    empty_list.frame_sequence = 1;
    empty_list.num_objects    = 0;

    auto pose   = make_origin_pose();
    auto result = compute_auto_track(empty_list, pose, config);

    EXPECT_FALSE(result.has_target);
    EXPECT_FLOAT_EQ(result.pitch_deg, 0.0f);
    EXPECT_FLOAT_EQ(result.yaw_deg, 0.0f);
}

TEST(GimbalAutoTracker, AllObjectsBelowConfidenceHoldsPosition) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.8f;

    DetectedObjectList list{};
    list.timestamp_ns   = 1000;
    list.frame_sequence = 1;
    list.num_objects    = 2;

    // Both objects below threshold
    for (uint32_t i = 0; i < 2; ++i) {
        auto& obj              = list.objects[i];
        obj.track_id           = i + 1;
        obj.class_id           = ObjectClass::PERSON;
        obj.confidence         = 0.3f + static_cast<float>(i) * 0.2f;  // 0.3, 0.5
        obj.position_x         = 10.0f;
        obj.position_y         = 0.0f;
        obj.position_z         = 0.0f;
        obj.velocity_x         = 0.0f;
        obj.velocity_y         = 0.0f;
        obj.velocity_z         = 0.0f;
        obj.heading            = 0.0f;
        obj.bbox_x             = 0.0f;
        obj.bbox_y             = 0.0f;
        obj.bbox_w             = 0.0f;
        obj.bbox_h             = 0.0f;
        obj.has_camera         = true;
        obj.has_radar          = false;
        obj.estimated_radius_m = 0.5f;
        obj.estimated_height_m = 1.0f;
        obj.radar_update_count = 0;
    }

    auto pose   = make_origin_pose();
    auto result = compute_auto_track(list, pose, config);

    EXPECT_FALSE(result.has_target);
}

TEST(GimbalAutoTracker, ConfigToggle) {
    auto objects = make_single_object(10.0f, 5.0f, -3.0f, 0.9f);
    auto pose    = make_origin_pose();

    // Disabled — no target
    AutoTrackConfig disabled_cfg{};
    disabled_cfg.enabled        = false;
    disabled_cfg.min_confidence = 0.5f;

    auto result_off = compute_auto_track(objects, pose, disabled_cfg);
    EXPECT_FALSE(result_off.has_target);

    // Enabled — should find target
    AutoTrackConfig enabled_cfg{};
    enabled_cfg.enabled        = true;
    enabled_cfg.min_confidence = 0.5f;

    auto result_on = compute_auto_track(objects, pose, enabled_cfg);
    EXPECT_TRUE(result_on.has_target);
}

// ═══════════════════════════════════════════════════════════
// num_objects bounds check
// ═══════════════════════════════════════════════════════════

TEST(GimbalAutoTracker, NumObjectsClampedToMax) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.1f;

    DetectedObjectList list{};
    list.timestamp_ns   = 1000;
    list.frame_sequence = 1;
    // Corrupt num_objects to exceed array bounds — must not crash
    list.num_objects = MAX_DETECTED_OBJECTS + 100;

    // Place a valid object at index 0
    auto& obj              = list.objects[0];
    obj.track_id           = 1;
    obj.class_id           = ObjectClass::PERSON;
    obj.confidence         = 0.9f;
    obj.position_x         = 10.0f;
    obj.position_y         = 0.0f;
    obj.position_z         = 0.0f;
    obj.velocity_x         = 0.0f;
    obj.velocity_y         = 0.0f;
    obj.velocity_z         = 0.0f;
    obj.heading            = 0.0f;
    obj.bbox_x             = 0.0f;
    obj.bbox_y             = 0.0f;
    obj.bbox_w             = 0.0f;
    obj.bbox_h             = 0.0f;
    obj.has_camera         = true;
    obj.has_radar          = false;
    obj.estimated_radius_m = 0.5f;
    obj.estimated_height_m = 1.0f;
    obj.radar_update_count = 0;

    auto pose   = make_origin_pose();
    auto result = compute_auto_track(list, pose, config);

    // Should still work — clamped to MAX_DETECTED_OBJECTS
    EXPECT_TRUE(result.has_target);
    EXPECT_EQ(result.target_track_id, 1u);
}

// ═══════════════════════════════════════════════════════════
// World-frame + pose transform tests
// ═══════════════════════════════════════════════════════════

TEST(GimbalAutoTracker, YawFromQuaternionIdentity) {
    double quat[4] = {1.0, 0.0, 0.0, 0.0};  // identity → yaw=0
    EXPECT_NEAR(yaw_from_quaternion(quat), 0.0f, 1e-6f);
}

TEST(GimbalAutoTracker, YawFromQuaternion90Degrees) {
    // 90 degrees yaw (pi/2): w=cos(pi/4), z=sin(pi/4)
    const double half    = std::acos(0.0);  // pi/2
    double       quat[4] = {std::cos(half / 2.0), 0.0, 0.0, std::sin(half / 2.0)};
    EXPECT_NEAR(yaw_from_quaternion(quat), static_cast<float>(half), 1e-5f);
}

TEST(GimbalAutoTracker, NonZeroDronePosition) {
    // Drone at world (5, 5, 10), yaw=0. Object at world (15, 5, 5).
    // World-relative: dx=10, dy=0, dz=-5. Body (yaw=0): forward=10, left=0, up=-5.
    // Expect: pitch negative (below), yaw ~0 (ahead).
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.3f;

    auto objects = make_single_object(15.0f, 5.0f, 5.0f, 0.9f);
    auto pose    = make_pose(5.0, 5.0, 10.0, 0.0);
    auto result  = compute_auto_track(objects, pose, config);

    EXPECT_TRUE(result.has_target);
    EXPECT_LT(result.pitch_deg, 0.0f);        // below
    EXPECT_NEAR(result.yaw_deg, 0.0f, 0.5f);  // straight ahead
}

TEST(GimbalAutoTracker, NonZeroDroneYaw90) {
    // Drone at origin, yaw=pi/2 (facing East).
    // Object at world (0, 10, 0) — 10m East.
    // World-relative: dx=0, dy=10, dz=0.
    // Body-frame (rotate by -pi/2):
    //   body_x = 0*cos(pi/2) + 10*sin(pi/2) = 10 (forward)
    //   body_y = -0*sin(pi/2) + 10*cos(pi/2) = 0 (no lateral offset)
    // Expect: pitch=0, yaw=0 (directly ahead in body frame).
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.3f;

    const double yaw_rad = std::acos(0.0);  // pi/2
    auto         objects = make_single_object(0.0f, 10.0f, 0.0f, 0.9f);
    auto         pose    = make_pose(0.0, 0.0, 0.0, yaw_rad);
    auto         result  = compute_auto_track(objects, pose, config);

    EXPECT_TRUE(result.has_target);
    EXPECT_NEAR(result.pitch_deg, 0.0f, 0.5f);
    EXPECT_NEAR(result.yaw_deg, 0.0f, 0.5f);  // ahead in body frame
}

TEST(GimbalAutoTracker, NonZeroDroneYaw90ObjectBehind) {
    // Drone at origin, yaw=pi/2 (facing East).
    // Object at world (0, -10, 0) — 10m West = behind the drone.
    // World-relative: dx=0, dy=-10, dz=0.
    // Body-frame (rotate by -pi/2):
    //   body_x = 0*cos(pi/2) + (-10)*sin(pi/2) = -10 (behind)
    //   body_y = -0*sin(pi/2) + (-10)*cos(pi/2) = 0
    // Expect: yaw = +/-180 (behind).
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.3f;

    const double yaw_rad = std::acos(0.0);  // pi/2
    auto         objects = make_single_object(0.0f, -10.0f, 0.0f, 0.9f);
    auto         pose    = make_pose(0.0, 0.0, 0.0, yaw_rad);
    auto         result  = compute_auto_track(objects, pose, config);

    EXPECT_TRUE(result.has_target);
    EXPECT_NEAR(std::fabs(result.yaw_deg), 180.0f, 0.5f);
}

TEST(GimbalAutoTracker, NonZeroDroneYawAndPosition) {
    // Drone at world (10, 0, 5), yaw=pi/2 (facing East).
    // Object at world (10, 10, 5) — 10m East of drone.
    // World-relative: dx=0, dy=10, dz=0.
    // Body-frame (rotate by -pi/2): body_x=10 (forward), body_y=0.
    // Expect: pitch=0, yaw=0 (directly ahead).
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.3f;

    const double yaw_rad = std::acos(0.0);  // pi/2
    auto         objects = make_single_object(10.0f, 10.0f, 5.0f, 0.9f);
    auto         pose    = make_pose(10.0, 0.0, 5.0, yaw_rad);
    auto         result  = compute_auto_track(objects, pose, config);

    EXPECT_TRUE(result.has_target);
    EXPECT_NEAR(result.pitch_deg, 0.0f, 0.5f);
    EXPECT_NEAR(result.yaw_deg, 0.0f, 0.5f);
}

TEST(GimbalAutoTracker, NonZeroDroneYaw45ObjectLeft) {
    // Drone at origin, yaw=pi/4 (45 deg, facing NE).
    // Object at world (0, 10, 0) — 10m East.
    // World-relative: dx=0, dy=10, dz=0.
    // Body-frame (rotate by -pi/4):
    //   body_x = 0*cos(pi/4) + 10*sin(pi/4) = 7.07 (forward component)
    //   body_y = -0*sin(pi/4) + 10*cos(pi/4) = 7.07 (left component)
    // Expect: yaw=45 degrees (forward-left in body frame).
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.3f;

    const double yaw_rad = std::acos(0.0) / 2.0;  // pi/4
    auto         objects = make_single_object(0.0f, 10.0f, 0.0f, 0.9f);
    auto         pose    = make_pose(0.0, 0.0, 0.0, yaw_rad);
    auto         result  = compute_auto_track(objects, pose, config);

    EXPECT_TRUE(result.has_target);
    EXPECT_NEAR(result.pitch_deg, 0.0f, 0.5f);
    EXPECT_NEAR(result.yaw_deg, 45.0f, 1.0f);  // 45 deg left in body frame
}
