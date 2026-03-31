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
// compute_auto_track tests
// ═══════════════════════════════════════════════════════════

TEST(GimbalAutoTracker, DisabledReturnsNoTarget) {
    AutoTrackConfig config{};
    config.enabled = false;

    auto objects = make_single_object(10.0f, 0.0f, -5.0f, 0.9f);
    auto result  = compute_auto_track(objects, config);

    EXPECT_FALSE(result.has_target);
}

TEST(GimbalAutoTracker, EnabledWithHighConfidence) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.5f;

    auto objects = make_single_object(10.0f, 0.0f, -5.0f, 0.9f);
    auto result  = compute_auto_track(objects, config);

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
    auto result  = compute_auto_track(objects, config);

    EXPECT_FALSE(result.has_target);  // Filtered out
}

TEST(GimbalAutoTracker, MinConfidenceAtExactThreshold) {
    AutoTrackConfig config{};
    config.enabled        = true;
    config.min_confidence = 0.5f;

    // Confidence exactly at threshold — should pass
    auto objects = make_single_object(10.0f, 0.0f, 0.0f, 0.5f);
    auto result  = compute_auto_track(objects, config);

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

    // Object 1: highest confidence, to the left
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

    auto result = compute_auto_track(list, config);

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

    auto result = compute_auto_track(empty_list, config);

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

    auto result = compute_auto_track(list, config);

    EXPECT_FALSE(result.has_target);
}

TEST(GimbalAutoTracker, ConfigToggle) {
    auto objects = make_single_object(10.0f, 5.0f, -3.0f, 0.9f);

    // Disabled — no target
    AutoTrackConfig disabled_cfg{};
    disabled_cfg.enabled        = false;
    disabled_cfg.min_confidence = 0.5f;

    auto result_off = compute_auto_track(objects, disabled_cfg);
    EXPECT_FALSE(result_off.has_target);

    // Enabled — should find target
    AutoTrackConfig enabled_cfg{};
    enabled_cfg.enabled        = true;
    enabled_cfg.min_confidence = 0.5f;

    auto result_on = compute_auto_track(objects, enabled_cfg);
    EXPECT_TRUE(result_on.has_target);
}
