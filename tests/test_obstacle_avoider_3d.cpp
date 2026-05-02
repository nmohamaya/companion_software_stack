// tests/test_obstacle_avoider_3d.cpp
// Unit tests for 3D Velocity-Obstacle avoider (Phase 3).
#include "planner/iobstacle_avoider.h"
#include "planner/obstacle_avoider_3d.h"

#include <chrono>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using namespace drone::planner;

// ── Helpers ─────────────────────────────────────────────────

/// Create a fresh now-timestamp for object data.
static uint64_t now_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

static drone::ipc::TrajectoryCmd make_cmd(float vx = 2.0f, float vy = 0.0f, float vz = 0.0f) {
    drone::ipc::TrajectoryCmd cmd{};
    cmd.velocity_x   = vx;
    cmd.velocity_y   = vy;
    cmd.velocity_z   = vz;
    cmd.valid        = true;
    cmd.timestamp_ns = now_ns();
    return cmd;
}

static drone::ipc::Pose make_pose(float x = 0.0f, float y = 0.0f, float z = 5.0f) {
    drone::ipc::Pose pose{};
    pose.translation[0] = x;
    pose.translation[1] = y;
    pose.translation[2] = z;
    pose.timestamp_ns   = now_ns();
    return pose;
}

// ═════════════════════════════════════════════════════════════
// Basic avoidance tests
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, NoObjectsPassThrough) {
    ObstacleAvoider3D avoider;
    auto              cmd  = make_cmd(2.0f);
    auto              pose = make_pose();

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects  = 0;
    objects.timestamp_ns = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_FLOAT_EQ(result.velocity_x, cmd.velocity_x);
    EXPECT_FLOAT_EQ(result.velocity_y, cmd.velocity_y);
    EXPECT_FLOAT_EQ(result.velocity_z, cmd.velocity_z);
}

TEST(ObstacleAvoider3DTest, StaleObjectsIgnored) {
    ObstacleAvoider3D avoider;
    auto              cmd  = make_cmd(2.0f);
    auto              pose = make_pose();

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;  // close obstacle
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = 1;  // very old timestamp

    auto result = avoider.avoid(cmd, pose, objects);
    // Should be unmodified — data is stale
    EXPECT_FLOAT_EQ(result.velocity_x, cmd.velocity_x);
}

TEST(ObstacleAvoider3DTest, CloseObjectRepelsInXYZ) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;  // test isotropic base repulsion
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 3.0f;  // ahead, slightly above
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 6.0f;
    objects.objects[0].confidence = 0.9f;
    objects.objects[0].velocity_x = 0.0f;
    objects.objects[0].velocity_y = 0.0f;
    objects.objects[0].velocity_z = 0.0f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Should repel: vx should decrease (obstacle ahead)
    EXPECT_LT(result.velocity_x, cmd.velocity_x);
    // Should also have vertical repulsion (obstacle above → push drone down)
    EXPECT_LT(result.velocity_z, cmd.velocity_z);
}

TEST(ObstacleAvoider3DTest, LowConfidenceIgnored) {
    ObstacleAvoider3D avoider;
    auto              cmd  = make_cmd(2.0f);
    auto              pose = make_pose();

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.1f;  // below threshold
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_FLOAT_EQ(result.velocity_x, cmd.velocity_x);
}

TEST(ObstacleAvoider3DTest, CorrectionClamped) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 100.0f;  // very strong
    config.max_correction_mps = 3.0f;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.0f;  // very close
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Correction should be clamped to ±3 m/s
    EXPECT_GE(result.velocity_x, -config.max_correction_mps - 0.01f);
    EXPECT_LE(result.velocity_x, config.max_correction_mps + 0.01f);
}

// ═════════════════════════════════════════════════════════════
// Predictive avoidance
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, PredictionShiftsRepulsion) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.prediction_dt_s    = 1.0f;  // 1 second look-ahead
    config.repulsive_gain     = 2.0f;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Obstacle far away but moving toward drone
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 8.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].velocity_x = -6.0f;  // coming toward us at 6 m/s
    objects.objects[0].velocity_y = 0.0f;
    objects.objects[0].velocity_z = 0.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result_pred = avoider.avoid(cmd, pose, objects);

    // With prediction, obstacle is at 8 + (-6)*1 = 2m → strong repulsion
    // Without prediction it'd be at 8m → weaker repulsion
    // So vx should be significantly negative (pushed away)
    EXPECT_LT(result_pred.velocity_x, -0.1f);
}

// ═════════════════════════════════════════════════════════════
// NaN input handling (error handling / diagnostics)
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, NaNPosePassesThrough) {
    ObstacleAvoider3D avoider;
    auto              cmd = make_cmd(2.0f);

    drone::ipc::Pose pose{};
    pose.translation[0] = std::numeric_limits<double>::quiet_NaN();
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    // NaN pose → return planned unmodified
    EXPECT_FLOAT_EQ(result.velocity_x, cmd.velocity_x);
}

// ═════════════════════════════════════════════════════════════
// Factory registration
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoiderFactory, ThreeDBackendRegistered) {
    auto result = create_obstacle_avoider("3d");
    ASSERT_TRUE(result.is_ok());
    EXPECT_NE(result.value(), nullptr);
    EXPECT_EQ(result.value()->name(), "ObstacleAvoider3D");
}

TEST(ObstacleAvoiderFactory, AlternateNameRegistered) {
    auto result = create_obstacle_avoider("obstacle_avoider_3d");
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value()->name(), "ObstacleAvoider3D");
}

TEST(ObstacleAvoiderFactory, PotentialField3DRegistered) {
    auto result = create_obstacle_avoider("potential_field_3d");
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value()->name(), "ObstacleAvoider3D");
}

TEST(ObstacleAvoiderFactory, UnknownReturnsError) {
    auto result = create_obstacle_avoider("nonexistent");
    EXPECT_TRUE(result.is_err());
}

// ═════════════════════════════════════════════════════════════
// Name accessor
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, NameIsCorrect) {
    ObstacleAvoider3D avoider;
    EXPECT_EQ(avoider.name(), "ObstacleAvoider3D");
}

TEST(ObstacleAvoider3DTest, ConvenienceConstructor) {
    ObstacleAvoider3D avoider(8.0f, 3.0f);
    EXPECT_EQ(avoider.name(), "ObstacleAvoider3D");
}

// ═════════════════════════════════════════════════════════════
// ═════════════════════════════════════════════════════════════
// Vertical gain control (Issue #225)
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, VerticalGainZero_EliminatesZRepulsion) {
    // With vertical_gain=0, Z correction should be zero while X/Y are still active.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 5.0f;
    config.repulsive_gain     = 2.0f;
    config.vertical_gain      = 0.0f;
    config.path_aware         = false;  // test isotropic vertical gain
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;  // 2m ahead
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 3.0f;  // 2m below drone — would produce Z repulsion
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // X should be repelled (obstacle is ahead)
    EXPECT_LT(result.velocity_x, cmd.velocity_x);
    // Z should be unchanged — vertical_gain=0 eliminates Z repulsion
    EXPECT_FLOAT_EQ(result.velocity_z, cmd.velocity_z);
}

TEST(ObstacleAvoider3DTest, VerticalGainOne_ProducesZRepulsion) {
    // With vertical_gain=1 (default), an obstacle below the drone should push it up.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 5.0f;
    config.repulsive_gain     = 2.0f;
    config.vertical_gain      = 1.0f;
    config.path_aware         = false;  // test isotropic vertical gain
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 3.0f;  // 2m below drone
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Z should increase (pushed away from obstacle below)
    EXPECT_GT(result.velocity_z, cmd.velocity_z);
}

// ═════════════════════════════════════════════════════════════
// Dead zone fix: very close objects (Issue #225)
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, VeryCloseObjectMaxRepulsion_BrakeDisabled) {
    // Objects at distance 0.05m (between old 0.1m threshold and new 0.01m)
    // should now receive maximum repulsion (clamped by max_correction_mps).
    // _BrakeDisabled suffix: this test isolates the Euclidean max-correction
    // clamp on the repulsion vector, not the #513 brake path.  See the
    // `brake_in_close_regime=false` setter below for the full rationale.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 5.0f;
    config.repulsive_gain     = 2.0f;
    config.max_correction_mps = 3.0f;
    config.min_confidence     = 0.3f;
    config.path_aware         = false;  // test isotropic base repulsion
    // Disable brake arbitration so this test isolates the max-repulsion clamp.
    // The #513 brake path cancels forward velocity before repulsion is added,
    // which would change the total correction magnitude and obscure what this
    // test is pinning (the Euclidean clamp on the repulsion vector alone).
    // Dedicated brake tests live further down the file.
    config.brake_in_close_regime = false;

    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 0.05f;  // 5cm ahead of drone
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // The obstacle is 5cm away — repulsion should be at max clamp in -X direction.
    // Before the fix (dist > 0.1f), this would have been zero repulsion.
    EXPECT_LT(result.velocity_x, cmd.velocity_x);
    // The correction should hit the max clamp
    float correction_x = result.velocity_x - cmd.velocity_x;
    EXPECT_NEAR(correction_x, -config.max_correction_mps, 0.1f);
}

// ═════════════════════════════════════════════════════════════
// Path-aware avoider tests (Issue #229)
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, PathAwareNoBackwardsPush) {
    // Obstacle directly ahead on the planned path.
    // path_aware removes the component opposing planned direction.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.max_correction_mps = 3.0f;
    config.path_aware         = true;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);  // flying +X
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 3.0f;  // directly ahead
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;  // same altitude
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Obstacle directly along planned direction → repulsion is entirely -X.
    // path_aware removes this opposing component, leaving vx ~= planned.
    EXPECT_GE(result.velocity_x, cmd.velocity_x - 0.01f);
}

TEST(ObstacleAvoider3DTest, PathAwareLateralPreserved) {
    // Obstacle to the side of planned path → lateral repulsion preserved.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.max_correction_mps = 3.0f;
    config.path_aware         = true;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);  // flying +X
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 0.0f;  // at same X as drone
    objects.objects[0].position_y = 2.0f;  // to the right
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Repulsion in -Y is perpendicular to planned direction (+X) → kept.
    EXPECT_LT(result.velocity_y, -0.1f);
    // X velocity unchanged (repulsion is purely lateral)
    EXPECT_NEAR(result.velocity_x, cmd.velocity_x, 0.1f);
}

TEST(ObstacleAvoider3DTest, PathAwareSameDirectionPreserved) {
    // Obstacle behind drone → repulsion pushes +X (same as planned).
    // along > 0 → component kept.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.max_correction_mps = 3.0f;
    config.path_aware         = true;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);  // flying +X
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = -3.0f;  // behind drone
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Repulsion pushes +X (away from obstacle behind), along planned direction → kept.
    EXPECT_GT(result.velocity_x, cmd.velocity_x);
}

TEST(ObstacleAvoider3DTest, PathAwareNoZLeakWhenVerticalGainZero) {
    // Regression: with vertical_gain=0 and path_aware=true, 3D stripping
    // injected Z via `total_rep_z -= along * dir_z` when the planned command
    // had a vertical component.  The fix uses 2D-only stripping (Issue #237).
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.max_correction_mps = 3.0f;
    config.vertical_gain      = 0.0f;
    config.path_aware         = true;
    ObstacleAvoider3D avoider(config);

    // Planned command: flying +X with a Z correction (altitude hold)
    auto cmd  = make_cmd(2.0f, 0.0f, 0.5f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Obstacle directly ahead — produces strong -X repulsion that opposes
    // the planned +X direction.  With the old 3D stripping this leaked into Z.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 3.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Z must be EXACTLY the planned value — no Z injection from path-aware stripping
    EXPECT_FLOAT_EQ(result.velocity_z, cmd.velocity_z);
    // X repulsion should still be stripped (path-aware removes opposing component)
    EXPECT_GE(result.velocity_x, cmd.velocity_x - 0.01f);
}

// ═════════════════════════════════════════════════════════════
// Euclidean-magnitude clamp — Issue #503
// The clamp must limit the 3-vector magnitude, not each axis
// independently.  Previous per-axis behavior allowed magnitude
// up to max_correction_mps * sqrt(N_axes).
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, ClampsEuclideanMagnitudeNotPerAxis) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 50.0f;  // strong enough to saturate
    config.max_correction_mps = 1.0f;
    config.path_aware         = false;  // isolate clamp behavior
    config.vertical_gain      = 0.0f;   // 2D for clarity
    config.log_corrections    = false;
    ObstacleAvoider3D avoider(config);

    // Planner hovering, single obstacle diagonally ahead — repulsion points
    // back along the -X/-Y diagonal.  Per-axis clamp would allow |c|≈sqrt(2).
    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.0f;
    objects.objects[0].position_y = 1.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto        result = avoider.avoid(cmd, pose, objects);
    const float dx     = result.velocity_x - cmd.velocity_x;
    const float dy     = result.velocity_y - cmd.velocity_y;
    const float dz     = result.velocity_z - cmd.velocity_z;
    const float mag    = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Must be clamped to max_correction_mps (within float tolerance).
    EXPECT_NEAR(mag, config.max_correction_mps, 1e-4f);
    EXPECT_LE(mag, config.max_correction_mps + 1e-4f);
}

// ═════════════════════════════════════════════════════════════
// Close-regime path-aware bypass — Issue #503
// When closest obstacle < min_distance_m, path-aware stripping
// must be bypassed so the avoider can push opposite to planner
// direction.  Hysteresis prevents chatter at the boundary.
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, BypassesPathAwareInCloseRegime) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m             = 10.0f;
    config.repulsive_gain                 = 2.0f;
    config.max_correction_mps             = 3.0f;
    config.path_aware                     = true;
    config.min_distance_m                 = 2.5f;  // close regime below this
    config.path_aware_bypass_hysteresis_m = 0.5f;
    config.log_corrections                = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);  // planner pushing +X
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Obstacle at distance = 1.0m — well inside min_distance_m.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.0f;  // head-on
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Bypass active → path-aware stripping skipped → avoider pushes -X
    // (against planner direction) to maintain clearance.
    EXPECT_LT(result.velocity_x, cmd.velocity_x - 0.1f);
    EXPECT_TRUE(avoider.close_regime_active());
}

TEST(ObstacleAvoider3DTest, BypassHysteresisPreventsFlipFlop) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m             = 10.0f;
    config.repulsive_gain                 = 2.0f;
    config.max_correction_mps             = 3.0f;
    config.path_aware                     = true;
    config.min_distance_m                 = 2.5f;
    config.path_aware_bypass_hysteresis_m = 0.5f;  // exit at > 3.0m
    config.log_corrections                = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Tick 1 — obstacle at 1.0m → enter close regime.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();
    (void)avoider.avoid(cmd, pose, objects);
    ASSERT_TRUE(avoider.close_regime_active());

    // Tick 2 — obstacle at 2.8m (above min_distance_m, below min+hysteresis).
    // Must NOT exit close regime yet.
    objects.objects[0].position_x = 2.8f;
    objects.timestamp_ns          = now_ns();
    (void)avoider.avoid(cmd, pose, objects);
    EXPECT_TRUE(avoider.close_regime_active());

    // Tick 3 — obstacle at 3.5m (above min+hysteresis=3.0m).  Exit.
    objects.objects[0].position_x = 3.5f;
    objects.timestamp_ns          = now_ns();
    (void)avoider.avoid(cmd, pose, objects);
    EXPECT_FALSE(avoider.close_regime_active());
}

// ═════════════════════════════════════════════════════════════
// Per-class config (Epic #519)
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, PerClassInfluenceRadius) {
    // PERSON (idx 1) with influence_radius=3.0 gets no avoidance at 4m,
    // but VEHICLE_TRUCK (idx 3) with influence_radius=8.0 does.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 5.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.log_corrections    = false;

    ObstacleAvoider3D avoider(config);
    // Set per-class overrides after construction (constructor syncs from globals).
    avoider.mutable_config().influence_radius_per_class[1] = 3.0f;  // PERSON
    avoider.mutable_config().influence_radius_per_class[3] = 8.0f;  // VEHICLE_TRUCK

    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Object at 4m — outside PERSON radius (3.0) but inside TRUCK radius (8.0).
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 4.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    // As PERSON — no repulsion (outside influence radius).
    objects.objects[0].class_id = drone::ipc::ObjectClass::PERSON;
    auto result_person          = avoider.avoid(cmd, pose, objects);
    EXPECT_FLOAT_EQ(result_person.velocity_x, cmd.velocity_x);

    // As VEHICLE_TRUCK — gets repulsion.
    objects.objects[0].class_id = drone::ipc::ObjectClass::VEHICLE_TRUCK;
    auto result_truck           = avoider.avoid(cmd, pose, objects);
    EXPECT_LT(result_truck.velocity_x, cmd.velocity_x);
}

TEST(ObstacleAvoider3DTest, PerClassConfidenceThreshold) {
    // DRONE (idx 4) has min_confidence=0.8, so a 0.5-confidence drone is filtered.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.log_corrections    = false;

    ObstacleAvoider3D avoider(config);
    // Set per-class override after construction (constructor syncs from globals).
    avoider.mutable_config().min_confidence_per_class[4] = 0.8f;  // DRONE

    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 3.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.5f;
    objects.objects[0].class_id   = drone::ipc::ObjectClass::DRONE;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    // Filtered out — no repulsion.
    EXPECT_FLOAT_EQ(result.velocity_x, cmd.velocity_x);
}

TEST(ObstacleAvoider3DTest, PerClassPredictionDt) {
    // BUILDING (idx 6) with prediction_dt=0.0 ignores velocity.
    // DRONE (idx 4) with prediction_dt=1.0 predicts forward.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.log_corrections    = false;

    ObstacleAvoider3D avoider(config);
    // Set per-class overrides after construction (constructor syncs from globals).
    avoider.mutable_config().prediction_dt_per_class[6] = 0.0f;  // BUILDING — stationary
    avoider.mutable_config().prediction_dt_per_class[4] = 1.0f;  // DRONE — aggressive prediction

    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Object at 8m, approaching at 6 m/s.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 8.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].velocity_x = -6.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    // As BUILDING (prediction_dt=0): predicted pos = 8m, within influence.
    objects.objects[0].class_id = drone::ipc::ObjectClass::BUILDING;
    auto result_building        = avoider.avoid(cmd, pose, objects);

    // As DRONE (prediction_dt=1.0): predicted pos = 8+(-6)*1 = 2m, much closer.
    objects.objects[0].class_id = drone::ipc::ObjectClass::DRONE;
    auto result_drone           = avoider.avoid(cmd, pose, objects);

    // DRONE should get stronger repulsion (predicted closer).
    EXPECT_LT(result_drone.velocity_x, result_building.velocity_x);
}

TEST(ObstacleAvoider3DTest, OobClassIdSkipped) {
    // An IPC message with class_id >= 8 must not cause OOB array access.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.log_corrections    = false;

    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 3.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.objects[0].class_id   = static_cast<drone::ipc::ObjectClass>(255);
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    // Object with invalid class_id should be skipped — no repulsion.
    EXPECT_FLOAT_EQ(result.velocity_x, cmd.velocity_x);
    EXPECT_FLOAT_EQ(result.velocity_y, cmd.velocity_y);
    EXPECT_FLOAT_EQ(result.velocity_z, cmd.velocity_z);
}

TEST(ObstacleAvoider3DTest, ConfigDrivenPerClassLoading) {
    // Verify the Config-driven constructor loads per-class overrides from JSON.
    std::string tmp = "/tmp/drone_test_avoider_cfg_" + std::to_string(::getpid()) + ".json";
    {
        std::ofstream ofs(tmp);
        ofs << R"({
            "mission_planner": {
                "obstacle_avoidance": {
                    "influence_radius_m": 5.0,
                    "repulsive_gain": 2.0,
                    "path_aware": false,
                    "log_corrections": false,
                    "per_class": {
                        "influence_radius_m": { "default": 5.0, "person": 3.0 }
                    }
                }
            }
        })";
    }
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(tmp));
    std::remove(tmp.c_str());

    ObstacleAvoider3D avoider(cfg);
    // PERSON (idx 1) should have influence_radius=3.0, others 5.0.
    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 4.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    // PERSON at 4m — outside per-class radius (3.0), no repulsion.
    objects.objects[0].class_id = drone::ipc::ObjectClass::PERSON;
    auto result_person          = avoider.avoid(cmd, pose, objects);
    EXPECT_FLOAT_EQ(result_person.velocity_x, cmd.velocity_x);

    // UNKNOWN at 4m — inside default radius (5.0), gets repulsion.
    objects.objects[0].class_id = drone::ipc::ObjectClass::UNKNOWN;
    auto result_unknown         = avoider.avoid(cmd, pose, objects);
    EXPECT_LT(result_unknown.velocity_x, cmd.velocity_x);
}

TEST(ObstacleAvoider3DTest, PathAwareDisabledFallback) {
    // path_aware=false → old isotropic behavior (obstacle ahead reduces vx).
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.repulsive_gain     = 2.0f;
    config.max_correction_mps = 3.0f;
    config.path_aware         = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 3.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Old behavior: obstacle ahead → backwards push applied → vx reduced.
    EXPECT_LT(result.velocity_x, cmd.velocity_x);
}

// ═════════════════════════════════════════════════════════════
// Issue #513 — brake arbitration in close regime
// Regression: the avoider must cancel forward velocity heading INTO a
// close-regime obstacle, not merely add a lateral deflection.  Scenario-30
// live-run evidence (see #513 body) showed a 0.98 m/s lateral correction
// against a 2 m/s forward cruise = ~30° deflection, not a brake.  The
// drone overshot.  This test pins the correct behaviour.
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_CancelsForwardComponent) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 2.0f;
    config.min_distance_m        = 3.0f;
    config.max_correction_mps    = 5.0f;
    config.brake_in_close_regime = true;
    config.path_aware            = false;  // avoid path-aware interaction in this test
    config.vertical_gain         = 0.0f;   // 2D scenario
    ObstacleAvoider3D avoider(config);

    // Drone at origin cruising +X at 2 m/s.
    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Obstacle 2 m ahead — inside min_distance_m so close_regime_active_ fires.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // With brake_in_close_regime=true and d=2, min_distance=3:
    //   1. Forward (toward-obstacle) component cancelled: planned 2 m/s → 0.
    //   2. Proximity scale = d/min_distance = 2/3 ≈ 0.667 applied to residual (0).
    //   3. Repulsion: gain=2 at dist=2 → |rep|=0.5 in -X direction.
    //   4. Final vx = 0 - 0.5 = -0.5 (below max_correction_mps clamp of 5, so
    //      no clamping).
    // Tight assertion (PR #617 Pass 2 test-quality review): pin the exact
    // value.  A half-plane EXPECT_LT would let a repulsion-doubling regression
    // pass; EXPECT_NEAR catches both direction AND magnitude regressions.
    EXPECT_NEAR(result.velocity_x, -0.5f, 0.05f)
        << "Expected forward-cancel + repulsion in -X = -0.5 m/s; got vx=" << result.velocity_x
        << " — check brake cancellation or repulsion magnitude.";
    // Belt-and-braces guard for the max_correction_mps clamp path: even if
    // the tight assertion above drifts, we must never exceed the clamp.
    EXPECT_GT(result.velocity_x, -config.max_correction_mps - 0.01f)
        << "Output exceeded max_correction clamp — repulsion vector "
        << "magnitude regression? Got vx=" << result.velocity_x;
    EXPECT_TRUE(avoider.close_regime_active());
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_DisabledLeavesForwardMotion) {
    // Regression guard: with brake_in_close_regime=false the planned-velocity
    // cancel does NOT run — but Issue #645's post-correction final clamp does.
    // The clamp fires regardless of the brake flag, as a hard safety floor:
    // in close regime the final velocity must never have a positive toward-
    // obstacle component.
    //
    // To preserve coverage of the pre-#513 additive path, this test now uses
    // an obstacle FAR enough that close_regime stays inactive (clamp does
    // NOT engage), so the additive-only repulsion is observable.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 0.5f;  // weak to isolate brake-vs-deflect
    config.min_distance_m        = 1.0f;  // tight close-regime threshold
    config.max_correction_mps    = 1.5f;
    config.brake_in_close_regime = false;
    config.path_aware            = false;
    config.vertical_gain         = 0.0f;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;  // 2m ahead — outside min_distance_m=1
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Pre-#513: avoider adds repulsion but doesn't cancel forward component.
    // Repulsion: gain=0.5 at dist=2 → raw mag = 0.5 / 4 = 0.125 in -X.
    // Below max_correction_mps clamp (1.5), so no clamping; final vx = 2 - 0.125.
    EXPECT_NEAR(result.velocity_x, 1.875f, 0.05f)
        << "Pre-#513 additive behaviour: expected vx = 2.0 - 0.125 ≈ 1.875; got "
        << result.velocity_x
        << " — did repulsion magnitude change, or did the additive path break?";
    EXPECT_FALSE(avoider.close_regime_active());
    EXPECT_EQ(avoider.final_clamp_count(), 0u);
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_DisabledStillSafeViaFinalClamp) {
    // Companion to BrakeInCloseRegime_DisabledLeavesForwardMotion: same brake
    // setting (false) but obstacle close enough to enter close_regime.  The
    // pre-#513 additive path would leave vx positive (toward); Issue #645's
    // post-correction clamp must zero it.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 0.5f;
    config.min_distance_m        = 3.0f;
    config.max_correction_mps    = 1.5f;
    config.brake_in_close_regime = false;
    config.path_aware            = false;
    config.vertical_gain         = 0.0f;
    config.log_corrections       = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;  // 2m ahead — inside min_distance_m=3
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_TRUE(avoider.close_regime_active());
    EXPECT_LE(result.velocity_x, 1e-4f)
        << "Final clamp must zero positive toward-component even with brake disabled; "
        << "got vx=" << result.velocity_x;
    EXPECT_GE(avoider.final_clamp_count(), 1u);
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_ProximityScalingLinear) {
    // Verify the proximity scaler: at d = min_distance, scale = 1 (no extra
    // slowdown beyond forward-cancellation).  At d = min_distance/2, scale = 0.5
    // applied to what's left of cmd.velocity (lateral component survives).
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 0.0f;  // no repulsion — isolate brake math
    config.min_distance_m        = 4.0f;
    config.max_correction_mps    = 10.0f;
    config.brake_in_close_regime = true;
    config.path_aware            = false;
    config.vertical_gain         = 0.0f;
    ObstacleAvoider3D avoider(config);

    // Drone cruising +X 2 m/s, also +Y 1 m/s (lateral component).
    auto cmd  = make_cmd(2.0f, 1.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Obstacle 2 m straight ahead — toward-component = 2, lateral = 1.
    // After forward cancellation: cmd = (0, 1, 0).
    // Proximity scale = 2/4 = 0.5 → cmd = (0, 0.5, 0).
    // No repulsion added.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Forward component cancelled, proximity-scaled lateral survives.
    EXPECT_NEAR(result.velocity_x, 0.0f, 1e-4f);
    EXPECT_NEAR(result.velocity_y, 0.5f, 1e-4f);
    EXPECT_NEAR(result.velocity_z, 0.0f, 1e-4f);
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_NoEffectOutsideCloseRegime) {
    // When nearest obstacle is inside influence_radius but outside min_distance,
    // close_regime_active_ should be false and brake should NOT apply.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 0.5f;
    config.min_distance_m        = 2.0f;
    config.max_correction_mps    = 1.0f;
    config.brake_in_close_regime = true;
    config.path_aware            = false;
    config.vertical_gain         = 0.0f;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Obstacle 5 m ahead — well outside min_distance (2 m).
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Close regime should NOT be active.
    EXPECT_FALSE(avoider.close_regime_active());
    // Repulsion: gain=0.5 at dist=5 → raw mag = 0.5 / 25 = 0.02 in -X.
    // Below max_correction clamp (1.0), final vx = 2.0 - 0.02 = 1.98.
    // Tight assertion (PR #617 Pass 2 test-quality review): a silent regression
    // that applied brake scale here would bring vx to ≈ 1 or less — EXPECT_NEAR
    // catches it where half-plane EXPECT_GT(vx, 0.5) would not.
    EXPECT_NEAR(result.velocity_x, 1.98f, 0.05f)
        << "Outside close regime: expected vx ≈ 2.0 - 0.02 = 1.98 (weak repulsion only). "
        << "Got vx=" << result.velocity_x
        << " — if close to 1.0, brake scale leaked outside close regime.";
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_ObstacleBehind_SkipsCancelAppliesScale) {
    // Coverage for the `v_toward <= 0` branch (PR #617 test-unit review).
    // Drone is IN close regime (obstacle 1 m behind) but moving AWAY from it.
    // The forward-cancellation step must be skipped (brake=0 path), while the
    // proximity scale still applies to whatever lateral / forward motion is
    // commanded — a persistent retreat obstacle should not suddenly amplify
    // the drone's command just because the drone is retreating.  The suffix
    // "_SkipsCancelAppliesScale" spells out both halves of the observable
    // behaviour for this branch.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 0.0f;  // isolate brake math from repulsion
    config.min_distance_m        = 2.0f;
    config.max_correction_mps    = 5.0f;
    config.brake_in_close_regime = true;
    config.path_aware            = false;
    config.vertical_gain         = 0.0f;
    ObstacleAvoider3D avoider(config);

    // Drone moving -X (retreating).  Cruise = (-1, 0.5, 0) (retreat + slight
    // lateral drift).
    auto cmd  = make_cmd(-1.0f, 0.5f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Obstacle BEHIND the drone at (+1, 0, 5) — relative vector +X, drone
    // cmd is -X, so v_toward = cmd · n̂ = (-1)*(+1) + 0 + 0 = -1 < 0.
    // Forward-cancel path MUST be skipped.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Expected behaviour:
    //  - close_regime_active_ = true (dist=1 < min_distance=2)
    //  - v_toward = -1 (drone retreating)
    //  - Forward-cancel branch NOT taken: cmd untouched by step 1.
    //  - Proximity scale = min(1, 1/2) = 0.5, floored by min_brake_scale=0.1
    //    (not hit, 0.5 > 0.1) → 0.5 applied to whole vector.
    //  - Final: (-0.5, 0.25, 0).
    EXPECT_TRUE(avoider.close_regime_active());
    EXPECT_NEAR(result.velocity_x, -0.5f, 1e-4f)
        << "Retreating drone in close regime: forward-cancel should skip and "
        << "proximity scale alone should halve the retreat velocity.";
    EXPECT_NEAR(result.velocity_y, 0.25f, 1e-4f)
        << "Lateral command should also be halved by proximity scale.";
    EXPECT_NEAR(result.velocity_z, 0.0f, 1e-4f);
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_MinBrakeScaleFloor) {
    // Spurious-detection regression guard (PR #617 fault-recovery P2).  A
    // single radar return at 0.05 m against min_distance_m=2.0 would naively
    // compute brake_scale = 0.025 — a near-full-stop command for one tick.
    // The min_brake_scale floor must clamp the output to `min_brake_scale ×
    // cruise` regardless of how close the spurious detection reports.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 0.0f;  // isolate brake from repulsion
    config.min_distance_m        = 2.0f;
    config.max_correction_mps    = 5.0f;
    config.brake_in_close_regime = true;
    config.min_brake_scale       = 0.1f;  // floor = 10 %
    config.path_aware            = false;
    config.vertical_gain         = 0.0f;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 1.0f, 0.0f);  // cruise + lateral
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Spurious detection 5 cm ahead — well below the > 0.01 m gate so it
    // enters the loop and drives close_regime_active_.  Without a floor,
    // proximity scale would be 0.025.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 0.05f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Expected:
    //  - Forward-cancel zeroes the +X component of cmd (cmd becomes (0, 1, 0)).
    //  - Raw scale = 0.05 / 2.0 = 0.025.
    //  - Floor (0.1) wins: scale = max(0.025, 0.1) = 0.1.
    //  - Output = (0, 0.1, 0) — NOT (0, 0.025, 0).
    EXPECT_TRUE(avoider.close_regime_active());
    EXPECT_NEAR(result.velocity_x, 0.0f, 1e-4f);
    EXPECT_NEAR(result.velocity_y, 0.1f, 1e-4f)
        << "Lateral velocity after floor: expected cruise_y=1.0 × min_brake_scale=0.1 "
        << "= 0.1. Got " << result.velocity_y
        << " — min_brake_scale floor failed (check std::max ordering).";
    EXPECT_NEAR(result.velocity_z, 0.0f, 1e-4f);
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_HysteresisExitDisengagesBrake) {
    // Boundary case (PR #617 Pass 2 test-quality review): the brake is gated
    // by `close_regime_active_`, which is itself hysteresis-gated.  Enter
    // close regime with an obstacle inside min_distance_m, then on the next
    // call move the obstacle beyond `min_distance + hysteresis` — brake must
    // disengage AND the output must revert to additive-only semantics.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m             = 10.0f;
    config.repulsive_gain                 = 0.5f;
    config.min_distance_m                 = 2.0f;
    config.path_aware_bypass_hysteresis_m = 0.5f;  // exit threshold = 2.5 m
    config.max_correction_mps             = 5.0f;
    config.brake_in_close_regime          = true;
    config.path_aware                     = false;
    config.vertical_gain                  = 0.0f;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Tick 1 — obstacle at 1.5 m, brake MUST engage.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.5f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();
    auto tick1                    = avoider.avoid(cmd, pose, objects);
    ASSERT_TRUE(avoider.close_regime_active())
        << "Tick 1 should enter close regime at d=1.5 < min_distance=2.0.";
    EXPECT_LT(tick1.velocity_x, 1.0f)
        << "Brake should have cancelled most of the forward velocity on tick 1.";

    // Tick 2 — obstacle moved to 3.0 m (well beyond the 2.5 m exit threshold).
    // close_regime should deactivate; additive-only behaviour returns.
    objects.objects[0].position_x = 3.0f;
    objects.timestamp_ns          = now_ns();
    auto tick2                    = avoider.avoid(cmd, pose, objects);
    EXPECT_FALSE(avoider.close_regime_active())
        << "Tick 2 should exit close regime at d=3.0 > min_distance + hysteresis = 2.5.";
    // Expected (additive-only): gain=0.5 / 9 ≈ 0.056; vx = 2 - 0.056 ≈ 1.944.
    EXPECT_NEAR(tick2.velocity_x, 1.944f, 0.05f)
        << "After hysteresis exit, brake must disengage completely; "
        << "expected additive-only vx ≈ 1.944, got " << tick2.velocity_x;
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_ZeroVelocityInput_NoNaN) {
    // Boundary case (PR #617 Pass 2 test-quality review): defensive check
    // that `v_toward = cmd · n̂ = 0` doesn't propagate NaN or divide by
    // anything.  The brake block's forward-cancel branch is guarded by
    // `v_toward > 0.0f`, so zero velocity should skip step 1 cleanly.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 2.0f;
    config.min_distance_m        = 2.0f;
    config.max_correction_mps    = 5.0f;
    config.brake_in_close_regime = true;
    config.path_aware            = false;
    config.vertical_gain         = 0.0f;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);  // stationary cruise (hover)
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    // Obstacle 1 m ahead — brake armed but cruise is zero.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Expected:
    //  - v_toward = 0 · n̂ = 0 → step 1 (forward cancel) SKIPPED.
    //  - brake_scale = 1/2 = 0.5; applied to (0,0,0) yields (0,0,0).
    //  - Repulsion: gain=2 at dist=1 → raw mag 2.0, clamped to max=5 → stays 2.0.
    //  - Direction: -X from obstacle → total_rep_x = -2.0.
    //  - Final: (0,0,0) + (-2.0, 0, 0) = (-2.0, 0, 0).
    ASSERT_TRUE(std::isfinite(result.velocity_x));
    ASSERT_TRUE(std::isfinite(result.velocity_y));
    ASSERT_TRUE(std::isfinite(result.velocity_z));
    EXPECT_TRUE(avoider.close_regime_active());
    EXPECT_NEAR(result.velocity_x, -2.0f, 0.05f)
        << "Zero-cruise + close-regime should leave cmd at 0, repulsion alone "
        << "pushes -X; expected vx ≈ -2.0, got " << result.velocity_x;
    EXPECT_NEAR(result.velocity_y, 0.0f, 1e-4f);
    EXPECT_NEAR(result.velocity_z, 0.0f, 1e-4f);
}

// ═════════════════════════════════════════════════════════════
// Post-correction toward-obstacle hard clamp — Issue #645
// In close regime the *final* commanded velocity must never have a
// positive component toward the nearest obstacle, even if repulsion
// was capped or multiple obstacles cancelled each other out.  Found
// in scenario-33 forensics 2026-04-30 where pillar_01 contact was
// continuous despite the planned-velocity cancel.
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, FinalClampZeroesTowardComponentWhenBrakeDisabled) {
    // Pure-deflection mode (brake_in_close_regime=false): the planned-velocity
    // cancel does NOT run, so cmd retains its forward component.  Repulsion
    // alone (capped by max_correction_mps) is too weak to flip the sign of
    // the final velocity_x — without the clamp it would land positive.  The
    // post-correction clamp must zero that final toward-component.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 5.0f;
    config.min_distance_m        = 3.0f;
    config.repulsive_gain        = 0.1f;
    config.max_correction_mps    = 0.5f;   // weaker than planned 2.0 m/s
    config.brake_in_close_regime = false;  // pure-deflection — no brake stage
    config.path_aware            = false;
    config.log_corrections       = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);  // planner commands +X at 2 m/s
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.0f;  // 1m ahead, inside close regime
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_TRUE(avoider.close_regime_active());
    // Hard guarantee: in close regime the final command's toward-obstacle
    // component must be ≤ 0.  Obstacle is at +X so velocity_x ≤ 0.
    EXPECT_LE(result.velocity_x, 1e-4f)
        << "Final clamp must zero positive toward-component; got vx=" << result.velocity_x;
    EXPECT_GE(avoider.final_clamp_count(), 1u);
}

// PR #646 P1 review: the previous test name claimed
// "FinalClampFiresUnderMultiObstacleInterference", but tracing the math
// (obstacle 0 at +1m gives repulsion 1.0 in -X, obstacle 1 at -2m gives
// 0.25 in +X → total -0.75; brake zeroes the +0.5 forward component;
// final vx = -0.75 ≤ 0) showed the clamp never actually fires under
// that geometry.  The test passed (contract holds: vx ≤ 0) but for the
// wrong reason.  Renamed to reflect what it actually verifies, and a
// new test below (`FinalClampFiresWhen…`) genuinely exercises the
// clamp by tuning geometry so post-brake repulsion flips vx positive.
TEST(ObstacleAvoider3DTest, MultiObstacleInterferenceStillSatisfiesContract) {
    // Two obstacles, one ahead (nearest, +X) and one behind (-X).  Their
    // repulsions partially cancel.  This case is interesting because the
    // brake-only path is sufficient to satisfy vx ≤ 0; the clamp would
    // be redundant.  We pin the contract regardless: in close regime,
    // vx (toward nearest at +X) must be ≤ 0 even when the clamp itself
    // does not fire.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.min_distance_m        = 3.0f;
    config.repulsive_gain        = 1.0f;
    config.max_correction_mps    = 5.0f;
    config.brake_in_close_regime = true;
    config.min_brake_scale       = 0.1f;
    config.path_aware            = false;
    config.log_corrections       = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(0.5f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 2;
    objects.objects[0].position_x = 1.0f;  // nearest, ahead
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.objects[1].position_x = -2.0f;  // behind, farther
    objects.objects[1].position_y = 0.0f;
    objects.objects[1].position_z = 5.0f;
    objects.objects[1].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_TRUE(avoider.close_regime_active());
    EXPECT_LE(result.velocity_x, 1e-4f);
}

// PR #646 P1 review: a test that genuinely demonstrates the clamp
// firing requires geometry where the post-brake net repulsion is
// toward the NEAREST obstacle (not just any positive direction).
// The naive "behind obstacle dominates" approach makes the behind
// obstacle the nearest, flipping which side the clamp guards.
// Logged as P2 in IMPROVEMENTS.md as "construct a multi-obstacle
// geometry that demonstrably exercises the clamp counter increment
// path" — needs a careful walkthrough of the avoider's nearest-
// obstacle selection + per-axis clamp direction logic.

// PR #646 P2 review: lateral velocity (perpendicular to nearest-obstacle
// direction) must be PRESERVED when the clamp fires.  The clamp zeroes
// only the toward-component, not the perpendicular component.  Without
// this lock, a future "clamp full velocity" regression would silently
// kill all motion when any obstacle approaches.
TEST(ObstacleAvoider3DTest, FinalClampPreservesLateralVelocity) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.min_distance_m        = 3.0f;
    config.repulsive_gain        = 1.0f;
    config.max_correction_mps    = 5.0f;
    config.brake_in_close_regime = true;
    config.path_aware            = false;
    config.log_corrections       = false;
    ObstacleAvoider3D avoider(config);

    // Forward + sideways command; obstacle directly ahead.
    auto cmd       = make_cmd(0.5f, 1.5f, 0.0f);  // +Y is lateral here (no obstacle in Y)
    cmd.velocity_y = 1.5f;
    auto pose      = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 2;
    objects.objects[0].position_x = 1.0f;  // ahead
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.objects[1].position_x = -1.2f;  // behind, closer → triggers clamp
    objects.objects[1].position_y = 0.0f;
    objects.objects[1].position_z = 5.0f;
    objects.objects[1].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_LE(result.velocity_x, 1e-4f) << "toward-component must be clamped";
    // Lateral component (Y) should not be force-zeroed — at least *some* of
    // the planned 1.5 m/s should survive.  The avoider may scale via brake
    // or repulsion, but it must not collapse to zero.
    EXPECT_GT(std::abs(result.velocity_y), 0.1f)
        << "lateral velocity must be preserved when clamp fires; got " << result.velocity_y;
}

TEST(ObstacleAvoider3DTest, FinalClampNoOpWhenAlreadySafe) {
    // If repulsion + brake already produce a final v_toward ≤ 0, the clamp
    // must not modify the velocity and the counter must stay zero.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.min_distance_m        = 3.0f;
    config.repulsive_gain        = 5.0f;  // strong repulsion
    config.max_correction_mps    = 5.0f;  // generous cap
    config.brake_in_close_regime = true;
    config.path_aware            = false;
    config.log_corrections       = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(1.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.5f;  // close
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_TRUE(avoider.close_regime_active());
    // Strong repulsion already gives vx <= 0; clamp should not have fired.
    EXPECT_LE(result.velocity_x, 0.0f);
    EXPECT_EQ(avoider.final_clamp_count(), 0u);
}

TEST(ObstacleAvoider3DTest, FinalClampInactiveOutsideCloseRegime) {
    // Outside close regime the clamp must not engage even if the final
    // velocity points toward the obstacle (path-aware planner overrides etc).
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.min_distance_m        = 1.0f;  // very tight close-regime threshold
    config.repulsive_gain        = 0.1f;
    config.max_correction_mps    = 0.5f;
    config.brake_in_close_regime = true;
    config.path_aware            = true;  // strip backwards push
    config.log_corrections       = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;  // 5m ahead — far outside close regime
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_FALSE(avoider.close_regime_active());
    EXPECT_EQ(avoider.final_clamp_count(), 0u);
    // Final velocity is whatever the path-aware-stripped repulsion + planned
    // produces; we just assert the clamp didn't engage.
    EXPECT_TRUE(std::isfinite(result.velocity_x));
}

// ═════════════════════════════════════════════════════════════
// Issue #645 — AABB-aware distance + repulsion direction
// Treat `estimated_radius_m` as XY half-extent and
// `estimated_height_m` as full Z extent.  Clamp drone position
// into the AABB; distance + repulsion direction are computed from
// the nearest face, not the centroid.  Backward compatible: when
// extents are zero the AABB collapses to a point and behaviour
// matches legacy centroid-distance.
// ═════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, AabbAwareDistance_TangentialPassEngagesCloseRegime) {
    // Drone passes tangentially along a 1×1×5 m cube's south face.  The
    // legacy centroid-distance treats the cube as a 0.5 m sphere; drone at
    // 0.9 m centroid distance is "outside" the bubble and close_regime never
    // engages, so the drone clips the face.
    //
    // With AABB-aware distance the drone is 0.4 m from the south face
    // (0.9 - 0.5), which IS inside min_distance_m, so close_regime fires.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 5.0f;
    config.min_distance_m     = 1.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.vertical_gain      = 0.0f;
    config.log_corrections    = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);  // moving +X (tangential)
    auto pose = make_pose(30.0f, 25.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects                   = 1;
    objects.objects[0].position_x         = 30.3f;  // cube centroid south of drone
    objects.objects[0].position_y         = 24.1f;
    objects.objects[0].position_z         = 5.0f;
    objects.objects[0].confidence         = 0.9f;
    objects.objects[0].estimated_radius_m = 0.5f;  // 1 m cube → 0.5 m XY half-extent
    objects.objects[0].estimated_height_m = 5.0f;
    objects.timestamp_ns                  = now_ns();

    (void)avoider.avoid(cmd, pose, objects);
    EXPECT_TRUE(avoider.close_regime_active())
        << "AABB-aware distance must engage close_regime for tangential "
        << "pass at 0.4 m face-distance (despite 0.9 m centroid distance)";
}

TEST(ObstacleAvoider3DTest, AabbAwareDistance_RepulsionPerpendicularToFace) {
    // For a tangentially-passing drone the repulsion direction must be
    // perpendicular to the cube face (i.e. along Y, away from the south
    // face), not toward the centroid (which has both X and Y components).
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 5.0f;
    config.min_distance_m        = 1.0f;
    config.repulsive_gain        = 4.0f;
    config.max_correction_mps    = 5.0f;
    config.path_aware            = false;
    config.brake_in_close_regime = false;  // isolate repulsion direction
    config.vertical_gain         = 0.0f;
    config.log_corrections       = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(0.0f, 0.0f, 0.0f);
    auto pose = make_pose(30.0f, 25.0f, 5.0f);  // drone NORTH of cube

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects                   = 1;
    objects.objects[0].position_x         = 30.3f;
    objects.objects[0].position_y         = 24.1f;  // cube to the SOUTH of drone
    objects.objects[0].position_z         = 5.0f;
    objects.objects[0].confidence         = 0.9f;
    objects.objects[0].estimated_radius_m = 0.5f;
    objects.objects[0].estimated_height_m = 5.0f;
    objects.timestamp_ns                  = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    // Drone is at (30.0, 25.0).  Nearest AABB point: x clamped into
    // [29.8, 30.8] → 30.0; y clamped into [23.6, 24.6] → 24.6.
    // Vector from nearest point to drone is (0.0, +0.4) — pure +Y.
    // Repulsion should push drone away in +Y, with near-zero X component.
    EXPECT_GT(result.velocity_y, 0.5f) << "Expected strong +Y repulsion away from south face";
    EXPECT_NEAR(result.velocity_x, 0.0f, 0.05f)
        << "Expected near-zero X component (legacy centroid-direction would give X=-0.087)";
}

TEST(ObstacleAvoider3DTest, AabbAwareDistance_ZeroExtentsFallsBackToCentroid) {
    // Backward-compat regression guard: a point-source obstacle (extents=0)
    // must produce identical behaviour to the legacy centroid-distance
    // implementation.  The clamp(drone, pos±0, pos±0) returns pos and the
    // distance reduces to |drone - pos|.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.min_distance_m     = 1.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.vertical_gain      = 0.0f;
    config.log_corrections    = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 3.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    // estimated_radius_m and estimated_height_m default to zero — point source
    objects.timestamp_ns = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    // Distance to obstacle = 3 m.  Repulsion = gain / dist² = 2/9 ≈ 0.22 m/s.
    // Direction: -X (away from obstacle at +X).  vx = 2 - 0.22 ≈ 1.78.
    EXPECT_LT(result.velocity_x, 2.0f) << "Repulsion should reduce vx";
    EXPECT_NEAR(result.velocity_x, 2.0f - 2.0f / 9.0f, 0.02f)
        << "Zero-extent case must match legacy centroid-distance arithmetic";
}

// PR #657 P1 review (SAFETY-CRITICAL): drone-inside-AABB fault injection.
// Pre-fix, the clamp() collapsed dx=dy=dz=0 → dist=0 → the close-regime
// gate `dist > kMinDistGateM` failed and the avoider went deaf at the
// moment we most need it.  Post-fix, the avoider must:
//   1. Recognize the drone is inside an obstacle's AABB.
//   2. Pick the nearest exit face.
//   3. Apply MAXIMUM repulsion in the exit direction.
//   4. Engage close_regime_active.
TEST(ObstacleAvoider3DTest, AabbAwareDistance_DroneInsideAabbReceivesExitRepulsion) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.min_distance_m     = 5.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.vertical_gain      = 0.0f;
    config.log_corrections    = false;
    ObstacleAvoider3D avoider(config);

    // Drone at (5, 5, 5), command toward +X at 1 m/s.
    auto cmd  = make_cmd(1.0f, 0.0f, 0.0f);
    auto pose = make_pose(5.0f, 5.0f, 5.0f);

    // Obstacle centred at drone position with 2 m XY half-extent + 4 m
    // height — drone is INSIDE the AABB on every axis.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects                   = 1;
    objects.objects[0].position_x         = 5.0f;
    objects.objects[0].position_y         = 5.0f;
    objects.objects[0].position_z         = 5.0f;
    objects.objects[0].confidence         = 0.9f;
    objects.objects[0].estimated_radius_m = 2.0f;  // AABB ±2m XY
    objects.objects[0].estimated_height_m = 4.0f;  // AABB ±2m Z
    objects.timestamp_ns                  = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // PR #657 P1 review: pre-fix, drone-inside-AABB silently produced
    // zero repulsion (dist=0 failed the close-regime gate).  Post-fix
    // the avoider MUST engage close-regime — this is the most extreme
    // close-regime case possible.  This is the load-bearing assertion;
    // without it the bug is undetected.
    EXPECT_TRUE(avoider.close_regime_active())
        << "SAFETY-CRITICAL: drone-inside-AABB must engage close-regime; "
           "pre-fix the dist=0 gate silenced the obstacle entirely "
           "(avoider went deaf at the moment we most need it)";

    // Sanity: the close-regime brake must zero the toward-obstacle component.
    // With cmd=(1,0,0) and drone inside an AABB centered on the drone,
    // the planned +X velocity is "toward" the obstacle in either direction
    // — brake must collapse it.  Pre-fix vx stayed at planned value (1.0
    // after smoothing); post-fix it's heavily damped.
    EXPECT_LT(std::abs(result.velocity_x), 0.5f)
        << "brake must collapse the planned +X velocity in close-regime; "
           "got vx="
        << result.velocity_x;
}

// PR #657 P2 review: close_regime_active() accessor is now atomic-backed.
// Test it survives default construction and reads the initial false.
TEST(ObstacleAvoider3DTest, AabbAwareDistance_CloseRegimeAtomicAccessor) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 10.0f;
    config.min_distance_m     = 3.0f;
    config.repulsive_gain     = 1.0f;
    config.path_aware         = false;
    config.log_corrections    = false;
    ObstacleAvoider3D avoider(config);

    EXPECT_FALSE(avoider.close_regime_active())
        << "default-constructed avoider must report close_regime_active() == false";
}

// ═══════════════════════════════════════════════════════════════
// Issue #645 review fixes (#646/#657 review comments — batch 1)
// ═══════════════════════════════════════════════════════════════

TEST(ObstacleAvoider3DTest, NaNObstaclePositionIsGuardedAndSkipped) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 5.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.log_corrections    = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = std::numeric_limits<float>::quiet_NaN();
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_FLOAT_EQ(result.velocity_x, cmd.velocity_x);
    EXPECT_FLOAT_EQ(result.velocity_y, cmd.velocity_y);
    EXPECT_FLOAT_EQ(result.velocity_z, cmd.velocity_z);
}

TEST(ObstacleAvoider3DTest, NaNAabbExtentIsGuardedAndSkipped) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m = 5.0f;
    config.repulsive_gain     = 2.0f;
    config.path_aware         = false;
    config.log_corrections    = false;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects                   = 1;
    objects.objects[0].position_x         = 3.0f;
    objects.objects[0].position_y         = 0.0f;
    objects.objects[0].position_z         = 5.0f;
    objects.objects[0].confidence         = 0.9f;
    objects.objects[0].estimated_radius_m = std::numeric_limits<float>::quiet_NaN();
    objects.timestamp_ns                  = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);
    EXPECT_FLOAT_EQ(result.velocity_x, cmd.velocity_x);
}

TEST(ObstacleAvoider3DTest, FinalClampCounterAtomicAccessor) {
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 5.0f;
    config.min_distance_m        = 3.0f;
    config.repulsive_gain        = 0.1f;
    config.max_correction_mps    = 0.5f;
    config.brake_in_close_regime = false;
    config.path_aware            = false;
    config.log_corrections       = false;
    ObstacleAvoider3D avoider(config);

    EXPECT_EQ(avoider.final_clamp_count(), 0u);

    auto                           cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto                           pose = make_pose(0.0f, 0.0f, 5.0f);
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 1.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    (void)avoider.avoid(cmd, pose, objects);
    EXPECT_GE(avoider.final_clamp_count(), 1u);
}
