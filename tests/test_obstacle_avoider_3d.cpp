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

TEST(ObstacleAvoider3DTest, VeryCloseObjectMaxRepulsion) {
    // Objects at distance 0.05m (between old 0.1m threshold and new 0.01m)
    // should now receive maximum repulsion (clamped by max_correction_mps).
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
    //   2. Proximity scale = d/min_distance = 2/3 ≈ 0.67 applied to residual.
    //   3. Repulsion adds negative-X push (away from obstacle) so final vx < 0.
    // Key assertion: the forward motion was REVERSED, not merely deflected.
    EXPECT_LT(result.velocity_x, 0.0f)
        << "Forward velocity should be cancelled AND pushed backward by repulsion; "
        << "got vx=" << result.velocity_x << " (additive-only correction would leave vx > 0)";
    // Upper bound — runaway repulsion regression guard (PR #617 test-unit review).
    // max_correction_mps clamps the repulsion vector to 5 m/s; with the 0.01
    // clamp-path tolerance, output vx cannot drop below -5.01.
    EXPECT_GT(result.velocity_x, -config.max_correction_mps - 0.01f)
        << "Output exceeded max_correction clamp — repulsion vector "
        << "magnitude regression? Got vx=" << result.velocity_x;
    EXPECT_TRUE(avoider.close_regime_active());
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_DisabledLeavesForwardMotion) {
    // Regression guard: with brake_in_close_regime=false, the pre-#513 additive-
    // only behaviour must be preserved.  Same geometry as above — verify vx is
    // only lightly reduced, not reversed.
    ObstacleAvoider3DConfig config;
    config.influence_radius_m    = 10.0f;
    config.repulsive_gain        = 0.5f;  // weak to isolate brake-vs-deflect
    config.min_distance_m        = 3.0f;
    config.max_correction_mps    = 1.5f;   // limit correction authority
    config.brake_in_close_regime = false;  // pre-#513 behaviour
    config.path_aware            = false;
    config.vertical_gain         = 0.0f;
    ObstacleAvoider3D avoider(config);

    auto cmd  = make_cmd(2.0f, 0.0f, 0.0f);
    auto pose = make_pose(0.0f, 0.0f, 5.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 2.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    objects.timestamp_ns          = now_ns();

    auto result = avoider.avoid(cmd, pose, objects);

    // Pre-#513: avoider adds repulsion but doesn't cancel forward component.
    // With max_correction capped at 1.5 and cruise at 2.0, vx stays positive.
    EXPECT_GT(result.velocity_x, 0.0f)
        << "With brake_in_close_regime=false, pre-#513 additive behaviour "
        << "should leave forward motion above zero; got vx=" << result.velocity_x;
    EXPECT_TRUE(avoider.close_regime_active());
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
    // Forward motion should still be largely intact — only weak repulsion applied.
    EXPECT_GT(result.velocity_x, 0.5f)
        << "Outside close regime, brake should not apply; vx=" << result.velocity_x;
}

TEST(ObstacleAvoider3DTest, BrakeInCloseRegime_ObstacleBehind_NoForwardCancel) {
    // Coverage for the `v_toward <= 0` branch (PR #617 test-unit review).
    // Drone is IN close regime (obstacle 1 m behind) but moving AWAY from it.
    // The forward-cancellation step must be skipped (brake=0 path), while the
    // proximity scale still applies to whatever lateral / forward motion is
    // commanded — a persistent retreat obstacle should not suddenly amplify
    // the drone's command just because the drone is retreating.
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
