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
    auto avoider = create_obstacle_avoider("3d");
    EXPECT_NE(avoider, nullptr);
    EXPECT_EQ(avoider->name(), "ObstacleAvoider3D");
}

TEST(ObstacleAvoiderFactory, AlternateNameRegistered) {
    auto avoider = create_obstacle_avoider("obstacle_avoider_3d");
    EXPECT_NE(avoider, nullptr);
    EXPECT_EQ(avoider->name(), "ObstacleAvoider3D");
}

TEST(ObstacleAvoiderFactory, PotentialField3DRegistered) {
    auto avoider = create_obstacle_avoider("potential_field_3d");
    EXPECT_NE(avoider, nullptr);
    EXPECT_EQ(avoider->name(), "ObstacleAvoider3D");
}

TEST(ObstacleAvoiderFactory, UnknownThrows) {
    EXPECT_THROW(create_obstacle_avoider("nonexistent"), std::runtime_error);
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
