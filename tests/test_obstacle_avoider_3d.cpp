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
