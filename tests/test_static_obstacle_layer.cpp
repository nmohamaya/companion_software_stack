// tests/test_static_obstacle_layer.cpp
// Unit tests for StaticObstacleLayer (Issue #154).
#include "planner/static_obstacle_layer.h"

#include <gtest/gtest.h>

using namespace drone::planner;

namespace {

/// Make a detected object list with a single detection at the given position.
drone::ipc::DetectedObjectList make_detection(float x, float y, float z) {
    drone::ipc::DetectedObjectList list{};
    list.num_objects           = 1;
    list.objects[0].position_x = x;
    list.objects[0].position_y = y;
    list.objects[0].position_z = z;
    list.objects[0].confidence = 0.9f;
    list.objects[0].class_id   = drone::ipc::ObjectClass::BUILDING;
    return list;
}

}  // namespace

// ═══════════════════════════════════════════════════════════
// Load tests
// ═══════════════════════════════════════════════════════════

TEST(StaticObstacleLayerTest, LoadEmpty) {
    StaticObstacleLayer               layer;
    std::vector<StaticObstacleRecord> empty;
    layer.load(empty);
    EXPECT_TRUE(layer.empty());
    EXPECT_EQ(layer.obstacles().size(), 0u);
}

TEST(StaticObstacleLayerTest, LoadSingle) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 10.0f, 0.75f, 3.0f}});
    ASSERT_EQ(layer.obstacles().size(), 1u);
    EXPECT_FLOAT_EQ(layer.obstacles()[0].x, 5.0f);
    EXPECT_FLOAT_EQ(layer.obstacles()[0].y, 10.0f);
    EXPECT_FALSE(layer.obstacles()[0].confirmed);
}

TEST(StaticObstacleLayerTest, LoadMultiple) {
    StaticObstacleLayer layer;
    layer.load({{1.0f, 2.0f, 0.5f, 2.0f}, {3.0f, 4.0f, 1.0f, 4.0f}, {5.0f, 6.0f, 0.75f, 3.0f}});
    EXPECT_EQ(layer.obstacles().size(), 3u);
}

// ═══════════════════════════════════════════════════════════
// Cross-check confirmation
// ═══════════════════════════════════════════════════════════

TEST(StaticObstacleLayerTest, CrossCheckNeedsTwoHitsToConfirm) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});

    // First detection near the obstacle — should increment count but not confirm
    auto det1 = make_detection(5.5f, 5.0f, 2.0f);
    layer.cross_check(det1, 1, 1000);
    EXPECT_EQ(layer.obstacles()[0].confirm_count, 1);
    EXPECT_FALSE(layer.obstacles()[0].confirmed);

    // Second detection — should confirm
    auto det2 = make_detection(5.2f, 5.1f, 1.5f);
    layer.cross_check(det2, 1, 2000);
    EXPECT_EQ(layer.obstacles()[0].confirm_count, 2);
    EXPECT_TRUE(layer.obstacles()[0].confirmed);
    EXPECT_EQ(layer.obstacles()[0].first_confirmed_ns, 2000u);
}

TEST(StaticObstacleLayerTest, CrossCheckSkipsLowQualityPose) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});

    auto det = make_detection(5.5f, 5.0f, 2.0f);
    layer.cross_check(det, 0, 1000);  // quality < 1 → skip
    EXPECT_EQ(layer.obstacles()[0].confirm_count, 0);
}

TEST(StaticObstacleLayerTest, CrossCheckIgnoresDistantDetection) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});

    // Detection at (20, 20) — too far from obstacle at (5, 5)
    auto det = make_detection(20.0f, 20.0f, 2.0f);
    layer.cross_check(det, 1, 1000);
    EXPECT_EQ(layer.obstacles()[0].confirm_count, 0);
}

// ═══════════════════════════════════════════════════════════
// Collision detection
// ═══════════════════════════════════════════════════════════

TEST(StaticObstacleLayerTest, CollisionDetectedWhenWithinMargin) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});

    auto now = std::chrono::steady_clock::now();
    // Drone at (5.3, 5.0, 2.0) — within radius_m(1.0) + margin(0.5) = 1.5m
    bool collision = layer.check_collision(5.3f, 5.0f, 2.0f, now);
    EXPECT_TRUE(collision);
}

TEST(StaticObstacleLayerTest, NoCollisionWhenFarAway) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});

    auto now = std::chrono::steady_clock::now();
    // Drone at (10.0, 10.0) — far from obstacle
    bool collision = layer.check_collision(10.0f, 10.0f, 2.0f, now);
    EXPECT_FALSE(collision);
}

TEST(StaticObstacleLayerTest, CollisionCooldownThrottles) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});

    auto now = std::chrono::steady_clock::now();
    // First collision — should fire
    EXPECT_TRUE(layer.check_collision(5.3f, 5.0f, 2.0f, now));
    // Immediate second call — should be throttled (within 2s cooldown)
    EXPECT_FALSE(layer.check_collision(5.3f, 5.0f, 2.0f, now));
}

TEST(StaticObstacleLayerTest, NoCollisionAboveObstacleHeight) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});  // height_m = 3.0

    auto now = std::chrono::steady_clock::now();
    // Drone at altitude 5.0 — above height_m(3.0) + margin(0.5) = 3.5
    bool collision = layer.check_collision(5.3f, 5.0f, 5.0f, now);
    EXPECT_FALSE(collision);
}

// ═══════════════════════════════════════════════════════════
// Unconfirmed approach warning
// ═══════════════════════════════════════════════════════════

TEST(StaticObstacleLayerTest, UnconfirmedApproachWarning) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});

    auto now = std::chrono::steady_clock::now();
    // Drone at (6.0, 5.0) — within radius_m(1.0) + approach_warn(3.0) = 4.0m
    bool warned = layer.check_unconfirmed_approach(6.0f, 5.0f, now);
    EXPECT_TRUE(warned);
}

TEST(StaticObstacleLayerTest, ConfirmedObstacleNoApproachWarning) {
    StaticObstacleLayer layer;
    layer.load({{5.0f, 5.0f, 1.0f, 3.0f}});

    // Confirm the obstacle
    layer.obstacles()[0].confirmed = true;

    auto now    = std::chrono::steady_clock::now();
    bool warned = layer.check_unconfirmed_approach(6.0f, 5.0f, now);
    EXPECT_FALSE(warned);
}
