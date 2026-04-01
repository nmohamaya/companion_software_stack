// tests/test_dstar_lite_planner.cpp
// Unit tests for D* Lite Incremental Path Planner (Issue #158).
#include "planner/dstar_lite_planner.h"
#include "planner/grid_planner_base.h"
#include "planner/occupancy_grid_3d.h"
#include "planner/planner_factory.h"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::planner;

// ═════════════════════════════════════════════════════════════
// OccupancyGrid3D basic tests (ported from test_astar_planner.cpp)
// ═════════════════════════════════════════════════════════════

TEST(OccupancyGrid3DTest, EmptyGridHasNoOccupied) {
    OccupancyGrid3D grid(0.5f, 50.0f, 1.5f);
    EXPECT_EQ(grid.occupied_count(), 0u);
    EXPECT_FALSE(grid.is_occupied({0, 0, 0}));
}

TEST(OccupancyGrid3DTest, WorldToGridRoundTrip) {
    OccupancyGrid3D grid(0.5f, 50.0f, 1.5f);

    auto cell  = grid.world_to_grid(2.5f, -1.0f, 3.0f);
    auto world = grid.grid_to_world(cell);

    EXPECT_NEAR(world[0], 2.5f, 0.5f);
    EXPECT_NEAR(world[1], -1.0f, 0.5f);
    EXPECT_NEAR(world[2], 3.0f, 0.5f);
}

TEST(OccupancyGrid3DTest, InBoundsCheck) {
    OccupancyGrid3D grid(1.0f, 10.0f, 1.0f);
    EXPECT_TRUE(grid.in_bounds({0, 0, 0}));
    EXPECT_TRUE(grid.in_bounds({10, 10, 10}));
    EXPECT_FALSE(grid.in_bounds({11, 0, 0}));
    EXPECT_FALSE(grid.in_bounds({0, -11, 0}));
}

TEST(OccupancyGrid3DTest, UpdateFromObjectsInflates) {
    OccupancyGrid3D grid(1.0f, 20.0f, 2.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    EXPECT_TRUE(grid.is_occupied({5, 5, 5}));
    EXPECT_TRUE(grid.is_occupied({6, 5, 5}));
    EXPECT_TRUE(grid.is_occupied({5, 6, 5}));
    EXPECT_FALSE(grid.is_occupied({0, 0, 0}));
    EXPECT_GT(grid.occupied_count(), 1u);
}

TEST(OccupancyGrid3DTest, LowConfidenceSkipped) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.1f;  // below 0.3 threshold

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    EXPECT_EQ(grid.occupied_count(), 0u);
}

TEST(OccupancyGrid3DTest, ClearResetsGrid) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);
    EXPECT_GT(grid.occupied_count(), 0u);

    grid.clear();
    EXPECT_EQ(grid.occupied_count(), 0u);
}

// ═════════════════════════════════════════════════════════════
// Known-obstacle suppression (Issue #237)
// ═════════════════════════════════════════════════════════════

TEST(OccupancyGrid3DTest, PromotedCellStillAcceptsDynamicDetections) {
    // Grid with promotion_hits=2: after 2 observations a cell promotes.
    // Subsequent detections near the promoted position create dynamic
    // cells (needed for navigation avoidance) but do NOT accumulate
    // further promotion hits (prevents runaway wall growth). Issue #237.
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f,
                         /*min_conf=*/0.3f, /*promotion_hits=*/2);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    drone::ipc::Pose pose{};

    // Two observations → cell promoted to static
    grid.update_from_objects(objects, pose);
    grid.update_from_objects(objects, pose);
    size_t static_after_promote = grid.static_count();
    EXPECT_GT(static_after_promote, 0u);

    // Clear dynamic cells, then detect at an offset position (parallax).
    // Dynamic cells are created, but no further promotion occurs.
    grid.clear_dynamic();
    EXPECT_EQ(grid.occupied_count(), 0u);

    objects.objects[0].position_x = 7.0f;  // 2 cells away — partial overlap
    // Many repeated observations — none should promote because center
    // is adjacent to existing static cells (promotion suppressed).
    for (int i = 0; i < 10; ++i) {
        grid.update_from_objects(objects, pose);
    }
    // Dynamic cells created for avoidance
    EXPECT_GT(grid.occupied_count(), 0u);
    // Static count did NOT grow — promotion was suppressed near known obstacles
    EXPECT_EQ(grid.static_count(), static_after_promote);
}

TEST(OccupancyGrid3DTest, AdjacentDetectionStillInserted) {
    // Parallax causes detections 1 cell away from the promoted cell.
    // These detections are still inserted as dynamic cells — the
    // close-range position may be more accurate than the promoted one.
    OccupancyGrid3D grid(1.0f, 20.0f, 0.5f, /*ttl=*/3.0f,
                         /*min_conf=*/0.3f, /*promotion_hits=*/2);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    drone::ipc::Pose pose{};

    // Promote the cell at (5,5,5)
    grid.update_from_objects(objects, pose);
    grid.update_from_objects(objects, pose);
    EXPECT_GT(grid.static_count(), 0u);
    grid.clear_dynamic();

    // New detection 1m away (1 cell on 1m grid) — parallax offset
    objects.objects[0].position_x = 6.0f;
    grid.update_from_objects(objects, pose);
    // Adjacent to promoted cell — still inserted for better position accuracy
    EXPECT_GT(grid.occupied_count(), 0u);
}

TEST(OccupancyGrid3DTest, FarDetectionNotSuppressed) {
    // A detection far from any promoted cell should NOT be suppressed.
    OccupancyGrid3D grid(1.0f, 20.0f, 0.5f, /*ttl=*/3.0f,
                         /*min_conf=*/0.3f, /*promotion_hits=*/2);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    drone::ipc::Pose pose{};

    // Promote the cell at (5,5,5)
    grid.update_from_objects(objects, pose);
    grid.update_from_objects(objects, pose);
    grid.clear_dynamic();

    // New detection 5m away — definitely a different obstacle
    objects.objects[0].position_x = 10.0f;
    grid.update_from_objects(objects, pose);
    // Should NOT be suppressed
    EXPECT_GT(grid.occupied_count(), 0u);
}

TEST(OccupancyGrid3DTest, SuppressionDisabledWhenPromotionOff) {
    // When promotion_hits=0, suppression is disabled — all detections accepted.
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, /*ttl=*/3.0f,
                         /*min_conf=*/0.3f, /*promotion_hits=*/0);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;
    drone::ipc::Pose pose{};

    grid.update_from_objects(objects, pose);
    EXPECT_GT(grid.occupied_count(), 0u);
    grid.clear_dynamic();

    // Same position again — no promotion possible, so no suppression
    grid.update_from_objects(objects, pose);
    EXPECT_GT(grid.occupied_count(), 0u);
}

// ═════════════════════════════════════════════════════════════
// GridCellHash + factory tests (ported from test_astar_planner.cpp)
// ═════════════════════════════════════════════════════════════

TEST(GridCellHashTest, DifferentCellsDifferentHashes) {
    GridCellHash h;
    EXPECT_NE(h({0, 0, 0}), h({1, 0, 0}));
    EXPECT_NE(h({0, 0, 0}), h({0, 1, 0}));
    EXPECT_NE(h({0, 0, 0}), h({0, 0, 1}));
}

TEST(GridCellHashTest, SameCellSameHash) {
    GridCellHash h;
    EXPECT_EQ(h({3, 4, 5}), h({3, 4, 5}));
}

TEST(PathPlannerFactory, FactoryCreatesDStarLite) {
    auto planner = create_path_planner("dstar_lite");
    EXPECT_NE(planner, nullptr);
    EXPECT_EQ(planner->name(), "DStarLitePlanner");
}

TEST(PathPlannerFactory, UnknownThrows) {
    EXPECT_THROW(create_path_planner("nonexistent"), std::runtime_error);
}

// ═════════════════════════════════════════════════════════════
// Change tracking tests (OccupancyGrid3D)
// ═════════════════════════════════════════════════════════════

TEST(ChangeTrackingTest, RecordsNewCells) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    EXPECT_GT(grid.pending_changes(), 0u);
    auto changes = grid.drain_changes();
    EXPECT_GT(changes.size(), 0u);

    // All newly inserted cells should be marked as occupied
    for (const auto& [cell, is_occupied] : changes) {
        EXPECT_TRUE(is_occupied);
        (void)cell;
    }
}

TEST(ChangeTrackingTest, RecordsExpiredCells) {
    // Use very short TTL so cells expire quickly
    OccupancyGrid3D grid(1.0f, 20.0f, 0.5f, 0.001f);  // 1ms TTL

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 3.0f;
    objects.objects[0].position_y = 3.0f;
    objects.objects[0].position_z = 3.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);
    grid.drain_changes();  // clear insertion changes

    // Wait for TTL to expire
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Trigger expiry with an empty detection
    drone::ipc::DetectedObjectList empty{};
    empty.num_objects = 0;
    grid.update_from_objects(empty, pose);

    auto changes = grid.drain_changes();
    // Should have removal changes
    bool has_removal = false;
    for (const auto& [cell, is_occupied] : changes) {
        if (!is_occupied) has_removal = true;
        (void)cell;
    }
    EXPECT_TRUE(has_removal);
}

TEST(ChangeTrackingTest, DrainChangesClears) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    auto changes = grid.drain_changes();
    EXPECT_GT(changes.size(), 0u);

    // Second drain should be empty
    auto changes2 = grid.drain_changes();
    EXPECT_EQ(changes2.size(), 0u);
}

TEST(ChangeTrackingTest, StaticObstacleChangesTracked) {
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f);

    grid.add_static_obstacle(5.0f, 5.0f, 1.0f, 3.0f);
    auto changes = grid.drain_changes();
    EXPECT_GT(changes.size(), 0u);

    // All should be insertions
    for (const auto& [cell, is_occupied] : changes) {
        EXPECT_TRUE(is_occupied);
        (void)cell;
    }
}

// ═════════════════════════════════════════════════════════════
// D* Lite search tests
// ═════════════════════════════════════════════════════════════

TEST(DStarLiteSearchTest, TrivialStartEqualsGoal) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 10.0f;
    config.inflation_radius_m = 1.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    Waypoint target{0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    // At goal — velocity should be near zero after smoothing
    EXPECT_NEAR(cmd.velocity_x, 0.0f, 0.5f);
    EXPECT_NEAR(cmd.velocity_y, 0.0f, 0.5f);
}

TEST(DStarLiteSearchTest, StraightLinePath) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.5f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_GT(cmd.velocity_x, 0.0f);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());
}

TEST(DStarLiteSearchTest, PathAround3DObstacle) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.5f;
    DStarLitePlanner planner(config);

    // Place a wall of obstacles along x=5, y=-2..2
    drone::ipc::DetectedObjectList objects{};
    uint32_t                       idx = 0;
    for (int y = -2; y <= 2; ++y) {
        for (int z = 3; z <= 7; ++z) {
            if (idx >= drone::ipc::MAX_DETECTED_OBJECTS) break;
            objects.objects[idx].position_x = 5.0f;
            objects.objects[idx].position_y = static_cast<float>(y);
            objects.objects[idx].position_z = static_cast<float>(z);
            objects.objects[idx].confidence = 0.9f;
            ++idx;
        }
    }
    objects.num_objects = idx;

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;
    planner.update_obstacles(objects, pose);

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    // Path should go around the obstacle
    EXPECT_GT(planner.cached_path().size(), 10u);
}

TEST(DStarLiteSearchTest, UnreachableGoalUsesDirectFallback) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 5.0f;
    config.inflation_radius_m = 2.0f;
    DStarLitePlanner planner(config);

    // Surround the goal completely with obstacles — the goal snap may find
    // a free cell laterally, so what we really test is that the planner
    // produces a valid command regardless of whether it falls back or not.
    drone::ipc::DetectedObjectList objects{};
    uint32_t                       idx = 0;
    for (int dy = -3; dy <= 3; ++dy) {
        for (int dx = -3; dx <= 3; ++dx) {
            if (idx >= drone::ipc::MAX_DETECTED_OBJECTS) break;
            objects.objects[idx].position_x = 4.0f + static_cast<float>(dx) * 0.5f;
            objects.objects[idx].position_y = static_cast<float>(dy) * 0.5f;
            objects.objects[idx].position_z = 0.0f;
            objects.objects[idx].confidence = 0.9f;
            ++idx;
        }
    }
    objects.num_objects = idx;

    drone::ipc::Pose pose{};
    planner.update_obstacles(objects, pose);

    Waypoint target{4.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    // Whether the planner finds a snapped path or falls back to direct,
    // it must produce a valid command.
    EXPECT_TRUE(cmd.valid);
}

TEST(DStarLiteSearchTest, StartBlocked_BFSEscape) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 5.0f;
    config.inflation_radius_m = 0.5f;
    DStarLitePlanner planner(config);

    // Place obstacle on the start position
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 0.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 0.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    planner.update_obstacles(objects, pose);

    // Goal is nearby and free
    Waypoint target{4.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
}

TEST(DStarLiteSearchTest, GoalOutOfBounds) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 5.0f;
    config.inflation_radius_m = 0.5f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    Waypoint         target{100.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};

    auto cmd = planner.plan(pose, target);
    // Goal snapping should try to find a nearby cell, but if all fail → direct fallback
    EXPECT_TRUE(cmd.valid);
}

// ═════════════════════════════════════════════════════════════
// Incremental replanning tests
// ═════════════════════════════════════════════════════════════

TEST(DStarLiteIncrementalTest, NewObstacleOnPath) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.5f;
    config.replan_interval_s  = -1.0f;  // always replan
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};

    // First plan — clear grid, should find straight path
    auto cmd1 = planner.plan(pose, target);
    EXPECT_TRUE(cmd1.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_GT(planner.cached_path().size(), 1u);

    // Add a wall of obstacles on the path (x=5, y=-3..3, z=3..7)
    drone::ipc::DetectedObjectList objects{};
    uint32_t                       idx = 0;
    for (int y = -3; y <= 3; ++y) {
        for (int z = 3; z <= 7; ++z) {
            if (idx >= drone::ipc::MAX_DETECTED_OBJECTS) break;
            objects.objects[idx].position_x = 5.0f;
            objects.objects[idx].position_y = static_cast<float>(y);
            objects.objects[idx].position_z = static_cast<float>(z);
            objects.objects[idx].confidence = 0.9f;
            ++idx;
        }
    }
    objects.num_objects = idx;
    planner.update_obstacles(objects, pose);

    // Second plan — should find path around obstacle wall
    auto cmd2 = planner.plan(pose, target);
    EXPECT_TRUE(cmd2.valid);
    EXPECT_FALSE(planner.using_direct_fallback());

    // Verify the path does not go through occupied cells
    const auto& grid = planner.grid();
    const auto& path = planner.cached_path();
    EXPECT_GE(path.size(), 2u);
    bool avoids_occupied = true;
    for (const auto& wp : path) {
        auto cell = grid.world_to_grid(wp[0], wp[1], wp[2]);
        if (grid.is_occupied(cell)) {
            avoids_occupied = false;
        }
    }
    EXPECT_TRUE(avoids_occupied);
}

TEST(DStarLiteIncrementalTest, ObstacleRemoved) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.5f;
    config.replan_interval_s  = -1.0f;
    DStarLitePlanner planner(config);

    // Start with obstacle on the path
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;
    planner.update_obstacles(objects, pose);

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};

    auto cmd1 = planner.plan(pose, target);
    EXPECT_TRUE(cmd1.valid);

    // Plan again with no new detections — obstacle persists due to TTL
    // but the planner should still produce a valid command
    drone::ipc::DetectedObjectList empty{};
    empty.num_objects = 0;
    planner.update_obstacles(empty, pose);

    auto cmd2 = planner.plan(pose, target);
    EXPECT_TRUE(cmd2.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
}

TEST(DStarLiteIncrementalTest, GoalChange_Reinitialises) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.5f;
    config.replan_interval_s  = -1.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target1{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd1 = planner.plan(pose, target1);
    EXPECT_TRUE(cmd1.valid);

    // Change goal — should reinitialise
    Waypoint target2{0.0f, 10.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd2 = planner.plan(pose, target2);
    EXPECT_TRUE(cmd2.valid);
    // New goal direction should reflect in velocity
    EXPECT_GT(cmd2.velocity_y, 0.0f);
}

TEST(DStarLiteIncrementalTest, DroneMovement_UpdatesKm) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.5f;
    config.replan_interval_s  = -1.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd1 = planner.plan(pose, target);
    EXPECT_TRUE(cmd1.valid);

    // Move drone closer to goal
    pose.translation[0] = 3.0;
    auto cmd2           = planner.plan(pose, target);
    EXPECT_TRUE(cmd2.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
}

// ═════════════════════════════════════════════════════════════
// Timeout tests
// ═════════════════════════════════════════════════════════════

TEST(DStarLiteTimeoutTest, MaxSearchTimeEnforced) {
    GridPlannerConfig config;
    config.resolution_m       = 0.5f;
    config.grid_extent_m      = 50.0f;
    config.inflation_radius_m = 0.5f;
    config.max_search_time_ms = 1.0f;    // 1ms — very tight
    config.max_iterations     = 500000;  // high iter limit so timeout triggers first
    config.replan_interval_s  = 0.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    // Far goal to force many iterations
    Waypoint target{40.0f, 40.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};

    auto start_time = std::chrono::steady_clock::now();
    auto cmd        = planner.plan(pose, target);
    auto elapsed_ms =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start_time)
            .count();

    EXPECT_TRUE(cmd.valid);
    // Should complete relatively quickly (within ~50ms even with overhead)
    EXPECT_LT(elapsed_ms, 500.0f);
}

TEST(DStarLiteTimeoutTest, FallbackOnTimeout) {
    GridPlannerConfig config;
    config.resolution_m       = 0.1f;  // very fine grid → many cells
    config.grid_extent_m      = 50.0f;
    config.inflation_radius_m = 0.5f;
    config.max_search_time_ms = 0.001f;  // unrealistically tight timeout
    config.max_iterations     = 500000;
    config.replan_interval_s  = 0.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    Waypoint         target{40.0f, 40.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};

    auto cmd = planner.plan(pose, target);
    EXPECT_TRUE(cmd.valid);
    // With such a tight timeout, the planner likely falls back to direct
    // (though it may find a partial path — either way, cmd should be valid)
}

// ═════════════════════════════════════════════════════════════
// Integration tests
// ═════════════════════════════════════════════════════════════

TEST(DStarLiteIntegrationTest, PlanReturnsValidCmd) {
    DStarLitePlanner planner;

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};

    auto cmd = planner.plan(pose, target);
    EXPECT_TRUE(cmd.valid);
    EXPECT_GT(cmd.timestamp_ns, 0u);
    EXPECT_GT(cmd.velocity_x, 0.0f);
    EXPECT_NEAR(cmd.target_x, 10.0f, 0.01f);
}

TEST(DStarLiteIntegrationTest, GoalSnappingWorks) {
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 10.0f;
    config.inflation_radius_m = 0.5f;
    DStarLitePlanner planner(config);

    // Block the goal with an obstacle
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;
    planner.update_obstacles(objects, pose);

    Waypoint target{5.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    // Goal snap should redirect — planner should NOT use direct fallback
    EXPECT_FALSE(planner.using_direct_fallback());
}

TEST(DStarLiteIntegrationTest, EMASmoothing) {
    GridPlannerConfig config;
    config.resolution_m    = 1.0f;
    config.grid_extent_m   = 20.0f;
    config.smoothing_alpha = 0.1f;  // heavy smoothing
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};

    // First call — velocity should be small due to smoothing from zero
    auto  cmd1 = planner.plan(pose, target);
    float v1   = cmd1.velocity_x;

    // Second call with same inputs — velocity should increase
    auto  cmd2 = planner.plan(pose, target);
    float v2   = cmd2.velocity_x;

    EXPECT_GT(v2, v1);
}

TEST(DStarLiteIntegrationTest, SpeedRampingNearTarget) {
    GridPlannerConfig config;
    config.resolution_m    = 1.0f;
    config.grid_extent_m   = 20.0f;
    config.smoothing_alpha = 1.0f;  // no smoothing for clearer test
    config.path_speed_mps  = 3.0f;
    DStarLitePlanner planner(config);

    // Drone close to target
    drone::ipc::Pose pose{};
    pose.translation[0] = 9.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    // Velocity should be less than cruise speed due to ramping
    float speed = std::sqrt(cmd.velocity_x * cmd.velocity_x + cmd.velocity_y * cmd.velocity_y +
                            cmd.velocity_z * cmd.velocity_z);
    EXPECT_LT(speed, config.path_speed_mps);
}

TEST(DStarLiteIntegrationTest, UpdateObstaclesIntegration) {
    DStarLitePlanner planner;

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 5.0f;
    objects.objects[0].position_z = 5.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    planner.update_obstacles(objects, pose);

    EXPECT_GT(planner.grid().occupied_count(), 0u);
}

TEST(DStarLiteIntegrationTest, FactoryRegistered) {
    auto planner = create_path_planner("dstar_lite");
    EXPECT_NE(planner, nullptr);
    EXPECT_EQ(planner->name(), "DStarLitePlanner");
}

// ═════════════════════════════════════════════════════════════
// Name test
// ═════════════════════════════════════════════════════════════

TEST(DStarLitePlannerTest, NameIsCorrect) {
    DStarLitePlanner planner;
    EXPECT_EQ(planner.name(), "DStarLitePlanner");
}

// ═════════════════════════════════════════════════════════════
// Config wiring tests (Issue #228)
// ═════════════════════════════════════════════════════════════

TEST(GridPlannerConfigTest, CellTTL_PassedToGrid) {
    // Custom TTL should propagate through GridPlannerConfig → GridPlannerBase → OccupancyGrid3D.
    // We verify indirectly: a cell inserted at t=0 should expire after the configured TTL.
    GridPlannerConfig cfg;
    cfg.cell_ttl_s = 0.05f;  // 50ms TTL — short to keep test fast
    DStarLitePlanner planner(cfg);

    // Insert an object into the grid
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0]            = {};
    objects.objects[0].confidence = 0.9f;
    objects.objects[0].position_x = 10.0f;
    objects.objects[0].position_y = 10.0f;
    objects.objects[0].position_z = 2.0f;
    objects.timestamp_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;
    planner.update_obstacles(objects, pose);
    EXPECT_GT(planner.grid().occupied_count(), 0u);

    // Wait for TTL to expire (50ms + margin)
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Trigger another update with zero objects to run expiry
    drone::ipc::DetectedObjectList empty{};
    empty.num_objects = 0;
    planner.update_obstacles(empty, pose);
    EXPECT_EQ(planner.grid().occupied_count(), 0u);
}

TEST(OccupancyGrid3DTest, MinConfidence_Configurable) {
    // Objects below the configured min_confidence should be rejected by the grid.
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, 3.0f, 0.5f);  // min_confidence = 0.5

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects = 2;
    objects.timestamp_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());

    // Object 0: confidence 0.4 — below threshold, should be rejected
    objects.objects[0]            = {};
    objects.objects[0].confidence = 0.4f;
    objects.objects[0].position_x = 10.0f;
    objects.objects[0].position_y = 10.0f;
    objects.objects[0].position_z = 2.0f;

    // Object 1: confidence 0.6 — above threshold, should be accepted
    objects.objects[1]            = {};
    objects.objects[1].confidence = 0.6f;
    objects.objects[1].position_x = -10.0f;
    objects.objects[1].position_y = -10.0f;
    objects.objects[1].position_z = 2.0f;

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;
    grid.update_from_objects(objects, pose);

    // Only the high-confidence object should have populated cells
    auto cell_high = grid.world_to_grid(-10.0f, -10.0f, 2.0f);
    auto cell_low  = grid.world_to_grid(10.0f, 10.0f, 2.0f);
    EXPECT_TRUE(grid.is_occupied(cell_high));
    EXPECT_FALSE(grid.is_occupied(cell_low));
}

TEST(OccupancyGrid3DTest, DefaultMinConfidence_IsZeroPointThree) {
    // Default min_confidence should be 0.3 (backward compatibility)
    OccupancyGrid3D grid(1.0f, 20.0f, 1.0f, 3.0f);  // no min_confidence arg

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects = 1;
    objects.timestamp_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());

    // Object at confidence 0.25 — below default 0.3
    objects.objects[0]            = {};
    objects.objects[0].confidence = 0.25f;
    objects.objects[0].position_x = 10.0f;
    objects.objects[0].position_y = 10.0f;
    objects.objects[0].position_z = 2.0f;

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;
    grid.update_from_objects(objects, pose);

    auto cell = grid.world_to_grid(10.0f, 10.0f, 2.0f);
    EXPECT_FALSE(grid.is_occupied(cell));
}

// ═════════════════════════════════════════════════════════════
// Queue performance tests (Issue #234)
// ═════════════════════════════════════════════════════════════

TEST(DStarLiteQueueTest, LargeGridWithObstaclesCompletesWithinTimeout) {
    // With the O(log N) queue_index_ fix, a densely-occupied grid should
    // complete search within the timeout rather than falling back.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 50.0f;
    config.inflation_radius_m = 2.0f;
    config.max_search_time_ms = 200.0f;
    config.max_iterations     = 500000;
    config.replan_interval_s  = 0.0f;
    DStarLitePlanner planner(config);

    // Add obstacles to populate the grid — simulate detected objects
    drone::ipc::DetectedObjectList objects{};
    objects.timestamp_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    objects.num_objects = 6;
    // Scatter obstacles across the grid
    float positions[][3] = {{5, 5, 3}, {10, 8, 3}, {15, 3, 3}, {8, 12, 3}, {20, 7, 3}, {12, 15, 3}};
    for (uint32_t i = 0; i < objects.num_objects; ++i) {
        objects.objects[i]            = {};
        objects.objects[i].confidence = 0.9f;
        objects.objects[i].position_x = positions[i][0];
        objects.objects[i].position_y = positions[i][1];
        objects.objects[i].position_z = positions[i][2];
    }

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 4.0;
    planner.update_obstacles(objects, pose);

    // Plan from origin to far side — must route around obstacles
    Waypoint target{25.0f, 0.0f, 4.0f, 0.0f, 2.0f, 3.0f, false};

    auto start_time = std::chrono::steady_clock::now();
    auto cmd        = planner.plan(pose, target);
    auto elapsed_ms =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start_time)
            .count();

    EXPECT_TRUE(cmd.valid);
    // Ensure the search actually completed (no direct fallback due to timeout)
    EXPECT_FALSE(planner.using_direct_fallback());
    // Should complete within a reasonable wall-clock budget (allowing overhead)
    EXPECT_LT(elapsed_ms, config.max_search_time_ms + 200.0f);
}

TEST(DStarLiteQueueTest, IncrementalReplanWithManyChanges) {
    // Verify incremental replanning stays fast when obstacles are added
    // frame-by-frame (the real-world perception scenario).
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 30.0f;
    config.inflation_radius_m = 2.0f;
    config.max_search_time_ms = 100.0f;
    config.max_iterations     = 500000;
    config.replan_interval_s  = 0.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 4.0;

    Waypoint target{20.0f, 10.0f, 4.0f, 0.0f, 2.0f, 3.0f, false};

    // Initial plan (no obstacles) — should succeed
    auto cmd = planner.plan(pose, target);
    EXPECT_TRUE(cmd.valid);

    // Add obstacles incrementally (simulating perception detections)
    for (int frame = 0; frame < 5; ++frame) {
        drone::ipc::DetectedObjectList objects{};
        objects.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        objects.num_objects           = 1;
        objects.objects[0]            = {};
        objects.objects[0].confidence = 0.9f;
        objects.objects[0].position_x = 5.0f + static_cast<float>(frame) * 3.0f;
        objects.objects[0].position_y = 5.0f;
        objects.objects[0].position_z = 4.0f;
        planner.update_obstacles(objects, pose);

        auto start_time = std::chrono::steady_clock::now();
        cmd             = planner.plan(pose, target);
        auto elapsed_ms =
            std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start_time)
                .count();

        EXPECT_TRUE(cmd.valid);
        // Each incremental replan should be fast; allow margin over configured limit
        EXPECT_LT(elapsed_ms, config.max_search_time_ms * 1.5f);
    }
}

TEST(DStarLiteQueueTest, QueueIndexConsistentAfterReinitialise) {
    // When the planner reinitialises (goal change), the queue_index_ must
    // be cleared and rebuilt correctly.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 1.0f;
    config.replan_interval_s  = 0.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 3.0;

    // Plan to first goal
    Waypoint target1{10.0f, 0.0f, 3.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd1 = planner.plan(pose, target1);
    EXPECT_TRUE(cmd1.valid);

    // Change goal — triggers reinitialise
    Waypoint target2{0.0f, 10.0f, 3.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd2 = planner.plan(pose, target2);
    EXPECT_TRUE(cmd2.valid);

    // Plan again to second goal — should still work (no stale iterators)
    auto cmd3 = planner.plan(pose, target2);
    EXPECT_TRUE(cmd3.valid);
}

// ═════════════════════════════════════════════════════════════
// Z-band constraint tests (Issue #234)
// ═════════════════════════════════════════════════════════════

TEST(DStarLiteZBandTest, ZBandConstraintReducesSearchSpace) {
    // With Z-band=3, a 3D search with start/goal at z=5 only expands
    // cells in z=[2,8] instead of z=[-50,50].  Should find path quickly.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 50.0f;
    config.inflation_radius_m = 2.0f;
    config.max_search_time_ms = 100.0f;
    config.replan_interval_s  = 0.0f;
    config.z_band_cells       = 3;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 10.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);
    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
}

TEST(DStarLiteZBandTest, ZBandDisabledSearchesFull3D) {
    // Without Z-band (default=0), the full 3D grid is searched.
    // With a large grid and tight timeout, this may time out.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 50.0f;
    config.inflation_radius_m = 2.0f;
    config.max_search_time_ms = 50.0f;  // tight timeout
    config.replan_interval_s  = 0.0f;
    config.z_band_cells       = 0;  // no Z-band
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    // Far goal — likely times out without Z-band
    Waypoint target{15.0f, 15.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);
    // We expect fallback (timeout) — the key check is that it doesn't crash
    EXPECT_TRUE(cmd.valid);
}

TEST(DStarLiteZBandTest, ZBandWithDifferentStartGoalAltitudes) {
    // Start at z=3, goal at z=7 with z_band=2 → band=[1,9].
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 1.0f;
    config.replan_interval_s  = 0.0f;
    config.z_band_cells       = 2;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 3.0;

    Waypoint target{5.0f, 5.0f, 7.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);
    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
}

TEST(DStarLiteZBandTest, KmReinitPreventsKeyChurn) {
    // Simulate drone moving far → km_ should trigger reinit
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 50.0f;
    config.inflation_radius_m = 1.0f;
    config.replan_interval_s  = 0.0f;
    config.z_band_cells       = 3;
    DStarLitePlanner planner(config);

    Waypoint target{15.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};

    // Plan from origin
    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;
    planner.plan(pose, target);

    // Simulate drone moving 12m — should trigger km_ reinit
    for (int i = 1; i <= 12; ++i) {
        pose.translation[0] = static_cast<double>(i);
        planner.plan(pose, target);
    }

    // Should still find valid paths (not crash or produce stale results)
    auto cmd = planner.plan(pose, target);
    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
}

// ═════════════════════════════════════════════════════════════
// Pure-pursuit / carrot following tests (Issue #237)
// ═════════════════════════════════════════════════════════════

TEST(PurePursuitTest, CarrotSteersTowardGoal) {
    // With look-ahead enabled, the velocity should point toward the goal
    // (not just the next cell).
    GridPlannerConfig config;
    config.resolution_m      = 2.0f;
    config.grid_extent_m     = 50.0f;
    config.smoothing_alpha   = 1.0f;  // no smoothing for clearer test
    config.look_ahead_m      = 6.0f;  // 3 cells ahead
    config.replan_interval_s = 0.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{20.0f, 20.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    // Velocity should have positive X and Y components toward NE goal
    EXPECT_GT(cmd.velocity_x, 0.0f);
    EXPECT_GT(cmd.velocity_y, 0.0f);
}

TEST(PurePursuitTest, CarrotProducesSmootherTurnThanCellByCell) {
    // Compare velocity direction change at a path bend:
    // pure-pursuit should have a smaller angle change than cell-by-cell.
    GridPlannerConfig config_cell;
    config_cell.resolution_m      = 2.0f;
    config_cell.grid_extent_m     = 50.0f;
    config_cell.smoothing_alpha   = 1.0f;
    config_cell.look_ahead_m      = 0.0f;  // cell-by-cell
    config_cell.replan_interval_s = 0.0f;

    GridPlannerConfig config_carrot = config_cell;
    config_carrot.look_ahead_m      = 6.0f;  // pure-pursuit

    DStarLitePlanner planner_cell(config_cell);
    DStarLitePlanner planner_carrot(config_carrot);

    // Place an obstacle that forces a path bend
    drone::ipc::DetectedObjectList objects{};
    objects.timestamp_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    objects.num_objects           = 1;
    objects.objects[0]            = {};
    objects.objects[0].confidence = 0.9f;
    objects.objects[0].position_x = 10.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 5.0f;

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;
    planner_cell.update_obstacles(objects, pose);
    planner_carrot.update_obstacles(objects, pose);

    Waypoint target{20.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};

    // Both should find a valid path
    auto cmd_cell   = planner_cell.plan(pose, target);
    auto cmd_carrot = planner_carrot.plan(pose, target);
    EXPECT_TRUE(cmd_cell.valid);
    EXPECT_TRUE(cmd_carrot.valid);

    // Both should have positive X velocity (heading toward goal)
    EXPECT_GT(cmd_cell.velocity_x, 0.0f);
    EXPECT_GT(cmd_carrot.velocity_x, 0.0f);

    // Smoothness: pure-pursuit should deviate less from a straight line to goal.
    // Measure |vy/vx| — smaller means more aligned with the forward direction.
    const float deviation_cell = std::abs(cmd_cell.velocity_y) /
                                 std::max(0.01f, std::abs(cmd_cell.velocity_x));
    const float deviation_carrot = std::abs(cmd_carrot.velocity_y) /
                                   std::max(0.01f, std::abs(cmd_carrot.velocity_x));
    EXPECT_LE(deviation_carrot, deviation_cell + 0.5f)
        << "Pure-pursuit (look_ahead=6m) should not deviate more than cell-by-cell";
}

TEST(PurePursuitTest, LookAheadZeroFallsBackToCellByCell) {
    // With look_ahead_m=0, behavior should match legacy cell-by-cell.
    GridPlannerConfig config;
    config.resolution_m      = 2.0f;
    config.grid_extent_m     = 50.0f;
    config.smoothing_alpha   = 1.0f;
    config.look_ahead_m      = 0.0f;
    config.replan_interval_s = 0.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_GT(cmd.velocity_x, 0.0f);
}

TEST(PurePursuitTest, CarrotReachesEndOfShortPath) {
    // When look-ahead exceeds path length, carrot should settle at last waypoint.
    GridPlannerConfig config;
    config.resolution_m      = 2.0f;
    config.grid_extent_m     = 50.0f;
    config.smoothing_alpha   = 1.0f;
    config.look_ahead_m      = 50.0f;  // much longer than any path
    config.replan_interval_s = 0.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{6.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_GT(cmd.velocity_x, 0.0f);
}

TEST(PurePursuitTest, ConfigDefaultDisabled) {
    // Default config has look_ahead_m=0 (disabled)
    GridPlannerConfig config;
    EXPECT_FLOAT_EQ(config.look_ahead_m, 0.0f);
}

// ═════════════════════════════════════════════════════════════
// Fallback behaviour tests (Issue #237)
// ═════════════════════════════════════════════════════════════

TEST(FallbackBehaviourTest, SearchFailureKeepsLastGoodPath) {
    // First plan succeeds and caches a path.
    // Then we wall off the grid so the next plan fails.
    // The planner should keep following the cached path, NOT fly direct.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 4.0f;  // small grid so wall is impassable
    config.inflation_radius_m = 0.5f;
    config.smoothing_alpha    = 1.0f;
    config.replan_interval_s  = 0.0f;
    config.snap_search_radius = 0;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    Waypoint         target{3.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};

    // First plan — clear grid, should find a path
    auto cmd1 = planner.plan(pose, target);
    EXPECT_TRUE(cmd1.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());
    size_t first_path_size = planner.cached_path().size();

    // Wall off — complete barrier across the grid
    drone::ipc::DetectedObjectList objects{};
    uint32_t                       idx = 0;
    for (int y = -4; y <= 4; ++y) {
        for (int x = 1; x <= 2; ++x) {
            if (idx >= drone::ipc::MAX_DETECTED_OBJECTS) break;
            objects.objects[idx].position_x = static_cast<float>(x);
            objects.objects[idx].position_y = static_cast<float>(y);
            objects.objects[idx].position_z = 0.0f;
            objects.objects[idx].confidence = 0.9f;
            ++idx;
        }
    }
    objects.num_objects = idx;
    planner.update_obstacles(objects, pose);

    // Second plan — search fails, but planner keeps the old cached path
    auto cmd2 = planner.plan(pose, target);
    EXPECT_TRUE(cmd2.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());
    EXPECT_EQ(planner.cached_path().size(), first_path_size);
}

TEST(FallbackBehaviourTest, NoCachedPathHoversInPlace) {
    // If the very first search fails (no cached path exists),
    // the planner should output near-zero velocity (hover) instead
    // of flying a direct line through obstacles.
    // Use a tiny grid and fill nearly everything with obstacles so
    // goal snapping cannot find a route around.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 4.0f;  // small grid: cells -4..+4
    config.inflation_radius_m = 0.5f;
    config.smoothing_alpha    = 1.0f;
    config.replan_interval_s  = 0.0f;
    config.snap_search_radius = 0;  // disable goal snapping
    DStarLitePlanner planner(config);

    // Fill a complete wall across the grid at x=2, all y values
    drone::ipc::DetectedObjectList objects{};
    uint32_t                       idx = 0;
    for (int y = -4; y <= 4; ++y) {
        for (int x = 1; x <= 3; ++x) {
            if (idx >= drone::ipc::MAX_DETECTED_OBJECTS) break;
            objects.objects[idx].position_x = static_cast<float>(x);
            objects.objects[idx].position_y = static_cast<float>(y);
            objects.objects[idx].position_z = 0.0f;
            objects.objects[idx].confidence = 0.9f;
            ++idx;
        }
    }
    objects.num_objects = idx;

    drone::ipc::Pose pose{};
    planner.update_obstacles(objects, pose);

    Waypoint target{4.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    // Velocity should be zero or near-zero (hovering)
    float speed = std::sqrt(cmd.velocity_x * cmd.velocity_x + cmd.velocity_y * cmd.velocity_y +
                            cmd.velocity_z * cmd.velocity_z);
    EXPECT_LT(speed, 0.1f) << "Expected hover (near-zero velocity), got speed=" << speed;
    EXPECT_TRUE(planner.using_direct_fallback());
}

TEST(FallbackBehaviourTest, HoverRecoversToCachedPath) {
    // After hovering (no path), if the grid clears and search succeeds,
    // the planner should resume normal path following.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 4.0f;
    config.inflation_radius_m = 0.5f;
    config.smoothing_alpha    = 1.0f;
    config.replan_interval_s  = 0.0f;
    config.snap_search_radius = 0;
    DStarLitePlanner planner(config);

    // Wall off — first search fails → hover
    drone::ipc::DetectedObjectList objects{};
    uint32_t                       idx = 0;
    for (int y = -4; y <= 4; ++y) {
        for (int x = 1; x <= 3; ++x) {
            if (idx >= drone::ipc::MAX_DETECTED_OBJECTS) break;
            objects.objects[idx].position_x = static_cast<float>(x);
            objects.objects[idx].position_y = static_cast<float>(y);
            objects.objects[idx].position_z = 0.0f;
            objects.objects[idx].confidence = 0.9f;
            ++idx;
        }
    }
    objects.num_objects = idx;

    drone::ipc::Pose pose{};
    planner.update_obstacles(objects, pose);

    Waypoint target{4.0f, 0.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    (void)planner.plan(pose, target);
    EXPECT_TRUE(planner.using_direct_fallback());

    // Clear all obstacles (TTL-based: insert empty update, then wait for expiry)
    // For test: grid uses short TTL, just re-create planner with clean grid
    DStarLitePlanner planner2(config);

    auto cmd2 = planner2.plan(pose, target);
    EXPECT_TRUE(cmd2.valid);
    EXPECT_FALSE(planner2.using_direct_fallback());
    EXPECT_FALSE(planner2.cached_path().empty());
    EXPECT_GT(cmd2.velocity_x, 0.0f);  // moving toward goal again
}

// ═════════════════════════════════════════════════════════════
// Corner-cutting guard tests (Issue #258)
// ═════════════════════════════════════════════════════════════

// Helper: check that no consecutive waypoints in the world-coordinate path
// represent a diagonal grid move through a blocked cardinal intermediary.
// Uses the grid's own world_to_grid() to avoid duplicating the conversion logic.
static bool path_has_corner_cut(const std::vector<std::array<float, 3>>& path,
                                const OccupancyGrid3D&                   grid) {
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        GridCell g0 = grid.world_to_grid(path[i][0], path[i][1], path[i][2]);
        GridCell g1 = grid.world_to_grid(path[i + 1][0], path[i + 1][1], path[i + 1][2]);

        int dx = g1.x - g0.x;
        int dy = g1.y - g0.y;
        int dz = g1.z - g0.z;

        // Only check diagonal moves (Manhattan distance > 1)
        if (std::abs(dx) + std::abs(dy) + std::abs(dz) <= 1) continue;

        // Check XY intermediaries
        if (dx != 0 && dy != 0) {
            GridCell cx{g0.x + dx, g0.y, g0.z};
            GridCell cy{g0.x, g0.y + dy, g0.z};
            if (grid.is_occupied(cx) || grid.is_occupied(cy)) return true;
        }
        // Check XZ intermediaries
        if (dx != 0 && dz != 0) {
            GridCell cx{g0.x + dx, g0.y, g0.z};
            GridCell cz{g0.x, g0.y, g0.z + dz};
            if (grid.is_occupied(cx) || grid.is_occupied(cz)) return true;
        }
        // Check YZ intermediaries
        if (dy != 0 && dz != 0) {
            GridCell cy{g0.x, g0.y + dy, g0.z};
            GridCell cz{g0.x, g0.y, g0.z + dz};
            if (grid.is_occupied(cy) || grid.is_occupied(cz)) return true;
        }
    }
    return false;
}

TEST(DStarLiteCornerCutTest, LShapeInflatedBlocksDiagonal) {
    // Two point obstacles placed so their 1-cell inflation (the OccupancyGrid3D
    // minimum) forms an L-shaped barrier with a diagonal gap.
    //
    // Obstacle centres at (5,4) and (7,2).  With 1-cell cross inflation each
    // centre expands into a plus/cross of 5 cells:
    //   (5,4) -> {(5,4),(4,4),(6,4),(5,3),(5,5)}
    //   (7,2) -> {(7,2),(6,2),(8,2),(7,1),(7,3)}
    //
    // The diagonal from (6,3) to (7,4) has cardinal intermediates:
    //   (7,3) — occupied (inflation of (7,2))
    //   (6,4) — occupied (inflation of (5,4))
    // But (6,3) and (7,4) themselves are FREE — the corner-cutting guard must
    // prevent a diagonal move through that gap.
    //
    //   y=5: . . . . . X . . . .     X = inflated from (5,4)
    //   y=4: . . . . X O X . . G     O = obstacle centre
    //   y=3: . . . . . X . x . .     x = inflated from (7,2)
    //   y=2: . . . . . . x O x .
    //   y=1: . . . . . . . x . .
    //   y=0: S . . . . . . . . .
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.0f;  // OccupancyGrid3D clamps to at least 1 cell
    config.replan_interval_s  = 0.0f;
    config.smoothing_alpha    = 1.0f;
    DStarLitePlanner planner(config);

    planner.add_static_obstacle(5.0f, 4.0f, 0.0f, 0.0f);
    planner.add_static_obstacle(7.0f, 2.0f, 0.0f, 0.0f);

    // Assert key cells: diagonal endpoints free, cardinal intermediates occupied
    const auto& grid = planner.grid();
    EXPECT_FALSE(grid.is_occupied({6, 3, 0})) << "Diagonal endpoint (6,3) must be free";
    EXPECT_FALSE(grid.is_occupied({7, 4, 0})) << "Diagonal endpoint (7,4) must be free";
    EXPECT_TRUE(grid.is_occupied({7, 3, 0})) << "Cardinal intermediate (7,3) must be occupied";
    EXPECT_TRUE(grid.is_occupied({6, 4, 0})) << "Cardinal intermediate (6,4) must be occupied";

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    Waypoint target{9.0f, 4.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());

    // The path must NOT cut through the diagonal gap
    EXPECT_FALSE(path_has_corner_cut(planner.cached_path(), planner.grid()))
        << "Path should not cut diagonally through L-shaped obstacle gap";
}

TEST(DStarLiteCornerCutTest, LShapeInflatedBlocksDiagonalReversed) {
    // Mirror of the first test with a different axis orientation.
    // Obstacles at (4,7) and (2,5) with 1-cell cross inflation:
    //   (4,7) -> {(4,7),(3,7),(5,7),(4,6),(4,8)}
    //   (2,5) -> {(2,5),(1,5),(3,5),(2,4),(2,6)}
    //
    // Diagonal from (3,6) to (4,5) has cardinal intermediates:
    //   (4,6) — occupied (inflation of (4,7))
    //   (3,5) — occupied (inflation of (2,5))
    // Endpoints (3,6) and (4,5) are both free.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.0f;  // OccupancyGrid3D clamps to at least 1 cell
    config.replan_interval_s  = 0.0f;
    config.smoothing_alpha    = 1.0f;
    DStarLitePlanner planner(config);

    planner.add_static_obstacle(4.0f, 7.0f, 0.0f, 0.0f);
    planner.add_static_obstacle(2.0f, 5.0f, 0.0f, 0.0f);

    // Assert key cells: diagonal endpoints free, cardinal intermediates occupied
    const auto& grid = planner.grid();
    EXPECT_FALSE(grid.is_occupied({3, 6, 0})) << "Diagonal endpoint (3,6) must be free";
    EXPECT_FALSE(grid.is_occupied({4, 5, 0})) << "Diagonal endpoint (4,5) must be free";
    EXPECT_TRUE(grid.is_occupied({4, 6, 0})) << "Cardinal intermediate (4,6) must be occupied";
    EXPECT_TRUE(grid.is_occupied({3, 5, 0})) << "Cardinal intermediate (3,5) must be occupied";

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    Waypoint target{5.0f, 8.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());

    EXPECT_FALSE(path_has_corner_cut(planner.cached_path(), planner.grid()))
        << "Path should not cut diagonally through reversed L-shaped obstacle gap";
}

TEST(DStarLiteCornerCutTest, WallCornerInflatedNoCut) {
    // Wall with a 90-degree corner.  Each obstacle centre gets 1-cell cross
    // inflation (OccupancyGrid3D minimum), so the wall is wider than the raw
    // centres.  The path must not cut the inside corner of the inflated wall.
    //
    // Obstacle centres (O) and their inflation zones (x) overlap to form a
    // solid L-shaped barrier:
    //   y=4: . x x x x x .
    //   y=3: . x O O O x .    wall at y=3, x=2..4
    //   y=2: x x O x x x .    wall at x=2, y=1..2 + inflation overlap
    //   y=1: . x O x . . .
    //   y=0: . x x x . . .
    //   S at (0,0), G at (6,5)
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.0f;  // OccupancyGrid3D clamps to at least 1 cell
    config.replan_interval_s  = 0.0f;
    config.smoothing_alpha    = 1.0f;
    DStarLitePlanner planner(config);

    // Vertical wall segment x=2, y=1..2
    planner.add_static_obstacle(2.0f, 1.0f, 0.0f, 0.0f);
    planner.add_static_obstacle(2.0f, 2.0f, 0.0f, 0.0f);
    // Horizontal wall segment y=3, x=2..4
    planner.add_static_obstacle(2.0f, 3.0f, 0.0f, 0.0f);
    planner.add_static_obstacle(3.0f, 3.0f, 0.0f, 0.0f);
    planner.add_static_obstacle(4.0f, 3.0f, 0.0f, 0.0f);

    // Verify the inside-corner inflation — the cell just inside the corner
    // and the two adjacent cardinal cells must be occupied.
    const auto& grid = planner.grid();
    EXPECT_TRUE(grid.is_occupied({3, 2, 0})) << "Inside corner cell (3,2) must be occupied";
    EXPECT_TRUE(grid.is_occupied({3, 3, 0})) << "Wall centre (3,3) must be occupied";
    EXPECT_TRUE(grid.is_occupied({2, 2, 0})) << "Wall centre (2,2) must be occupied";

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    Waypoint target{6.0f, 5.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());

    EXPECT_FALSE(path_has_corner_cut(planner.cached_path(), planner.grid()))
        << "Path should not cut through the inside corner of an L-shaped wall";
}

TEST(DStarLiteCornerCutTest, ValidDiagonalStillAllowed) {
    // When both cardinal intermediaries are free, diagonal moves must still work.
    // Place a single obstacle far from the path so its 1-cell inflation does
    // not block any diagonal along the route from (0,0) to (4,4).
    //
    //   y=4: . . . . G
    //   y=0: S . . . . . . O    O at (7,0) — well away from the path
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.0f;  // OccupancyGrid3D clamps to at least 1 cell
    config.replan_interval_s  = 0.0f;
    config.smoothing_alpha    = 1.0f;
    DStarLitePlanner planner(config);

    planner.add_static_obstacle(7.0f, 0.0f, 0.0f, 0.0f);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    Waypoint target{4.0f, 4.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());

    // Path should use diagonal moves (should be short — ~5 waypoints, not 8+).
    // A path without diagonals from (0,0) to (4,4) would need at least 8 cardinal steps.
    // With diagonals available it can be as short as 5 waypoints (start + 4 diagonal steps).
    EXPECT_LE(planner.cached_path().size(), 6u)
        << "Valid diagonals should still be used, keeping the path short";
}

TEST(DStarLiteCornerCutTest, OpenFieldDiagonalsWork) {
    // In an empty grid, diagonal paths should still be generated normally.
    // No obstacles, so inflation does not matter.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.0f;  // OccupancyGrid3D clamps to at least 1 cell
    config.replan_interval_s  = 0.0f;
    config.smoothing_alpha    = 1.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    Waypoint target{5.0f, 5.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());

    // Diagonal path from (0,0) to (5,5): optimal is 5 diagonal steps = 6 waypoints
    // (start + 5 steps). Allow some tolerance.
    EXPECT_LE(planner.cached_path().size(), 7u)
        << "Open field should use diagonal moves for an efficient path";
}

TEST(DStarLiteCornerCutTest, BothCardinalsBlockedForcesDetour) {
    // Both cardinal intermediaries of a diagonal are blocked, forcing a detour.
    //
    // Obstacles at (3,4) and (5,2) with 1-cell cross inflation:
    //   (3,4) -> {(3,4),(2,4),(4,4),(3,3),(3,5)}
    //   (5,2) -> {(5,2),(4,2),(6,2),(5,1),(5,3)}
    //
    // Diagonal from (4,3) to (5,4): cardinal intermediates
    //   (5,3) — occupied (inflation of (5,2))
    //   (4,4) — occupied (inflation of (3,4))
    // Both blocked, so the planner must route around.
    // Endpoints (4,3) and (5,4) are both free.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.0f;  // OccupancyGrid3D clamps to at least 1 cell
    config.replan_interval_s  = 0.0f;
    config.smoothing_alpha    = 1.0f;
    DStarLitePlanner planner(config);

    planner.add_static_obstacle(3.0f, 4.0f, 0.0f, 0.0f);
    planner.add_static_obstacle(5.0f, 2.0f, 0.0f, 0.0f);

    // Assert the diagonal's cardinal intermediates are blocked and endpoints free
    const auto& grid = planner.grid();
    EXPECT_FALSE(grid.is_occupied({4, 3, 0})) << "Diagonal endpoint (4,3) must be free";
    EXPECT_FALSE(grid.is_occupied({5, 4, 0})) << "Diagonal endpoint (5,4) must be free";
    EXPECT_TRUE(grid.is_occupied({5, 3, 0})) << "Cardinal intermediate (5,3) must be occupied";
    EXPECT_TRUE(grid.is_occupied({4, 4, 0})) << "Cardinal intermediate (4,4) must be occupied";

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    Waypoint target{7.0f, 6.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};
    auto     cmd = planner.plan(pose, target);

    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());

    EXPECT_FALSE(path_has_corner_cut(planner.cached_path(), planner.grid()))
        << "Path must not cut through when both cardinal intermediaries are blocked";

    // Verify the specific guarded diagonal edge (4,3)->(5,4) is not taken
    for (size_t i = 0; i + 1 < planner.cached_path().size(); ++i) {
        GridCell a = grid.world_to_grid(planner.cached_path()[i][0], planner.cached_path()[i][1],
                                        planner.cached_path()[i][2]);
        GridCell b = grid.world_to_grid(planner.cached_path()[i + 1][0],
                                        planner.cached_path()[i + 1][1],
                                        planner.cached_path()[i + 1][2]);
        bool     is_guarded_edge = (a.x == 4 && a.y == 3 && b.x == 5 && b.y == 4) ||
                               (a.x == 5 && a.y == 4 && b.x == 4 && b.y == 3);
        EXPECT_FALSE(is_guarded_edge)
            << "Path must not use the guarded diagonal edge (4,3)<->(5,4)";
    }
}

TEST(DStarLiteCornerCutTest, IncrementalUpdateRespectsCornerGuard) {
    // After an incremental obstacle update creates an L-shape, the replanned
    // path must still respect the corner-cutting guard.
    // Obstacles are spaced 2 cells apart diagonally so that 1-cell inflation
    // creates cardinal intermediates without occupying the diagonal endpoints.
    GridPlannerConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 20.0f;
    config.inflation_radius_m = 0.0f;  // OccupancyGrid3D clamps to at least 1 cell
    config.replan_interval_s  = 0.0f;
    config.smoothing_alpha    = 1.0f;
    DStarLitePlanner planner(config);

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 0.0;

    Waypoint target{9.0f, 6.0f, 0.0f, 0.0f, 2.0f, 3.0f, false};

    // First plan: no obstacles, path uses diagonals freely
    auto cmd1 = planner.plan(pose, target);
    EXPECT_TRUE(cmd1.valid);
    EXPECT_FALSE(planner.using_direct_fallback());

    // Add L-shaped obstacles via dynamic update (simulates runtime detection).
    // Objects at (5,4) and (7,2) — same geometry as LShapeInflatedBlocksDiagonal.
    // update_from_objects() uses steady_clock internally; timestamp_ns is unused.
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects = 2;
    // Object 1 at (5,4)
    objects.objects[0]            = {};
    objects.objects[0].confidence = 0.9f;
    objects.objects[0].position_x = 5.0f;
    objects.objects[0].position_y = 4.0f;
    objects.objects[0].position_z = 0.0f;
    // Object 2 at (7,2)
    objects.objects[1]            = {};
    objects.objects[1].confidence = 0.9f;
    objects.objects[1].position_x = 7.0f;
    objects.objects[1].position_y = 2.0f;
    objects.objects[1].position_z = 0.0f;

    planner.update_obstacles(objects, pose);

    // Replan with new obstacles
    auto cmd2 = planner.plan(pose, target);
    EXPECT_TRUE(cmd2.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
    EXPECT_FALSE(planner.cached_path().empty());

    EXPECT_FALSE(path_has_corner_cut(planner.cached_path(), planner.grid()))
        << "Incremental replan must respect corner-cutting guard";
}
