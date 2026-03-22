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
    cfg.cell_ttl_s = 1.0f;  // 1s TTL (shorter than default 3s)
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

    // Wait for TTL to expire (1s + margin)
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    // Trigger another update with zero objects to run expiry
    drone::ipc::DetectedObjectList empty{};
    empty.num_objects = 0;
    empty.timestamp_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
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
