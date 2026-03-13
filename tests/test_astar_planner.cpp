// tests/test_astar_planner.cpp
// Unit tests for A* 3D Grid Path Planner (Phase 3).
#include "planner/astar_planner.h"
#include "planner/ipath_planner.h"

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::planner;

// ═════════════════════════════════════════════════════════════
// OccupancyGrid3D tests
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
    // grid extent = 10m, resolution = 1m → half_extent_cells = 10
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

    // Center cell should be occupied
    EXPECT_TRUE(grid.is_occupied({5, 5, 5}));
    // Inflated cells should also be occupied (within radius)
    EXPECT_TRUE(grid.is_occupied({6, 5, 5}));
    EXPECT_TRUE(grid.is_occupied({5, 6, 5}));
    // Far away should be clear
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
// A* search tests
// ═════════════════════════════════════════════════════════════

TEST(AStarSearchTest, TrivialStartEqualsGoal) {
    OccupancyGrid3D grid(1.0f, 10.0f, 1.0f);
    GridCell        start{0, 0, 0};
    auto            result = astar_search(grid, start, start);

    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.path.size(), 1u);
}

TEST(AStarSearchTest, StraightLinePath) {
    OccupancyGrid3D grid(1.0f, 10.0f, 1.0f);  // empty grid
    GridCell        start{0, 0, 0};
    GridCell        goal{5, 0, 0};

    auto result = astar_search(grid, start, goal);
    EXPECT_TRUE(result.found);
    EXPECT_GT(result.path.size(), 1u);
    EXPECT_EQ(result.path.front(), start);
    EXPECT_EQ(result.path.back(), goal);
    EXPECT_GT(result.iterations, 0);
}

TEST(AStarSearchTest, PathAround3DObstacle) {
    OccupancyGrid3D grid(1.0f, 20.0f, 0.5f);  // small inflation

    // Place a wall of obstacles along x=5, y=-2..2, z=-2..2
    drone::ipc::DetectedObjectList objects{};
    uint32_t                       idx = 0;
    for (int y = -2; y <= 2; ++y) {
        for (int z = -2; z <= 2; ++z) {
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
    grid.update_from_objects(objects, pose);

    GridCell start{0, 0, 0};
    GridCell goal{10, 0, 0};

    auto result = astar_search(grid, start, goal, 100000);
    EXPECT_TRUE(result.found);
    // Path should go around the obstacle (longer than straight line)
    EXPECT_GT(result.path.size(), 10u);
    EXPECT_EQ(result.path.front(), start);
    EXPECT_EQ(result.path.back(), goal);
}

TEST(AStarSearchTest, UnreachableGoal) {
    OccupancyGrid3D grid(1.0f, 5.0f, 0.5f);

    // Occupy the goal cell
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 4.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 0.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    GridCell start{0, 0, 0};
    GridCell goal{4, 0, 0};

    auto result = astar_search(grid, start, goal);
    EXPECT_FALSE(result.found);
}

TEST(AStarSearchTest, StartBlocked) {
    // Fix #22: BFS start-escape finds the nearest free cell when the start is
    // occupied, so a path *is* found despite the start cell being blocked.
    OccupancyGrid3D grid(1.0f, 10.0f, 0.5f);

    drone::ipc::DetectedObjectList objects{};
    objects.num_objects           = 1;
    objects.objects[0].position_x = 0.0f;
    objects.objects[0].position_y = 0.0f;
    objects.objects[0].position_z = 0.0f;
    objects.objects[0].confidence = 0.9f;

    drone::ipc::Pose pose{};
    grid.update_from_objects(objects, pose);

    auto result = astar_search(grid, {0, 0, 0}, {5, 0, 0});
    EXPECT_TRUE(result.found);          // BFS escapes to adjacent free cell
    EXPECT_FALSE(result.path.empty());  // valid path returned
    // The effective start should differ from the blocked (0,0,0) cell
    auto first = result.path.front();
    EXPECT_FALSE(first.x == 0 && first.y == 0 && first.z == 0);
}

TEST(AStarSearchTest, GoalOutOfBounds) {
    OccupancyGrid3D grid(1.0f, 5.0f, 1.0f);
    auto            result = astar_search(grid, {0, 0, 0}, {100, 0, 0});
    EXPECT_FALSE(result.found);
}

TEST(AStarSearchTest, MaxIterationsLimitsSearch) {
    OccupancyGrid3D grid(1.0f, 50.0f, 1.0f);
    GridCell        start{0, 0, 0};
    GridCell        goal{40, 40, 0};

    auto result = astar_search(grid, start, goal, 10);  // very low limit
    // May or may not find path, but iterations should be <= 10
    EXPECT_LE(result.iterations, 10);
}

TEST(AStarSearchTest, DiagonalPathWorldCoords) {
    OccupancyGrid3D grid(1.0f, 10.0f, 1.0f);
    GridCell        start{0, 0, 0};
    GridCell        goal{3, 3, 3};

    auto result = astar_search(grid, start, goal);
    EXPECT_TRUE(result.found);
    EXPECT_EQ(result.world_path.size(), result.path.size());

    // World path start/end should correspond to grid start/end
    auto ws = result.world_path.front();
    auto we = result.world_path.back();
    EXPECT_NEAR(ws[0], 0.0f, 0.5f);
    EXPECT_NEAR(we[0], 3.0f, 0.5f);
    EXPECT_NEAR(we[1], 3.0f, 0.5f);
    EXPECT_NEAR(we[2], 3.0f, 0.5f);
}

// ═════════════════════════════════════════════════════════════
// AStarPathPlanner (IPathPlanner) tests
// ═════════════════════════════════════════════════════════════

TEST(AStarPathPlannerTest, PlanReturnsValidCmd) {
    AStarPathPlanner planner;

    drone::ipc::Pose pose{};
    pose.translation[0] = 0.0;
    pose.translation[1] = 0.0;
    pose.translation[2] = 5.0;

    Waypoint target{10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, false};

    auto cmd = planner.plan(pose, target);
    EXPECT_TRUE(cmd.valid);
    EXPECT_GT(cmd.timestamp_ns, 0u);
    // Should have some velocity toward the target (positive X)
    EXPECT_GT(cmd.velocity_x, 0.0f);
    EXPECT_NEAR(cmd.target_x, 10.0f, 0.01f);
}

TEST(AStarPathPlannerTest, FallbackWhenObstacleBlocks) {
    // Goal-snap logic redirects occupied waypoints to the nearest lateral
    // free cell, so A* succeeds instead of falling back to direct flight.
    AStarConfig config;
    config.resolution_m       = 1.0f;
    config.grid_extent_m      = 10.0f;
    config.inflation_radius_m = 0.5f;
    AStarPathPlanner planner(config);

    // Block the goal with obstacles
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

    // Goal snap finds a nearby free cell → A* succeeds without direct fallback
    EXPECT_TRUE(cmd.valid);
    EXPECT_FALSE(planner.using_direct_fallback());
}

TEST(AStarPathPlannerTest, NameIsCorrect) {
    AStarPathPlanner planner;
    EXPECT_EQ(planner.name(), "AStarPathPlanner");
}

TEST(AStarPathPlannerTest, GridAccessor) {
    AStarPathPlanner planner;
    EXPECT_EQ(planner.grid().occupied_count(), 0u);
}

TEST(AStarPathPlannerTest, CachedPathAccessor) {
    AStarPathPlanner planner;
    EXPECT_TRUE(planner.cached_path().empty());
}

// ═════════════════════════════════════════════════════════════
// Factory registration
// ═════════════════════════════════════════════════════════════

TEST(PathPlannerFactory, AStarBackendRegistered) {
    auto planner = create_path_planner("astar");
    EXPECT_NE(planner, nullptr);
    EXPECT_EQ(planner->name(), "AStarPathPlanner");
}

TEST(PathPlannerFactory, UnknownThrows) {
    EXPECT_THROW(create_path_planner("nonexistent"), std::runtime_error);
}

// ═════════════════════════════════════════════════════════════
// GridCellHash test
// ═════════════════════════════════════════════════════════════

TEST(GridCellHashTest, DifferentCellsDifferentHashes) {
    GridCellHash h;
    // Not strictly required, but very likely for nearby cells
    EXPECT_NE(h({0, 0, 0}), h({1, 0, 0}));
    EXPECT_NE(h({0, 0, 0}), h({0, 1, 0}));
    EXPECT_NE(h({0, 0, 0}), h({0, 0, 1}));
}

TEST(GridCellHashTest, SameCellSameHash) {
    GridCellHash h;
    EXPECT_EQ(h({3, 4, 5}), h({3, 4, 5}));
}
