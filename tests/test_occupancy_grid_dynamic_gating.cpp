// tests/test_occupancy_grid_dynamic_gating.cpp
//
// Issue #764 — unit tests for the camera dynamic-add gates added to
// OccupancyGrid3D::update_from_objects():
//   1. Ground-plane reject  (position_z < min_obstacle_altitude_m)
//   2. Non-finite (NaN) position guard
//   3. Dynamic-confirmation hits (N observations before a camera cell occupies)
//   4. Default behaviour preserved (confirmation_hits == 1 → immediate occupy)
//
// These gates bring the camera/objects path up to the safety the voxel path
// already had, stopping color_contour ground/depth-noise ghosts from flooding
// the dynamic planning grid (D*Lite path-extraction failures, ~50% scenario-02
// mission-timeout). Safety lens: every gate biases toward KEEPING obstacles —
// see DESIGN_RATIONALE DR for the asymmetric-cost analysis.
#include "planner/occupancy_grid_3d.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using drone::planner::GridCell;
using drone::planner::OccupancyGrid3D;

namespace {

// Grid wired for the gating tests: 1 m resolution, 1-cell inflation, promotion
// disabled (cells stay dynamic), prediction off. Only the two trailing gate
// params vary per test.
OccupancyGrid3D make_grid(float min_obstacle_altitude_m, int dynamic_confirmation_hits) {
    return OccupancyGrid3D(/*resolution=*/1.0f, /*extent=*/20.0f, /*inflation=*/1.0f,
                           /*cell_ttl_s=*/3.0f, /*min_confidence=*/0.3f, /*promotion_hits=*/0,
                           /*radar_promotion_hits=*/3, /*min_promotion_depth_confidence=*/0.5f,
                           /*max_static_cells=*/0, /*prediction_enabled=*/false,
                           /*prediction_dt_s=*/2.0f, /*require_radar_for_promotion=*/false,
                           /*voxel_promotion_hits=*/3, /*static_cell_ttl_s=*/0.0f,
                           /*voxel_instance_promotion_observations=*/0, min_obstacle_altitude_m,
                           dynamic_confirmation_hits);
}

// Single camera detection (no radar) at (px,py,pz), confidence 0.9. Placed away
// from the origin so it never lands in the drone's self-exclusion zone.
drone::ipc::DetectedObjectList make_detection(float px, float py, float pz) {
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects                   = 1;
    objects.objects[0].track_id           = 1;
    objects.objects[0].class_id           = drone::ipc::ObjectClass::UNKNOWN;
    objects.objects[0].confidence         = 0.9f;
    objects.objects[0].position_x         = px;
    objects.objects[0].position_y         = py;
    objects.objects[0].position_z         = pz;
    objects.objects[0].has_camera         = true;
    objects.objects[0].has_radar          = false;
    objects.objects[0].radar_update_count = 0;
    return objects;
}

}  // namespace

// ── 1. Ground-plane reject ───────────────────────────────────────────────
TEST(OccupancyGridDynamicGating, GroundDetectionBelowFloorRejected) {
    OccupancyGrid3D  grid   = make_grid(/*min_alt=*/0.5f, /*confirm_hits=*/1);
    auto             ground = make_detection(5.0f, 5.0f, 0.1f);  // below the 0.5 m floor
    drone::ipc::Pose pose{};
    grid.update_from_objects(ground, pose);
    EXPECT_FALSE(grid.is_occupied({5, 5, 0}))
        << "Ground-texture detection below min_obstacle_altitude_m must not occupy the grid";
    EXPECT_EQ(grid.occupied_count(), 0u);
}

TEST(OccupancyGridDynamicGating, ObstacleAboveFloorOccupies) {
    OccupancyGrid3D  grid     = make_grid(/*min_alt=*/0.5f, /*confirm_hits=*/1);
    auto             obstacle = make_detection(5.0f, 5.0f, 5.0f);  // well above the floor
    drone::ipc::Pose pose{};
    grid.update_from_objects(obstacle, pose);
    EXPECT_TRUE(grid.is_occupied({5, 5, 5})) << "A detection above the altitude floor must still "
                                                "occupy the grid (no real obstacle lost)";
}

// ── 2. Non-finite position guard ─────────────────────────────────────────
TEST(OccupancyGridDynamicGating, NaNPositionProducesNoPhantomCell) {
    OccupancyGrid3D  grid = make_grid(/*min_alt=*/0.0f, /*confirm_hits=*/1);
    auto             bad  = make_detection(std::numeric_limits<float>::quiet_NaN(), 5.0f, 5.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(bad, pose);
    EXPECT_EQ(grid.occupied_count(), 0u)
        << "A non-finite detection position must be dropped, not land as a phantom grid cell";
}

// ── 3. Dynamic-confirmation hits ─────────────────────────────────────────
TEST(OccupancyGridDynamicGating, RequiresNObservationsBeforeOccupying) {
    OccupancyGrid3D  grid     = make_grid(/*min_alt=*/0.0f, /*confirm_hits=*/3);
    auto             obstacle = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose pose{};

    // First two observations: pending, not yet a planning obstacle.
    grid.update_from_objects(obstacle, pose);
    EXPECT_FALSE(grid.is_occupied({5, 5, 5})) << "1/3 observations: cell must stay free (pending)";
    EXPECT_EQ(grid.occupied_count(), 0u);

    grid.update_from_objects(obstacle, pose);
    EXPECT_FALSE(grid.is_occupied({5, 5, 5})) << "2/3 observations: cell must stay free (pending)";

    // Third observation confirms the stable obstacle.
    grid.update_from_objects(obstacle, pose);
    EXPECT_TRUE(grid.is_occupied({5, 5, 5}))
        << "3/3 observations: a stable obstacle must confirm and occupy";
}

// ── 4. Default behaviour preserved (confirmation_hits == 1) ───────────────
TEST(OccupancyGridDynamicGating, DefaultHitsOneOccupiesImmediately) {
    OccupancyGrid3D  grid     = make_grid(/*min_alt=*/0.0f, /*confirm_hits=*/1);
    auto             obstacle = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose pose{};
    grid.update_from_objects(obstacle, pose);
    EXPECT_TRUE(grid.is_occupied({5, 5, 5}))
        << "confirmation_hits=1 must preserve prior behaviour: occupy on first observation";
}
