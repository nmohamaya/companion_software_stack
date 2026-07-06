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
#include "util/mock_clock.h"  // Issue #799 Phase B — deterministic static-cell decay

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

// Grid wired for the takeoff promotion-gate tests (#789): static promotion
// ENABLED (hit-count path, 2 hits), depth gate open, generous static cap.
// Only min_promotion_altitude_m varies per test (last param).
OccupancyGrid3D make_promoting_grid(float min_promotion_altitude_m) {
    return OccupancyGrid3D(/*resolution=*/1.0f, /*extent=*/20.0f, /*inflation=*/1.0f,
                           /*cell_ttl_s=*/3.0f, /*min_confidence=*/0.3f, /*promotion_hits=*/2,
                           /*radar_promotion_hits=*/3, /*min_promotion_depth_confidence=*/0.0f,
                           /*max_static_cells=*/100, /*prediction_enabled=*/false,
                           /*prediction_dt_s=*/2.0f, /*require_radar_for_promotion=*/false,
                           /*voxel_promotion_hits=*/3, /*static_cell_ttl_s=*/0.0f,
                           /*voxel_instance_promotion_observations=*/0,
                           /*min_obstacle_altitude_m=*/0.0f, /*dynamic_confirmation_hits=*/1,
                           min_promotion_altitude_m);
}

// Grid wired for the static-cell decay tests (#799 Phase B): static promotion
// ENABLED (2 hits), no takeoff gate, generous cap. static_cell_ttl_s varies.
OccupancyGrid3D make_decaying_grid(float static_cell_ttl_s) {
    return OccupancyGrid3D(/*resolution=*/1.0f, /*extent=*/20.0f, /*inflation=*/1.0f,
                           /*cell_ttl_s=*/3.0f, /*min_confidence=*/0.3f, /*promotion_hits=*/2,
                           /*radar_promotion_hits=*/3, /*min_promotion_depth_confidence=*/0.0f,
                           /*max_static_cells=*/100, /*prediction_enabled=*/false,
                           /*prediction_dt_s=*/2.0f, /*require_radar_for_promotion=*/false,
                           /*voxel_promotion_hits=*/3, static_cell_ttl_s,
                           /*voxel_instance_promotion_observations=*/0,
                           /*min_obstacle_altitude_m=*/0.0f, /*dynamic_confirmation_hits=*/1,
                           /*min_promotion_altitude_m=*/0.0f);
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

// ── 5. TTL=0 must not break confirmation (Copilot PR #787) ────────────────
// With dynamic_obstacle_ttl_s=0 (cell_ttl_ns_==0) AND dynamic_confirmation_hits>1,
// the staleness reset `now_ns - last_ns > 0` would fire on every later observation,
// resetting the pending count to 0 each time → the cell could NEVER reach the
// threshold → camera obstacles suppressed entirely. The fix guards the reset on
// `cell_ttl_ns_ > 0`, so a stable obstacle still confirms when TTL is disabled.
TEST(OccupancyGridDynamicGating, TtlZeroDoesNotBreakConfirmation) {
    // ttl_s = 0 (TTL disabled), confirmation_hits = 3, promotion off.
    OccupancyGrid3D  grid(/*resolution=*/1.0f, /*extent=*/20.0f, /*inflation=*/1.0f,
                         /*cell_ttl_s=*/0.0f, /*min_confidence=*/0.3f, /*promotion_hits=*/0,
                         /*radar_promotion_hits=*/3, /*min_promotion_depth_confidence=*/0.5f,
                         /*max_static_cells=*/0, /*prediction_enabled=*/false,
                         /*prediction_dt_s=*/2.0f, /*require_radar_for_promotion=*/false,
                         /*voxel_promotion_hits=*/3, /*static_cell_ttl_s=*/0.0f,
                         /*voxel_instance_promotion_observations=*/0,
                         /*min_obstacle_altitude_m=*/0.0f,
                         /*dynamic_confirmation_hits=*/3);
    auto             obstacle = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose pose{};

    grid.update_from_objects(obstacle, pose);
    EXPECT_FALSE(grid.is_occupied({5, 5, 5})) << "1/3: pending";
    grid.update_from_objects(obstacle, pose);
    EXPECT_FALSE(grid.is_occupied({5, 5, 5})) << "2/3: pending (must NOT reset under TTL=0)";
    grid.update_from_objects(obstacle, pose);
    EXPECT_TRUE(grid.is_occupied({5, 5, 5}))
        << "3/3 with TTL=0 must still confirm — the staleness reset must not suppress entirely";
}

// ── 6. Issue #789 — takeoff promotion gate (min_promotion_altitude_m) ──────
// While the drone is below the promotion-altitude floor (climbing off the pad),
// camera detections must NOT promote to the permanent static layer — but the
// dynamic TTL layer must still populate so the reactive avoider keeps working.
TEST(OccupancyGridTakeoffGate, BelowPromotionAltitudeSuppressesStaticPromotion) {
    OccupancyGrid3D  grid     = make_promoting_grid(/*min_promotion_alt=*/2.0f);
    auto             obstacle = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose pose{};
    pose.translation[2] = 1.0;  // drone at 1 m — below the 2 m promotion floor (taking off)

    // Feed more than promotion_hits (=2) observations; none must promote.
    for (int i = 0; i < 4; ++i) grid.update_from_objects(obstacle, pose);

    EXPECT_EQ(grid.promoted_count(), 0)
        << "static promotion must be suppressed while the drone is below the promotion floor";
    EXPECT_GT(grid.occupied_count(), 0u) << "fail-safe: the dynamic TTL layer must still populate "
                                            "(reactive avoider backstop intact)";
}

TEST(OccupancyGridTakeoffGate, AbovePromotionAltitudeAllowsPromotion) {
    OccupancyGrid3D  grid     = make_promoting_grid(/*min_promotion_alt=*/2.0f);
    auto             obstacle = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose pose{};
    pose.translation[2] = 3.0;  // drone at 3 m — above the floor (cruise/survey)

    for (int i = 0; i < 4; ++i) grid.update_from_objects(obstacle, pose);

    EXPECT_GT(grid.promoted_count(), 0)
        << "once the drone clears the promotion floor, a stable obstacle must promote to static";
}

TEST(OccupancyGridTakeoffGate, DisabledGatePromotesEvenWhenLow) {
    OccupancyGrid3D  grid     = make_promoting_grid(/*min_promotion_alt=*/0.0f);  // gate OFF
    auto             obstacle = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose pose{};
    pose.translation[2] = 1.0;  // low, but the gate is disabled (default behaviour)

    for (int i = 0; i < 4; ++i) grid.update_from_objects(obstacle, pose);

    EXPECT_GT(grid.promoted_count(), 0) << "min_promotion_altitude_m=0 must preserve prior "
                                           "behaviour: promote regardless of altitude";
}

// ── 7. Issue #792 (SAFETY) — radar estimated_radius_m must be finite + clamped ──
// A pathological radar radius (huge or non-finite, e.g. a gpu_lidar/UKF degenerate
// size estimate) must NOT make inflate_disk_at_cell_'s O(N^2) disk loop run
// unbounded — that hangs the planning_loop thread (uncontrolled hover, Issue #792).
// The grid here is 20m half-extent / 1m res → at most (2*20+1)^2 = 1681 cells per
// z-layer; without the clamp a 1000 km radius would try ~10^12 iterations and hang.
namespace {
drone::ipc::DetectedObjectList make_radar_detection(float px, float py, float pz, float radius_m) {
    drone::ipc::DetectedObjectList objects{};
    objects.num_objects                   = 1;
    objects.objects[0].track_id           = 1;
    objects.objects[0].class_id           = drone::ipc::ObjectClass::UNKNOWN;
    objects.objects[0].confidence         = 0.9f;
    objects.objects[0].position_x         = px;
    objects.objects[0].position_y         = py;
    objects.objects[0].position_z         = pz;
    objects.objects[0].has_camera         = true;
    objects.objects[0].has_radar          = true;
    objects.objects[0].radar_update_count = 5;
    objects.objects[0].estimated_radius_m = radius_m;
    return objects;
}
}  // namespace

TEST(OccupancyGridRadarInflation, HugeRadarRadiusIsClampedNotUnbounded) {
    OccupancyGrid3D  grid = make_grid(/*min_alt=*/0.0f, /*confirm_hits=*/1);
    auto             objs = make_radar_detection(5.0f, 5.0f, 5.0f, /*radius_m=*/1.0e6f);  // 1000 km
    drone::ipc::Pose pose{};
    grid.update_from_objects(objs, pose);  // must RETURN PROMPTLY (clamped), not hang
    EXPECT_GT(grid.occupied_count(), 0u);
    EXPECT_LE(grid.occupied_count(), static_cast<size_t>(41u * 41u))
        << "huge radar radius must be clamped to the grid half-extent, not run unbounded";
}

TEST(OccupancyGridRadarInflation, NonFiniteRadarRadiusFallsBackToConfigInflation) {
    OccupancyGrid3D grid = make_grid(/*min_alt=*/0.0f, /*confirm_hits=*/1);
    auto objs = make_radar_detection(5.0f, 5.0f, 5.0f, std::numeric_limits<float>::infinity());
    drone::ipc::Pose pose{};
    grid.update_from_objects(objs, pose);  // isfinite guard → config inflation, no UB/hang
    EXPECT_GT(grid.occupied_count(), 0u);
    EXPECT_LE(grid.occupied_count(), static_cast<size_t>(41u * 41u))
        << "non-finite radar radius must fall back to the config inflation, not cast to UB";
}

// add_static_obstacle (HD-map path) is the unguarded twin of the radar bug —
// radius_m/height_m come from config JSON, so a typo must not hang the planner.
TEST(OccupancyGridRadarInflation, StaticObstacleHugeRadiusIsClampedNotUnbounded) {
    OccupancyGrid3D grid = make_grid(/*min_alt=*/0.0f, /*confirm_hits=*/1);
    grid.add_static_obstacle(0.0f, 0.0f, /*radius_m=*/1.0e9f, /*height_m=*/1.0e9f);  // config typo
    EXPECT_GT(grid.static_count(), 0u);
    EXPECT_LE(grid.static_count(), static_cast<size_t>(41u * 41u * 41u))
        << "huge HD-map radius must be clamped to the grid, not run unbounded";
}

TEST(OccupancyGridRadarInflation, StaticObstacleNonFiniteGeometryIgnored) {
    OccupancyGrid3D grid = make_grid(/*min_alt=*/0.0f, /*confirm_hits=*/1);
    grid.add_static_obstacle(0.0f, 0.0f, std::numeric_limits<float>::infinity(), 2.0f);
    EXPECT_EQ(grid.static_count(), 0u) << "non-finite HD-map geometry must be ignored, not cast UB";
}

// ── 6. Static-cell decay (Issue #799 Phase B) ────────────────────────────
// Promoted (static-layer) cells now decay after `static_cell_ttl_s` (default
// 30 s) without re-observation — bounds the perception-ghost accumulation that
// sealed the grid in scenario 18 (25→650 static cells → D*Lite no-path).
// Deterministic via ScopedMockClock: the grid now reads drone::util::get_clock()
// (Phase B migration), so time is driven in microseconds — no wall-clock sleeps
// (the flaky-under-tsan anti-pattern). Re-observed cells refresh; 0 = disabled.

TEST(OccupancyGridStaticDecay, PromotedGhostDecaysAfterTtl) {
    drone::util::ScopedMockClock clock{1'000'000'000ULL};
    OccupancyGrid3D              grid  = make_decaying_grid(/*static_cell_ttl_s=*/30.0f);
    auto                         ghost = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose             pose{};

    // Promote the ghost into the static layer (>= promotion_hits observations).
    for (int i = 0; i < 3; ++i) grid.update_from_objects(ghost, pose);
    ASSERT_GT(grid.promoted_count(), 0) << "ghost must promote to static after the hit threshold";

    // Advance past the TTL, then tick the grid with a detection FAR from the
    // ghost — this runs the sweep without re-observing (refreshing) the ghost.
    clock.mock().advance_s(31);
    grid.update_from_objects(make_detection(-8.0f, -8.0f, 5.0f), pose);

    EXPECT_EQ(grid.promoted_count(), 0)
        << "a promoted cell not re-observed for its TTL must decay out of the static layer "
           "(the #799 fix that bounds ghost accumulation)";
}

TEST(OccupancyGridStaticDecay, ReobservedPromotedCellSurvivesPastTtl) {
    drone::util::ScopedMockClock clock{1'000'000'000ULL};
    OccupancyGrid3D              grid     = make_decaying_grid(/*static_cell_ttl_s=*/30.0f);
    auto                         obstacle = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose             pose{};

    for (int i = 0; i < 3; ++i) grid.update_from_objects(obstacle, pose);
    ASSERT_GT(grid.promoted_count(), 0);
    const int promoted = grid.promoted_count();

    // Re-observe at 20 s (refreshes the cell's timestamp), then reach 40 s total.
    // 40 s > the 30 s TTL, but only 20 s since the last observation → must persist.
    clock.mock().advance_s(20);
    grid.update_from_objects(obstacle, pose);  // refresh
    clock.mock().advance_s(20);
    grid.update_from_objects(make_detection(-8.0f, -8.0f, 5.0f),
                             pose);  // sweep, no re-obs of obstacle

    EXPECT_EQ(grid.promoted_count(), promoted)
        << "a continuously re-observed real obstacle refreshes its timestamp and must persist "
           "past the raw TTL — decay only clears the drone's un-re-observed wake";
}

TEST(OccupancyGridStaticDecay, TtlZeroKeepsLegacyPermanentPromotion) {
    drone::util::ScopedMockClock clock{1'000'000'000ULL};
    OccupancyGrid3D              grid = make_decaying_grid(/*static_cell_ttl_s=*/0.0f);  // sentinel
    auto                         ghost = make_detection(5.0f, 5.0f, 5.0f);
    drone::ipc::Pose             pose{};

    for (int i = 0; i < 3; ++i) grid.update_from_objects(ghost, pose);
    ASSERT_GT(grid.promoted_count(), 0);
    const int promoted = grid.promoted_count();

    clock.mock().advance_s(600);  // far past any TTL
    grid.update_from_objects(make_detection(-8.0f, -8.0f, 5.0f), pose);

    EXPECT_EQ(grid.promoted_count(), promoted)
        << "static_cell_ttl_s == 0 is the documented 'disabled' sentinel — permanent promotion";
}
