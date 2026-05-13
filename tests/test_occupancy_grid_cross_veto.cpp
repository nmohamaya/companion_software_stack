// tests/test_occupancy_grid_cross_veto.cpp
//
// Integration tests for OccupancyGrid3D::insert_voxels() with the cross-veto
// gate engaged.  Validates that the three-row promotion table from
// ADR-013 §2 item 4 is enforced end-to-end through the actual grid path,
// not just the pure decide_promotion() function (covered by
// test_cross_veto_decision.cpp).
//
// Issue: #698 Fix #1, Phase 2 of plan-scenario-33-pass.md.

#include "ipc/ipc_types.h"
#include "planner/occupancy_grid_3d.h"
#include "planner/radar_fov_gate.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

using drone::ipc::Pose;
using drone::ipc::RadarDetection;
using drone::ipc::RadarDetectionList;
using drone::ipc::SemanticVoxel;
using drone::planner::CrossVetoPolicy;
using drone::planner::GridCell;
using drone::planner::OccupancyGrid3D;
using drone::planner::RadarFovConfig;
using drone::planner::RadarFovGate;

namespace {

constexpr float    kRes        = 1.0f;  // 1 m resolution → grid coords == world metres
constexpr uint32_t kPromoHits  = 1;     // promote on first observation in tests for brevity
constexpr int      kInstanceId = 7;     // arbitrary non-zero instance id (skip noise gate)

// Construct an OccupancyGrid3D wired for the cross-veto tests.  Resolution,
// extent, inflation, cell_ttl, min_conf, promotion_hits, radar_promotion_hits,
// min_promotion_depth_conf, max_static, prediction_enabled, prediction_dt,
// require_radar, voxel_promotion_hits, static_cell_ttl,
// voxel_instance_promotion_observations.
OccupancyGrid3D make_grid(float cell_ttl_s = 2.0f) {
    return OccupancyGrid3D(
        /*resolution=*/kRes,
        /*extent=*/200.0f,
        /*inflation=*/0.5f,
        /*cell_ttl_s=*/cell_ttl_s,
        /*min_confidence=*/0.0f,
        /*promotion_hits=*/0,
        /*radar_promotion_hits=*/3,
        /*min_promotion_depth_confidence=*/0.0f,
        /*max_static_cells=*/0,
        /*prediction_enabled=*/false,
        /*prediction_dt_s=*/2.0f,
        /*require_radar_for_promotion=*/false,
        /*voxel_promotion_hits=*/static_cast<int>(kPromoHits),
        /*static_cell_ttl_s=*/0.0f,
        /*voxel_instance_promotion_observations=*/0);
}

Pose pose_at_origin() {
    Pose p{};
    p.translation[0] = 0.0;
    p.translation[1] = 0.0;
    p.translation[2] = 0.0;
    p.quaternion[0]  = 1.0;
    p.quaternion[1]  = 0.0;
    p.quaternion[2]  = 0.0;
    p.quaternion[3]  = 0.0;
    return p;
}

SemanticVoxel voxel_at(float wx, float wy, float wz) {
    SemanticVoxel v{};
    v.position_x     = wx;
    v.position_y     = wy;
    v.position_z     = wz;
    v.confidence     = 1.0f;
    v.semantic_label = drone::ipc::ObjectClass::UNKNOWN;
    v.occupancy      = 1.0f;
    v.instance_id    = kInstanceId;
    return v;
}

RadarDetectionList empty_radar() {
    RadarDetectionList list{};
    list.timestamp_ns   = 1;
    list.num_detections = 0;
    return list;
}

RadarDetectionList radar_with(float range_m, float az_rad, float el_rad) {
    RadarDetectionList list{};
    list.timestamp_ns                = 1;
    list.num_detections              = 1;
    list.detections[0].range_m       = range_m;
    list.detections[0].azimuth_rad   = az_rad;
    list.detections[0].elevation_rad = el_rad;
    list.detections[0].confidence    = 1.0f;
    return list;
}

uint64_t now_real_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

}  // namespace

// ── Row 1 — short range (stereo alone) ───────────────────────

TEST(OccupancyGridCrossVeto, ShortRange_PromoteWithoutRadar) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    auto v     = voxel_at(5.0f, 0.0f, 1.0f);
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);

    EXPECT_EQ(stats.cross_veto_deferred, 0u);
    EXPECT_EQ(stats.fov_silence_promoted, 0u);
    EXPECT_EQ(stats.age_cap_evicted, 0u);
    EXPECT_GT(grid.static_count(), 0u);
}

// ── Row 2 — long range, in FOV ───────────────────────────────

TEST(OccupancyGridCrossVeto, LongRangeInFov_RadarAgrees_Promote) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    const float el = std::atan2(1.0f, 15.0f);
    const float r  = std::sqrt(15.0f * 15.0f + 1.0f * 1.0f);
    gate.set_radar_detections(radar_with(r, 0.0f, el), now_real_ns());

    auto v     = voxel_at(15.0f, 0.0f, 1.0f);
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);

    EXPECT_EQ(stats.cross_veto_deferred, 0u);
    EXPECT_EQ(stats.fov_silence_promoted, 0u);
    EXPECT_EQ(stats.age_cap_evicted, 0u);
    EXPECT_GT(grid.static_count(), 0u);
}

TEST(OccupancyGridCrossVeto, LongRangeInFov_RadarSilent_Defer) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    auto v     = voxel_at(15.0f, 0.0f, 1.0f);
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);

    EXPECT_GT(stats.cross_veto_deferred, 0u);
    EXPECT_EQ(grid.static_count(), 0u);
}

// ── Row 3 — long range, outside FOV ──────────────────────────

TEST(OccupancyGridCrossVeto, LongRangeOutsideFov_NoResidency_NoAge_Defer) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    auto v     = voxel_at(-15.0f, 0.0f, 1.0f);  // behind drone
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);

    EXPECT_GT(stats.cross_veto_deferred, 0u);
    EXPECT_EQ(stats.fov_silence_promoted, 0u);
    EXPECT_EQ(stats.age_cap_evicted, 0u);
    EXPECT_EQ(grid.static_count(), 0u);
}

// ── Row 3 escape hatch (a) — FOV-silence promotion ───────────

TEST(OccupancyGridCrossVeto, FovSilencePromote_AfterEnoughInFovTime) {
    // Exercises the `PromoteFovSilence` branch end-to-end.  Sequence:
    //   1. Drone faces +X, cell at (15,0,1) is in FOV.  Tick residency
    //      enough to cross the threshold.
    //   2. Yaw drone 180°.  Cell is now behind drone (outside FOV).
    //   3. Insert voxel — query_cell sees !in_fov + residency >= threshold
    //      → row 3 escape hatch fires (PromoteFovSilence).
    auto policy                     = CrossVetoPolicy{};
    policy.fov_residency_promote_ns = 100'000'000ULL;  // 100 ms for fast test
    auto         grid = make_grid(/*cell_ttl_s=*/600.0f);
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, policy);
    gate.set_pose(pose);

    const GridCell c{15, 0, 1};

    // Step 1: accrue residency while cell is in FOV.
    std::vector<GridCell> live{c};
    gate.set_radar_detections(empty_radar(), 1'000'000'000ULL);
    gate.tick_residency(live, kRes, 1'000'000'000ULL);
    gate.set_radar_detections(empty_radar(), 1'060'000'000ULL);
    gate.tick_residency(live, kRes, 1'060'000'000ULL);
    gate.set_radar_detections(empty_radar(), 1'120'000'000ULL);
    gate.tick_residency(live, kRes, 1'120'000'000ULL);
    EXPECT_GE(gate.residency_ns(c), policy.fov_residency_promote_ns);

    // Step 2: yaw drone 180° — cell now outside FOV.  Refresh radar so
    // the staleness gate doesn't trip (insert_voxels uses real steady_clock).
    Pose yawed_pose            = pose;
    yawed_pose.quaternion[0]   = 0.0;  // w
    yawed_pose.quaternion[1]   = 0.0;
    yawed_pose.quaternion[2]   = 0.0;
    yawed_pose.quaternion[3]   = 1.0;  // z
    gate.set_pose(yawed_pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    // Step 3: insert at the same world position — now outside FOV.
    auto v = voxel_at(15.0f, 0.0f, 1.0f);
    auto s = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);
    EXPECT_GT(s.fov_silence_promoted, 0u);
    EXPECT_EQ(s.age_cap_evicted, 0u);
    EXPECT_GT(grid.static_count(), 0u);
    EXPECT_GT(grid.single_modality_static_count(), 0u);
}

// ── Row 3 escape hatch (b) — age-cap eviction ────────────────

TEST(OccupancyGridCrossVeto, AgeCapEvict_FiresIndependentOfResidency) {
    // Cell is behind drone (outside FOV) → never accrues residency.
    // Force `cell_age_ns >= dynamic_age_cap_ns` via fake timestamps and
    // verify age-cap eviction promotes with the dedicated counter.
    auto policy              = CrossVetoPolicy{};
    policy.dynamic_age_cap_ns = 50'000'000ULL;  // 50 ms for fast test
    auto         grid = make_grid(/*cell_ttl_s=*/600.0f);
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, policy);
    gate.set_pose(pose);

    const GridCell c{-15, 0, 1};
    std::vector<GridCell> live{c};

    // First tick stamps first_seen.
    gate.set_radar_detections(empty_radar(), 0ULL);
    gate.tick_residency(live, kRes, 0ULL);
    EXPECT_EQ(gate.residency_ns(c), 0u);
    // Cell stays outside FOV — residency stays at 0.  Age advances with now_ns.
    gate.set_radar_detections(empty_radar(), 60'000'000ULL);
    gate.tick_residency(live, kRes, 60'000'000ULL);
    EXPECT_EQ(gate.residency_ns(c), 0u);
    EXPECT_GE(gate.cell_age_ns(c, 60'000'000ULL), policy.dynamic_age_cap_ns);

    // Now insert and check that age-cap eviction fires (using the gate's
    // cached pose — caller doesn't pass pose).
    auto v = voxel_at(-15.0f, 0.0f, 1.0f);
    auto s = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);
    // Note: insert_voxels uses steady_clock::now() so cell_age may be much
    // larger than our fake 60 ms — that just makes the >= cap check fire
    // even more emphatically.  We can't compare exactly, only that the
    // age-cap branch fired and not the FOV-silence one.
    EXPECT_EQ(s.fov_silence_promoted, 0u);
    EXPECT_GT(s.age_cap_evicted, 0u);
    EXPECT_GT(grid.static_count(), 0u);
}

// ── Cross-cutting: total_* getters + clear_static() reset ────

TEST(OccupancyGridCrossVeto, AccumulatorsSurviveAcrossBatches) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    auto v = voxel_at(15.0f, 0.0f, 1.0f);
    grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);
    grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);
    EXPECT_GE(grid.total_cross_veto_deferred(), 2u);

    grid.clear_static();
    EXPECT_EQ(grid.total_cross_veto_deferred(), 0u);
    EXPECT_EQ(grid.total_fov_silence_promoted(), 0u);
    EXPECT_EQ(grid.total_age_cap_evicted(), 0u);
}

TEST(OccupancyGridCrossVeto, AccumulatorsStartAtZero) {
    auto grid = make_grid();
    EXPECT_EQ(grid.total_cross_veto_deferred(), 0u);
    EXPECT_EQ(grid.total_fov_silence_promoted(), 0u);
    EXPECT_EQ(grid.total_age_cap_evicted(), 0u);
    EXPECT_EQ(grid.single_modality_static_count(), 0u);
}

// ── dynamic_cell_keys() coverage ─────────────────────────────

TEST(OccupancyGridCrossVeto, DynamicCellKeys_EmptyAfterInit) {
    auto grid = make_grid();
    EXPECT_TRUE(grid.dynamic_cell_keys().empty());
}

TEST(OccupancyGridCrossVeto, DynamicCellKeys_ListsVetoedDeferredCells) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    auto v = voxel_at(15.0f, 0.0f, 1.0f);
    grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);

    auto keys = grid.dynamic_cell_keys();
    EXPECT_FALSE(keys.empty()) << "Vetoed cell should remain in dynamic layer.";
}

// ── NaN voxel rejection — must not reach the gate query ──────

TEST(OccupancyGridCrossVeto, NanVoxelPosition_ClampedDropped_DoesNotReachGate) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    auto v = voxel_at(std::numeric_limits<float>::quiet_NaN(), 0.0f, 1.0f);
    auto s = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);

    EXPECT_EQ(s.cross_veto_deferred, 0u);
    EXPECT_EQ(s.fov_silence_promoted, 0u);
    EXPECT_EQ(s.age_cap_evicted, 0u);
    EXPECT_GT(s.clamped_dropped + s.out_of_bounds, 0u)
        << "NaN voxel must drop early via clamp/oob filter, never reach the gate.";
    EXPECT_EQ(grid.static_count(), 0u);
}

// ── Empty input boundary ─────────────────────────────────────

TEST(OccupancyGridCrossVeto, NullInput_NoOp) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    auto s = grid.insert_voxels(nullptr, 0, 100.0f, 0.3f, &gate);
    EXPECT_EQ(s.inserted, 0u);
    EXPECT_EQ(s.cross_veto_deferred, 0u);
}

TEST(OccupancyGridCrossVeto, ZeroCount_NoOp) {
    auto         grid = make_grid();
    auto         pose = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    auto v = voxel_at(15.0f, 0.0f, 1.0f);
    auto s = grid.insert_voxels(&v, 0, 100.0f, 0.3f, &gate);
    EXPECT_EQ(s.inserted, 0u);
}

// ── Legacy null-gate path ────────────────────────────────────

TEST(OccupancyGridCrossVeto, NullGate_PromotesUnconditionally) {
    auto grid  = make_grid();
    auto v     = voxel_at(15.0f, 0.0f, 1.0f);
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, nullptr);

    EXPECT_EQ(stats.cross_veto_deferred, 0u);
    EXPECT_GT(grid.static_count(), 0u);
}

TEST(OccupancyGridCrossVeto, GateWithoutPose_PromotesUnconditionally) {
    auto         grid = make_grid();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});  // no set_pose() called
    auto         v    = voxel_at(15.0f, 0.0f, 1.0f);
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &gate);
    EXPECT_EQ(stats.cross_veto_deferred, 0u);
    EXPECT_GT(grid.static_count(), 0u)
        << "Gate without pose must not block — caller asked for veto but the gate isn't ready.";
}
