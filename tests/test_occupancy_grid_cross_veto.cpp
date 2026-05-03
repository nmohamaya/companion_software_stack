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
#include <thread>

using drone::ipc::Pose;
using drone::ipc::RadarDetection;
using drone::ipc::RadarDetectionList;
using drone::ipc::SemanticVoxel;
using drone::planner::CrossVetoPolicy;
using drone::planner::OccupancyGrid3D;
using drone::planner::RadarFovConfig;
using drone::planner::RadarFovGate;

namespace {

constexpr float    kRes        = 1.0f;  // 1 m resolution → grid coords == world metres
constexpr uint32_t kPromoHits  = 1;     // promote on first observation in tests for brevity
constexpr int      kInstanceId = 7;     // arbitrary non-zero instance id (skip noise gate)

// Construct an OccupancyGrid3D wired for the cross-veto tests.  We use the
// real ctor signature (resolution, extent, inflation, cell_ttl, min_conf,
// promotion_hits, radar_promotion_hits, min_promotion_depth_conf, max_static,
// prediction_enabled, prediction_dt, require_radar, voxel_promotion_hits,
// static_cell_ttl, voxel_instance_promotion_observations).  Inflation is
// kept positive (the ctor max()'s to 1 cell anyway).
OccupancyGrid3D make_grid() {
    return OccupancyGrid3D(
        /*resolution=*/kRes,
        /*extent=*/200.0f,
        /*inflation=*/0.5f,
        /*cell_ttl_s=*/2.0f,
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
    p.quaternion[0]  = 1.0;  // w (identity)
    p.quaternion[1]  = 0.0;
    p.quaternion[2]  = 0.0;
    p.quaternion[3]  = 0.0;
    return p;
}

// Build a single PATH A voxel at (wx, wy, wz) with non-zero instance_id so
// the instance gate doesn't reject it.
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
    list.timestamp_ns           = 1;
    list.num_detections         = 1;
    list.detections[0].range_m       = range_m;
    list.detections[0].azimuth_rad   = az_rad;
    list.detections[0].elevation_rad = el_rad;
    list.detections[0].confidence    = 1.0f;
    return list;
}

constexpr uint64_t kMs(uint64_t ms) { return ms * 1'000'000ULL; }

// insert_voxels() uses steady_clock::now() internally for the gate query,
// so the radar timestamp in tests must use the same clock source — otherwise
// the radar appears arbitrarily old / from-the-future and the staleness gate
// trips spuriously.
uint64_t now_real_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

}  // namespace

// ── Row 1 — short range (stereo alone) ───────────────────────

TEST(OccupancyGridCrossVeto, ShortRange_PromoteWithoutRadar) {
    auto grid  = make_grid();
    auto pose  = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    // Voxel at 5 m forward — within the default 10 m short-range threshold.
    auto v = voxel_at(5.0f, 0.0f, 1.0f);  // z=1.0 > kMinObstacleZ
    auto stats = grid.insert_voxels(&v, 1, /*clamp_m=*/100.0f, /*min_confidence=*/0.3f, &pose, &gate);

    EXPECT_EQ(stats.cross_veto_deferred, 0u);
    EXPECT_EQ(stats.single_modality_promoted, 0u);
    EXPECT_GT(grid.static_count(), 0u) << "Short-range voxel must promote on stereo alone.";
}

// ── Row 2 — long range, in FOV ───────────────────────────────

TEST(OccupancyGridCrossVeto, LongRangeInFov_RadarAgrees_Promote) {
    auto grid  = make_grid();
    auto pose  = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    // Radar return co-located with the voxel at 15 m forward.  The voxel
    // is at z=1 m, so use the corresponding elevation angle (~3.8°).
    const float el = std::atan2(1.0f, 15.0f);
    const float r  = std::sqrt(15.0f * 15.0f + 1.0f * 1.0f);
    gate.set_radar_detections(radar_with(r, 0.0f, el), now_real_ns());

    auto v     = voxel_at(15.0f, 0.0f, 1.0f);  // z=1.0 > kMinObstacleZ=0.3
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &pose, &gate);

    EXPECT_EQ(stats.cross_veto_deferred, 0u);
    EXPECT_EQ(stats.single_modality_promoted, 0u);
    EXPECT_GT(grid.static_count(), 0u);
}

TEST(OccupancyGridCrossVeto, LongRangeInFov_RadarSilent_Defer) {
    auto grid  = make_grid();
    auto pose  = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    auto v     = voxel_at(15.0f, 0.0f, 1.0f);  // z=1.0 > kMinObstacleZ=0.3
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &pose, &gate);

    EXPECT_GT(stats.cross_veto_deferred, 0u)
        << "Long-range voxel in radar FOV with no radar return must be vetoed.";
    EXPECT_EQ(grid.static_count(), 0u) << "Vetoed voxel must NOT cement.";
}

TEST(OccupancyGridCrossVeto, LongRangeInFov_RadarStale_Defer) {
    auto grid  = make_grid();
    auto pose  = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    // Radar set "now"; sleep past the 100 ms staleness threshold before
    // calling insert_voxels (which queries with steady_clock::now() internally).
    const float el = std::atan2(1.0f, 15.0f);
    const float r  = std::sqrt(15.0f * 15.0f + 1.0f * 1.0f);
    gate.set_radar_detections(radar_with(r, 0.0f, el), now_real_ns());

    auto v = voxel_at(15.0f, 0.0f, 1.0f);  // z=1.0 > kMinObstacleZ
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &pose, &gate);

    EXPECT_GT(stats.cross_veto_deferred, 0u)
        << "Stale radar must veto promotion even with a co-located return.";
    EXPECT_EQ(grid.static_count(), 0u);
}

// ── Row 3 — long range, outside FOV ──────────────────────────

TEST(OccupancyGridCrossVeto, LongRangeOutsideFov_NoResidency_Defer) {
    auto grid  = make_grid();
    auto pose  = pose_at_origin();
    RadarFovGate gate(RadarFovConfig{}, CrossVetoPolicy{});
    gate.set_pose(pose);
    gate.set_radar_detections(empty_radar(), now_real_ns());

    // Voxel at 15 m behind the drone (outside ±60° azimuth FOV).
    auto v     = voxel_at(-15.0f, 0.0f, 1.0f);  // z=1.0 > kMinObstacleZ
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &pose, &gate);

    EXPECT_GT(stats.cross_veto_deferred, 0u);
    EXPECT_EQ(stats.single_modality_promoted, 0u);
    EXPECT_EQ(grid.static_count(), 0u)
        << "Long-range voxel outside FOV with no residency must not promote.";
}

// ── Legacy null-pose / null-gate path (preserves backward compat) ──

TEST(OccupancyGridCrossVeto, NullPose_PreservesLegacyBehaviour) {
    auto grid = make_grid();

    // Same long-range-in-FOV voxel that would be vetoed with the gate.
    auto v     = voxel_at(15.0f, 0.0f, 1.0f);  // z=1.0 > kMinObstacleZ=0.3
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, nullptr, nullptr);

    EXPECT_EQ(stats.cross_veto_deferred, 0u);
    EXPECT_EQ(stats.single_modality_promoted, 0u);
    EXPECT_GT(grid.static_count(), 0u)
        << "When pose+gate are null, insert_voxels must promote unconditionally — "
           "this is the path used by all pre-#698 unit tests.";
}

TEST(OccupancyGridCrossVeto, NullGateOnly_PreservesLegacyBehaviour) {
    auto grid = make_grid();
    auto pose = pose_at_origin();

    auto v     = voxel_at(15.0f, 0.0f, 1.0f);  // z=1.0 > kMinObstacleZ=0.3
    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f, &pose, /*radar_gate=*/nullptr);

    EXPECT_EQ(stats.cross_veto_deferred, 0u);
    EXPECT_GT(grid.static_count(), 0u)
        << "When only the gate is null, the veto must still be skipped.";
}

// ── Single-modality tracking surfaces correctly ──────────────

TEST(OccupancyGridCrossVeto, SingleModalityCounter_StartsAtZero) {
    auto grid = make_grid();
    EXPECT_EQ(grid.single_modality_static_count(), 0u);
}
