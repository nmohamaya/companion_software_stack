// tests/test_radar_fov_gate.cpp
//
// Unit tests for the RadarFovGate cross-veto helper (issue #698 Fix #1,
// Phase 1 of tasks/plan-scenario-33-pass.md).
//
// Coverage:
//   - FOV in/out at boresight, FOV edges, beyond max range
//   - Pose rotation: yaw the drone 90° and verify in/out flips
//   - Radar association: detection within / outside the gate radius
//   - Staleness gate: scans older than max_staleness_ns flagged stale
//   - Residency tracker: counter accrues while in-FOV-with-no-return,
//     resets on leaving FOV, untouched when radar return is present.

#include "planner/radar_fov_gate.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

using drone::ipc::Pose;
using drone::ipc::RadarDetection;
using drone::ipc::RadarDetectionList;
using drone::planner::CrossVetoPolicy;
using drone::planner::GridCell;
using drone::planner::RadarFovConfig;
using drone::planner::RadarFovGate;

namespace {

// Identity-orientation pose at the world origin.
Pose make_pose_origin() {
    Pose p{};
    p.translation[0] = 0.0;
    p.translation[1] = 0.0;
    p.translation[2] = 0.0;
    p.quaternion[0]  = 1.0;  // w
    p.quaternion[1]  = 0.0;
    p.quaternion[2]  = 0.0;
    p.quaternion[3]  = 0.0;
    return p;
}

// Yaw-only pose at the world origin.  yaw = rotation about +Z (up).
Pose make_pose_yawed(float yaw_rad) {
    Pose        p    = make_pose_origin();
    const float half = 0.5f * yaw_rad;
    p.quaternion[0]  = std::cos(half);  // w
    p.quaternion[1]  = 0.0;
    p.quaternion[2]  = 0.0;
    p.quaternion[3]  = std::sin(half);  // z
    return p;
}

RadarDetectionList make_radar_with(float range_m, float az_rad, float el_rad) {
    RadarDetectionList list{};
    list.timestamp_ns                = 1;
    list.num_detections              = 1;
    list.detections[0]               = RadarDetection{};
    list.detections[0].range_m       = range_m;
    list.detections[0].azimuth_rad   = az_rad;
    list.detections[0].elevation_rad = el_rad;
    list.detections[0].confidence    = 1.0f;
    return list;
}

RadarFovConfig default_fov() {
    return RadarFovConfig{};  // ±60° az, ±40° el, 0.5–100 m
}

CrossVetoPolicy default_policy() {
    return CrossVetoPolicy{};  // 10 m short-range, 100 ms staleness, 2 s residency, 30 s cap
}

constexpr uint64_t kMs(uint64_t ms) {
    return ms * 1'000'000ULL;
}
constexpr uint64_t kS(uint64_t s) {
    return s * 1'000'000'000ULL;
}

}  // namespace

// ── FOV geometry ─────────────────────────────────────────────

TEST(RadarFovGate, NoPoseYet_QueryReturnsConservativeVeto) {
    RadarFovGate g(default_fov(), default_policy());
    auto         q = g.query(15.0f, 0.0f, 0.0f, kMs(0));
    EXPECT_FALSE(q.in_fov);
    EXPECT_TRUE(q.radar_stale);
    EXPECT_FALSE(q.radar_present);
}

TEST(RadarFovGate, PointDirectlyForward_InFov) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // Point 15 m straight ahead in +X (forward, body and world coincide).
    auto q = g.query(15.0f, 0.0f, 0.0f, kMs(0));
    EXPECT_TRUE(q.in_fov);
    EXPECT_NEAR(q.range_to_drone_m, 15.0f, 1e-4f);
}

TEST(RadarFovGate, PointBehindDrone_OutOfFov) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // 15 m behind in -X.
    auto q = g.query(-15.0f, 0.0f, 0.0f, kMs(0));
    EXPECT_FALSE(q.in_fov);
}

TEST(RadarFovGate, PointBeyondMaxRange_OutOfFov) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    auto q = g.query(150.0f, 0.0f, 0.0f, kMs(0));
    EXPECT_FALSE(q.in_fov);
}

TEST(RadarFovGate, PointInsideMinRange_OutOfFov) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    auto q = g.query(0.3f, 0.0f, 0.0f, kMs(0));
    EXPECT_FALSE(q.in_fov);  // inside min_range_m = 0.5
}

TEST(RadarFovGate, FovEdgeAzimuth_Inside) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // 10 m at +50° azimuth (well inside ±60° default).
    const float az = 50.0f * static_cast<float>(M_PI) / 180.0f;
    auto        q  = g.query(10.0f * std::cos(az), 10.0f * std::sin(az), 0.0f, kMs(0));
    EXPECT_TRUE(q.in_fov);
}

TEST(RadarFovGate, FovEdgeAzimuth_Outside) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // 10 m at +75° azimuth (outside ±60° default).
    const float az = 75.0f * static_cast<float>(M_PI) / 180.0f;
    auto        q  = g.query(10.0f * std::cos(az), 10.0f * std::sin(az), 0.0f, kMs(0));
    EXPECT_FALSE(q.in_fov);
}

TEST(RadarFovGate, FovEdgeElevation_Outside) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // 10 m at +50° elevation (outside ±40° default).
    const float el = 50.0f * static_cast<float>(M_PI) / 180.0f;
    auto        q  = g.query(10.0f * std::cos(el), 0.0f, 10.0f * std::sin(el), kMs(0));
    EXPECT_FALSE(q.in_fov);
}

TEST(RadarFovGate, YawDrone90Deg_FlipsForwardSide) {
    RadarFovGate g(default_fov(), default_policy());
    // Yaw the drone +90° about +Z — drone now faces world +Y.
    g.set_pose(make_pose_yawed(static_cast<float>(M_PI) / 2.0f));

    // Point at world +Y at 15 m: should now be "forward" in body frame.
    auto q_forward = g.query(0.0f, 15.0f, 0.0f, kMs(0));
    EXPECT_TRUE(q_forward.in_fov);

    // Point at world +X at 15 m: should now be "right" → outside ±60° body azimuth.
    // World +X with body yawed +90° lands at body azimuth -90° (right side).
    auto q_side = g.query(15.0f, 0.0f, 0.0f, kMs(0));
    EXPECT_FALSE(q_side.in_fov);
}

// ── Radar-association window ─────────────────────────────────

TEST(RadarFovGate, RadarPresent_WithinGateRadius) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // Radar return at 15 m forward, 0 az, 0 el — exactly co-located with the
    // query point at (15, 0, 0).
    g.set_radar_detections(make_radar_with(15.0f, 0.0f, 0.0f), kMs(50));
    auto q = g.query(15.0f, 0.0f, 0.0f, kMs(60));
    EXPECT_TRUE(q.in_fov);
    EXPECT_FALSE(q.radar_stale);
    EXPECT_TRUE(q.radar_present);
}

// Issue #348 / scenario 33 run 2026-05-03_123538 — Cosys-AirSim publishes
// azimuth in FRD (right-positive); the gate must negate to match the
// stack's FLU convention or radar returns and same-position camera cells
// land on opposite sides of the body axis and never match.
TEST(RadarFovGate, AzimuthSignFlip_MatchesUkfConvention) {
    RadarFovConfig fov = default_fov();
    fov.negate_azimuth = true;  // explicit (matches UKF default for Cosys)
    RadarFovGate g(fov, default_policy());
    g.set_pose(make_pose_origin());

    // Cube at world (15, +5, 0) — left of the drone-forward axis (FLU body Y+).
    // Cosys radar reports this as azimuth = -atan2(+5, 15) = -0.32 rad
    // (FRD right-positive → left = negative).  With negate_azimuth, the gate
    // converts to FLU az = +0.32 rad, places the return at (cy = +5),
    // matches the cell at body (15, +5, 0).
    const float frd_az_rad = -std::atan2(5.0f, 15.0f);
    const float range      = std::sqrt(15.0f * 15.0f + 5.0f * 5.0f);
    g.set_radar_detections(make_radar_with(range, frd_az_rad, 0.0f), kMs(50));

    auto q = g.query(15.0f, 5.0f, 0.0f, kMs(60));
    EXPECT_TRUE(q.in_fov);
    EXPECT_TRUE(q.radar_present)
        << "negate_azimuth=true must convert FRD radar return to FLU body frame "
           "so the same-position camera cell matches.  Without this flip the "
           "gate places the return at body Y=-5 and a +Y=+5 cell never matches.";
}

// With negate_azimuth=false, the same FRD-published return DOES end up on
// the wrong side — proves the flip is the load-bearing piece, not some other
// implicit transform.
TEST(RadarFovGate, AzimuthSignFlip_DisabledMisMatchesFrdRadar) {
    RadarFovConfig fov = default_fov();
    fov.negate_azimuth = false;
    RadarFovGate g(fov, default_policy());
    g.set_pose(make_pose_origin());

    const float frd_az_rad = -std::atan2(5.0f, 15.0f);
    const float range      = std::sqrt(15.0f * 15.0f + 5.0f * 5.0f);
    g.set_radar_detections(make_radar_with(range, frd_az_rad, 0.0f), kMs(50));

    auto q = g.query(15.0f, 5.0f, 0.0f, kMs(60));
    EXPECT_TRUE(q.in_fov);
    EXPECT_FALSE(q.radar_present)
        << "Without the sign flip, FRD radar returns and FLU camera cells "
           "land on opposite body-Y sides and the spatial gate never matches.";
}

TEST(RadarFovGate, RadarAbsent_OutsideGateRadius) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // Radar return at 15 m forward but offset 10 m laterally — far outside
    // any reasonable gate radius at R=15 (gate ≈ max(1.0, 0.25*15+0.1) = 3.85 m).
    const float az = std::atan2(10.0f, 15.0f);
    g.set_radar_detections(make_radar_with(std::sqrt(15.0f * 15.0f + 10.0f * 10.0f), az, 0.0f),
                           kMs(50));
    auto q = g.query(15.0f, 0.0f, 0.0f, kMs(60));
    EXPECT_TRUE(q.in_fov);
    EXPECT_FALSE(q.radar_stale);
    EXPECT_FALSE(q.radar_present);
}

TEST(RadarFovGate, GateRadiusGrowsWithRange) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    auto near_q = g.query(2.0f, 0.0f, 0.0f, kMs(0));
    auto far_q  = g.query(40.0f, 0.0f, 0.0f, kMs(0));
    EXPECT_GT(far_q.gate_radius_m, near_q.gate_radius_m)
        << "Gate radius must grow with range to track radar's own cross-range resolution.";
    // At 2 m: gate = max(1.0, 0.25*2 + 0.1) = max(1.0, 0.6) = 1.0
    EXPECT_NEAR(near_q.gate_radius_m, 1.0f, 1e-4f);
    // At 40 m: gate = max(1.0, 0.25*40 + 0.1) = 10.1
    EXPECT_NEAR(far_q.gate_radius_m, 10.1f, 1e-4f);
}

// ── Staleness gate ────────────────────────────────────────────

TEST(RadarFovGate, NoRadarYet_Stale) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // No set_radar_detections() call — last_radar_t_ns_ stays 0.
    auto q = g.query(15.0f, 0.0f, 0.0f, kMs(50));
    EXPECT_TRUE(q.radar_stale);
    EXPECT_FALSE(q.radar_present);
}

TEST(RadarFovGate, RadarFresh_NotStale) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    g.set_radar_detections(make_radar_with(50.0f, 0.0f, 0.0f), kMs(100));
    auto q = g.query(15.0f, 0.0f, 0.0f, kMs(150));  // 50 ms old
    EXPECT_FALSE(q.radar_stale);
}

TEST(RadarFovGate, RadarOlderThanThreshold_Stale) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    g.set_radar_detections(make_radar_with(50.0f, 0.0f, 0.0f), kMs(100));
    auto q = g.query(15.0f, 0.0f, 0.0f, kMs(250));  // 150 ms old, > 100 ms threshold
    EXPECT_TRUE(q.radar_stale);
    EXPECT_FALSE(q.radar_present) << "Stale radar must NOT short-circuit to radar_present even if "
                                     "a co-located return exists.";
}

// ── Residency tracker ─────────────────────────────────────────

// Helper: refresh radar each tick to mimic the real ~20 Hz radar receiver
// thread. Without this, the staleness gate (default 100 ms) trips between
// ticks and residency accrual stops — by design, research note §6.
static void tick_with_fresh_radar(RadarFovGate& g, const std::vector<GridCell>& cells,
                                  float resolution_m, uint64_t now_ns) {
    g.set_radar_detections(RadarDetectionList{}, now_ns);
    g.tick_residency(cells, resolution_m, now_ns);
}

TEST(RadarFovGate, ResidencyAccruesWhileInFovWithNoRadar) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());

    // Cell at (15, 0, 0) world coords; with resolution 1 m → GridCell{15,0,0}.
    const std::vector<GridCell> cells{{15, 0, 0}};

    tick_with_fresh_radar(g, cells, 1.0f, kMs(0));  // baseline; dt=0 first tick
    EXPECT_EQ(g.residency_ns(GridCell{15, 0, 0}), 0u);

    tick_with_fresh_radar(g, cells, 1.0f, kMs(50));  // +50 ms (radar still fresh)
    EXPECT_EQ(g.residency_ns(GridCell{15, 0, 0}), kMs(50));

    tick_with_fresh_radar(g, cells, 1.0f, kMs(150));  // +100 ms (radar refreshed each tick)
    EXPECT_EQ(g.residency_ns(GridCell{15, 0, 0}), kMs(150));
}

TEST(RadarFovGate, ResidencyResetsWhenCellLeavesFov) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());

    // Cell directly forward — in FOV.
    const std::vector<GridCell> cells_in{{15, 0, 0}};
    tick_with_fresh_radar(g, cells_in, 1.0f, kMs(0));
    tick_with_fresh_radar(g, cells_in, 1.0f, kMs(50));
    EXPECT_EQ(g.residency_ns(GridCell{15, 0, 0}), kMs(50));

    // Now yaw drone 180° — same world cell is now behind, out of FOV.
    g.set_pose(make_pose_yawed(static_cast<float>(M_PI)));
    tick_with_fresh_radar(g, cells_in, 1.0f, kMs(100));
    EXPECT_EQ(g.residency_ns(GridCell{15, 0, 0}), 0u)
        << "Residency must reset to 0 when the cell leaves the FOV.";
}

TEST(RadarFovGate, ResidencyUntouchedWhenRadarPresent) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());
    // Radar return at the cell.
    g.set_radar_detections(make_radar_with(15.0f, 0.0f, 0.0f), kMs(0));

    const std::vector<GridCell> cells{{15, 0, 0}};
    g.tick_residency(cells, 1.0f, kMs(0));
    g.tick_residency(cells, 1.0f, kMs(500));
    EXPECT_EQ(g.residency_ns(GridCell{15, 0, 0}), 0u)
        << "Cell with a co-located radar return must not accrue residency — it will promote "
           "via the regular Promote path on its next insert_voxels() round.";
}

TEST(RadarFovGate, ClearResidencyDropsEntry) {
    RadarFovGate g(default_fov(), default_policy());
    g.set_pose(make_pose_origin());

    const std::vector<GridCell> cells{{15, 0, 0}};
    tick_with_fresh_radar(g, cells, 1.0f, kMs(0));
    tick_with_fresh_radar(g, cells, 1.0f, kMs(50));
    EXPECT_GT(g.residency_ns(GridCell{15, 0, 0}), 0u);

    g.clear_residency(GridCell{15, 0, 0});
    EXPECT_EQ(g.residency_ns(GridCell{15, 0, 0}), 0u);
}

TEST(RadarFovGate, NoTickWithoutPose) {
    RadarFovGate                g(default_fov(), default_policy());
    const std::vector<GridCell> cells{{15, 0, 0}};
    // No set_pose() call.
    g.tick_residency(cells, 1.0f, kMs(0));
    g.tick_residency(cells, 1.0f, kMs(500));
    EXPECT_EQ(g.residency_ns(GridCell{15, 0, 0}), 0u)
        << "Without a pose the FOV check is unreliable; residency must not accrue.";
}
