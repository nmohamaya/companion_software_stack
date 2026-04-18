// tests/test_mission_fsm.cpp
// Unit tests for MissionFSM state machine and waypoint logic.
#include "planner/mission_fsm.h"

#include <cmath>
#include <limits>
#include <optional>

#include <gtest/gtest.h>

using namespace drone::planner;

TEST(MissionFSMTest, StartsInIdle) {
    MissionFSM fsm;
    EXPECT_EQ(fsm.state(), MissionState::IDLE);
}

TEST(MissionFSMTest, NormalMissionLifecycle) {
    MissionFSM fsm;

    fsm.on_arm();
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    fsm.on_takeoff();
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);

    fsm.on_navigate();
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);

    fsm.on_loiter();
    EXPECT_EQ(fsm.state(), MissionState::LOITER);

    fsm.on_navigate();
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);

    fsm.on_rtl();
    EXPECT_EQ(fsm.state(), MissionState::RTL);

    fsm.on_land();
    EXPECT_EQ(fsm.state(), MissionState::LAND);

    fsm.on_landed();
    EXPECT_EQ(fsm.state(), MissionState::IDLE);
}

TEST(MissionFSMTest, EmergencyFromAnyState) {
    MissionFSM fsm;

    fsm.on_arm();
    fsm.on_takeoff();
    fsm.on_navigate();
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);

    fsm.on_emergency();
    EXPECT_EQ(fsm.state(), MissionState::EMERGENCY);
}

TEST(MissionFSMTest, WaypointReached) {
    MissionFSM fsm;

    Waypoint wp{10.0f, 20.0f, 5.0f, 0.0f, 1.0f, 2.0f, false};

    // Exactly at waypoint
    EXPECT_TRUE(fsm.waypoint_reached(10.0f, 20.0f, 5.0f, wp));

    // Within acceptance radius
    EXPECT_TRUE(fsm.waypoint_reached(10.5f, 20.5f, 5.0f, wp));

    // Outside acceptance radius
    EXPECT_FALSE(fsm.waypoint_reached(15.0f, 25.0f, 5.0f, wp));
}

TEST(MissionFSMTest, WaypointLoadAndAdvance) {
    MissionFSM fsm;

    std::vector<Waypoint> wps = {
        {0, 0, 5, 0, 1.0f, 2.0f, false},
        {10, 0, 5, 0, 1.0f, 2.0f, true},
        {10, 10, 5, 0, 1.0f, 2.0f, false},
    };
    fsm.load_mission(wps);

    EXPECT_EQ(fsm.total_waypoints(), 3u);
    EXPECT_EQ(fsm.current_wp_index(), 0u);

    const auto* wp0 = fsm.current_waypoint();
    ASSERT_NE(wp0, nullptr);
    EXPECT_FLOAT_EQ(wp0->x, 0.0f);

    EXPECT_TRUE(fsm.advance_waypoint());
    EXPECT_EQ(fsm.current_wp_index(), 1u);

    const auto* wp1 = fsm.current_waypoint();
    ASSERT_NE(wp1, nullptr);
    EXPECT_TRUE(wp1->trigger_payload);

    EXPECT_TRUE(fsm.advance_waypoint());
    EXPECT_EQ(fsm.current_wp_index(), 2u);

    // Can't advance past last
    EXPECT_FALSE(fsm.advance_waypoint());
}

TEST(MissionFSMTest, EmptyMissionNoWaypoint) {
    MissionFSM fsm;
    EXPECT_EQ(fsm.current_waypoint(), nullptr);
    EXPECT_EQ(fsm.total_waypoints(), 0u);
}

TEST(MissionFSMTest, StateNames) {
    EXPECT_STREQ(state_name(MissionState::IDLE), "IDLE");
    EXPECT_STREQ(state_name(MissionState::TAKEOFF), "TAKEOFF");
    EXPECT_STREQ(state_name(MissionState::NAVIGATE), "NAVIGATE");
    EXPECT_STREQ(state_name(MissionState::EMERGENCY), "EMERGENCY");
    EXPECT_STREQ(state_name(MissionState::RTL), "RTL");
    EXPECT_STREQ(state_name(MissionState::LAND), "LAND");
    EXPECT_STREQ(state_name(MissionState::COLLISION_RECOVERY), "COLLISION_RECOVERY");
}

// ═══════════════════════════════════════════════════════════
// Waypoint overshoot detection (Issue #236)
// ═══════════════════════════════════════════════════════════

TEST(MissionFSMTest, NoOvershootOnFirstWaypoint) {
    MissionFSM fsm;
    fsm.load_mission({{10, 0, 5, 0, 2.0f, 3.0f, false}, {20, 0, 5, 0, 2.0f, 3.0f, false}});

    // WP0 has no previous waypoint — overshoot never triggers for first WP
    EXPECT_FALSE(fsm.waypoint_overshot(15.0f, 0.0f, 5.0f));
}

TEST(MissionFSMTest, OvershootDetectedWhenPastWaypoint) {
    // 3 waypoints: (0,0,5) → (10,0,5) → (20,0,5) — all collinear along X
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();  // Now at WP1 (10,0,5)

    // Drone at (15,0,5) — past WP1 along approach direction (WP0→WP1 = +X)
    EXPECT_TRUE(fsm.waypoint_overshot(15.0f, 0.0f, 5.0f));
}

TEST(MissionFSMTest, NoOvershootWhenBeforeWaypoint) {
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();  // Now at WP1 (10,0,5)

    // Drone at (5,0,5) — behind WP1 along approach direction (WP0→WP1 = +X)
    EXPECT_FALSE(fsm.waypoint_overshot(5.0f, 0.0f, 5.0f));
}

TEST(MissionFSMTest, NoOvershootOnLastWaypoint) {
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();  // WP1
    (void)fsm.advance_waypoint();  // WP2 (last)
    EXPECT_EQ(fsm.current_wp_index(), 2u);

    // Drone past last WP — must NOT report overshoot (require acceptance radius)
    EXPECT_FALSE(fsm.waypoint_overshot(25.0f, 0.0f, 5.0f));
}

TEST(MissionFSMTest, OvershootWithLateralOffset) {
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();  // Now at WP1 (10,0,5)

    // Drone at (15,3,5) — past WP1 along approach vector but laterally offset
    EXPECT_TRUE(fsm.waypoint_overshot(15.0f, 3.0f, 5.0f));
}

TEST(MissionFSMTest, NoOvershootWhenFarFromWaypoint) {
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();  // Now at WP1 (10,0,5)

    // Drone at (20,0,5) — "past" WP1 in dot-product sense but 10m away
    // (proximity zone = 3 × 2.0 = 6m), so overshoot must NOT trigger
    EXPECT_FALSE(fsm.waypoint_overshot(20.0f, 0.0f, 5.0f));
}

TEST(MissionFSMTest, ConfigurableProximityFactorAllowsFarOvershoot) {
    // With factor=7.0, proximity zone = 7 × 2.0 = 14m — 10m away should trigger
    MissionFSM fsm(7.0f);
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();  // Now at WP1 (10,0,5)

    // Same position as NoOvershootWhenFarFromWaypoint (10m from WP1),
    // but larger proximity factor now allows detection
    EXPECT_TRUE(fsm.waypoint_overshot(20.0f, 0.0f, 5.0f));
}

TEST(MissionFSMTest, NextWaypointAccessor) {
    MissionFSM fsm;
    fsm.load_mission({{10, 0, 5, 0, 2.0f, 3.0f, false}, {20, 0, 5, 0, 2.0f, 3.0f, false}});

    const auto* nwp = fsm.next_waypoint();
    ASSERT_NE(nwp, nullptr);
    EXPECT_FLOAT_EQ(nwp->x, 20.0f);

    // Advance to last — no next waypoint
    EXPECT_TRUE(fsm.advance_waypoint());
    EXPECT_EQ(fsm.next_waypoint(), nullptr);
}

// ═══════════════════════════════════════════════════════════
// Snap offset acceptance — Issue #394
// When the planner snaps a waypoint to avoid occupied cells,
// waypoint_reached() must check against the snapped position,
// not the original (possibly unreachable) waypoint.
// ═══════════════════════════════════════════════════════════

TEST(MissionFSMTest, WaypointReachedAtSnappedPosition) {
    MissionFSM fsm;

    // Original WP at (10, 0, 5) with acceptance_radius = 3.0m
    Waypoint wp{10.0f, 0.0f, 5.0f, 0.0f, 3.0f, 2.0f, false};

    // Snapped position at (16, -2, 5) — 6.32m from original.
    // Without snap override, drone at snapped position would NOT satisfy radius.
    const std::array<float, 3> snap_xyz = {16.0f, -2.0f, 5.0f};

    // Drone at the snapped position — should be reached when snap is provided
    EXPECT_TRUE(fsm.waypoint_reached(16.0f, -2.0f, 5.0f, wp, &snap_xyz));

    // Same position without snap override — should NOT be reached (6.32m > 3.0m)
    EXPECT_FALSE(fsm.waypoint_reached(16.0f, -2.0f, 5.0f, wp, nullptr));

    // Also verify the original waypoint check still works without snap
    EXPECT_TRUE(fsm.waypoint_reached(10.0f, 0.0f, 5.0f, wp, nullptr));

    // Drone near ORIGINAL position with snap provided — OR logic means
    // the waypoint is still reached even though the drone isn't near the snap.
    // This is the unreachable-snap regression case: D* can't pathfind to snap,
    // but the drone is close to the original WP position.
    EXPECT_TRUE(fsm.waypoint_reached(10.0f, 0.0f, 5.0f, wp, &snap_xyz));
}

TEST(MissionFSMTest, WaypointNotReachedFarFromBothOriginalAndSnapped) {
    MissionFSM fsm;

    Waypoint                   wp{10.0f, 0.0f, 5.0f, 0.0f, 3.0f, 2.0f, false};
    const std::array<float, 3> snap_xyz = {16.0f, -2.0f, 5.0f};

    // Drone at (30, 30, 5) — far from both original and snapped positions
    EXPECT_FALSE(fsm.waypoint_reached(30.0f, 30.0f, 5.0f, wp, &snap_xyz));
}

TEST(MissionFSMTest, WaypointReachedExplicitNullSnapMatchesDefault) {
    MissionFSM fsm;

    // Contract: explicit nullptr snap must behave identically to omitting the parameter
    Waypoint wp{10.0f, 20.0f, 5.0f, 0.0f, 1.0f, 2.0f, false};

    EXPECT_EQ(fsm.waypoint_reached(10.0f, 20.0f, 5.0f, wp, nullptr),
              fsm.waypoint_reached(10.0f, 20.0f, 5.0f, wp));

    EXPECT_EQ(fsm.waypoint_reached(15.0f, 25.0f, 5.0f, wp, nullptr),
              fsm.waypoint_reached(15.0f, 25.0f, 5.0f, wp));
}

TEST(MissionFSMTest, WaypointReachedWithinRadiusOfSnap) {
    MissionFSM fsm;

    Waypoint                   wp{10.0f, 0.0f, 5.0f, 0.0f, 3.0f, 2.0f, false};
    const std::array<float, 3> snap_xyz = {16.0f, 0.0f, 5.0f};

    // Drone ~2.24m from snapped position (√(2²+1²)) — within 3.0m radius
    EXPECT_TRUE(fsm.waypoint_reached(18.0f, 1.0f, 5.0f, wp, &snap_xyz));

    // Drone exactly at radius boundary (3.0m) — strict < means NOT reached
    EXPECT_FALSE(fsm.waypoint_reached(19.0f, 0.0f, 5.0f, wp, &snap_xyz));

    // Drone 4.0m from snapped position — outside 3.0m radius
    EXPECT_FALSE(fsm.waypoint_reached(20.0f, 0.0f, 5.0f, wp, &snap_xyz));
}

// ═══════════════════════════════════════════════════════════
// Waypoint overshot with snap support — Issue #398
// When the planner snaps a waypoint, waypoint_overshot() must
// also check against the snapped position (OR-logic).
// ═══════════════════════════════════════════════════════════

TEST(MissionFSMTest, OvershotAtSnappedPosition) {
    // 3 waypoints collinear along X: (0,0,5) → (10,0,5) → (20,0,5)
    // Snap WP1 to (10,3,5) — 3m lateral offset in Y.
    // Drone at (11,3,5) is past the snap along the approach direction (+X)
    // and within proximity zone of the original WP.
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();  // Now at WP1 (10,0,5)

    std::optional<std::array<float, 3>> snap = std::array<float, 3>{10.0f, 3.0f, 5.0f};

    // Drone at (11,3,5) — 1m past snap along +X, within proximity of WP1
    EXPECT_TRUE(fsm.waypoint_overshot(11.0f, 3.0f, 5.0f, snap));
}

TEST(MissionFSMTest, OvershotDisabledWhenFarFromSnap) {
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();

    // Snap at (10,3,5). Drone at (25,3,5) — 15m from snap, 15m from original.
    // proximity_r = 2.0 * 3.0 = 6.0m — both positions are outside proximity zone.
    std::optional<std::array<float, 3>> snap = std::array<float, 3>{10.0f, 3.0f, 5.0f};
    EXPECT_FALSE(fsm.waypoint_overshot(25.0f, 3.0f, 5.0f, snap));
}

TEST(MissionFSMTest, OvershotFallbackToOriginalWhenNoSnap) {
    // Without snap (nullopt), overshoot behaves exactly like the original code.
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();

    // Drone past WP1 along approach direction, within proximity
    EXPECT_TRUE(fsm.waypoint_overshot(15.0f, 0.0f, 5.0f, std::nullopt));

    // Drone before WP1
    EXPECT_FALSE(fsm.waypoint_overshot(5.0f, 0.0f, 5.0f, std::nullopt));
}

TEST(MissionFSMTest, OvershotWithZAxisSnapDisplacement) {
    // Snap differs in Z: original WP at z=5, snap at z=8.
    // Drone at (11,0,8) — near snap, past in +X direction.
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();

    std::optional<std::array<float, 3>> snap = std::array<float, 3>{10.0f, 0.0f, 8.0f};

    // Drone at (11,0,8) — 1m past snap in X, at same Z as snap
    EXPECT_TRUE(fsm.waypoint_overshot(11.0f, 0.0f, 8.0f, snap));
}

// ═══════════════════════════════════════════════════════════
// NaN/Inf robustness — Issue #398 item 4
// Non-finite snap coordinates must be safely rejected.
// ═══════════════════════════════════════════════════════════

TEST(MissionFSMTest, NaNSnappedCoordinatesReachedFallsBackToOriginal) {
    MissionFSM fsm;
    Waypoint   wp{10.0f, 0.0f, 5.0f, 0.0f, 3.0f, 2.0f, false};

    // NaN snapped position — should be ignored, fall back to original check
    const std::array<float, 3> nan_snap = {std::numeric_limits<float>::quiet_NaN(), 0.0f, 5.0f};

    // Drone at original WP — reached via original check despite NaN snap
    EXPECT_TRUE(fsm.waypoint_reached(10.0f, 0.0f, 5.0f, wp, &nan_snap));

    // Drone far from original — NOT reached (NaN snap is ignored)
    EXPECT_FALSE(fsm.waypoint_reached(50.0f, 0.0f, 5.0f, wp, &nan_snap));
}

TEST(MissionFSMTest, InfSnappedCoordinatesReachedFallsBackToOriginal) {
    MissionFSM fsm;
    Waypoint   wp{10.0f, 0.0f, 5.0f, 0.0f, 3.0f, 2.0f, false};

    // Inf snapped position — should be ignored
    const std::array<float, 3> inf_snap = {std::numeric_limits<float>::infinity(), 0.0f, 5.0f};
    EXPECT_TRUE(fsm.waypoint_reached(10.0f, 0.0f, 5.0f, wp, &inf_snap));
    EXPECT_FALSE(fsm.waypoint_reached(50.0f, 0.0f, 5.0f, wp, &inf_snap));
}

TEST(MissionFSMTest, NaNSnappedCoordinatesOvershotFallsBackToOriginal) {
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();

    std::optional<std::array<float, 3>> nan_snap =
        std::array<float, 3>{std::numeric_limits<float>::quiet_NaN(), 0.0f, 5.0f};

    // NaN snap is ignored — overshoot check uses only original WP
    EXPECT_TRUE(fsm.waypoint_overshot(15.0f, 0.0f, 5.0f, nan_snap));
    EXPECT_FALSE(fsm.waypoint_overshot(5.0f, 0.0f, 5.0f, nan_snap));
}

TEST(MissionFSMTest, StaleZeroSnapDoesNotTriggerFalseOvershoot) {
    // A stale snap at (0,0,0) should not trigger overshoot if the drone
    // happens to be near the origin and "past" it in the dot-product sense.
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();  // WP1 at (10,0,5)

    // Stale snap at (0,0,0). Drone at (5,0,5) — before WP1 in approach direction.
    // The drone IS past the stale snap (0,0,0) in the +X approach direction,
    // but 5m > proximity_r (6.0m from original, 7.07m from snap) — so the snap
    // check should NOT trigger because dist from snap is outside proximity zone.
    std::optional<std::array<float, 3>> stale_snap = std::array<float, 3>{0.0f, 0.0f, 0.0f};

    // Drone at (5,0,5) — 5m from original WP, within proximity (6m) BUT behind
    // WP in approach direction, AND snap at origin is 7.07m away (>6m proximity).
    EXPECT_FALSE(fsm.waypoint_overshot(5.0f, 0.0f, 5.0f, stale_snap));
}

TEST(MissionFSMTest, ExactBoundaryAtAcceptanceRadiusOvershotStrictLessThan) {
    // The proximity check uses <= (not <), so exactly at proximity boundary
    // DOES qualify for the proximity test. But the dot-product determines direction.
    MissionFSM fsm;
    fsm.load_mission({{0, 0, 5, 0, 2.0f, 3.0f, false},
                      {10, 0, 5, 0, 2.0f, 3.0f, false},
                      {20, 0, 5, 0, 2.0f, 3.0f, false}});
    (void)fsm.advance_waypoint();

    // proximity_r = 2.0 * 3.0 = 6.0m
    // Drone at exactly 6.0m past WP1 along +X: (16,0,5)
    EXPECT_TRUE(fsm.waypoint_overshot(16.0f, 0.0f, 5.0f));

    // Drone at 6.001m — just outside proximity zone
    EXPECT_FALSE(fsm.waypoint_overshot(16.01f, 0.0f, 5.0f));
}

// ═════════════════════════════════════════════════════════════
// StuckDetector tests — Issue #503
// ═════════════════════════════════════════════════════════════

TEST(StuckDetectorTest, TriggersWhenPoseStationaryAndAvoiderActive) {
    StuckDetector::Config cfg;
    cfg.enabled        = true;
    cfg.window_s       = 1.0f;
    cfg.min_movement_m = 0.5f;
    StuckDetector det{cfg};

    const auto t0 = StuckDetector::Clock::now();
    // Feed 12 samples over >window_s, all at the same pose, avoider active.
    for (int i = 0; i <= 12; ++i) {
        auto t = t0 + std::chrono::milliseconds(100 * i);
        det.push_sample(t, 5.0f, 5.0f, 5.0f, true);
    }
    const auto now = t0 + std::chrono::milliseconds(1200);
    EXPECT_TRUE(det.is_stuck(now));
}

TEST(StuckDetectorTest, DoesNotTriggerWhenAvoiderInactive) {
    StuckDetector::Config cfg;
    cfg.enabled        = true;
    cfg.window_s       = 1.0f;
    cfg.min_movement_m = 0.5f;
    StuckDetector det{cfg};

    const auto t0 = StuckDetector::Clock::now();
    for (int i = 0; i <= 12; ++i) {
        auto t = t0 + std::chrono::milliseconds(100 * i);
        det.push_sample(t, 5.0f, 5.0f, 5.0f, /*avoider_active=*/false);
    }
    const auto now = t0 + std::chrono::milliseconds(1200);
    EXPECT_FALSE(det.is_stuck(now));
}

TEST(StuckDetectorTest, ResetsOnMovement) {
    StuckDetector::Config cfg;
    cfg.enabled        = true;
    cfg.window_s       = 1.0f;
    cfg.min_movement_m = 0.5f;
    StuckDetector det{cfg};

    const auto t0 = StuckDetector::Clock::now();
    // Stationary samples long enough to trigger.
    for (int i = 0; i <= 12; ++i) {
        auto t = t0 + std::chrono::milliseconds(100 * i);
        det.push_sample(t, 0.0f, 0.0f, 5.0f, true);
    }
    EXPECT_TRUE(det.is_stuck(t0 + std::chrono::milliseconds(1200)));

    // Drone moves 0.1 m/tick (×12 ticks = 1.2 m) — well beyond min_movement_m.
    // Samples span a full new window at the new location.
    for (int i = 13; i <= 24; ++i) {
        auto        t = t0 + std::chrono::milliseconds(100 * i);
        const float x = 0.1f * static_cast<float>(i - 12);
        det.push_sample(t, x, 0.0f, 5.0f, true);
    }
    EXPECT_FALSE(det.is_stuck(t0 + std::chrono::milliseconds(2400)));
}

TEST(StuckDetectorTest, DisabledNeverTriggers) {
    StuckDetector::Config cfg;
    cfg.enabled        = false;
    cfg.window_s       = 1.0f;
    cfg.min_movement_m = 0.5f;
    StuckDetector det{cfg};

    const auto t0 = StuckDetector::Clock::now();
    for (int i = 0; i <= 12; ++i) {
        auto t = t0 + std::chrono::milliseconds(100 * i);
        det.push_sample(t, 5.0f, 5.0f, 5.0f, true);
    }
    EXPECT_FALSE(det.is_stuck(t0 + std::chrono::milliseconds(1200)));
}

TEST(StuckDetectorTest, WindowNotYetFull) {
    StuckDetector::Config cfg;
    cfg.enabled        = true;
    cfg.window_s       = 3.0f;
    cfg.min_movement_m = 0.5f;
    StuckDetector det{cfg};

    const auto t0 = StuckDetector::Clock::now();
    // Only 1s of samples — below window_s=3s.
    for (int i = 0; i <= 10; ++i) {
        auto t = t0 + std::chrono::milliseconds(100 * i);
        det.push_sample(t, 0.0f, 0.0f, 5.0f, true);
    }
    EXPECT_FALSE(det.is_stuck(t0 + std::chrono::milliseconds(1000)));
}

TEST(MissionFSMTest, StuckEventTransitionsToNavigateUnstuck) {
    MissionFSM fsm;
    fsm.on_arm();
    fsm.on_takeoff();
    fsm.on_navigate();
    ASSERT_EQ(fsm.state(), MissionState::NAVIGATE);

    fsm.on_stuck();
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE_UNSTUCK);

    fsm.on_unstuck();
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}
