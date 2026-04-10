// tests/test_mission_fsm.cpp
// Unit tests for MissionFSM state machine and waypoint logic.
#include "planner/mission_fsm.h"

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
}

TEST(MissionFSMTest, WaypointNotReachedFarFromBothOriginalAndSnapped) {
    MissionFSM fsm;

    Waypoint                   wp{10.0f, 0.0f, 5.0f, 0.0f, 3.0f, 2.0f, false};
    const std::array<float, 3> snap_xyz = {16.0f, -2.0f, 5.0f};

    // Drone at (30, 30, 5) — far from both original and snapped positions
    EXPECT_FALSE(fsm.waypoint_reached(30.0f, 30.0f, 5.0f, wp, &snap_xyz));
}

TEST(MissionFSMTest, WaypointReachedNoSnapNormalBehavior) {
    MissionFSM fsm;

    // Normal case — no snap, nullptr override
    Waypoint wp{10.0f, 20.0f, 5.0f, 0.0f, 2.0f, 2.0f, false};

    // Within radius of original
    EXPECT_TRUE(fsm.waypoint_reached(10.5f, 20.5f, 5.0f, wp, nullptr));

    // Outside radius of original
    EXPECT_FALSE(fsm.waypoint_reached(15.0f, 25.0f, 5.0f, wp, nullptr));
}

TEST(MissionFSMTest, WaypointReachedWithinRadiusOfSnap) {
    MissionFSM fsm;

    Waypoint                   wp{10.0f, 0.0f, 5.0f, 0.0f, 3.0f, 2.0f, false};
    const std::array<float, 3> snap_xyz = {16.0f, 0.0f, 5.0f};

    // Drone 2.5m from snapped position — within 3.0m radius
    EXPECT_TRUE(fsm.waypoint_reached(18.0f, 1.0f, 5.0f, wp, &snap_xyz));

    // Drone 4.0m from snapped position — outside 3.0m radius
    EXPECT_FALSE(fsm.waypoint_reached(20.0f, 0.0f, 5.0f, wp, &snap_xyz));
}
