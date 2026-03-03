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
}
