// tests/test_mission_state_tick.cpp
// Unit tests for MissionStateTick (Issue #154).
#include "planner/mission_state_tick.h"
#include "planner/obstacle_avoider_3d.h"
#include "planner/planner_factory.h"

#include <vector>

#include <gtest/gtest.h>

using namespace drone::planner;
using namespace drone::ipc;

namespace {

template<typename T>
class MockPublisher : public IPublisher<T> {
public:
    void                             publish(const T& msg) override { messages_.push_back(msg); }
    [[nodiscard]] bool               is_ready() const override { return true; }
    [[nodiscard]] const std::string& topic_name() const override { return topic_; }
    const std::vector<T>&            messages() const { return messages_; }
    void                             clear() { messages_.clear(); }

private:
    std::vector<T> messages_;
    std::string    topic_ = "mock";
};

struct FCCallRecord {
    FCCommandType cmd;
    float         param;
};

Pose make_pose(float x, float y, float z) {
    Pose p{};
    p.translation[0] = x;
    p.translation[1] = y;
    p.translation[2] = z;
    p.quality        = 2;
    p.timestamp_ns   = 1000;
    return p;
}

FCState make_fc(bool armed, float rel_alt) {
    FCState fc{};
    fc.armed             = armed;
    fc.connected         = true;
    fc.rel_alt           = rel_alt;
    fc.battery_remaining = 80.0f;
    fc.timestamp_ns      = 1000;
    return fc;
}

}  // namespace

class MissionStateTickTest : public ::testing::Test {
protected:
    StateTickConfig               config{10.0f, 1.5f, 0.5f, 5};
    MissionStateTick              state_tick{config};
    MockPublisher<TrajectoryCmd>  traj_pub;
    MockPublisher<PayloadCommand> payload_pub;
    MissionFSM                    fsm;
    StaticObstacleLayer           obstacle_layer;
    std::vector<FCCallRecord>     fc_calls;

    FCSendFn send_fc = [this](FCCommandType cmd, float p) {
        fc_calls.push_back({cmd, p});
    };

    void SetUp() override {
        fsm.load_mission({{10, 0, 5, 0, 2, 3, true}, {20, 0, 5, 0, 2, 3, false}});
        fsm.on_arm();  // → PREFLIGHT
    }

    std::unique_ptr<IPathPlanner>     planner_ = create_path_planner("dstar_lite");
    std::unique_ptr<IObstacleAvoider> avoider_ = create_obstacle_avoider("potential_field_3d", 5.0f,
                                                                         2.0f);

    void do_tick(const Pose& pose, const FCState& fc_state) {
        DetectedObjectList            objects{};
        drone::util::FrameDiagnostics diag(0);

        state_tick.tick(fsm, pose, fc_state, objects, *planner_, nullptr, *avoider_, obstacle_layer,
                        traj_pub, payload_pub, send_fc, 0, diag);
    }
};

// ═══════════════════════════════════════════════════════════
// PREFLIGHT: ARM retry
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, PreflightSendsARM) {
    auto pose = make_pose(0, 0, 0);
    auto fc   = make_fc(false, 0);

    do_tick(pose, fc);

    // Should have sent ARM command
    ASSERT_GE(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);
}

TEST_F(MissionStateTickTest, PreflightTransitionsOnArmed) {
    auto pose = make_pose(0, 0, 0);
    auto fc   = make_fc(true, 0);  // armed!

    do_tick(pose, fc);

    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);
}

// ═══════════════════════════════════════════════════════════
// TAKEOFF: transition to NAVIGATE at 90% altitude
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, TakeoffTransitionsAtTargetAlt) {
    // First get to TAKEOFF
    fsm.on_takeoff();
    ASSERT_EQ(fsm.state(), MissionState::TAKEOFF);

    auto pose = make_pose(0, 0, 0);
    auto fc   = make_fc(true, 9.5f);  // >= 10.0 * 0.9 = 9.0

    do_tick(pose, fc);

    // Should have sent TAKEOFF command and transitioned
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

TEST_F(MissionStateTickTest, TakeoffStaysIfBelowAlt) {
    fsm.on_takeoff();
    auto pose = make_pose(0, 0, 0);
    auto fc   = make_fc(true, 5.0f);  // below 9.0

    do_tick(pose, fc);

    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);
}

// ═══════════════════════════════════════════════════════════
// HOME RECORDING: race condition guards
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, HomeNotRecordedFromDefaultPose) {
    fsm.on_takeoff();
    // Default-constructed pose has timestamp_ns=0 — must NOT record home
    Pose default_pose{};
    auto fc = make_fc(true, 9.5f);  // altitude reached

    do_tick(default_pose, fc);

    // Should stay in TAKEOFF because home was not recorded
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);
}

TEST_F(MissionStateTickTest, TakeoffDefersNavigateUntilHomeRecorded) {
    fsm.on_takeoff();

    // Tick 1: altitude reached but no real pose yet
    Pose default_pose{};
    auto fc = make_fc(true, 9.5f);
    do_tick(default_pose, fc);
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);

    // Tick 2: real pose arrives → home recorded → transition
    auto real_pose = make_pose(5.0f, 10.0f, 0.0f);
    do_tick(real_pose, fc);
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

TEST_F(MissionStateTickTest, HomeRecordedDuringPreflight) {
    // Home should be recorded from any state once a real pose arrives
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    auto pose = make_pose(7.0f, 3.0f, 0.0f);
    auto fc   = make_fc(true, 0.0f);
    do_tick(pose, fc);

    // Now transition through to TAKEOFF and reach altitude
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);
    fc = make_fc(true, 9.5f);
    do_tick(pose, fc);

    // Should transition directly to NAVIGATE since home was already recorded
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

// ═══════════════════════════════════════════════════════════
// NAVIGATE: waypoint reached + payload trigger
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, WaypointReachedTriggersPayload) {
    fsm.on_takeoff();
    fsm.on_navigate();
    ASSERT_EQ(fsm.state(), MissionState::NAVIGATE);

    // Pose at first waypoint (10, 0, 5) with radius 2
    auto pose = make_pose(10.0f, 0.0f, 5.0f);
    auto fc   = make_fc(true, 10.0f);

    do_tick(pose, fc);

    // First waypoint has trigger_payload=true → should publish payload command
    ASSERT_GE(payload_pub.messages().size(), 1u);
    EXPECT_EQ(payload_pub.messages().back().action, PayloadAction::CAMERA_CAPTURE);
    // Should advance to next waypoint
    EXPECT_EQ(fsm.current_wp_index(), 1u);
}

// ═══════════════════════════════════════════════════════════
// NAVIGATE: mission complete → RTL
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, MissionCompleteTriggersRTL) {
    // Load single-waypoint mission
    fsm.load_mission({{10, 0, 5, 0, 2, 3, false}});
    fsm.on_arm();
    fsm.on_takeoff();
    fsm.on_navigate();

    auto pose = make_pose(10.0f, 0.0f, 5.0f);
    auto fc   = make_fc(true, 10.0f);

    do_tick(pose, fc);

    EXPECT_EQ(fsm.state(), MissionState::RTL);
    ASSERT_GE(fc_calls.size(), 1u);
    // Last FC call should be RTL
    EXPECT_EQ(fc_calls.back().cmd, FCCommandType::RTL);
}

// ═══════════════════════════════════════════════════════════
// NAVIGATE: disarm detection → IDLE (cannot recover without motors)
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, DisarmDuringNavigateDetected) {
    fsm.on_takeoff();
    fsm.on_navigate();
    state_tick.flight_state().nav_was_armed = true;

    auto pose = make_pose(5.0f, 0.0f, 5.0f);
    auto fc   = make_fc(false, 5.0f);  // disarmed!

    // Disarm during NAVIGATE transitions to IDLE — FC can't execute recovery commands
    do_tick(pose, fc);
    EXPECT_EQ(fsm.state(), MissionState::IDLE);
}

// ═══════════════════════════════════════════════════════════
// RTL: disarm detection → IDLE
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, RTLDisarmGoesToIdle) {
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_rtl();
    state_tick.flight_state().nav_was_armed = true;

    auto pose = make_pose(0, 0, 1);
    auto fc   = make_fc(false, 1.0f);  // disarmed during RTL

    do_tick(pose, fc);

    EXPECT_EQ(fsm.state(), MissionState::IDLE);
}

// ═══════════════════════════════════════════════════════════
// LAND: landed transition
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, LandTransitionsToIdleWhenTouchdown) {
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_rtl();
    fsm.on_land();
    state_tick.flight_state().land_sent = true;

    auto pose = make_pose(0, 0, 0);
    auto fc   = make_fc(true, 0.3f);  // below landed_alt_m (0.5)

    do_tick(pose, fc);

    EXPECT_EQ(fsm.state(), MissionState::IDLE);
    EXPECT_TRUE(state_tick.consume_fault_reset());
}

// ═══════════════════════════════════════════════════════════
// LAND: stays in LAND if land_sent is false
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, LandStaysIfLandNotSent) {
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_rtl();
    fsm.on_land();
    // land_sent defaults to false

    auto pose = make_pose(0, 0, 0);
    auto fc   = make_fc(true, 0.3f);

    do_tick(pose, fc);

    EXPECT_EQ(fsm.state(), MissionState::LAND);
}

// ═══════════════════════════════════════════════════════════
// NAVIGATE_UNSTUCK: disarm during backoff transitions to IDLE
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, NavigateUnstuckAbortsOnDisarm) {
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_stuck();
    ASSERT_EQ(fsm.state(), MissionState::NAVIGATE_UNSTUCK);

    auto pose = make_pose(5, 5, 5);
    auto fc   = make_fc(false, 5.0f);  // disarmed
    do_tick(pose, fc);
    EXPECT_EQ(fsm.state(), MissionState::IDLE);
}

// ═══════════════════════════════════════════════════════════
// NAVIGATE: waypoint overshoot advances to next (Issue #236)
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, WaypointOvershootAdvancesToNext) {
    // Override mission with 3 waypoints so WP1 has a previous WP for overshoot detection
    fsm.load_mission(
        {{0, 0, 5, 0, 2, 3, false}, {10, 0, 5, 0, 2, 3, true}, {20, 0, 5, 0, 2, 3, false}});

    // Get to NAVIGATE and advance to WP1
    fsm.on_takeoff();
    fsm.on_navigate();
    (void)fsm.advance_waypoint();  // Now at WP1 (10,0,5)
    ASSERT_EQ(fsm.state(), MissionState::NAVIGATE);
    ASSERT_EQ(fsm.current_wp_index(), 1u);

    // Place drone at (15,0,5) — past WP1 along approach direction (WP0→WP1 = +X)
    auto pose = make_pose(15, 0, 5);
    auto fc   = make_fc(true, 5.0f);

    do_tick(pose, fc);

    // Should have advanced past WP1 due to overshoot detection
    EXPECT_EQ(fsm.current_wp_index(), 2u);
}
