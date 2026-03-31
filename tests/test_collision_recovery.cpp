// tests/test_collision_recovery.cpp
// Unit tests for collision recovery FSM state and logic (Issue #226).
#include "planner/mission_fsm.h"
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

// ═══════════════════════════════════════════════════════════
// FSM transition tests
// ═══════════════════════════════════════════════════════════

TEST(CollisionRecoveryFSMTest, TransitionToCollisionRecovery) {
    MissionFSM fsm;
    fsm.on_arm();
    fsm.on_takeoff();
    fsm.on_navigate();
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);

    fsm.on_collision_recovery();
    EXPECT_EQ(fsm.state(), MissionState::COLLISION_RECOVERY);
}

TEST(CollisionRecoveryFSMTest, RecoveryCompleteReturnsToNavigate) {
    MissionFSM fsm;
    fsm.on_arm();
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_collision_recovery();
    EXPECT_EQ(fsm.state(), MissionState::COLLISION_RECOVERY);

    fsm.on_recovery_complete();
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

TEST(CollisionRecoveryFSMTest, StateNameReturnsCorrectString) {
    EXPECT_STREQ(state_name(MissionState::COLLISION_RECOVERY), "COLLISION_RECOVERY");
}

TEST(CollisionRecoveryFSMTest, CollisionRecoveryIsInFaultState) {
    MissionFSM fsm;
    fsm.on_arm();
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_collision_recovery();
    fsm.set_fault_triggered(true);
    EXPECT_TRUE(fsm.is_in_fault_state());
}

TEST(CollisionRecoveryFSMTest, WaypointPreservedAcrossRecovery) {
    MissionFSM fsm;
    fsm.load_mission(
        {{0, 0, 5, 0, 2, 3, false}, {10, 0, 5, 0, 2, 3, false}, {20, 0, 5, 0, 2, 3, false}});
    fsm.on_arm();
    fsm.on_takeoff();
    fsm.on_navigate();
    (void)fsm.advance_waypoint();  // WP1
    EXPECT_EQ(fsm.current_wp_index(), 1u);

    fsm.on_collision_recovery();
    EXPECT_EQ(fsm.current_wp_index(), 1u);  // preserved

    fsm.on_recovery_complete();
    EXPECT_EQ(fsm.current_wp_index(), 1u);  // still preserved
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

// ═══════════════════════════════════════════════════════════
// State tick collision recovery logic tests
// ═══════════════════════════════════════════════════════════

class CollisionRecoveryTickTest : public ::testing::Test {
protected:
    StateTickConfig               config{10.0f, 1.5f, 0.5f, 5, 0.0f, 0.3f,
                           true,   // collision_recovery_enabled
                           3.0f,   // collision_climb_delta_m
                           0.5f};  // collision_hover_duration_s (short for tests)
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
        fsm.load_mission({{10, 0, 5, 0, 2, 3, false}, {20, 0, 5, 0, 2, 3, false}});
        fsm.on_arm();
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

TEST_F(CollisionRecoveryTickTest, DisarmDuringNavigateTriggersRecovery) {
    fsm.on_takeoff();
    fsm.on_navigate();
    state_tick.flight_state().nav_was_armed = true;

    auto pose = make_pose(5.0f, 0.0f, 5.0f);
    auto fc   = make_fc(false, 5.0f);  // disarmed!

    do_tick(pose, fc);

    EXPECT_EQ(fsm.state(), MissionState::COLLISION_RECOVERY);
}

TEST_F(CollisionRecoveryTickTest, RecoveryDisabledKeepsNavigate) {
    // Re-create with recovery disabled
    StateTickConfig  disabled_config{10.0f, 1.5f, 0.5f, 5, 0.0f, 0.3f,
                                    false,  // collision_recovery_enabled = false
                                    3.0f,  0.5f};
    MissionStateTick disabled_tick{disabled_config};

    fsm.on_takeoff();
    fsm.on_navigate();
    disabled_tick.flight_state().nav_was_armed = true;

    auto pose = make_pose(5.0f, 0.0f, 5.0f);
    auto fc   = make_fc(false, 5.0f);

    DetectedObjectList            objects{};
    drone::util::FrameDiagnostics diag(0);
    disabled_tick.tick(fsm, pose, fc, objects, *planner_, nullptr, *avoider_, obstacle_layer,
                       traj_pub, payload_pub, send_fc, 0, diag);

    // Should stay in NAVIGATE (old behavior)
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

TEST_F(CollisionRecoveryTickTest, HoverPhasePublishesZeroVelocity) {
    fsm.on_takeoff();
    fsm.on_navigate();

    // Enter recovery manually
    fsm.on_collision_recovery();
    ASSERT_EQ(fsm.state(), MissionState::COLLISION_RECOVERY);

    auto pose = make_pose(5.0f, 0.0f, 5.0f);
    auto fc   = make_fc(true, 5.0f);
    traj_pub.clear();

    do_tick(pose, fc);

    // Should publish a trajectory with zero velocity (hover)
    ASSERT_GE(traj_pub.messages().size(), 1u);
    const auto& cmd = traj_pub.messages().back();
    EXPECT_TRUE(cmd.valid);
    EXPECT_FLOAT_EQ(cmd.velocity_x, 0.0f);
    EXPECT_FLOAT_EQ(cmd.velocity_y, 0.0f);
    EXPECT_FLOAT_EQ(cmd.velocity_z, 0.0f);
}

TEST_F(CollisionRecoveryTickTest, ClimbPhaseTargetsHigherAltitude) {
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_collision_recovery();

    auto pose = make_pose(5.0f, 0.0f, 5.0f);
    auto fc   = make_fc(true, 5.0f);

    // Tick several times to get past hover phase (0.5s config, 10Hz ticks ~5 ticks)
    // With short hover_duration_s=0.5, one tick at time=0 starts hover,
    // we need to wait for real time. Instead, just tick enough times.
    // Actually the hover phase uses real clock, so we need to wait.
    // For this test, let's manually verify the climb target by ticking once
    // (HOVER phase), then sleeping past the hover duration.

    // First tick enters hover
    do_tick(pose, fc);
    EXPECT_EQ(fsm.state(), MissionState::COLLISION_RECOVERY);

    // Sleep past hover duration
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    do_tick(pose, fc);  // This tick transitions HOVER→CLIMB

    // Now tick again in CLIMB phase to get the climb trajectory cmd
    traj_pub.clear();
    do_tick(pose, fc);

    // Should be in climb phase now — target_z should be 5.0 + 3.0 = 8.0
    ASSERT_GE(traj_pub.messages().size(), 1u);
    const auto& cmd = traj_pub.messages().back();
    EXPECT_TRUE(cmd.valid);
    EXPECT_FLOAT_EQ(cmd.target_z, 8.0f);
    EXPECT_GT(cmd.velocity_z, 0.0f);  // climbing
}

TEST_F(CollisionRecoveryTickTest, ReplanPhaseTransitionsToNavigate) {
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_collision_recovery();

    auto pose = make_pose(5.0f, 0.0f, 5.0f);
    auto fc   = make_fc(true, 5.0f);

    // Tick to enter hover phase
    do_tick(pose, fc);

    // Sleep past hover duration
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // Tick to enter climb phase
    do_tick(pose, fc);

    // Tick again to be in CLIMB phase
    do_tick(pose, fc);

    // Now pose at target altitude (5 + 3 = 8, minus 0.5 tolerance)
    auto high_pose = make_pose(5.0f, 0.0f, 7.6f);
    do_tick(high_pose, fc);  // CLIMB→REPLAN transition

    // Tick once more — REPLAN phase transitions to NAVIGATE
    do_tick(high_pose, fc);
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
}

TEST_F(CollisionRecoveryTickTest, FullRecoveryCyclePreservesWaypoint) {
    fsm.load_mission(
        {{10, 0, 5, 0, 2, 3, false}, {20, 0, 5, 0, 2, 3, false}, {30, 0, 5, 0, 2, 3, false}});
    fsm.on_takeoff();
    fsm.on_navigate();
    (void)fsm.advance_waypoint();  // WP1
    size_t wp_before                        = fsm.current_wp_index();
    state_tick.flight_state().nav_was_armed = true;

    // Trigger collision via disarm
    auto pose = make_pose(15.0f, 0.0f, 5.0f);
    auto fc   = make_fc(false, 5.0f);
    do_tick(pose, fc);
    EXPECT_EQ(fsm.state(), MissionState::COLLISION_RECOVERY);
    EXPECT_EQ(fsm.current_wp_index(), wp_before);

    // Run through recovery phases
    auto armed_fc = make_fc(true, 5.0f);

    // Hover
    do_tick(pose, armed_fc);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // Transition HOVER→CLIMB
    do_tick(pose, armed_fc);

    // Tick in CLIMB phase
    do_tick(pose, armed_fc);

    // Reach altitude — CLIMB→REPLAN transition
    auto high_pose = make_pose(15.0f, 0.0f, 7.6f);
    do_tick(high_pose, armed_fc);

    // REPLAN→NAVIGATE transition
    do_tick(high_pose, armed_fc);

    // Should be back in NAVIGATE with same waypoint
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
    EXPECT_EQ(fsm.current_wp_index(), wp_before);
}
