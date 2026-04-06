// tests/test_fault_response_executor.cpp
// Unit tests for FaultResponseExecutor (Issue #154).
#include "planner/fault_response_executor.h"

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

private:
    std::vector<T> messages_;
    std::string    topic_ = "mock";
};

struct FCCallRecord {
    FCCommandType cmd;
    float         param;
};

}  // namespace

class FaultResponseExecutorTest : public ::testing::Test {
protected:
    MockPublisher<TrajectoryCmd> traj_pub;
    MissionFSM                   fsm;
    SharedFlightState            flight_state;
    std::vector<FCCallRecord>    fc_calls;
    FaultResponseExecutor        executor;

    FCSendFn send_fc = [this](FCCommandType cmd, float p) {
        fc_calls.push_back({cmd, p});
    };

    void SetUp() override {
        fsm.load_mission({{10, 0, 5, 0, 2, 3, false}});
        fsm.on_arm();       // IDLE → PREFLIGHT
        fsm.on_takeoff();   // PREFLIGHT → TAKEOFF
        fsm.on_navigate();  // TAKEOFF → NAVIGATE (airborne)
    }
};

// ═══════════════════════════════════════════════════════════
// WARN action — log only, no FC commands
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, WarnActionNoFCCommand) {
    FaultState fault;
    fault.recommended_action = FaultAction::WARN;
    fault.reason             = "test warning";
    fault.active_faults      = to_uint(FAULT_BATTERY_LOW);

    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 1000);

    EXPECT_TRUE(fc_calls.empty());
    EXPECT_EQ(fsm.state(), MissionState::NAVIGATE);
    EXPECT_EQ(executor.last_action(), FaultAction::WARN);
}

// ═══════════════════════════════════════════════════════════
// LOITER action — stops trajectory, transitions FSM
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, LoiterStopsTrajAndTransitions) {
    FaultState fault;
    fault.recommended_action = FaultAction::LOITER;
    fault.reason             = "critical process died";
    fault.active_faults      = to_uint(FAULT_CRITICAL_PROCESS);

    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 1000);

    EXPECT_TRUE(fc_calls.empty());  // LOITER doesn't send FC command
    EXPECT_EQ(fsm.state(), MissionState::LOITER);
    EXPECT_TRUE(fsm.fault_triggered());
    EXPECT_GE(traj_pub.messages().size(), 1u);
    EXPECT_TRUE(
        traj_pub.messages().back().valid);  // stop = zero-velocity valid cmd (P5 skips valid=false)
}

// ═══════════════════════════════════════════════════════════
// RTL action — sends FC command, stops trajectory
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, RTLSendsFCAndTransitions) {
    FaultState fault;
    fault.recommended_action = FaultAction::RTL;
    fault.reason             = "battery low";
    fault.active_faults      = to_uint(FAULT_BATTERY_RTL);

    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 1000);

    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::RTL);
    EXPECT_EQ(fsm.state(), MissionState::RTL);
    EXPECT_TRUE(flight_state.nav_was_armed);
}

// ═══════════════════════════════════════════════════════════
// EMERGENCY_LAND — sends LAND command
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, EmergencyLandSendsLandCommand) {
    FaultState fault;
    fault.recommended_action = FaultAction::EMERGENCY_LAND;
    fault.reason             = "battery critical";
    fault.active_faults      = to_uint(FAULT_BATTERY_CRITICAL);

    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 1000);

    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::LAND);
    EXPECT_EQ(fsm.state(), MissionState::LAND);
    EXPECT_TRUE(flight_state.land_sent);
}

// ═══════════════════════════════════════════════════════════
// Escalation-only — doesn't downgrade
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, EscalationOnlyNeverDowngrades) {
    FaultState fault_rtl;
    fault_rtl.recommended_action = FaultAction::RTL;
    fault_rtl.reason             = "battery";
    fault_rtl.active_faults      = to_uint(FAULT_BATTERY_RTL);
    executor.execute(fault_rtl, fsm, send_fc, traj_pub, flight_state, 1000);
    EXPECT_EQ(executor.last_action(), FaultAction::RTL);
    fc_calls.clear();

    // Try to apply WARN (lower) — should be ignored
    FaultState fault_warn;
    fault_warn.recommended_action = FaultAction::WARN;
    fault_warn.reason             = "thermal";
    fault_warn.active_faults      = to_uint(FAULT_THERMAL_WARNING);
    executor.execute(fault_warn, fsm, send_fc, traj_pub, flight_state, 2000);
    EXPECT_TRUE(fc_calls.empty());  // no new commands
    EXPECT_EQ(executor.last_action(), FaultAction::RTL);
}

// ═══════════════════════════════════════════════════════════
// Non-airborne skip — IDLE/PREFLIGHT states
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, SkipsNonAirborneStates) {
    MissionFSM preflight_fsm;
    preflight_fsm.load_mission({{10, 0, 5, 0, 2, 3, false}});
    preflight_fsm.on_arm();  // → PREFLIGHT

    FaultState fault;
    fault.recommended_action = FaultAction::RTL;
    fault.reason             = "test";
    fault.active_faults      = to_uint(FAULT_BATTERY_RTL);

    executor.execute(fault, preflight_fsm, send_fc, traj_pub, flight_state, 1000);
    EXPECT_TRUE(fc_calls.empty());
    EXPECT_EQ(preflight_fsm.state(), MissionState::PREFLIGHT);
}

// ═══════════════════════════════════════════════════════════
// Reset clears state
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, ResetClearsState) {
    FaultState fault;
    fault.recommended_action = FaultAction::RTL;
    fault.reason             = "test";
    fault.active_faults      = to_uint(FAULT_BATTERY_RTL);
    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 1000);
    EXPECT_EQ(executor.last_action(), FaultAction::RTL);

    executor.reset();
    EXPECT_EQ(executor.last_action(), FaultAction::NONE);
}

// ═══════════════════════════════════════════════════════════
// Collision recovery: low-severity actions (WARN, LOITER) skipped
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, CollisionRecoverySkipsLowSeverity) {
    fsm.on_collision_recovery();  // NAVIGATE → COLLISION_RECOVERY
    EXPECT_EQ(fsm.state(), MissionState::COLLISION_RECOVERY);

    FaultState fault;
    fault.recommended_action = FaultAction::LOITER;
    fault.reason             = "process died";
    fault.active_faults      = to_uint(FAULT_CRITICAL_PROCESS);

    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 1000);

    // LOITER should be skipped during collision recovery
    EXPECT_TRUE(fc_calls.empty());
    EXPECT_EQ(fsm.state(), MissionState::COLLISION_RECOVERY);
    EXPECT_EQ(executor.last_action(), FaultAction::NONE);
}

// ═══════════════════════════════════════════════════════════
// Collision recovery: RTL is allowed (high-severity)
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, CollisionRecoveryAllowsRTL) {
    fsm.on_collision_recovery();

    FaultState fault;
    fault.recommended_action = FaultAction::RTL;
    fault.reason             = "battery low";
    fault.active_faults      = to_uint(FAULT_BATTERY_RTL);

    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 1000);

    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::RTL);
    EXPECT_EQ(executor.last_action(), FaultAction::RTL);
}

// ═══════════════════════════════════════════════════════════
// log_new_faults: detects new fault flags
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, LogNewFaultsTracksNewFlags) {
    // log_new_faults() updates internal last_active_faults_ and logs via spdlog.
    // The internal state is not directly observable without a getter, but we verify:
    //   1. No crash/UB on first call (all flags are new)
    //   2. No crash/UB on second call with additional flag
    //   3. Internal tracking is implicitly tested: reset() clears last_active_faults_,
    //      and the execution path through new_flags = active & ~last_ is exercised.
    executor.log_new_faults(to_uint(FAULT_BATTERY_LOW));

    // Add a new flag — exercises the new_flags != 0 path for FAULT_THERMAL_WARNING
    executor.log_new_faults(to_uint(FAULT_BATTERY_LOW) | to_uint(FAULT_THERMAL_WARNING));

    // Verify reset clears the tracked faults (re-logging same flags should not crash)
    executor.reset();
    executor.log_new_faults(to_uint(FAULT_BATTERY_LOW));
}

// ═══════════════════════════════════════════════════════════
// log_new_faults: no-ops on same flags
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, LogNewFaultsIgnoresKnownFlags) {
    // Verifies the new_flags == 0 path (no log output) when flags are unchanged.
    // log_new_faults() only produces spdlog output and has no return value or public
    // accessor for last_active_faults_. The test verifies no-crash behavior and that
    // the internal state tracking is exercised (new_flags = active & ~last_ == 0).
    uint32_t flags = to_uint(FAULT_BATTERY_LOW) | to_uint(FAULT_POSE_STALE);
    executor.log_new_faults(flags);
    // Call again with same flags — exercises the new_flags == 0 early-exit path
    executor.log_new_faults(flags);
}

// ═══════════════════════════════════════════════════════════
// Multiple fault flags in single execution
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, MultipleFaultFlagsInSingleExecution) {
    FaultState fault;
    fault.recommended_action = FaultAction::EMERGENCY_LAND;
    fault.reason             = "multiple faults";
    fault.active_faults      = to_uint(FAULT_BATTERY_CRITICAL) | to_uint(FAULT_FC_LINK_LOST) |
                          to_uint(FAULT_VIO_LOST);

    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 1000);

    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::LAND);
    EXPECT_EQ(fsm.state(), MissionState::LAND);
    EXPECT_TRUE(flight_state.land_sent);
}

// ═══════════════════════════════════════════════════════════
// Stop trajectory has valid=true and zero velocity
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, StopTrajectoryHasValidTrueZeroVelocity) {
    FaultState fault;
    fault.recommended_action = FaultAction::LOITER;
    fault.reason             = "pose stale";
    fault.active_faults      = to_uint(FAULT_POSE_STALE);

    executor.execute(fault, fsm, send_fc, traj_pub, flight_state, 42000);

    ASSERT_GE(traj_pub.messages().size(), 1u);
    const auto& stop = traj_pub.messages().back();
    EXPECT_TRUE(stop.valid);
    EXPECT_FLOAT_EQ(stop.velocity_x, 0.0f);
    EXPECT_FLOAT_EQ(stop.velocity_y, 0.0f);
    EXPECT_FLOAT_EQ(stop.velocity_z, 0.0f);
    EXPECT_EQ(stop.timestamp_ns, 42000u);
}

// ═══════════════════════════════════════════════════════════
// LOITER works from TAKEOFF state (not just NAVIGATE)
// ═══════════════════════════════════════════════════════════
TEST_F(FaultResponseExecutorTest, TakeoffStateLoiterWorks) {
    MissionFSM takeoff_fsm;
    takeoff_fsm.load_mission({{10, 0, 5, 0, 2, 3, false}});
    takeoff_fsm.on_arm();
    takeoff_fsm.on_takeoff();  // → TAKEOFF (airborne, but not NAVIGATE)

    FaultResponseExecutor fresh_executor;

    FaultState fault;
    fault.recommended_action = FaultAction::LOITER;
    fault.reason             = "process died";
    fault.active_faults      = to_uint(FAULT_CRITICAL_PROCESS);

    fresh_executor.execute(fault, takeoff_fsm, send_fc, traj_pub, flight_state, 1000);

    EXPECT_EQ(takeoff_fsm.state(), MissionState::LOITER);
    EXPECT_TRUE(takeoff_fsm.fault_triggered());
    // LOITER should not send FC commands
    EXPECT_TRUE(fc_calls.empty());
    // A stop trajectory should have been published with valid=true and zero velocity
    ASSERT_GE(traj_pub.messages().size(), 1u);
    const auto& stop = traj_pub.messages().back();
    EXPECT_TRUE(stop.valid);
    EXPECT_FLOAT_EQ(stop.velocity_x, 0.0f);
    EXPECT_FLOAT_EQ(stop.velocity_y, 0.0f);
    EXPECT_FLOAT_EQ(stop.velocity_z, 0.0f);
}
