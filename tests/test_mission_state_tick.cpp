// tests/test_mission_state_tick.cpp
// Unit tests for MissionStateTick (Issue #154).
#include "planner/grid_planner_base.h"
#include "planner/iobstacle_avoider.h"
#include "planner/mission_state_tick.h"
#include "planner/obstacle_avoider_3d.h"
#include "planner/planner_factory.h"
#include "util/mock_clock.h"

#include <chrono>
#include <limits>  // std::numeric_limits — NaN test for SettleGateNonFiniteResetsCounter
#include <thread>
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

// Issue #716 — `armable` defaults to true so the bulk of existing tests
// (which exercise post-preflight behaviour) do not need to know about the
// PREFLIGHT gate.  Tests that specifically exercise the gate override
// `armable=false` after construction.
FCState make_fc(bool armed, float rel_alt) {
    FCState fc{};
    fc.armed             = armed;
    fc.connected         = true;
    fc.armable           = true;
    fc.rel_alt           = rel_alt;
    fc.battery_remaining = 80.0f;
    fc.timestamp_ns      = 1000;
    return fc;
}

}  // namespace

// Issue #740 — existing tests in this fixture exercise ARM-retry semantics
// orthogonal to the cold-start debounce gate.  The debounce is covered by
// MissionStateTickDebounceTest below (which installs a ScopedMockClock to
// drive the stability window deterministically).  Here we disable the
// debounce so existing tests continue to see "ARM fires on first armable
// tick" without needing per-test clock manipulation.
inline StateTickConfig make_default_test_config() {
    StateTickConfig c{10.0f, 1.5f, 0.5f, 5};
    c.preflight_armable_stable_s = 0.0f;
    // Issue #740 Layer 4 — disable the post-ARM settle gate by default so
    // tests exercising post-TAKEOFF behaviour transition immediately on
    // `fc_state.armed` (legacy semantics).  The dedicated
    // `MissionStateTickTakeoffSettleTest` fixture below re-enables it.
    c.takeoff_settle_observations = 0;
    return c;
}

class MissionStateTickTest : public ::testing::Test {
protected:
    StateTickConfig               config = make_default_test_config();
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

    std::unique_ptr<IPathPlanner>     planner_ = create_path_planner("dstar_lite").value();
    std::unique_ptr<IObstacleAvoider> avoider_ =
        create_obstacle_avoider("potential_field_3d", 5.0f, 2.0f).value();

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
// Issue #716 — ARM gated on FC preflight readiness
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, PreflightWaitsWhenFCNotArmable) {
    // FC not yet ready (EKF2 converging, sensors warming).  Planner must
    // NOT send ARM in this state — sending ARM produces PX4's
    // "Arming denied: Resolve system health failures first" log spam and,
    // worse, can arm the vehicle in a degraded state when health flickers
    // through OK.  See #713 for the cold-start race this guards against.
    auto pose  = make_pose(0, 0, 0);
    auto fc    = make_fc(false, 0);
    fc.armable = false;

    do_tick(pose, fc);

    // No ARM command should have been issued
    EXPECT_TRUE(fc_calls.empty())
        << "Planner sent ARM before FC reported armable=true (Issue #716 regression)";
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);
}

TEST_F(MissionStateTickTest, PreflightSendsARMWhenFCBecomesArmable) {
    auto pose = make_pose(0, 0, 0);

    // Tick 1: FC not yet armable — no ARM sent
    auto fc_not_ready    = make_fc(false, 0);
    fc_not_ready.armable = false;
    do_tick(pose, fc_not_ready);
    EXPECT_TRUE(fc_calls.empty());
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);  // Issue #716 review — assert state too

    // Tick 2: FC reports armable — ARM should be sent immediately on the
    // very next tick (no 3-second wait imposed by the wait-log path).
    auto fc_ready = make_fc(false, 0);  // make_fc defaults armable=true
    do_tick(pose, fc_ready);
    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);
}

// Issue #716 review (test-quality / test-unit P2) — two consecutive ticks
// with armable=true must produce only ONE ARM within the retry interval.
// Prior to the fix, last_arm_time_ being constructor-initialised to
// `now - 10s` guarantees the first ARM fires; a follow-up tick within 3 s
// must NOT add a second ARM.
TEST_F(MissionStateTickTest, PreflightDoesNotResendArmWithinRetryInterval) {
    auto pose = make_pose(0, 0, 0);
    auto fc   = make_fc(false, 0);  // armable=true (default), not armed yet

    do_tick(pose, fc);
    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);

    // Second tick within the 3-second retry window — must not fire again
    do_tick(pose, fc);
    EXPECT_EQ(fc_calls.size(), 1u) << "Planner sent a duplicate ARM within the retry interval "
                                      "(Issue #716 retry-dedup regression)";
}

// Issue #716 review (test-unit P2 + fault-recovery P3) — armable can flicker
// true → false → true during EKF lock loss or sensor reinitialisation mid-
// PREFLIGHT.  After a brief false dip, the planner must NOT spuriously
// re-arm (the retry window protects against that), and it must STILL
// eventually issue the ARM once armable recovers (the gate must not
// permanently latch on the false transition).
TEST_F(MissionStateTickTest, PreflightHandlesArmableFlicker) {
    auto pose = make_pose(0, 0, 0);

    // Tick 1: armable=true → ARM fires
    auto fc_ok = make_fc(false, 0);
    do_tick(pose, fc_ok);
    ASSERT_EQ(fc_calls.size(), 1u);

    // Tick 2: armable goes false (EKF flicker) → no new ARM, still PREFLIGHT
    auto fc_lost    = make_fc(false, 0);
    fc_lost.armable = false;
    do_tick(pose, fc_lost);
    EXPECT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // Tick 3: armable recovers — within the retry window, so no immediate
    // re-arm (consistent with PreflightDoesNotResendArmWithinRetryInterval).
    // The FSM stays in PREFLIGHT until either the retry interval elapses
    // (production path) or fc_state.armed flips true (PX4 acknowledged the
    // earlier ARM after the flicker cleared).
    do_tick(pose, fc_ok);
    EXPECT_LE(fc_calls.size(), 1u);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);
}

// ═══════════════════════════════════════════════════════════
// Issue #740 (epic #727 Layer 1) — ARM-gate stability debounce
//
// Cold-start failure on Gazebo: PX4's `health_all_ok` flickers true while
// EKF2 attitude is still settling (gyro/accel bias estimates wandering).
// Arming on a single-tick flicker produces asymmetric mixer commands and
// the drone tips on the ground at takeoff.  Fix: require N consecutive
// seconds of continuous `fc_state.armable=true` before sending ARM.
//
// These tests use ScopedMockClock to drive the stability window without
// real-time sleeps.  The fixture sets `preflight_armable_stable_s = 3.0`
// (the production default) so the tests exercise the exact production
// behaviour.
// ═══════════════════════════════════════════════════════════
class MissionStateTickDebounceTest : public ::testing::Test {
protected:
    // ScopedMockClock must be initialised BEFORE state_tick — the planner's
    // first construction-time clock query (any of) happens via
    // drone::util::get_clock(), which ScopedMockClock overrides.
    drone::util::ScopedMockClock mock_clock_guard;

    StateTickConfig config = [] {
        StateTickConfig c{10.0f, 1.5f, 0.5f, 5};
        c.preflight_armable_stable_s = 3.0f;  // production default
        // This fixture exercises the #741 pre-ARM debounce.  Disable the
        // #740 Layer 4 post-ARM settle gate so the armed→takeoff transition
        // fires immediately — Layer 4 has its own fixture
        // (MissionStateTickTakeoffSettleTest) below.
        c.takeoff_settle_observations = 0;
        return c;
    }();
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

    std::unique_ptr<IPathPlanner>     planner_ = create_path_planner("dstar_lite").value();
    std::unique_ptr<IObstacleAvoider> avoider_ =
        create_obstacle_avoider("potential_field_3d", 5.0f, 2.0f).value();

    void do_tick(const Pose& pose, const FCState& fc_state) {
        DetectedObjectList            objects{};
        drone::util::FrameDiagnostics diag(0);

        state_tick.tick(fsm, pose, fc_state, objects, *planner_, nullptr, *avoider_, obstacle_layer,
                        traj_pub, payload_pub, send_fc, 0, diag);
    }
};

// Tick 1: armable=true → start stability window, no ARM yet.
// Tick 2: armable=false (the EKF2/health flicker) → reset window, no ARM.
// Tick 3+: armable=true again → window restarts.  After full window
// elapsed, ARM finally fires.  This is the actual production-equivalent
// of a Gazebo cold-start where `health_all_ok` blips through true for
// one MAVLink update before EKF2 actually converges.
TEST_F(MissionStateTickDebounceTest, ArmableFlickerResetsStabilityWindow) {
    auto pose = make_pose(0, 0, 0);

    // Tick 1: armable flickers true once.  Debounce starts, no ARM.
    auto fc_ok = make_fc(false, 0);
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty())
        << "ARM fired on first armable=true tick — debounce gate not engaged "
           "(Issue #740 regression).  EKF2-flicker cold-start race re-opens.";

    // Tick 2: armable drops back to false (the flicker).  Window should reset.
    auto fc_lost    = make_fc(false, 0);
    fc_lost.armable = false;
    do_tick(pose, fc_lost);
    EXPECT_TRUE(fc_calls.empty());

    // Tick 3: armable recovers — fresh window starts at THIS observation.
    // Advance the clock first so a buggy "resume from old first_seen"
    // implementation would see the original window already satisfied.
    mock_clock_guard.mock().advance_ms(1500);
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty());

    // Tick 4: only 1.5s elapsed since the fresh window started.  Must NOT
    // arm yet — proves the window restarted, not resumed.
    mock_clock_guard.mock().advance_ms(1500);
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty())
        << "ARM fired before the stability window restarted after flicker — "
           "the debounce is resuming rather than resetting (Issue #740 logic bug).";

    // Tick 5: another 1.7s (total 3.2s since the fresh window started in
    // tick 3) — window satisfied, ARM fires exactly once.
    mock_clock_guard.mock().advance_ms(1700);
    do_tick(pose, fc_ok);
    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);
}

// Continuous armable=true for the full stability window → ARM fires once.
TEST_F(MissionStateTickDebounceTest, ArmableStableForWindowFiresArm) {
    auto pose  = make_pose(0, 0, 0);
    auto fc_ok = make_fc(false, 0);

    // Tick 1: starts the stability window.
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty());

    // Advance clock past the 3.0s window.
    mock_clock_guard.mock().advance_ms(3100);

    // Tick 2: window elapsed → ARM fires.
    do_tick(pose, fc_ok);
    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);
}

// Continuous armable=true but below the stability window → no ARM.
// Verifies the gate does not fire prematurely under sub-window stable
// conditions (e.g. PX4 momentarily ok but EKF2 still pre-converged).
TEST_F(MissionStateTickDebounceTest, ArmableStableBelowWindowDoesNotArm) {
    auto pose  = make_pose(0, 0, 0);
    auto fc_ok = make_fc(false, 0);

    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty());

    // Advance clock by 2.5s — still under the 3.0s window.
    mock_clock_guard.mock().advance_ms(2500);

    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty()) << "ARM fired before the stability window elapsed (Issue #740 — "
                                     "debounce window too short).";
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);
}

// Sanity: with `preflight_armable_stable_s = 0.0`, the debounce is
// effectively disabled and ARM fires on the first armable=true tick.
// This proves the gate is *purely* configurable — operators can turn it
// off (e.g. for headless dev) without code changes.
TEST(MissionStateTickDebounceConfigTest, ZeroWindowDisablesDebounce) {
    drone::util::ScopedMockClock mock_clock;

    StateTickConfig cfg{10.0f, 1.5f, 0.5f, 5};
    cfg.preflight_armable_stable_s = 0.0f;
    MissionStateTick state_tick{cfg};

    MissionFSM fsm;
    fsm.load_mission({{10, 0, 5, 0, 2, 3, true}});
    fsm.on_arm();

    StaticObstacleLayer           obstacle_layer;
    MockPublisher<TrajectoryCmd>  traj_pub;
    MockPublisher<PayloadCommand> payload_pub;
    std::vector<FCCallRecord>     fc_calls;
    FCSendFn                      send_fc = [&](FCCommandType cmd, float p) {
        fc_calls.push_back({cmd, p});
    };
    auto planner = create_path_planner("dstar_lite").value();
    auto avoider = create_obstacle_avoider("potential_field_3d", 5.0f, 2.0f).value();

    DetectedObjectList            objects{};
    drone::util::FrameDiagnostics diag(0);
    auto                          pose = make_pose(0, 0, 0);
    auto                          fc   = make_fc(false, 0);  // armable=true

    state_tick.tick(fsm, pose, fc, objects, *planner, nullptr, *avoider, obstacle_layer, traj_pub,
                    payload_pub, send_fc, 0, diag);

    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);
}

// PR #741 review (test-unit + test-quality P2, 2 convergent): the production
// code at mission_state_tick.h:319 has an explicit underflow-guard clause
// `now_ns < armable_first_seen_ns_` that treats clock-backward as a fresh
// first-observation rather than wrapping the unsigned subtraction.  Without
// a test, a buggy refactor that drops this guard would still pass all the
// other 4 debounce tests — false-green risk.  This test pins the guard.
TEST_F(MissionStateTickDebounceTest, ArmableClockRewindRestartsWindow) {
    auto pose  = make_pose(0, 0, 0);
    auto fc_ok = make_fc(false, 0);

    // Advance the mock clock so we have a non-trivial baseline.
    mock_clock_guard.mock().advance_s(10);

    // Tick 1: armable=true → starts window at t=10s.
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty());

    // Rewind the mock clock to t=5s (e.g. test reset, host clock skew).
    // The production code MUST restart the window from this fresh point,
    // not compute `5s - 10s` as a huge uint64_t and falsely "elapse" the gate.
    mock_clock_guard.mock().reset(5'000'000'000ULL);

    // Tick 2: armable still true, but clock has rewound → window restarts.
    // No ARM should fire (window just started fresh at t=5s).
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty())
        << "ARM fired after clock rewind — underflow guard at "
           "mission_state_tick.h:319 has been removed or broken.  Production "
           "code would compute a huge `elapsed_ns` from the unsigned wrap and "
           "fire ARM immediately (Issue #740 regression).";

    // Tick 3 (at t=5s + 3.1s = 8.1s) — the freshly-started window has now
    // elapsed past the 3.0s threshold → ARM fires.
    mock_clock_guard.mock().advance_ms(3100);
    do_tick(pose, fc_ok);
    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);
}

// PR #741 review (test-unit + test-quality P2, 2 convergent): existing
// debounce tests use `make_default_test_config()` which sets stable_s=0.0,
// so the interaction between the debounce gate (#740) and the ARM-retry
// throttle (#716, `last_arm_time_ns_`) is never exercised at production
// defaults.  After PR #743's mixed-clock migration both throttles share a
// single mockable clock domain, so we can finally drive both deterministically.
TEST_F(MissionStateTickDebounceTest, DebounceAndRetryComposeAtProductionDefaults) {
    auto pose  = make_pose(0, 0, 0);
    auto fc_ok = make_fc(false, 0);

    // Tick 1 at t=0: starts the 3.0s stability window.
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty());

    // Tick 2 at t=3.1s: window has elapsed → first ARM fires.  PX4
    // hasn't acknowledged yet (test doesn't flip `fc_state.armed=true`),
    // so the FSM stays in PREFLIGHT and the ARM-retry throttle now
    // protects against spamming.
    mock_clock_guard.mock().advance_ms(3100);
    do_tick(pose, fc_ok);
    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);

    // Tick 3 at t=4.1s (only 1s after first ARM): inside the 3s retry
    // window → MUST NOT re-fire ARM (Issue #716 retry-dedup contract).
    // Pre-PR #743 this required wall-clock sleeps; now it's mockable.
    mock_clock_guard.mock().advance_ms(1000);
    do_tick(pose, fc_ok);
    EXPECT_EQ(fc_calls.size(), 1u) << "Duplicate ARM fired within the 3.0s retry interval — the "
                                      "#716 throttle is not gating the debounce-driven ARM path "
                                      "(PR #743 mixed-clock regression).";

    // Tick 4 at t=6.2s (3.1s after first ARM): retry interval has elapsed
    // → second ARM fires.  This is the safety-net retry for the case
    // where PX4 dropped the first MAVLink message.
    mock_clock_guard.mock().advance_ms(2100);
    do_tick(pose, fc_ok);
    ASSERT_EQ(fc_calls.size(), 2u);
    EXPECT_EQ(fc_calls[1].cmd, FCCommandType::ARM);
}

// PR #741 review (test-quality P3): the existing tests use offsets of
// 2500 ms (below) and 3100 ms (above) for the 3.0s window — they don't
// pin the exact-equality semantics at the boundary.  A refactor that
// changes `<` to `<=` in the comparison at mission_state_tick.h:332 would
// not be caught.  This test pins the boundary: at exactly 3.0s elapsed,
// the gate must consider the window satisfied (i.e. `< window_ns` is the
// correct shape — equality counts as "elapsed").
TEST_F(MissionStateTickDebounceTest, ArmableStableAtExactWindowBoundaryFiresArm) {
    auto pose  = make_pose(0, 0, 0);
    auto fc_ok = make_fc(false, 0);

    // Tick 1: starts the window.
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty());

    // Advance EXACTLY 3.0s (the configured stable window).  The check is
    // `(now_ns - first_seen_ns_) < window_ns` — at exact equality the
    // condition is false, so the code falls through to ARM.
    mock_clock_guard.mock().advance_ns(3'000'000'000ULL);
    do_tick(pose, fc_ok);
    ASSERT_EQ(fc_calls.size(), 1u)
        << "ARM did not fire at exactly the window boundary (3.0s).  The "
           "comparison at mission_state_tick.h:332 may have flipped from "
           "`<` to `<=` — pre-window-equality should count as elapsed.";
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);
}

// PR #741 review (test-quality + test-unit P3): the `armable_first_seen_ns_
// = 0` reset on the `fc_state.armed=true` early return (mission_state_tick.h
// line ~286) is not exercised at production stable_s.  Without coverage, a
// regression that removes this reset would only manifest on the rare re-
// PREFLIGHT path (currently unreachable in production but possible via
// future GCS re-arm).  This test verifies the reset by simulating that path:
// (1) build up to a near-elapsed window, (2) transition to armed (which
// must reset the tracker), (3) hypothetically re-enter PREFLIGHT (we
// can't really; instead we directly assert the field state after the
// armed branch by observing that a second post-armed cycle starts a
// fresh window).
TEST_F(MissionStateTickDebounceTest, ArmedTransitionResetsStabilityWindowForReentry) {
    auto pose = make_pose(0, 0, 0);

    // Phase 1: build a stability window then trigger the armed transition.
    auto fc_ok = make_fc(false, 0);
    do_tick(pose, fc_ok);  // starts window at t=0
    mock_clock_guard.mock().advance_ms(2500);
    do_tick(pose, fc_ok);  // still within window — no ARM
    EXPECT_TRUE(fc_calls.empty());

    auto fc_armed = make_fc(true, 0);  // armed=true → fsm.on_takeoff() path
    do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);

    // Reset FSM back to PREFLIGHT to simulate re-entry (this is a test-
    // only manoeuvre — production has no re-PREFLIGHT path today, but the
    // code defensively resets `armable_first_seen_ns_` to support a future
    // re-arm flow).  Reload mission to reset FSM to IDLE first, then on_arm().
    fsm.load_mission({{10, 0, 5, 0, 2, 3, true}});
    fsm.on_arm();
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // Phase 2: fresh tick at production stable_s.  If the reset on armed
    // didn't happen, `armable_first_seen_ns_` still holds the t=0 value
    // and elapsed-from-then is already > 3s — ARM would fire immediately.
    // The reset must have happened, so this tick starts a FRESH window.
    do_tick(pose, fc_ok);
    EXPECT_TRUE(fc_calls.empty())
        << "ARM fired immediately on re-PREFLIGHT — the `armable_first_seen_"
           "ns_ = 0` reset on the armed transition (mission_state_tick.h "
           "line ~286) was skipped, leaving stale state.  Production safety "
           "regression for future re-arm flows.";

    // Window must elapse from THIS tick, not from the original t=0.
    mock_clock_guard.mock().advance_ms(3100);
    do_tick(pose, fc_ok);
    ASSERT_EQ(fc_calls.size(), 1u);
    EXPECT_EQ(fc_calls[0].cmd, FCCommandType::ARM);
}

// ═══════════════════════════════════════════════════════════
// Issue #740 (epic #727) Layer 4 — post-ARM, pre-TAKEOFF settle gate.
//
// The #746 smoke sweep proved the #741 `armable` debounce is necessary-
// but-insufficient: PX4 can report armed with a marginal EKF2 attitude
// estimate, and commanding TAKEOFF onto it makes the attitude controller
// fight a phantom tilt → asymmetric rotor spin-up → sideways skid.  Layer
// 4 holds after `fc_state.armed` until the FC-reported attitude + velocity
// estimate has been within thresholds for N *consecutive* FCState
// observations.  The gate is observation-counted (not wall-timed), so
// these tests need no ScopedMockClock — they just feed FCState sequences.
// ═══════════════════════════════════════════════════════════
class MissionStateTickTakeoffSettleTest : public ::testing::Test {
protected:
    // settle_observations = 5 (small for fast tests), production-shaped
    // tilt/velocity limits.  preflight_armable_stable_s = 0 so the pre-ARM
    // debounce is out of the picture — these tests exercise only the
    // post-ARM settle gate.
    StateTickConfig config = [] {
        StateTickConfig c{10.0f, 1.5f, 0.5f, 5};
        c.preflight_armable_stable_s  = 0.0f;
        c.takeoff_settle_observations = 5;
        c.takeoff_max_tilt_deg        = 5.0f;
        c.takeoff_max_velocity_mps    = 0.3f;
        return c;
    }();
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

    std::unique_ptr<IPathPlanner>     planner_ = create_path_planner("dstar_lite").value();
    std::unique_ptr<IObstacleAvoider> avoider_ =
        create_obstacle_avoider("potential_field_3d", 5.0f, 2.0f).value();

    void do_tick(const Pose& pose, const FCState& fc_state) {
        DetectedObjectList            objects{};
        drone::util::FrameDiagnostics diag(0);
        state_tick.tick(fsm, pose, fc_state, objects, *planner_, nullptr, *avoider_, obstacle_layer,
                        traj_pub, payload_pub, send_fc, 0, diag);
    }
};

// A settled (armed, level, near-zero velocity) FCState held for N
// consecutive observations fires takeoff exactly on the Nth — not before.
TEST_F(MissionStateTickTakeoffSettleTest, SettleGateHoldsUntilNConsecutiveObservations) {
    auto pose     = make_pose(0, 0, 0);
    auto fc_armed = make_fc(true, 0);  // roll=pitch=0, vx=vy=vz=0 → settled

    // Ticks 1..4: settled but below the 5-observation threshold → stay PREFLIGHT.
    for (int i = 1; i <= 4; ++i) {
        do_tick(pose, fc_armed);
        EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT)
            << "Takeoff fired after only " << i
            << " settled observations — Layer 4 gate released early (need 5).";
    }

    // Tick 5: 5th consecutive settled observation → takeoff.
    do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF)
        << "Takeoff did NOT fire after 5 consecutive settled observations — "
           "Layer 4 gate stuck.";
}

// An attitude excursion mid-window resets the counter: the FC estimate
// must settle *continuously*, not cumulatively.
TEST_F(MissionStateTickTakeoffSettleTest, SettleGateExcursionResetsCounter) {
    auto pose     = make_pose(0, 0, 0);
    auto fc_armed = make_fc(true, 0);

    // 4 settled observations — one short of the threshold.
    for (int i = 0; i < 4; ++i) do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // Excursion: roll = 0.30 rad (~17°) — well past the 5° tilt limit.
    // Must reset the counter to 0.
    auto fc_tilted = make_fc(true, 0);
    fc_tilted.roll = 0.30f;
    do_tick(pose, fc_tilted);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // After the excursion, 4 more settled observations must still NOT fire
    // takeoff — proving the counter restarted from 0, not resumed from 4.
    for (int i = 0; i < 4; ++i) do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT)
        << "Takeoff fired with only 4 post-excursion settled observations — "
           "the settle counter resumed instead of resetting (Layer 4 logic bug).";

    // The 5th post-excursion settled observation fires it.
    do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);
}

// `takeoff_settle_observations = 0` disables the gate — takeoff fires on
// the first armed observation (legacy behaviour, used by headless dev
// configs and the default unit-test fixture).
TEST(MissionStateTickTakeoffSettleConfigTest, ZeroObservationsDisablesGate) {
    StateTickConfig c{10.0f, 1.5f, 0.5f, 5};
    c.preflight_armable_stable_s  = 0.0f;
    c.takeoff_settle_observations = 0;  // gate disabled
    MissionStateTick              state_tick{c};
    MockPublisher<TrajectoryCmd>  traj_pub;
    MockPublisher<PayloadCommand> payload_pub;
    MissionFSM                    fsm;
    StaticObstacleLayer           obstacle_layer;
    std::vector<FCCallRecord>     fc_calls;
    FCSendFn                      send_fc = [&](FCCommandType cmd, float p) {
        fc_calls.push_back({cmd, p});
    };
    auto planner = create_path_planner("dstar_lite").value();
    auto avoider = create_obstacle_avoider("potential_field_3d", 5.0f, 2.0f).value();

    fsm.load_mission({{10, 0, 5, 0, 2, 3, true}});
    fsm.on_arm();  // → PREFLIGHT

    DetectedObjectList            objects{};
    drone::util::FrameDiagnostics diag(0);
    state_tick.tick(fsm, make_pose(0, 0, 0), make_fc(true, 0), objects, *planner, nullptr, *avoider,
                    obstacle_layer, traj_pub, payload_pub, send_fc, 0, diag);
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF)
        << "With takeoff_settle_observations = 0 the gate must be disabled — takeoff "
           "should fire on the first armed observation (legacy behaviour).";
}

// A disarm mid-window resets the settle counter: a subsequent re-arm must
// re-accumulate the full N consecutive observations.  Covers the disarm /
// re-PREFLIGHT path (the counter is cleared whenever `fc_state.armed` is
// false).
TEST_F(MissionStateTickTakeoffSettleTest, SettleGateResetsWhenDisarmed) {
    auto pose     = make_pose(0, 0, 0);
    auto fc_armed = make_fc(true, 0);

    // 4 settled armed observations — one short of the threshold.
    for (int i = 0; i < 4; ++i) do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // Disarm: armed=false, armable=false → falls through to the pre-ARM
    // path, which clears the Layer 4 settle counter.
    auto fc_disarmed    = make_fc(false, 0);
    fc_disarmed.armable = false;
    do_tick(pose, fc_disarmed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // Re-arm: 4 settled observations must still NOT fire takeoff — the
    // disarm reset the counter, so we need the full 5 again.
    for (int i = 0; i < 4; ++i) do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT)
        << "Takeoff fired with only 4 post-re-arm settled observations — the "
           "settle counter was not cleared on disarm (Layer 4 re-entry bug).";

    // 5th post-re-arm settled observation fires it.
    do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);
}

// PR #763 review (test-unit + test-quality P2): velocity-excursion path
// was untested — only roll excursion was exercised.  This pins the
// `vel_mag <= vel_limit` branch: a mid-window velocity spike resets the
// counter, just like a tilt excursion does.  Without this test, a refactor
// that dropped the velocity check entirely (`vel_mag` from the
// `attitude_settled` predicate) would still pass the existing 4 tests.
TEST_F(MissionStateTickTakeoffSettleTest, SettleGateVelocityExcursionResetsCounter) {
    auto pose     = make_pose(0, 0, 0);
    auto fc_armed = make_fc(true, 0);  // settled (vx=vy=vz=0)

    for (int i = 0; i < 4; ++i) do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // Velocity excursion: |v| = 1.0 m/s — well past the 0.3 m/s limit.
    // Roll/pitch stay zero so this isolates the velocity branch.
    auto fc_moving = make_fc(true, 0);
    fc_moving.vx   = 1.0f;
    do_tick(pose, fc_moving);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // 4 more settled observations must NOT fire — counter reset, not resumed.
    for (int i = 0; i < 4; ++i) do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT)
        << "Takeoff fired after only 4 post-velocity-excursion settled observations — "
           "the velocity-branch reset is not happening (Layer 4 logic bug).";

    // 5th fires it.
    do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);
}

// PR #763 review (test-unit + test-quality P2): the `std::isfinite` guards
// in the gate predicate were uncovered — a NaN attitude (the exact
// cold-start hazard the gate exists to defend against — corrupted EKF2
// estimate publishing a NaN field) needs to count as an excursion, not
// silently pass the threshold check.  This pins the non-finite branch.
TEST_F(MissionStateTickTakeoffSettleTest, SettleGateNonFiniteResetsCounter) {
    auto pose     = make_pose(0, 0, 0);
    auto fc_armed = make_fc(true, 0);

    for (int i = 0; i < 4; ++i) do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // NaN roll — exactly the kind of garbage a cold-start EKF2 estimate
    // could produce.  Must NOT pass the threshold check (else a bad FC
    // estimate could short-circuit the gate).
    auto fc_nan = make_fc(true, 0);
    fc_nan.roll = std::numeric_limits<float>::quiet_NaN();
    do_tick(pose, fc_nan);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT);

    // Counter must have reset — 4 more settled observations should not fire.
    for (int i = 0; i < 4; ++i) do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::PREFLIGHT)
        << "Takeoff fired after only 4 post-NaN settled observations — the `std::isfinite` "
           "guard isn't resetting the counter, so a NaN FC estimate could skip the gate.";

    // 5th fires it.
    do_tick(pose, fc_armed);
    EXPECT_EQ(fsm.state(), MissionState::TAKEOFF);
}

// PR #763 review (fault-recovery P3 + test-quality): the "never settles"
// failure mode (FC attitude estimate persistently outside thresholds — a
// genuine sensor fault, EKF2 stuck, or persistent vibration) was untested.
// Pin the safety contract: the FSM stays in PREFLIGHT indefinitely, no
// crash, no spurious transition.  The corresponding fault-escalation work
// (timeout + escalate to disarm-with-fault) is tracked in #718 — that
// ISSUE expansion is OUT of this PR's scope; the test here just verifies
// the current fail-safe behaviour (grounded > bad takeoff).
TEST_F(MissionStateTickTakeoffSettleTest, SettleGateNeverSettlesHoldsPreflight) {
    auto pose = make_pose(0, 0, 0);

    // 50 consecutive armed-but-tilted observations — far more than the
    // 5-observation threshold.  None should ever count as settled.
    auto fc_tilted = make_fc(true, 0);
    fc_tilted.roll = 0.30f;  // ~17° — well past the 5° tilt limit

    for (int i = 0; i < 50; ++i) {
        do_tick(pose, fc_tilted);
        ASSERT_EQ(fsm.state(), MissionState::PREFLIGHT)
            << "FSM left PREFLIGHT on tick " << (i + 1)
            << " despite attitude never settling — Layer 4 fail-safe contract violated "
               "(should hold PREFLIGHT until either attitude settles or the operator "
               "intervenes; #718 will add a timeout-escalation path on top of this).";
    }

    // Confirm the gate does eventually release once attitude settles —
    // proves we held PREFLIGHT for the right reason (excursion), not
    // because the gate is broken.
    auto fc_armed = make_fc(true, 0);
    for (int i = 0; i < 5; ++i) do_tick(pose, fc_armed);
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
// NAVIGATE_UNSTUCK: timer expiry transitions back to NAVIGATE
// (Issue #503 — was a P1 coverage gap: tick loop's timer-based
//  exit path was not exercised by any test.)
// ═══════════════════════════════════════════════════════════
TEST(MissionStateTickUnstuckTest, ExitsOnTimerExpiry) {
    // Short backoff, wide safety margin for CI scheduler jitter (Issue #503
    // review: 50 ms margin was flaky under ASAN/TSAN/parallel ctest).
    StateTickConfig cfg{};
    cfg.takeoff_alt_m                     = 10.0f;
    cfg.stuck_detector.enabled            = true;
    cfg.stuck_detector.backoff_duration_s = 0.05f;  // 50 ms
    cfg.stuck_detector.backoff_speed_mps  = 1.0f;
    MissionStateTick short_tick(cfg);

    MissionFSM local_fsm;
    local_fsm.load_mission({{10, 0, 5, 0, 2, 3, false}});
    local_fsm.on_arm();
    local_fsm.on_takeoff();
    local_fsm.on_navigate();
    local_fsm.on_stuck();
    ASSERT_EQ(local_fsm.state(), MissionState::NAVIGATE_UNSTUCK);

    auto local_planner = create_path_planner("dstar_lite").value();
    auto local_avoider = create_obstacle_avoider("potential_field_3d", 5.0f, 2.0f).value();
    StaticObstacleLayer local_layer;

    Pose                          pose = make_pose(5, 5, 5);
    FCState                       fc   = make_fc(true, 5.0f);
    MockPublisher<TrajectoryCmd>  traj_p;
    MockPublisher<PayloadCommand> pay_p;
    auto                          null_fc = [](FCCommandType, float) {
    };
    drone::util::FrameDiagnostics diag(0);

    // Tick once to set unstuck_start_time_, then wait well past
    // backoff_duration_s (4× the backoff).  Under heavy CI load the sleep
    // may overshoot; extra margin tolerates that without requiring a mock
    // clock.  We only care that the transition fires eventually.
    short_tick.tick(local_fsm, pose, fc, DetectedObjectList{}, *local_planner, nullptr,
                    *local_avoider, local_layer, traj_p, pay_p, null_fc, 0, diag);
    ASSERT_EQ(local_fsm.state(), MissionState::NAVIGATE_UNSTUCK);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    short_tick.tick(local_fsm, pose, fc, DetectedObjectList{}, *local_planner, nullptr,
                    *local_avoider, local_layer, traj_p, pay_p, null_fc, 0, diag);
    EXPECT_EQ(local_fsm.state(), MissionState::NAVIGATE);
}

// ═══════════════════════════════════════════════════════════
// End-to-end: stationary pose samples through tick_navigate()
// cause StuckDetector to fire and the FSM to auto-transition
// to NAVIGATE_UNSTUCK (Issue #503 review: bypass of the detector
// via direct on_stuck() was leaving this wiring untested).
// ═══════════════════════════════════════════════════════════
TEST(MissionStateTickUnstuckTest, NavigateTickDetectsStuckAndTransitions) {
    StateTickConfig cfg{};
    cfg.takeoff_alt_m                     = 10.0f;
    cfg.stuck_detector.enabled            = true;
    cfg.stuck_detector.window_s           = 0.2f;  // small so 3 ticks are enough
    cfg.stuck_detector.min_movement_m     = 0.5f;
    cfg.stuck_detector.backoff_duration_s = 0.5f;
    cfg.stuck_detector.max_stuck_count    = 100;  // don't escalate to LOITER in this test
    MissionStateTick tick_obj(cfg);

    MissionFSM local_fsm;
    local_fsm.load_mission({{10, 0, 5, 0, 2, 3, false}});
    local_fsm.on_arm();
    local_fsm.on_takeoff();
    local_fsm.on_navigate();
    ASSERT_EQ(local_fsm.state(), MissionState::NAVIGATE);

    auto local_planner = create_path_planner("dstar_lite").value();
    auto local_avoider = create_obstacle_avoider("potential_field_3d", 5.0f, 2.0f).value();
    StaticObstacleLayer local_layer;

    Pose                          pose = make_pose(5, 5, 5);
    FCState                       fc   = make_fc(true, 5.0f);
    MockPublisher<TrajectoryCmd>  traj_p;
    MockPublisher<PayloadCommand> pay_p;
    auto                          null_fc = [](FCCommandType, float) {
    };
    drone::util::FrameDiagnostics diag(0);

    // Feed stationary pose over a window_s + margin (0.2 s + margin).  Each
    // tick pushes a sample; after samples span >= 0.8 × window_s and the drone
    // hasn't moved, is_stuck() should return true and tick_navigate() must
    // call fsm.on_stuck(), producing the NAVIGATE → NAVIGATE_UNSTUCK
    // transition automatically.
    for (int i = 0; i < 8; ++i) {
        tick_obj.tick(local_fsm, pose, fc, DetectedObjectList{}, *local_planner, nullptr,
                      *local_avoider, local_layer, traj_p, pay_p, null_fc, 0, diag);
        if (local_fsm.state() == MissionState::NAVIGATE_UNSTUCK) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    EXPECT_EQ(local_fsm.state(), MissionState::NAVIGATE_UNSTUCK)
        << "tick_navigate should auto-transition to NAVIGATE_UNSTUCK when the "
           "drone is stationary and is_stuck() fires — this was the P1 coverage "
           "gap flagged in the #503 review round.";
}

// ═══════════════════════════════════════════════════════════
// NAVIGATE_UNSTUCK: zero-velocity entry commands hover, not garbage
// (Defensive: drone should not fly off in a random direction if
//  it got stuck during a hover with zero planned velocity.)
// ═══════════════════════════════════════════════════════════
TEST_F(MissionStateTickTest, NavigateUnstuckHoversWhenEnteredAtZeroVelocity) {
    fsm.on_takeoff();
    fsm.on_navigate();
    fsm.on_stuck();  // enter NAVIGATE_UNSTUCK
    ASSERT_EQ(fsm.state(), MissionState::NAVIGATE_UNSTUCK);

    // unstuck_vx_/vy_/vz_ were never set (default-zero'd) since we didn't go
    // through the stuck_detector's is_stuck() path in the tick.  This mirrors
    // "stuck fired while planned velocity was ≈0" — hover is the right
    // defensive behaviour.
    auto pose = make_pose(5, 5, 5);
    auto fc   = make_fc(true, 5.0f);
    do_tick(pose, fc);

    ASSERT_FALSE(traj_pub.messages().empty());
    const auto& cmd = traj_pub.messages().back();
    EXPECT_NEAR(cmd.velocity_x, 0.0f, 1e-3f);
    EXPECT_NEAR(cmd.velocity_y, 0.0f, 1e-3f);
    EXPECT_NEAR(cmd.velocity_z, 0.0f, 1e-3f);
    // Hover must also hold position, not just zero velocity — verify target
    // is the current pose so a confused-FC doesn't drift (Issue #503 review).
    EXPECT_NEAR(cmd.target_x, pose.translation[0], 1e-3f);
    EXPECT_NEAR(cmd.target_y, pose.translation[1], 1e-3f);
    EXPECT_NEAR(cmd.target_z, pose.translation[2], 1e-3f);
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

// ═══════════════════════════════════════════════════════════════════
// Issue #624 — post-avoider yaw-towards-velocity refresh
// ═══════════════════════════════════════════════════════════════════
//
// When the planner's `yaw_towards_velocity` is on AND the avoider
// deflects velocity during NAVIGATE, `target_yaw` must follow the
// deflected velocity — not stay on the planner's pre-avoidance
// heading.  Verified against scenario 33 failure mode where the drone
// body was yawed at +9° while the avoider deflected motion to -91°
// (= 100° misalignment) — camera lost the cube to the blind spot and
// the drone drifted back into it from the side.

namespace {

/// Stub IObstacleAvoider that overwrites the planned velocity with a
/// caller-provided pure-sideways deflection (+Y direction).  Used to
/// exercise the post-avoider yaw refresh in isolation from real
/// avoider dynamics.
class DeflectingAvoider : public drone::planner::IObstacleAvoider {
public:
    drone::ipc::TrajectoryCmd avoid(const drone::ipc::TrajectoryCmd& planned,
                                    const drone::ipc::Pose& /*pose*/,
                                    const drone::ipc::DetectedObjectList& /*objects*/) override {
        auto out       = planned;
        out.velocity_x = 0.0f;  // cancel planned forward motion
        out.velocity_y = 1.5f;  // pure +Y deflection (= East in ENU)
        out.velocity_z = 0.0f;
        // target_yaw deliberately LEFT at the planner's value — the
        // whole point of the test is that mission_state_tick refreshes
        // it from the (new) velocity, NOT the avoider.
        return out;
    }
    std::string name() const override { return "DeflectingAvoider"; }
};

}  // namespace

class Issue624YawRefreshTest : public ::testing::Test {
protected:
    // GridPlannerBase config with yaw_towards_velocity ON + low threshold
    // so the refresh fires at any non-hover speed.
    drone::planner::GridPlannerConfig grid_cfg = [] {
        drone::planner::GridPlannerConfig c;
        c.yaw_towards_travel         = true;
        c.yaw_towards_velocity       = true;
        c.yaw_velocity_threshold_mps = 0.1f;
        return c;
    }();

    // Minimal grid planner (D* Lite accepts GridPlannerConfig; we only
    // use it for the two accessors #624 added — never for real search).
    std::unique_ptr<drone::planner::IPathPlanner> planner =
        drone::planner::create_path_planner("dstar_lite", grid_cfg).value();

    // The planner in mission_state_tick is typed as IPathPlanner*,
    // and grid_planner as IGridPlanner* — same underlying object here.
    drone::planner::IGridPlanner* grid_planner =
        dynamic_cast<drone::planner::IGridPlanner*>(planner.get());

    DeflectingAvoider                         avoider;
    drone::planner::StaticObstacleLayer       obstacle_layer;
    drone::planner::StateTickConfig           config{10.0f, 1.5f, 0.5f, 5};
    drone::planner::MissionStateTick          state_tick{config};
    MockPublisher<drone::ipc::TrajectoryCmd>  traj_pub;
    MockPublisher<drone::ipc::PayloadCommand> payload_pub;
    drone::planner::MissionFSM                fsm;
    std::vector<FCCallRecord>                 fc_calls;
    drone::planner::FCSendFn send_fc = [this](drone::ipc::FCCommandType cmd, float p) {
        fc_calls.push_back({cmd, p});
    };

    void SetUp() override {
        ASSERT_NE(grid_planner, nullptr) << "test harness needs grid_planner downcast to work";
        ASSERT_TRUE(grid_planner->yaw_towards_velocity_enabled());
        // Mission with a waypoint east of origin so NAVIGATE is exercised.
        fsm.load_mission({{20.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3, false}});
        fsm.on_arm();
        fsm.on_takeoff();
        fsm.on_navigate();
    }

    void do_navigate_tick(const drone::ipc::Pose& pose) {
        drone::ipc::DetectedObjectList objects{};
        drone::util::FrameDiagnostics  diag(0);
        auto                           fc = make_fc(true, 5.0f);
        state_tick.tick(fsm, pose, fc, objects, *planner, grid_planner, avoider, obstacle_layer,
                        traj_pub, payload_pub, send_fc, 0, diag);
    }
};

TEST_F(Issue624YawRefreshTest, TargetYawFollowsAvoiderDeflectedVelocity) {
    // Drone is mid-mission at (0,0,5), planner would normally emit a
    // +X-ish velocity toward the waypoint at (20,0,5) → bee-line yaw 0.
    // DeflectingAvoider forces the velocity to pure +Y (1.5 m/s).
    // Expected: target_yaw = atan2(1.5, 0) = +π/2 (≈ 1.5708 rad, East).
    auto pose = make_pose(0.0f, 0.0f, 5.0f);
    do_navigate_tick(pose);

    ASSERT_FALSE(traj_pub.messages().empty()) << "NAVIGATE tick should have published a trajectory";
    const auto& cmd = traj_pub.messages().back();
    EXPECT_NEAR(cmd.velocity_x, 0.0f, 1e-5f) << "avoider cancelled +X velocity";
    EXPECT_NEAR(cmd.velocity_y, 1.5f, 1e-5f) << "avoider emitted +Y velocity";
    EXPECT_NEAR(cmd.target_yaw, M_PI_2, 1e-3f)
        << "target_yaw must track avoider-deflected velocity (atan2(1.5, 0) = π/2). "
           "If this fires with target_yaw ≈ 0 instead, the post-avoider yaw refresh "
           "in mission_state_tick.h has regressed (Issue #624).";
}

TEST_F(Issue624YawRefreshTest, YawRefreshSkippedBelowVelocityThreshold) {
    // Below-threshold avoider: low velocity in a direction (+X +Y, 45°) that
    // would produce target_yaw ≈ π/4 if the refresh fired unconditionally.
    // The avoider deliberately does NOT touch target_yaw — so whatever value
    // ends up in the published trajectory comes from either:
    //   - the planner's own yaw_towards_travel output (≈ atan2(0, 20) = 0,
    //     bee-line east toward waypoint at (20, 0, 5)) when the refresh is
    //     correctly skipped, OR
    //   - atan2(0.01, 0.01) ≈ π/4 when the refresh fires despite |v| < thr,
    //     OR
    //   - atan2(0.01, 0.01) ≈ π/4 when the #624 yaw-refresh code is removed
    //     entirely AND the planner separately yaw-towards-velocity'd to that
    //     value (which it would, since the planner reads the same
    //     yaw_towards_velocity flag) — so this test alone doesn't prove
    //     #624 is wired up; that's the job of the companion test
    //     `TargetYawFollowsAvoiderDeflectedVelocity`.
    //
    // What this test specifically locks in: the *threshold* guard at the
    // mission_state_tick refresh site (mission_state_tick.h ~line 414) is
    // honoured.  Pre-#624 (no refresh code), planner emits its own yaw based
    // on its own smoothed velocity; with #624's threshold guard correctly
    // applied, target_yaw stays at the planner's value, NOT atan2(vx, vy)
    // of the avoider's tiny (0.01, 0.01) deflection.
    class HoverAvoider : public drone::planner::IObstacleAvoider {
    public:
        drone::ipc::TrajectoryCmd avoid(const drone::ipc::TrajectoryCmd& planned,
                                        const drone::ipc::Pose& /*pose*/,
                                        const drone::ipc::DetectedObjectList& /*objects*/) override {
            auto out       = planned;
            out.velocity_x = 0.01f;  // << 0.1 threshold (= 0.0001 < 0.01²)
            out.velocity_y = 0.01f;
            // target_yaw deliberately NOT modified — flows through from
            // planner so we can observe whether the refresh overwrites it.
            return out;
        }
        std::string name() const override { return "HoverAvoider"; }
    } hover_avoider;

    auto                           pose = make_pose(0.0f, 0.0f, 5.0f);
    drone::ipc::DetectedObjectList objects{};
    drone::util::FrameDiagnostics  diag(0);
    auto                           fc = make_fc(true, 5.0f);
    state_tick.tick(fsm, pose, fc, objects, *planner, grid_planner, hover_avoider, obstacle_layer,
                    traj_pub, payload_pub, send_fc, 0, diag);

    ASSERT_FALSE(traj_pub.messages().empty());
    const auto& cmd = traj_pub.messages().back();
    // If the threshold guard fires correctly, target_yaw is the planner's
    // emitted value (NOT atan2(0.01, 0.01) ≈ 0.785).  Use a wide-margin
    // assertion on the failure mode rather than the success mode — the
    // planner's exact yaw depends on smoothing state we don't want to
    // reach into.  Anything within 0.1 rad of π/4 indicates the refresh
    // fired in spite of the threshold check.
    EXPECT_GT(std::abs(cmd.target_yaw - M_PI_4), 0.1f)
        << "target_yaw is suspiciously close to atan2(0.01, 0.01) = π/4 — "
           "the post-avoider yaw refresh appears to have fired despite "
           "|v|² = 0.0002 < threshold² = 0.01.  The threshold guard at "
           "mission_state_tick.h ~line 414 has regressed (Issue #624).";
}
