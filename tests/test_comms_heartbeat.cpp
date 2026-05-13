// tests/test_comms_heartbeat.cpp
//
// PR #651 P1 review fix: regression tests for the SimpleFlight 60 ms
// api_goal_timeout heartbeat decision rule.  The original PR shipped
// the heartbeat without any test coverage — disabling it would not
// have broken any test.
//
// PR #684 Copilot review: signature now takes TWO durations
// (since_last_send for the heartbeat-period gate, since_last_new_traj
// for the stale-bound).  Tests pin both gates and the boundary
// conditions, plus a dedicated "P4 silent forever — heartbeat never
// resets the stale-bound" test that the previous single-staleness API
// failed silently.
//
// Pinned contract:
//   1. No cached trajectory  → No-op
//   2. RTL/LAND blocks       → No-op
//   3. Fresh send this tick  → No-op (don't double-send)
//   4. Within heartbeat period → No-op
//   5. Past heartbeat period, within stale bound → Send
//   6. Past stale bound       → SuppressStale (don't fly drone into terrain)
//   7. Boundary at exactly heartbeat_period → Send (>= per the rule)
//   8. Boundary at exactly max_stale → Send (> per the rule, not >=)
//   9. P4 silent for 6 s while heartbeat fires every 40 ms → SuppressStale
//      (was the actual bug in the pre-Copilot-fix design)

#include "comms/heartbeat_decision.h"

#include <chrono>

#include <gtest/gtest.h>

using drone::comms::evaluate_heartbeat;
using drone::comms::HeartbeatAction;
using namespace std::chrono_literals;

namespace {
constexpr auto kHeartbeatPeriod = 40ms;
constexpr auto kMaxStale        = 5000ms;
}  // namespace

TEST(HeartbeatDecision, NoCachedTrajectoryReturnsNone) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/false,
                                 /*blocked=*/false,
                                 /*since_last_send=*/100ms,
                                 /*since_last_new_traj=*/100ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::None);
}

TEST(HeartbeatDecision, RtlLandBlockReturnsNone) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/true,
                                 /*since_last_send=*/100ms,
                                 /*since_last_new_traj=*/100ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::None)
        << "RTL/LAND sentinel must suppress heartbeat — otherwise stale velocity "
           "commands re-enter offboard during descent (Issue #340 violation)";
}

TEST(HeartbeatDecision, FreshSendThisTickReturnsNone) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/true,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*since_last_send=*/100ms,
                                 /*since_last_new_traj=*/100ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::None)
        << "fresh trajectory just sent — heartbeat must not double-send";
}

TEST(HeartbeatDecision, WithinHeartbeatPeriodReturnsNone) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*since_last_send=*/30ms,
                                 /*since_last_new_traj=*/30ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::None)
        << "30ms < 40ms heartbeat period — too early to fire";
}

TEST(HeartbeatDecision, PastHeartbeatPeriodWithinStaleBoundReturnsSend) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*since_last_send=*/45ms,
                                 /*since_last_new_traj=*/45ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::Send)
        << "45ms > 40ms heartbeat period AND < 5000ms stale bound → SEND";
}

TEST(HeartbeatDecision, PastStaleBoundReturnsSuppressStale) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*since_last_send=*/45ms,
                                 /*since_last_new_traj=*/6000ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::SuppressStale)
        << "6000ms > 5000ms stale bound on since_last_new_traj → suppress; "
           "P4 may have died, replaying stale velocity commands forever is unsafe";
}

TEST(HeartbeatDecision, BoundaryAtExactlyHeartbeatPeriodReturnsSend) {
    // Rule uses `since_last_send >= kHeartbeatPeriod` (inclusive) so exactly
    // 40ms must fire.  Off-by-one regression would change `>=` to `>`.
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*since_last_send=*/kHeartbeatPeriod,
                                 /*since_last_new_traj=*/kHeartbeatPeriod, kHeartbeatPeriod,
                                 kMaxStale),
              HeartbeatAction::Send);
}

TEST(HeartbeatDecision, BoundaryAtExactlyMaxStaleReturnsSend) {
    // Rule uses `since_last_new_traj > kMaxStale` (strict) so exactly
    // the bound is still SEND, not SuppressStale.  Off-by-one regression
    // would make exactly-bound suppress.
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*since_last_send=*/kHeartbeatPeriod,
                                 /*since_last_new_traj=*/kMaxStale, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::Send);
}

TEST(HeartbeatDecision, JustPastMaxStaleReturnsSuppressStale) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*since_last_send=*/kHeartbeatPeriod,
                                 /*since_last_new_traj=*/kMaxStale + 1ms, kHeartbeatPeriod,
                                 kMaxStale),
              HeartbeatAction::SuppressStale);
}

// PR #684 Copilot review: this is the actual P4-dead safety net test.
// In the pre-fix single-staleness design, `staleness` was reset on
// every heartbeat send, so a permanently-silent P4 with the heartbeat
// firing every 40 ms kept staleness near 0 and the stale-bound NEVER
// fired — the safety net did nothing.  With two timestamps, a
// since_last_send of 40 ms (heartbeat just fired) but
// since_last_new_traj of 6000 ms (P4 silent the whole time) MUST
// trigger SuppressStale.
TEST(HeartbeatDecision, P4SilentForeverWithHeartbeatActiveStillSuppresses) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*since_last_send=*/40ms,        // heartbeat just fired
                                 /*since_last_new_traj=*/6000ms,  // P4 silent for 6 s
                                 kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::SuppressStale)
        << "P4 silent for 6 s WITH heartbeat-period reset must STILL trigger "
           "SuppressStale.  Pre-Copilot-fix this case spurious-passed because "
           "since_last_send was used for both gates and reset on heartbeat send.";
}
