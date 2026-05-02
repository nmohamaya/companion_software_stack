// tests/test_comms_heartbeat.cpp
//
// PR #651 P1 review fix: regression tests for the SimpleFlight 60 ms
// api_goal_timeout heartbeat decision rule.  The original PR shipped
// the heartbeat without any test coverage — disabling it would not
// have broken any test.  These tests pin the contract:
//
//   1. No cached trajectory  → no-op
//   2. RTL/LAND blocks       → no-op
//   3. Fresh send this tick  → no-op (don't double-send)
//   4. Within heartbeat period → no-op
//   5. Past heartbeat period, within stale bound → Send
//   6. Past stale bound       → SuppressStale (don't fly drone into terrain)
//   7. Boundary at exactly heartbeat_period → Send (>= per the rule)
//   8. Boundary at exactly max_stale → Send (> per the rule, not >=)

#include "comms/heartbeat_decision.h"

#include <chrono>

#include <gtest/gtest.h>

using drone::comms::HeartbeatAction;
using drone::comms::evaluate_heartbeat;
using namespace std::chrono_literals;

namespace {
constexpr auto kHeartbeatPeriod = 40ms;
constexpr auto kMaxStale        = 5000ms;
}  // namespace

TEST(HeartbeatDecision, NoCachedTrajectoryReturnsNone) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/false,
                                 /*blocked=*/false,
                                 /*staleness=*/100ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::None);
}

TEST(HeartbeatDecision, RtlLandBlockReturnsNone) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/true,
                                 /*staleness=*/100ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::None)
        << "RTL/LAND sentinel must suppress heartbeat — otherwise stale velocity "
           "commands re-enter offboard during descent (Issue #340 violation)";
}

TEST(HeartbeatDecision, FreshSendThisTickReturnsNone) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/true,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*staleness=*/100ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::None)
        << "fresh trajectory just sent — heartbeat must not double-send";
}

TEST(HeartbeatDecision, WithinHeartbeatPeriodReturnsNone) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*staleness=*/30ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::None)
        << "30ms < 40ms heartbeat period — too early to fire";
}

TEST(HeartbeatDecision, PastHeartbeatPeriodWithinStaleBoundReturnsSend) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*staleness=*/45ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::Send)
        << "45ms > 40ms heartbeat period AND < 5000ms stale bound → SEND";
}

TEST(HeartbeatDecision, PastStaleBoundReturnsSuppressStale) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*staleness=*/6000ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::SuppressStale)
        << "6000ms > 5000ms stale bound → suppress; P4 may have died, "
           "replaying stale velocity commands forever is unsafe";
}

TEST(HeartbeatDecision, BoundaryAtExactlyHeartbeatPeriodReturnsSend) {
    // Rule uses `staleness >= kHeartbeatPeriod` (inclusive) so exactly
    // 40ms must fire.  Off-by-one regression would change `>=` to `>`.
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*staleness=*/kHeartbeatPeriod, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::Send);
}

TEST(HeartbeatDecision, BoundaryAtExactlyMaxStaleReturnsSend) {
    // Rule uses `staleness > kMaxStale` (strict) so exactly the bound
    // is still SEND, not SuppressStale.  Off-by-one regression would
    // make exactly-bound suppress.
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*staleness=*/kMaxStale, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::Send);
}

TEST(HeartbeatDecision, JustPastMaxStaleReturnsSuppressStale) {
    EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false,
                                 /*have_cached=*/true,
                                 /*blocked=*/false,
                                 /*staleness=*/kMaxStale + 1ms, kHeartbeatPeriod, kMaxStale),
              HeartbeatAction::SuppressStale);
}
