// process5_comms/include/comms/heartbeat_decision.h
//
// Pure decision helper for the comms heartbeat mechanism.  Extracted
// from fc_tx_thread so the timing + staleness logic is unit-testable
// without an IFCLink + ISubscriber set-up.
//
// PR #651 P1 review fix: the original heartbeat shipped without any
// regression test ("disabling the feature would not break any test").
// This helper makes the decision rule directly addressable from
// test_comms_heartbeat.cpp; the thread function calls into it once
// per tick and acts on the returned Action.
//
// Decision rule (matches `fc_tx_thread`):
//   - If a fresh trajectory was sent this tick, OR no trajectory has
//     ever been cached, OR the RTL/LAND sentinel is set → No-op.
//   - Else if the staleness exceeds the max-stale bound → Suppress
//     (P4 has likely silently died; replaying a stale velocity command
//     indefinitely would fly the drone into terrain).
//   - Else if the staleness exceeds the heartbeat period → Send.
//   - Otherwise → No-op (within-period since last send).
#pragma once

#include <chrono>

namespace drone::comms {

enum class HeartbeatAction {
    None,           ///< Do nothing this tick.
    Send,           ///< Re-send the cached trajectory.
    SuppressStale,  ///< Cache too stale; suppress + WARN (P4 may be dead).
};

[[nodiscard]] inline HeartbeatAction evaluate_heartbeat(
    bool sent_this_tick, bool have_cached_trajectory, bool blocked_by_mode_change,
    std::chrono::milliseconds staleness, std::chrono::milliseconds heartbeat_period,
    std::chrono::milliseconds max_stale) noexcept {
    if (sent_this_tick || !have_cached_trajectory || blocked_by_mode_change) {
        return HeartbeatAction::None;
    }
    if (staleness > max_stale) {
        return HeartbeatAction::SuppressStale;
    }
    if (staleness >= heartbeat_period) {
        return HeartbeatAction::Send;
    }
    return HeartbeatAction::None;
}

}  // namespace drone::comms
