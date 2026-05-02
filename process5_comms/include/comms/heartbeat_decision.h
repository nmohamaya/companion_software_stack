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
// PR #684 Copilot review fix: the previous single-`staleness` API
// measured `now - last_traj_send`, but `last_traj_send` updates on
// every successful heartbeat resend — so the stale-bound never
// triggered when P4 was silent (heartbeat just kept replaying the
// last trajectory forever).  Two distinct durations are needed:
//
//   * `since_last_send` — used to gate the heartbeat-period throttle.
//     Ticks forward on every successful send (heartbeat OR new traj).
//   * `since_last_new_trajectory` — used to gate the stale-suppression
//     bound.  Updated ONLY when a brand-new trajectory cmd is received
//     from P4.  Heartbeat replays do NOT reset this.
//
// Decision rule (matches `fc_tx_thread`):
//   - If a fresh trajectory was sent this tick, OR no trajectory has
//     ever been cached, OR the RTL/LAND sentinel is set → No-op.
//   - Else if `since_last_new_trajectory > max_stale` → SuppressStale
//     (P4 has likely silently died; replaying a stale velocity command
//     indefinitely would fly the drone into terrain).
//   - Else if `since_last_send >= heartbeat_period` → Send.
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
    std::chrono::milliseconds since_last_send, std::chrono::milliseconds since_last_new_trajectory,
    std::chrono::milliseconds heartbeat_period, std::chrono::milliseconds max_stale) noexcept {
    if (sent_this_tick || !have_cached_trajectory || blocked_by_mode_change) {
        return HeartbeatAction::None;
    }
    // Stale-bound is decided against the time since the LAST NEW
    // trajectory — heartbeat resends do not reset this counter, so a
    // permanently-silent P4 will eventually trip the bound regardless
    // of how often the heartbeat fires.
    if (since_last_new_trajectory > max_stale) {
        return HeartbeatAction::SuppressStale;
    }
    // Heartbeat-period gate is decided against the time since the LAST
    // SEND (heartbeat or new trajectory) — we don't want to spam the
    // RPC layer at the loop-poll rate.
    if (since_last_send >= heartbeat_period) {
        return HeartbeatAction::Send;
    }
    return HeartbeatAction::None;
}

}  // namespace drone::comms
