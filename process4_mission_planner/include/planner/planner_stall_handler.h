// process4_mission_planner/include/planner/planner_stall_handler.h
//
// Issue #765 (acceptance #1 partial) — diagnostic enrichment + LOITER
// escalation when `ThreadWatchdog` detects the `planning_loop` thread
// stuck for more than the watchdog's own `heartbeat_timeout_s`
// threshold (default 5 s for critical threads).  No separate config
// key for the planner-stall threshold — see config_keys.h comment on
// the dropped `planner_stall_loiter_s` for the rationale.
//
// What this does
// ──────────────
//   1. Installed as the `ThreadWatchdog::set_stuck_callback`.  Fires on
//      the watchdog scan thread when a registered thread is detected
//      stuck — the existing ERROR log line at thread_watchdog.h:148
//      already prints the thread name + stuck duration, so this
//      callback's job is the ENRICHMENT: dump LatencyProfiler snapshot
//      + the rest of the diagnostic context that lets us triage on
//      first recurrence.
//
//   2. Sets the thread-safe `stalled_` flag for the `planning_loop`
//      thread specifically (other threads stuck are logged but do not
//      raise the FAULT_PLANNER_STALL bit — those are handled by their
//      own per-thread escalation paths or by process restart).
//
//   3. The planning loop in `process4_mission_planner/src/main.cpp`
//      reads `consume_event()` each tick and passes it to
//      `FaultManager::set_planner_stall()`.  FaultManager escalates to
//      LOITER — vehicle is in the air, NEVER disarm.
//
// Stack-trace capture (Issue #765 PR 2 — DONE)
// ────────────────────────────────────────────
//   When a `TraceCapturer` is injected via `set_trace_capturer()`,
//   `on_stuck()` now captures the stuck thread's stack trace via
//   `drone::util::StackTraceCapture` (SIGUSR1 + `backtrace` + symbolise;
//   `tgkill` delivery, not `pthread_kill` — see stack_trace_capture.h for
//   the async-signal-safe state machine and why).  main.cpp wires the
//   capturer when `watchdog.stack_trace.enabled` is set.
//
// What this still does NOT do (deferred — separate epic)
// ──────────────────────────────────────────────────────
//   - Mutex-snapshot ("which locks does the stuck thread hold").
//     Requires codebase-wide instrumented mutex wrappers; separate epic.
//
// Threading model
// ───────────────
//   - Callback fires on the watchdog scan thread.
//   - `consume_event()` runs on the planning loop thread.
//   - `stalled_` is `std::atomic<bool>` with explicit acquire/release.
//   - The LatencyProfiler dump (logged inside the callback) takes the
//     profiler's own mutex; the watchdog thread is not in a hot path
//     so this is acceptable.  Per CLAUDE.md §"Observability on flight-
//     critical threads" the constraint is about NOT calling mutex-
//     protected observability FROM a flight-critical thread; the
//     watchdog thread is the observer, not the observed.
#pragma once

#include "util/diagnostic.h"
#include "util/latency_profiler.h"
#include "util/stack_trace_capture.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

namespace drone::planner {

/// Encapsulates the watchdog stuck-callback for the planning_loop
/// thread, the FAULT_PLANNER_STALL event flag, and the diagnostic
/// dump emitted on stuck-detection.
///
/// Usage in `main.cpp`:
///   1. Construct the handler (with the watched-thread name) BEFORE
///      the `ThreadWatchdog` so destruction order is correct
///      (see `make_callback()` lifetime contract below).
///   2. Build the callback via `make_callback(profiler_ptr)` after the
///      `LatencyProfiler` exists.  Pass `nullptr` if benchmark mode is
///      disabled — the dump degrades gracefully.
///   3. Install the callback via `watchdog.set_stuck_callback(...)`.
///   4. Each planning tick: read `consume_event()` and pass to
///      `FaultManager::set_planner_stall()`.
class PlannerStallHandler {
public:
    /// @param watched_thread_name  Name of the thread whose stall
    ///                              triggers the FAULT_PLANNER_STALL
    ///                              event flag.  Other threads stuck
    ///                              are logged (by the existing watchdog
    ///                              ERROR line) but do not set the flag.
    explicit PlannerStallHandler(std::string watched_thread_name)
        : watched_thread_name_(std::move(watched_thread_name)) {}

    PlannerStallHandler(const PlannerStallHandler&)            = delete;
    PlannerStallHandler& operator=(const PlannerStallHandler&) = delete;
    PlannerStallHandler(PlannerStallHandler&&)                 = delete;
    PlannerStallHandler& operator=(PlannerStallHandler&&)      = delete;

    /// Returns the callback to install via `ThreadWatchdog::set_stuck_callback`.
    /// Capture-by-this — the handler instance must outlive the watchdog.
    /// In main.cpp this is enforced by declaring the handler BEFORE the
    /// watchdog so the watchdog destructor (which joins the scan thread)
    /// runs before the handler is destroyed (PR #775 review fix).
    ///
    /// PR #775 review fix (memory-safety P2): explicit `std::function`
    /// return type instead of `auto` so the lifetime contract is visible
    /// at the call site without needing to follow the deduced type.
    [[nodiscard]] drone::util::ThreadWatchdog::StuckCallback make_callback(
        drone::util::LatencyProfiler* profiler) {
        return [this, profiler](const drone::util::ThreadHeartbeat& beat) {
            on_stuck(beat, profiler);
        };
    }

    /// Read + clear the FAULT_PLANNER_STALL event flag.  Called once
    /// per planning tick from `main.cpp` and passed into
    /// `FaultManager::set_planner_stall()`.  Returns true exactly once
    /// per stuck-detection event; subsequent calls return false until
    /// the watchdog fires again.  (Latched-on-fire is intentional.)
    ///
    /// PR #775 review fix (api-contract P2): the
    /// "FaultManager's escalation-only policy handles the persistence"
    /// claim used to be on this docstring — it is TRUE for the LOITER
    /// action (preserved by `high_water_mark_`) but FALSE for the
    /// FAULT_PLANNER_STALL bit in `active_faults` unless FaultManager's
    /// `set_planner_stall()` is OR-latched (which it now is, post-fix).
    /// See fault_manager.h::set_planner_stall for the latch contract.
    [[nodiscard]] bool consume_event() {
        return stalled_.exchange(false, std::memory_order_acq_rel);
    }

    /// Test seam: peek at the flag without clearing.
    [[nodiscard]] bool peek_event() const { return stalled_.load(std::memory_order_acquire); }

    /// Injectable stack-trace capturer (Issue #765 PR 2).  When set,
    /// `on_stuck()` invokes it with the stuck thread's kernel tid + name
    /// so a SIGUSR1-driven backtrace of WHERE the thread is wedged is
    /// captured and logged.  Signature matches
    /// `StackTraceCapture::capture_and_log(pid_t, const char*)`.
    ///
    /// Dependency-injected (rather than calling the singleton directly) so
    /// the existing `on_stuck_for_test()` seam stays hermetic — tests
    /// inject a fake capturer to assert invocation without standing up the
    /// real SIGUSR1 machinery.  Null (the default) = capture disabled; the
    /// rest of the diagnostic dump + FAULT_PLANNER_STALL escalation still
    /// run, so the feature degrades gracefully when unset / config-off.
    ///
    /// THREADING CONTRACT (Issue #765 PR 2 review): `trace_capturer_` is a
    /// non-atomic member READ by `on_stuck()` on the watchdog scan thread.
    /// Call this ONCE, BEFORE `ThreadWatchdog::set_stuck_callback()` makes
    /// the scan thread able to invoke `on_stuck()` (ideally before the
    /// ThreadWatchdog even exists, as main.cpp does).  Setting it after the
    /// callback is installed races the scan thread's read.  Not re-entrant.
    using TraceCapturer = std::function<drone::util::TraceCaptureStatus(pid_t, const char*)>;
    void set_trace_capturer(TraceCapturer capturer) { trace_capturer_ = std::move(capturer); }

    /// Test seam: directly invoke the stuck-handler logic with a
    /// synthetic heartbeat record + optional profiler.  Used by
    /// `tests/test_planner_stall_handler.cpp` so tests don't need to
    /// stand up a real ThreadWatchdog scan loop.
    ///
    /// **DO NOT call from production code** — production callers must
    /// install via `make_callback()` so the lifetime contract (handler
    /// outlives watchdog) is enforced by main.cpp's declaration order.
    /// PR #775 review fix (code-quality P3).
    void on_stuck_for_test(const drone::util::ThreadHeartbeat& beat,
                           drone::util::LatencyProfiler*       profiler = nullptr) {
        on_stuck(beat, profiler);
    }

private:
    void on_stuck(const drone::util::ThreadHeartbeat& beat,
                  drone::util::LatencyProfiler*       profiler) {
        // The watchdog already logged an ERROR line with name + stuck
        // duration at the moment of detection (thread_watchdog.h:148).
        // This callback adds the diagnostic context for triage.
        const bool is_watched_thread = std::strncmp(beat.name, watched_thread_name_.c_str(),
                                                    sizeof(beat.name)) == 0;

        DRONE_LOG_ERROR("[Watchdog/STUCK-DIAG] thread='{}' is_critical={} watched={}", beat.name,
                        beat.is_critical, is_watched_thread);

        if (profiler != nullptr) {
            // LatencyProfiler::to_json() takes the profiler's own mutex.
            // Watchdog thread is not in a hot path — mutex cost is fine.
            // Per CLAUDE.md §"Observability on flight-critical threads"
            // the constraint applies to control-loop callers; the
            // watchdog is the observer side.
            DRONE_LOG_ERROR("[Watchdog/STUCK-DIAG] LatencyProfiler snapshot: {}",
                            profiler->to_json());
        } else {
            DRONE_LOG_ERROR("[Watchdog/STUCK-DIAG] LatencyProfiler unavailable "
                            "(benchmark mode not enabled — no per-stage stats to dump)");
        }

        // Issue #765 PR 2 — capture the stuck thread's stack trace.  Runs
        // for ANY stuck thread (not only the watched one): a trace of where
        // a stuck thread is wedged is the load-bearing diagnostic for #765,
        // regardless of whether it raises the planner-stall fault.  Skipped
        // when no capturer is injected (capture disabled / config-off) or
        // the heartbeat slot has no captured tid (tid==0 sentinel).
        if (trace_capturer_ && beat.tid != 0) {
            (void)trace_capturer_(beat.tid, beat.name);  // logs internally
        }

        if (is_watched_thread) {
            // Only the watched thread (planning_loop) raises the
            // FAULT_PLANNER_STALL event.  Other stuck threads are
            // logged but do not trigger the LOITER escalation —
            // they're handled by their own paths (process restart by
            // P7 / etc.).
            stalled_.store(true, std::memory_order_release);
        }
    }

    const std::string watched_thread_name_;
    std::atomic<bool> stalled_{false};
    TraceCapturer     trace_capturer_{};  // null = capture disabled (#765 PR 2)
};

}  // namespace drone::planner
