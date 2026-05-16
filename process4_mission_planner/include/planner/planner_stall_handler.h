// process4_mission_planner/include/planner/planner_stall_handler.h
//
// Issue #765 (acceptance #1 partial) — diagnostic enrichment + LOITER
// escalation when ThreadWatchdog detects the `planning_loop` thread
// stuck for more than `planner_stall_loiter_s`.
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
// What this does NOT do (deferred to follow-up #765 work)
// ───────────────────────────────────────────────────────
//   - Stack-trace capture via `pthread_kill(SIGUSR1)` +
//     `backtrace_symbols`.  Requires async-signal-safe handler + thread-
//     local buffer + symbolisation outside signal context; non-trivial.
//     Defer until first-recurrence dump proves insufficient.
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
#include "util/thread_heartbeat.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>

namespace drone::planner {

/// Encapsulates the watchdog stuck-callback for the planning_loop
/// thread, the FAULT_PLANNER_STALL event flag, and the diagnostic
/// dump emitted on stuck-detection.
///
/// Construct once in `main.cpp` and call `install(...)` with the
/// LatencyProfiler pointer (may be nullptr — dump degrades gracefully)
/// and the `ThreadWatchdog`.  Read `consume_event()` once per planning
/// tick and pass to `FaultManager::set_planner_stall()`.
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
    [[nodiscard]] auto make_callback(drone::util::LatencyProfiler* profiler) {
        return [this, profiler](const drone::util::ThreadHeartbeat& beat) {
            on_stuck(beat, profiler);
        };
    }

    /// Read + clear the FAULT_PLANNER_STALL event flag.  Called once
    /// per planning tick from `main.cpp` and passed into
    /// `FaultManager::set_planner_stall()`.  Returns true exactly once
    /// per stuck-detection event; subsequent calls return false until
    /// the watchdog fires again.  (Latched-on-fire is intentional —
    /// FaultManager's escalation-only policy handles the persistence.)
    [[nodiscard]] bool consume_event() {
        return stalled_.exchange(false, std::memory_order_acq_rel);
    }

    /// Test seam: peek at the flag without clearing.
    [[nodiscard]] bool peek_event() const { return stalled_.load(std::memory_order_acquire); }

    /// Test seam: directly invoke the stuck-handler logic with a
    /// synthetic heartbeat record + optional profiler.  Used by
    /// `tests/test_planner_stall_handler.cpp` so tests don't need to
    /// stand up a real ThreadWatchdog scan loop.
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
};

}  // namespace drone::planner
