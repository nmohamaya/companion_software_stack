// tests/test_planner_stall_handler.cpp
//
// Unit tests for `drone::planner::PlannerStallHandler` — the watchdog
// stuck-callback handler that ships in PR #775 (closes Issue #718,
// partially closes #765 P4 mid-flight planning_loop stall).
//
// What we test:
//   - `consume_event()` returns false initially, true after a stuck
//     event on the watched thread, then false again (single-shot).
//   - Stuck events for non-watched threads do NOT set the flag (only
//     planning_loop stalls escalate to FAULT_PLANNER_STALL — other
//     threads have their own paths).
//   - The handler's `make_callback()` produces a stable function object
//     that captures the handler safely (the watchdog callback fires on
//     the watchdog scan thread; the handler must outlive it).
//   - LatencyProfiler dump path is exercised both with `nullptr`
//     (benchmark profiler disabled) and a real profiler instance.

#include "planner/planner_stall_handler.h"
#include "util/latency_profiler.h"
#include "util/stack_trace_capture.h"  // TraceCaptureStatus (capturer tests)
#include "util/thread_heartbeat.h"

#include <atomic>
#include <cstring>
#include <thread>

#include <gtest/gtest.h>
#include <sys/types.h>  // pid_t

using drone::planner::PlannerStallHandler;
using drone::util::LatencyProfiler;
using drone::util::ThreadHeartbeat;

namespace {

// Helper — build a ThreadHeartbeat with a given name + is_critical.
// `last_touch_ns` is not used by the handler (the watchdog already did
// the stuck calculation by the time the callback fires).
ThreadHeartbeat make_beat(const char* name, bool is_critical) {
    ThreadHeartbeat beat;
    drone::util::safe_name_copy(beat.name, name);
    beat.is_critical = is_critical;
    beat.last_touch_ns.store(0, std::memory_order_relaxed);
    beat.initialized.store(true, std::memory_order_release);
    return beat;
}

}  // namespace

TEST(PlannerStallHandlerTest, ConsumeEventIsFalseInitially) {
    PlannerStallHandler handler("planning_loop");
    EXPECT_FALSE(handler.consume_event());
    EXPECT_FALSE(handler.peek_event());
}

TEST(PlannerStallHandlerTest, WatchedThreadStallSetsEventFlag) {
    PlannerStallHandler handler("planning_loop");

    auto beat = make_beat("planning_loop", /*is_critical=*/true);
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);

    EXPECT_TRUE(handler.peek_event())
        << "Watched-thread stall did not set the FAULT_PLANNER_STALL event flag";
    EXPECT_TRUE(handler.consume_event());
    // Single-shot: subsequent reads return false until the next stuck event.
    EXPECT_FALSE(handler.consume_event());
}

TEST(PlannerStallHandlerTest, NonWatchedThreadStallDoesNotSetEventFlag) {
    PlannerStallHandler handler("planning_loop");

    // A different thread is stuck (e.g. the fc_state_subscriber thread).
    // The handler should LOG the diagnostic but NOT raise
    // FAULT_PLANNER_STALL — only the watched thread escalates to LOITER.
    auto beat = make_beat("fc_state_subscriber", /*is_critical=*/false);
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);

    EXPECT_FALSE(handler.peek_event())
        << "Non-watched thread stall incorrectly raised the FAULT_PLANNER_STALL "
           "event — only the watched thread (planning_loop) should escalate.";
    EXPECT_FALSE(handler.consume_event());
}

TEST(PlannerStallHandlerTest, MultipleStuckEventsLatchToSingleConsume) {
    PlannerStallHandler handler("planning_loop");

    auto beat = make_beat("planning_loop", /*is_critical=*/true);
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);

    // Latched-on-fire: 3 events collapse to one consume.  This is
    // intentional — FaultManager's escalation-only policy handles the
    // persistence; we don't want to keep raising the fault each tick.
    EXPECT_TRUE(handler.consume_event());
    EXPECT_FALSE(handler.consume_event());
}

TEST(PlannerStallHandlerTest, DiagnosticDumpWithProfilerDoesNotCrash) {
    // The handler logs a JSON snapshot from the profiler.  With a real
    // (but empty) profiler this exercises the to_json() path; the
    // important property is that the callback completes without
    // exceptions / asan trips.  Log output is not asserted (gtest
    // doesn't capture spdlog output without extra wiring).
    PlannerStallHandler handler("planning_loop");
    LatencyProfiler     profiler;
    profiler.record("TestStage", /*correlation_id=*/0, /*start_ns=*/0,
                    /*end_ns=*/100'000);

    auto beat = make_beat("planning_loop", /*is_critical=*/true);
    handler.on_stuck_for_test(beat, &profiler);

    EXPECT_TRUE(handler.consume_event());
}

TEST(PlannerStallHandlerTest, MakeCallbackProducesValidStuckCallback) {
    // The callback is captured by-this — the handler instance must
    // outlive any thread holding the callback.  This test fires the
    // callback synchronously to verify the wiring (the real watchdog
    // fires it from the scan thread; that path is integration-tested
    // when the full process runs).
    PlannerStallHandler handler("planning_loop");
    auto                callback = handler.make_callback(/*profiler=*/nullptr);

    auto beat = make_beat("planning_loop", /*is_critical=*/true);
    callback(beat);

    EXPECT_TRUE(handler.consume_event())
        << "make_callback()-produced callback did not propagate to the handler "
           "(this-capture wiring broken)";
}

TEST(PlannerStallHandlerTest, ConsumeEventIsThreadSafeAcrossWatchdogAndPlanningLoop) {
    // The watchdog callback runs on the scan thread; consume_event()
    // runs on the planning loop thread.  std::atomic<bool> with
    // explicit acquire/release ordering must keep them coherent.
    // This test races a stuck-setter and a consumer; the consumer must
    // observe `true` exactly once across all events fired.
    PlannerStallHandler handler("planning_loop");
    constexpr int       kIterations = 1000;
    std::atomic<int>    consumed{0};

    std::thread consumer([&handler, &consumed] {
        for (int i = 0; i < kIterations * 10; ++i) {
            if (handler.consume_event()) {
                ++consumed;
            }
            std::this_thread::yield();
        }
    });

    auto beat = make_beat("planning_loop", /*is_critical=*/true);
    for (int i = 0; i < kIterations; ++i) {
        handler.on_stuck_for_test(beat, /*profiler=*/nullptr);
        std::this_thread::yield();
    }
    consumer.join();

    // Drain any remaining latched event.
    if (handler.consume_event()) ++consumed;

    // Latched-on-fire semantics: each consume returns true only if a
    // stuck event has fired since the last consume.  Under contention
    // some setter calls may collapse with concurrent consumes; the
    // contract is "at least one consume succeeds when events occur"
    // and "no consume returns true without an event preceding it".
    EXPECT_GE(consumed.load(), 1) << "No consume_event() returned true across " << kIterations
                                  << " stuck events — atomic ordering broken.";
    EXPECT_LE(consumed.load(), kIterations)
        << "consume_event() returned true more times than events were fired — "
           "spurious-true bug.";
}

// ─── Issue #765 PR 2 — stack-trace capturer injection ───────────────

// Capturer is invoked with the stuck thread's tid + name when set.
TEST(PlannerStallHandlerTest, TraceCapturerInvokedWithBeatTidAndName) {
    PlannerStallHandler handler("planning_loop");

    std::atomic<pid_t> seen_tid{0};
    std::string        seen_name;
    handler.set_trace_capturer([&](pid_t tid, const char* name) {
        seen_tid.store(tid, std::memory_order_release);
        seen_name = name;
        return drone::util::TraceCaptureStatus::kOk;
    });

    auto beat = make_beat("planning_loop", /*is_critical=*/true);
    beat.tid  = 4242;
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);

    EXPECT_EQ(seen_tid.load(std::memory_order_acquire), 4242);
    EXPECT_EQ(seen_name, "planning_loop");
    // The stall event still fires for the watched thread.
    EXPECT_TRUE(handler.consume_event());
}

// No capturer installed → on_stuck() is safe (no crash) and the existing
// escalation still runs.  Also: a zero tid is never passed to a capturer.
TEST(PlannerStallHandlerTest, NoCapturerInstalledIsSafe) {
    PlannerStallHandler handler("planning_loop");
    auto                beat = make_beat("planning_loop", /*is_critical=*/true);
    beat.tid                 = 99;
    // No set_trace_capturer() call.
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);  // must not crash
    EXPECT_TRUE(handler.consume_event());
}

TEST(PlannerStallHandlerTest, CapturerNotCalledWhenTidIsZero) {
    PlannerStallHandler handler("planning_loop");
    std::atomic<int>    calls{0};
    handler.set_trace_capturer([&](pid_t, const char*) {
        calls.fetch_add(1, std::memory_order_acq_rel);
        return drone::util::TraceCaptureStatus::kOk;
    });
    auto beat = make_beat("planning_loop", /*is_critical=*/true);
    beat.tid  = 0;  // unregistered / synthetic — no valid tid to signal
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);
    EXPECT_EQ(calls.load(std::memory_order_acquire), 0);
}

// A non-watched stuck thread is STILL traced (the trace is the
// load-bearing diagnostic regardless of which thread), but it does NOT
// raise the planner-stall fault flag.
TEST(PlannerStallHandlerTest, NonWatchedThreadStillTracedButNoFault) {
    PlannerStallHandler handler("planning_loop");

    std::atomic<pid_t> seen_tid{0};
    handler.set_trace_capturer([&](pid_t tid, const char*) {
        seen_tid.store(tid, std::memory_order_release);
        return drone::util::TraceCaptureStatus::kOk;
    });

    auto beat = make_beat("some_other_thread", /*is_critical=*/false);
    beat.tid  = 7;
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);

    EXPECT_EQ(seen_tid.load(std::memory_order_acquire), 7);  // traced
    EXPECT_FALSE(handler.consume_event());                   // but no fault
}

// Escalation is independent of capture OUTCOME: a capturer that fails
// (kTimeout — the D-state thread we most need to LOITER for) must NOT
// suppress the FAULT_PLANNER_STALL event.  (#765 PR 2 review #5)
TEST(PlannerStallHandlerTest, CapturerFailureDoesNotBlockEscalation) {
    PlannerStallHandler handler("planning_loop");
    handler.set_trace_capturer([](pid_t, const char*) {
        return drone::util::TraceCaptureStatus::kTimeout;  // capture failed
    });
    auto beat = make_beat("planning_loop", /*is_critical=*/true);
    beat.tid  = 555;
    handler.on_stuck_for_test(beat, /*profiler=*/nullptr);
    // The stall still escalates even though the trace could not be captured.
    EXPECT_TRUE(handler.consume_event());
}
