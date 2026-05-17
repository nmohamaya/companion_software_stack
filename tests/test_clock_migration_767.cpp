// tests/test_clock_migration_767.cpp
//
// Documentation tests for issue #767 (Clock #2: tactical migration of
// P3 cosys-passthrough pose stamping + P5 fc_tx_thread heartbeat suppressor
// + P4 pose-staleness consumer to drone::util::get_clock(); sub-issue of
// clock-modernisation epic #766).
//
// Honest scope statement
// ──────────────────────
// These tests do NOT regression-lock the migrated production lines.  The
// production sites in process3_slam_vio_nav/src/main.cpp,
// process5_comms/src/main.cpp, and process4_mission_planner/src/main.cpp
// are inside long thread loops that are not extracted into separately-
// linkable functions, so a unit test cannot detect a revert from
// `drone::util::get_clock()` back to `std::chrono::steady_clock::now()` at
// any specific call site.  The PR review for #767 surfaced this gap
// (review-test-quality + Copilot, multiple-agent agreement); the honest
// fix is the framing here, not a fake regression-lock.
//
// What these tests DO verify
// ──────────────────────────
//   * The IClock abstraction can be installed via ScopedMockClock and
//     used to drive both producer-side (P3 pose stamp) and consumer-side
//     (P4 FaultManager pose-staleness check) of the same fault path
//     deterministically in microseconds, without sleep_for.
//
//   * Three production threads now share one clock domain for this fault
//     path: the cosys-passthrough P3 producer, the P5 fc_tx heartbeat
//     suppressor, and the P4 FaultManager consumer.  Pre-migration the
//     consumer compared a get_clock()-stamped producer value against
//     a raw std::chrono::steady_clock::now() consumer value — coherent
//     in production (both reduce to the same syscall) but split under
//     any future MockClock / GazeboSimClock (#769) / FCSystemTimeClock
//     (#770) backend.
//
// Real regression coverage for the call sites would require either:
//   * A static text-grep CI gate (tracked under the bulk-migration
//     enforcement issue, clock-modernisation epic #766).
//   * An integration test running each process with mockable instrumentation.
//   * A refactor extracting the time source into a separately-testable seam.
// All three are out of scope for the tactical migration tracked here.

#include "comms/heartbeat_decision.h"
#include "ipc/ipc_types.h"
#include "planner/fault_manager.h"
#include "util/iclock.h"
#include "util/mock_clock.h"

#include <chrono>
#include <cstdint>

#include <gtest/gtest.h>

using drone::comms::evaluate_heartbeat;
using drone::comms::HeartbeatAction;
using drone::planner::FaultManager;
using drone::util::get_clock;
using drone::util::ScopedMockClock;

namespace {

// ──────────────────────────────────────────────────────────────────────
// Shared test fixture (DRY — addresses review-code-quality finding on #767)
// ──────────────────────────────────────────────────────────────────────

constexpr uint64_t kInitialNs = 1'000'000'000ULL;  // 1 s

class ClockMigration767Test : public ::testing::Test {
protected:
    ScopedMockClock guard_{kInitialNs};
};

// ──────────────────────────────────────────────────────────────────────
// P3 cosys-passthrough pose stamping (process3_slam_vio_nav/src/main.cpp:376)
// ──────────────────────────────────────────────────────────────────────

// Mirrors the production stamp:
//     shm_pose.timestamp_ns = drone::util::get_clock().now_ns();
TEST_F(ClockMigration767Test, P3PoseStampReadsActiveClock) {
    drone::ipc::Pose shm_pose{};
    shm_pose.timestamp_ns = get_clock().now_ns();
    EXPECT_EQ(shm_pose.timestamp_ns, kInitialNs);

    guard_.mock().advance_ms(250);
    drone::ipc::Pose later_pose{};
    later_pose.timestamp_ns = get_clock().now_ns();
    EXPECT_EQ(later_pose.timestamp_ns, kInitialNs + 250'000'000ULL);
}

// ──────────────────────────────────────────────────────────────────────
// End-to-end producer (P3) ↔ consumer (P4 FaultManager) coherence
// ──────────────────────────────────────────────────────────────────────
//
// Pre-migration only the producer side (cosys-passthrough pose stamp)
// went through get_clock().  The consumer in P4 main.cpp:887 used raw
// std::chrono::steady_clock::now(), so an age computation under MockClock
// would be (real_wall - mocked_stamp) — meaningless.  This PR migrates
// both sides; this test exercises the actual FaultManager.evaluate code
// path (not the inline check in main.cpp's loop, which is not extracted).
TEST_F(ClockMigration767Test, P4FaultManagerStaleCheckIsMockClockDriven) {
    // Configure a 500 ms pose-stale timeout (matches FaultManager default).
    drone::planner::FaultConfig cfg;
    cfg.pose_stale_timeout_ns = 500'000'000ULL;
    FaultManager mgr(cfg);

    drone::ipc::SystemHealth health{};
    drone::ipc::FCState      fc_state{};
    fc_state.armed     = true;
    fc_state.connected = true;

    const uint64_t pose_ts = get_clock().now_ns();  // producer-side stamp

    // T = 0: not stale (age == 0 < 500 ms threshold).
    {
        const uint64_t now_ns = get_clock().now_ns();
        const auto     state  = mgr.evaluate(health, fc_state, pose_ts, now_ns, /*pose_quality=*/2);
        EXPECT_EQ(state.active_faults & drone::planner::FAULT_POSE_STALE, 0u)
            << "Pose should not be stale at age=0; producer/consumer clocks must be coherent";
    }

    // T = 501 ms: just past the threshold → FAULT_POSE_STALE fires.
    guard_.mock().advance_ms(501);
    {
        const uint64_t now_ns = get_clock().now_ns();
        const auto     state  = mgr.evaluate(health, fc_state, pose_ts, now_ns, /*pose_quality=*/2);
        EXPECT_NE(state.active_faults & drone::planner::FAULT_POSE_STALE, 0u)
            << "Pose should be stale at age=501ms; if this fails the producer/consumer "
               "clocks have desynced (the bug this PR exists to prevent)";
    }
}

// ──────────────────────────────────────────────────────────────────────
// P5 fc_tx_thread heartbeat suppressor (process5_comms/src/main.cpp)
// ──────────────────────────────────────────────────────────────────────
//
// Mirrors the production wiring exactly (constants taken from
// process5_comms/src/main.cpp:154-155):
//   constexpr std::chrono::milliseconds kHeartbeatPeriod{40};
//   constexpr std::chrono::milliseconds kMaxHeartbeatStaleMs{5000};
//
// Pre-migration, exercising the (None → Send → SuppressStale) sequence
// for these specific gates required std::this_thread::sleep_for(>5040 ms)
// of real wall time.  Post-migration the same sequence runs in
// microseconds via ScopedMockClock + advance_ms().
TEST_F(ClockMigration767Test, P5HeartbeatStaleBoundIsMockClockDriven) {
    constexpr auto kHeartbeatPeriod   = std::chrono::milliseconds(40);
    constexpr auto kMaxHeartbeatStale = std::chrono::milliseconds(5'000);

    const auto                            now_init      = get_clock().now();
    std::chrono::steady_clock::time_point last_send     = now_init;
    std::chrono::steady_clock::time_point last_new_traj = now_init;

    // T = 0 ms: nothing has been sent, nothing is stale → None.
    {
        const auto now = get_clock().now();
        const auto since_last_send =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send);
        const auto since_last_new =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_new_traj);
        EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false, /*have_cached_trajectory=*/true,
                                     /*blocked_by_mode_change=*/false, since_last_send,
                                     since_last_new, kHeartbeatPeriod, kMaxHeartbeatStale),
                  HeartbeatAction::None);
    }

    // T = 50 ms: past the heartbeat period (40 ms), within the stale
    // bound (5'000 ms) → Send.  Pre-migration this test could never use
    // a 50 ms wall delay reliably under CI load; mock-clock makes it deterministic.
    guard_.mock().advance_ms(50);
    {
        const auto now = get_clock().now();
        const auto since_last_send =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send);
        const auto since_last_new =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_new_traj);
        EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false, /*have_cached_trajectory=*/true,
                                     /*blocked_by_mode_change=*/false, since_last_send,
                                     since_last_new, kHeartbeatPeriod, kMaxHeartbeatStale),
                  HeartbeatAction::Send);
    }

    // Send the heartbeat: last_send moves forward, last_new_traj does NOT
    // (heartbeat replays don't reset the stale-bound — the bug PR #684
    // fixed in production was that the original single-timestamp design
    // never tripped the stale bound under heartbeat replay).
    last_send = get_clock().now();

    // T = 50 + 4'951 = 5'001 ms: past the stale bound measured from
    // last_new_traj → SuppressStale.  Pre-migration this exercise would
    // have required a real >5 s sleep_for and made CI flaky on slow runners.
    guard_.mock().advance_ms(4'951);
    {
        const auto now = get_clock().now();
        const auto since_last_send =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send);
        const auto since_last_new =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_new_traj);
        EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false, /*have_cached_trajectory=*/true,
                                     /*blocked_by_mode_change=*/false, since_last_send,
                                     since_last_new, kHeartbeatPeriod, kMaxHeartbeatStale),
                  HeartbeatAction::SuppressStale);
    }
}

}  // namespace
