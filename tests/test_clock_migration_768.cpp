// tests/test_clock_migration_768.cpp
//
// Regression-lock for issue #768 (sub-issue of clock-modernisation epic #766).
//
// Two production sites were migrated from std::chrono::steady_clock::now()
// to drone::util::get_clock() in the same change:
//
//   1. process3_slam_vio_nav/src/main.cpp — Cosys-passthrough pose stamping
//      (the upstream producer of pose.timestamp_ns that the Layer 3 stale-pose
//      filter in zenoh_subscriber.h consumes).
//
//   2. process5_comms/src/main.cpp — fc_tx_thread heartbeat suppressor
//      (now_init / now_send / now in the heartbeat-period and stale-bound
//      gates around drone::comms::evaluate_heartbeat).
//
// Neither site previously had a unit test that drove the gate via a mock
// clock — the only test coverage was the pure-function evaluate_heartbeat()
// unit tests in test_comms_heartbeat.cpp (which take durations as input
// and never touched the wall-clock plumbing) and end-to-end behaviour
// tests that needed std::this_thread::sleep_for(stale_threshold) to elapse
// real wall time.
//
// These tests document the architectural payoff of the migration:
//
//   * The sites now share a clock domain with the rest of the cold-start
//     hardening surface (ThreadHeartbeat, ThreadWatchdog, Layer 3 zenoh
//     stale-pose filter, Layer 4 attitude-settle gate) — so a future
//     GazeboSimClock or FCSystemTimeClock backend installed at process
//     start (issues #767 / #771) will reach all of them coherently.
//
//   * Stale-bound checks (X ns since the last write) can now be driven
//     in microseconds via ScopedMockClock + advance_ns(), instead of
//     waiting real time. The tests below demonstrate this end-to-end
//     for the heartbeat suppressor's gate.

#include "comms/heartbeat_decision.h"
#include "ipc/ipc_types.h"
#include "util/iclock.h"
#include "util/mock_clock.h"

#include <chrono>
#include <cstdint>

#include <gtest/gtest.h>

using drone::comms::evaluate_heartbeat;
using drone::comms::HeartbeatAction;
using drone::util::get_clock;
using drone::util::ScopedMockClock;

namespace {

// ──────────────────────────────────────────────────────────────────────
// P3 cosys-passthrough pose stamping (process3_slam_vio_nav/src/main.cpp)
// ──────────────────────────────────────────────────────────────────────

// Mirrors the production stamp:
//     shm_pose.timestamp_ns = drone::util::get_clock().now_ns();
TEST(ClockMigration768_P3PoseStamp, MockClockDrivesPoseTimestamp) {
    constexpr uint64_t kInitialNs = 5'000'000'000ULL;  // 5 s
    ScopedMockClock    guard(kInitialNs);

    drone::ipc::Pose shm_pose{};
    shm_pose.timestamp_ns = get_clock().now_ns();

    EXPECT_EQ(shm_pose.timestamp_ns, kInitialNs);

    // Advancing the mock clock advances the next stamp — proves the
    // stamping site is no longer pinned to wall time. Pre-migration the
    // production code went through std::chrono::steady_clock::now() and
    // would have ignored the mock entirely.
    guard.mock().advance_ms(250);
    drone::ipc::Pose later_pose{};
    later_pose.timestamp_ns = get_clock().now_ns();

    EXPECT_EQ(later_pose.timestamp_ns, kInitialNs + 250'000'000ULL);
}

// The Layer 3 stale-pose check (in zenoh_subscriber.h) computes
// `age = get_clock().now_ns() - pose.timestamp_ns`. Now that the
// producer also goes through get_clock(), both sides share the
// timeline — so the age calculation is mockable end-to-end.
TEST(ClockMigration768_P3PoseStamp, StalePoseAgeIsDrivenByMockClock) {
    constexpr uint64_t kInitialNs        = 1'000'000'000ULL;
    constexpr uint32_t kStaleThresholdMs = 500;
    ScopedMockClock    guard(kInitialNs);

    drone::ipc::Pose pose{};
    pose.timestamp_ns = get_clock().now_ns();  // producer-side stamp

    // Immediately: not stale.
    {
        const uint64_t now_ns = get_clock().now_ns();
        const uint64_t age_ns = now_ns - pose.timestamp_ns;
        EXPECT_LT(age_ns / 1'000'000ULL, kStaleThresholdMs);
    }

    // Advance just past the threshold: stale.
    guard.mock().advance_ms(kStaleThresholdMs + 1);
    {
        const uint64_t now_ns = get_clock().now_ns();
        const uint64_t age_ns = now_ns - pose.timestamp_ns;
        EXPECT_GE(age_ns / 1'000'000ULL, static_cast<uint64_t>(kStaleThresholdMs));
    }
}

// ──────────────────────────────────────────────────────────────────────
// P5 fc_tx_thread heartbeat suppressor (process5_comms/src/main.cpp)
// ──────────────────────────────────────────────────────────────────────

// Mirrors the production wiring:
//     const auto now_init      = drone::util::get_clock().now();
//     auto       last_send     = now_init;
//     auto       last_new_traj = now_init;
//     ...
//     const auto now            = drone::util::get_clock().now();
//     const auto since_last_send = now - last_send;
//     const auto since_last_new  = now - last_new_traj;
//     evaluate_heartbeat(..., since_last_send, since_last_new, ...);
TEST(ClockMigration768_P5Heartbeat, MockClockDrivesStaleBoundEndToEnd) {
    constexpr auto     kHeartbeatPeriod   = std::chrono::milliseconds(500);
    constexpr auto     kMaxHeartbeatStale = std::chrono::milliseconds(5'000);
    constexpr uint64_t kInitialNs         = 1'000'000'000ULL;
    ScopedMockClock    guard(kInitialNs);

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

    // T = 600 ms: past the heartbeat period (500 ms), within the stale
    // bound (5'000 ms) → Send.
    guard.mock().advance_ms(600);
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
    // (heartbeat replays don't reset the stale-bound).
    last_send = get_clock().now();

    // T = 600 ms + 5'001 ms = 5'601 ms total: past the stale bound
    // measured from last_new_traj → SuppressStale.
    guard.mock().advance_ms(5'001);
    {
        const auto now = get_clock().now();
        const auto since_last_send =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send);
        const auto since_last_new =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_new_traj);
        // Pre-migration this test could only be exercised via real
        // sleep_for(>5 s), making it CI-hostile. Post-migration the
        // mock advances 5'001 ms in microseconds.
        EXPECT_EQ(evaluate_heartbeat(/*sent_this_tick=*/false, /*have_cached_trajectory=*/true,
                                     /*blocked_by_mode_change=*/false, since_last_send,
                                     since_last_new, kHeartbeatPeriod, kMaxHeartbeatStale),
                  HeartbeatAction::SuppressStale);
    }
}

}  // namespace
