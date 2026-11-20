// tests/test_perception_drain.cpp
// Unit tests for per-stage graceful pipeline drain on P2 shutdown (Issue #446).
//
// These tests verify that the phased shutdown mechanism works correctly:
//   Phase 0 = running, 1 = inference stopped, 2 = tracker stopped,
//   3 = fusion stopped, 4 = all stopped.
//
// Each test uses std::atomic<int> shutdown_phase and TripleBuffer to simulate
// the pipeline stages without needing real HAL backends or IPC.

#include "perception/drain_timeout.h"
#include "util/mock_clock.h"
#include "util/triple_buffer.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using drone::TripleBuffer;

// ── Test 1: Phased shutdown ordering ────────────────────────
// Verify that phase transitions happen in strict order and each
// simulated stage stops only when its phase threshold is reached.
TEST(PerceptionDrainTest, PhasedShutdownOrdering) {
    std::atomic<int> shutdown_phase{0};

    // Record the order in which stages observe their stop condition.
    // Each element: 1=inference, 2=tracker, 3=fusion
    std::vector<int> stop_order;
    std::mutex       order_mutex;

    auto record_stop = [&](int stage_id) {
        std::lock_guard<std::mutex> lock(order_mutex);
        stop_order.push_back(stage_id);
    };

    // Simulated inference thread: stops at phase >= 1
    std::thread t_inference([&] {
        while (shutdown_phase.load(std::memory_order_acquire) < 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        record_stop(1);
    });

    // Simulated tracker thread: stops at phase >= 2
    std::thread t_tracker([&] {
        while (shutdown_phase.load(std::memory_order_acquire) < 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        record_stop(2);
    });

    // Simulated fusion thread: stops at phase >= 3
    std::thread t_fusion([&] {
        while (shutdown_phase.load(std::memory_order_acquire) < 3) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        record_stop(3);
    });

    // Execute phased shutdown — same sequence as main.cpp
    shutdown_phase.store(1, std::memory_order_release);
    t_inference.join();

    shutdown_phase.store(2, std::memory_order_release);
    t_tracker.join();

    shutdown_phase.store(3, std::memory_order_release);
    t_fusion.join();

    // Verify strict ordering: inference -> tracker -> fusion
    ASSERT_EQ(stop_order.size(), 3u);
    EXPECT_EQ(stop_order[0], 1);
    EXPECT_EQ(stop_order[1], 2);
    EXPECT_EQ(stop_order[2], 3);
}

// ── Test 2: Tracker drains buffered data before stopping ────
// When inference stops (phase 1), the tracker should consume any
// remaining data in the TripleBuffer before exiting.
TEST(PerceptionDrainTest, TrackerDrainsBeforeStopping) {
    std::atomic<int>  shutdown_phase{0};
    TripleBuffer<int> inference_to_tracker;

    uint64_t          items_processed = 0;
    std::atomic<bool> exited_via_drain{false};

    // Write data to the buffer BEFORE starting the tracker
    inference_to_tracker.write(42);

    // Simulated tracker: reads from buffer, drains on phase >= 1
    std::thread t_tracker([&] {
        while (shutdown_phase.load(std::memory_order_acquire) < 2) {
            const bool draining = (shutdown_phase.load(std::memory_order_acquire) >= 1);

            auto val = inference_to_tracker.read();
            if (val.has_value()) {
                ++items_processed;
            } else if (draining) {
                // No data left and upstream is done — exit via drain path
                exited_via_drain.store(true, std::memory_order_release);
                break;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    });

    // Allow tracker to start running
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Phase 1: stop inference — tracker enters drain mode.
    // Do NOT immediately set phase 2 — give the drain path time to trigger.
    shutdown_phase.store(1, std::memory_order_release);

    t_tracker.join();

    // Tracker must have processed the buffered item AND exited via drain path
    // (not via the outer while condition reaching phase >= 2).
    EXPECT_GE(items_processed, 1u);
    EXPECT_TRUE(exited_via_drain.load(std::memory_order_acquire));
}

// ── Test 3: Drain timeout prevents hanging ──────────────────
// If the TripleBuffer somehow keeps producing data during drain,
// the drain timeout should force the thread to exit.
TEST(PerceptionDrainTest, DrainTimeoutPreventsHanging) {
    drone::util::ScopedMockClock clock;
    const auto                   start   = clock.mock().now();
    constexpr auto               timeout = std::chrono::milliseconds(100);
    TripleBuffer<int>            buffer;

    // Supply data deterministically on each iteration so the production
    // timeout predicate is tested independently of the scheduler.
    for (int elapsed_ms = 0; elapsed_ms < timeout.count(); ++elapsed_ms) {
        buffer.write(elapsed_ms);
        ASSERT_TRUE(buffer.read().has_value());
        EXPECT_FALSE(drone::perception::drain_timed_out(start, timeout));
        clock.mock().advance_ms(1);
    }
    buffer.write(42);
    ASSERT_TRUE(buffer.read().has_value());
    EXPECT_TRUE(drone::perception::drain_timed_out(start, timeout));
}

// ── Test 4: Fusion drains after tracker stops ───────────────
// Verify fusion consumes buffered tracked data when tracker stops.
TEST(PerceptionDrainTest, FusionDrainsAfterTrackerStops) {
    std::atomic<int>  shutdown_phase{0};
    TripleBuffer<int> tracker_to_fusion;

    uint64_t fusion_items_processed = 0;

    // Buffer data before fusion sees the drain signal
    tracker_to_fusion.write(100);

    // Simulated fusion: reads from buffer, drains on phase >= 2
    std::thread t_fusion([&] {
        bool fusion_draining    = false;
        auto fusion_drain_start = std::chrono::steady_clock::now();

        while (shutdown_phase.load(std::memory_order_acquire) < 3) {
            if (!fusion_draining && shutdown_phase.load(std::memory_order_acquire) >= 2) {
                fusion_draining    = true;
                fusion_drain_start = std::chrono::steady_clock::now();
            }

            auto val = tracker_to_fusion.read();
            if (val.has_value()) {
                ++fusion_items_processed;
            } else if (fusion_draining) {
                break;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    });

    // Allow fusion to start
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Phase 2: tracker stopped — fusion enters drain mode
    shutdown_phase.store(2, std::memory_order_release);

    // Give fusion time to drain
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Phase 3: allow fusion to exit
    shutdown_phase.store(3, std::memory_order_release);
    t_fusion.join();

    // Fusion must have processed the buffered item
    EXPECT_GE(fusion_items_processed, 1u);
}

// ── Test 5: Optional threads handle phase correctly ─────────
// Verify that depth/radar threads (which stop at phase >= 3)
// exit promptly when phase 3 is reached, even without drain.
TEST(PerceptionDrainTest, OptionalThreadsStopAtPhase3) {
    std::atomic<int> shutdown_phase{0};

    std::atomic<bool> depth_stopped{false};
    std::atomic<bool> radar_stopped{false};

    // Simulated depth thread: stops at phase >= 3
    std::thread t_depth([&] {
        while (shutdown_phase.load(std::memory_order_acquire) < 3) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        depth_stopped.store(true, std::memory_order_release);
    });

    // Simulated radar thread: stops at phase >= 3
    std::thread t_radar([&] {
        while (shutdown_phase.load(std::memory_order_acquire) < 3) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        radar_stopped.store(true, std::memory_order_release);
    });

    // Phases 1 and 2 should not stop depth/radar
    shutdown_phase.store(1, std::memory_order_release);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(depth_stopped.load(std::memory_order_acquire));
    EXPECT_FALSE(radar_stopped.load(std::memory_order_acquire));

    shutdown_phase.store(2, std::memory_order_release);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(depth_stopped.load(std::memory_order_acquire));
    EXPECT_FALSE(radar_stopped.load(std::memory_order_acquire));

    // Phase 3 should stop them
    shutdown_phase.store(3, std::memory_order_release);
    t_depth.join();
    t_radar.join();

    EXPECT_TRUE(depth_stopped.load(std::memory_order_acquire));
    EXPECT_TRUE(radar_stopped.load(std::memory_order_acquire));
}
