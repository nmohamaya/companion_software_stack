// common/util/include/util/thread_watchdog.h
// Background watchdog that scans ThreadHeartbeatRegistry for stuck threads.
// ADR-004 Layer 1 — ThreadWatchdog.
//
// Runs a dedicated low-priority thread that periodically compares each
// thread's last_touch_ns against steady_clock::now().  If the delta
// exceeds stuck_threshold and the thread has been touched at least once,
// the stuck callback fires.
#pragma once

#include "util/thread_heartbeat.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::util {

class ThreadWatchdog {
public:
    /// Configuration for the watchdog thread.
    struct Config {
        /// How long a thread can go without calling touch() before being
        /// considered stuck.  Default: 5 seconds.
        std::chrono::milliseconds stuck_threshold = std::chrono::milliseconds{5000};

        /// How often the watchdog scans the heartbeat registry.
        /// Default: 1 second.
        std::chrono::milliseconds scan_interval = std::chrono::milliseconds{1000};

        Config() = default;
        Config(std::chrono::milliseconds thresh, std::chrono::milliseconds interval)
            : stuck_threshold(thresh), scan_interval(interval) {}
    };

    /// Callback signature for stuck-thread notification.
    /// Receives a copy of the ThreadHeartbeat that is stuck.
    using StuckCallback = std::function<void(const ThreadHeartbeat& beat)>;

    /// Construct and start the watchdog scan thread.
    explicit ThreadWatchdog(Config cfg) : cfg_(cfg), running_(true) {
        scan_thread_ = std::thread([this] { scan_loop(); });
    }

    /// Construct with defaults: 5s stuck threshold, 1s scan interval.
    ThreadWatchdog() : ThreadWatchdog(Config{}) {}

    /// Stop the scan thread and join.
    ~ThreadWatchdog() {
        running_.store(false, std::memory_order_relaxed);
        if (scan_thread_.joinable()) {
            scan_thread_.join();
        }
    }

    // Non-copyable, non-movable
    ThreadWatchdog(const ThreadWatchdog&)            = delete;
    ThreadWatchdog& operator=(const ThreadWatchdog&) = delete;
    ThreadWatchdog(ThreadWatchdog&&)                 = delete;
    ThreadWatchdog& operator=(ThreadWatchdog&&)      = delete;

    /// Set the callback invoked when a stuck thread is detected.
    /// Thread-safe — may be called while the watchdog is running.
    void set_stuck_callback(StuckCallback cb) {
        std::lock_guard<std::mutex> lock(cb_mutex_);
        callback_ = std::move(cb);
    }

    /// Returns the names of threads currently detected as stuck.
    /// Thread-safe snapshot.
    [[nodiscard]] std::vector<std::string> get_stuck_threads() const {
        std::lock_guard<std::mutex> lock(stuck_mutex_);
        return stuck_names_;
    }

private:
    void scan_loop() {
        while (running_.load(std::memory_order_relaxed)) {
            scan_once();

            // Sleep in small increments so we can exit quickly.
            // Use min(remaining, 50ms) so sub-50ms scan intervals are honored.
            const auto deadline = std::chrono::steady_clock::now() + cfg_.scan_interval;
            while (running_.load(std::memory_order_relaxed)) {
                const auto now       = std::chrono::steady_clock::now();
                auto       remaining = deadline - now;
                if (remaining <= std::chrono::milliseconds::zero()) {
                    break;
                }
                const auto sleep_duration = std::min(
                    remaining, std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::milliseconds(50)));
                std::this_thread::sleep_for(sleep_duration);
            }
        }
    }

    void scan_once() {
        const auto now_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());

        const auto threshold_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(cfg_.stuck_threshold).count());

        auto snap = ThreadHeartbeatRegistry::instance().snapshot();

        std::vector<std::string> new_stuck;

        // Copy callback under lock, then invoke outside to avoid deadlock
        StuckCallback cb_copy;
        {
            std::lock_guard<std::mutex> lock(cb_mutex_);
            cb_copy = callback_;
        }

        for (size_t i = 0; i < snap.count; ++i) {
            const auto&    beat = snap[i];
            const uint64_t last = beat.last_touch_ns.load(std::memory_order_relaxed);

            // Skip threads that haven't started yet (last_touch_ns == 0)
            if (last == 0) continue;

            // If the thread's timestamp is in the future (grace period),
            // it's not stuck
            if (last > now_ns) continue;

            const uint64_t delta = now_ns - last;
            if (delta > threshold_ns) {
                new_stuck.emplace_back(beat.name);

                // Log only on healthy→stuck transition to avoid log storms
                bool was_previously_stuck = false;
                {
                    std::lock_guard<std::mutex> lock(stuck_mutex_);
                    for (const auto& s : stuck_names_) {
                        if (s == beat.name) {
                            was_previously_stuck = true;
                            break;
                        }
                    }
                }
                if (!was_previously_stuck) {
                    spdlog::error("[Watchdog] Thread '{}' stuck for {:.1f}s "
                                  "(threshold: {:.1f}s, critical: {})",
                                  beat.name, static_cast<double>(delta) / 1e9,
                                  static_cast<double>(threshold_ns) / 1e9, beat.is_critical);
                }

                // Fire callback (outside cb_mutex_ to avoid deadlock)
                if (cb_copy) {
                    cb_copy(beat);
                }
            }
        }

        // Update stuck list
        {
            std::lock_guard<std::mutex> lock(stuck_mutex_);
            stuck_names_ = std::move(new_stuck);
        }
    }

    Config            cfg_;
    std::atomic<bool> running_{false};
    std::thread       scan_thread_;

    mutable std::mutex cb_mutex_;
    StuckCallback      callback_;

    mutable std::mutex       stuck_mutex_;
    std::vector<std::string> stuck_names_;
};

}  // namespace drone::util
