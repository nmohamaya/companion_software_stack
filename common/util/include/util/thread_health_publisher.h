// common/util/include/util/thread_health_publisher.h
// Bridges ThreadHeartbeatRegistry + ThreadWatchdog → ShmThreadHealth → IPublisher.
//
// NOTE: Consumers must link both drone_util and drone_ipc, since this header
//       references types from both libraries (both are header-only INTERFACE
//       targets, so no CMake cross-dependency is required).
#pragma once

#include "ipc/shm_types.h"
#include "util/safe_name_copy.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <algorithm>
#include <chrono>
#include <cstring>

namespace drone::util {

/// Publishes ThreadHeartbeatRegistry snapshots as ShmThreadHealth via the
/// provided IPublisher.  Automatically marks threads that the watchdog has
/// flagged as stuck with `healthy = false`.
///
/// Usage in a process's main loop:
///
///   ThreadHealthPublisher health_pub(publisher, "video_capture", watchdog);
///   while (running) {
///       // ... normal work ...
///       health_pub.publish_snapshot();
///       sleep(1);
///   }
template<typename Publisher>
class ThreadHealthPublisher {
public:
    /// @param pub       Reference to an IPublisher<ShmThreadHealth>.
    /// @param process   Null-terminated process name (max 31 chars).
    /// @param watchdog  Reference to the process's ThreadWatchdog instance.
    ThreadHealthPublisher(Publisher& pub, const char* process, const ThreadWatchdog& watchdog)
        : pub_(pub), watchdog_(watchdog) {
        safe_name_copy(process_name_, process);
    }

    /// Take a snapshot of the heartbeat registry, cross-reference with
    /// watchdog stuck-thread detection, and publish the result.
    void publish_snapshot() {
        drone::ipc::ShmThreadHealth health{};
        std::memcpy(health.process_name, process_name_, sizeof(health.process_name));

        // Snapshot registered heartbeats
        auto beats = ThreadHeartbeatRegistry::instance().snapshot();
        auto stuck = watchdog_.get_stuck_threads();

        // Use beats.size() — not a separate count() call — to avoid a
        // TOCTOU race where a concurrent register_thread() bumps count_
        // past beats.size(), causing an out-of-bounds read.
        health.num_threads = static_cast<uint8_t>(
            std::min(beats.size(), static_cast<size_t>(drone::ipc::kMaxTrackedThreads)));

        for (uint8_t i = 0; i < health.num_threads; ++i) {
            auto& dst = health.threads[i];
            auto& src = beats[i];

            std::memset(dst.name, 0, sizeof(dst.name));
            std::memcpy(dst.name, src.name, sizeof(dst.name));
            dst.critical = src.is_critical;
            dst.last_ns  = src.last_touch_ns.load(std::memory_order_relaxed);

            // A thread is unhealthy if the watchdog flagged it as stuck
            dst.healthy = true;
            for (const auto& s : stuck) {
                if (std::strncmp(s.c_str(), src.name, sizeof(src.name)) == 0) {
                    dst.healthy = false;
                    break;
                }
            }
        }

        health.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());

        pub_.publish(health);
    }

private:
    Publisher&            pub_;
    const ThreadWatchdog& watchdog_;
    char                  process_name_[32] = {};
};

}  // namespace drone::util
