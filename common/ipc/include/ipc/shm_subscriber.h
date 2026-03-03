// common/ipc/include/ipc/shm_subscriber.h
// SHM-backed subscriber — wraps ShmReader<T> behind ISubscriber<T>.
#pragma once

#include "ipc/isubscriber.h"
#include "ipc/shm_reader.h"
#include "util/latency_tracker.h"

#include <chrono>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Subscribes to a POSIX shared-memory topic using SeqLock reads.
template<typename T>
class ShmSubscriber final : public ISubscriber<T> {
public:
    /// Construct and attempt to open the SHM segment.
    /// Retries up to max_retries times with retry_ms between attempts.
    /// @param topic        The SHM name (e.g. "/drone_mission_cam").
    /// @param max_retries  Number of connection attempts (0 = try once).
    /// @param retry_ms     Milliseconds between retries.
    /// @param track_latency  Enable IPC latency tracking (default: true).
    explicit ShmSubscriber(const std::string& topic, int max_retries = 50, int retry_ms = 200,
                           bool track_latency = true)
        : topic_(topic), track_latency_(track_latency) {
        for (int attempt = 0; attempt <= max_retries; ++attempt) {
            if (reader_.open(topic)) {
                spdlog::info("[ShmSubscriber] Connected to topic '{}'", topic);
                return;
            }
            if (attempt < max_retries) {
                if (attempt % 10 == 0) {
                    spdlog::warn("[ShmSubscriber] Waiting for '{}' "
                                 "(attempt {}/{})...",
                                 topic, attempt + 1, max_retries);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_ms));
            }
        }
        spdlog::warn("[ShmSubscriber] Could not connect to '{}' after "
                     "{} attempts",
                     topic, max_retries + 1);
    }

    /// Construct without connection — call connect() later.
    ShmSubscriber() = default;

    /// Manually attempt to connect (for lazy/optional subscribers).
    [[nodiscard]] bool connect(const std::string& topic) {
        topic_ = topic;
        return reader_.open(topic);
    }

    [[nodiscard]] bool receive(T& out, uint64_t* timestamp_ns = nullptr) const override {
        if (!is_connected()) return false;
        uint64_t msg_ts = 0;
        bool     ok     = reader_.read(out, &msg_ts);
        if (ok) {
            if (timestamp_ns) *timestamp_ns = msg_ts;
            if (track_latency_ && msg_ts > 0) {
                uint64_t now = drone::util::LatencyTracker::now_ns();
                if (now > msg_ts) {
                    latency_tracker_.record(now - msg_ts);
                }
            }
        }
        return ok;
    }

    [[nodiscard]] bool is_connected() const override { return reader_.is_open(); }

    [[nodiscard]] const std::string& topic_name() const override { return topic_; }

    /// Access the latency tracker for periodic reporting.
    [[nodiscard]] drone::util::LatencyTracker& latency_tracker() const { return latency_tracker_; }

    /// Log latency summary if enough samples have been collected.
    /// Returns true if a summary was logged (and the tracker was reset).
    bool log_latency_if_due(size_t min_samples = 100) const {
        if (!track_latency_) return false;
        return latency_tracker_.log_summary_if_due(topic_, min_samples);
    }

private:
    ShmReader<T>                        reader_;
    std::string                         topic_;
    bool                                track_latency_ = true;
    mutable drone::util::LatencyTracker latency_tracker_{1024};
};

}  // namespace drone::ipc
