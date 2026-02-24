// common/ipc/include/ipc/shm_subscriber.h
// SHM-backed subscriber — wraps ShmReader<T> behind ISubscriber<T>.
#pragma once

#include "ipc/isubscriber.h"
#include "ipc/shm_reader.h"

#include <string>
#include <chrono>
#include <thread>
#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Subscribes to a POSIX shared-memory topic using SeqLock reads.
template <typename T>
class ShmSubscriber final : public ISubscriber<T> {
public:
    /// Construct and attempt to open the SHM segment.
    /// Retries up to max_retries times with retry_ms between attempts.
    /// @param topic        The SHM name (e.g. "/drone_mission_cam").
    /// @param max_retries  Number of connection attempts (0 = try once).
    /// @param retry_ms     Milliseconds between retries.
    explicit ShmSubscriber(const std::string& topic,
                           int max_retries = 50,
                           int retry_ms = 200)
        : topic_(topic)
    {
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
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(retry_ms));
            }
        }
        spdlog::warn("[ShmSubscriber] Could not connect to '{}' after "
                     "{} attempts", topic, max_retries + 1);
    }

    /// Construct without connection — call connect() later.
    ShmSubscriber() = default;

    /// Manually attempt to connect (for lazy/optional subscribers).
    bool connect(const std::string& topic) {
        topic_ = topic;
        return reader_.open(topic);
    }

    bool receive(T& out, uint64_t* timestamp_ns = nullptr) const override {
        return reader_.read(out, timestamp_ns);
    }

    bool is_connected() const override { return reader_.is_open(); }

    const std::string& topic_name() const override { return topic_; }

private:
    ShmReader<T> reader_;
    std::string topic_;
};

}  // namespace drone::ipc
