// common/ipc/include/ipc/shm_publisher.h
// SHM-backed publisher — wraps ShmWriter<T> behind IPublisher<T>.
#pragma once

#include "ipc/ipublisher.h"
#include "ipc/shm_writer.h"

#include <string>

#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Publishes typed messages to a POSIX shared-memory segment using SeqLock.
template<typename T>
class ShmPublisher final : public IPublisher<T> {
public:
    /// Construct and create the SHM segment.
    /// @param topic  The SHM name (e.g. "/drone_mission_cam").
    explicit ShmPublisher(const std::string& topic) : topic_(topic) {
        ready_ = writer_.create(topic);
        if (ready_) {
            spdlog::info("[ShmPublisher] Created topic '{}'", topic);
        } else {
            spdlog::error("[ShmPublisher] Failed to create topic '{}'", topic);
        }
    }

    void publish(const T& msg) override { writer_.write(msg); }

    [[nodiscard]] const std::string& topic_name() const override { return topic_; }

    [[nodiscard]] bool is_ready() const override { return ready_; }

private:
    ShmWriter<T> writer_;
    std::string  topic_;
    bool         ready_ = false;
};

}  // namespace drone::ipc
