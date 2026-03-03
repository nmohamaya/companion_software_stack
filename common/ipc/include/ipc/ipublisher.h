// common/ipc/include/ipc/ipublisher.h
// Abstract publisher interface for typed pub-sub messaging.
// Decouples process code from the concrete transport (SHM, DDS, ZeroMQ, etc.)
#pragma once

#include <string>

namespace drone::ipc {

/// Abstract typed publisher — write one message at a time.
/// Concrete implementations: ShmPublisher<T>, (future) DdsPublisher<T>.
template<typename T>
class IPublisher {
public:
    virtual ~IPublisher() = default;

    /// Publish a message. Non-blocking, overwrites previous value.
    virtual void publish(const T& msg) = 0;

    /// Returns the topic/channel name this publisher writes to.
    [[nodiscard]] virtual const std::string& topic_name() const = 0;

    /// Returns true if the publisher is ready to accept data.
    [[nodiscard]] virtual bool is_ready() const = 0;
};

}  // namespace drone::ipc
