// common/ipc/include/ipc/isubscriber.h
// Abstract subscriber interface for typed pub-sub messaging.
// Decouples process code from the concrete transport (SHM, DDS, ZeroMQ, etc.)
#pragma once

#include <cstdint>
#include <string>

namespace drone::ipc {

/// Abstract typed subscriber — read latest message from a topic.
/// Concrete implementations: ShmSubscriber<T>, (future) DdsSubscriber<T>.
template<typename T>
class ISubscriber {
public:
    virtual ~ISubscriber() = default;

    /// Read the latest message. Returns true if a consistent read was obtained.
    /// @param out       Destination for the message payload.
    /// @param timestamp_ns  Optional output for the publisher timestamp.
    virtual bool receive(T& out, uint64_t* timestamp_ns = nullptr) const = 0;

    /// Returns true if the subscriber is connected to its data source.
    virtual bool is_connected() const = 0;

    /// Returns the topic/channel name this subscriber reads from.
    virtual const std::string& topic_name() const = 0;
};

}  // namespace drone::ipc
