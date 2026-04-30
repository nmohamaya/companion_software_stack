// common/ipc/include/ipc/isubscriber.h
// Abstract subscriber interface for typed pub-sub messaging.
// Decouples process code from the concrete transport (Zenoh, DDS, etc.)
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace drone::ipc {

/// Abstract typed subscriber — read latest message from a topic.
/// Concrete implementations: ZenohSubscriber<T>, (future) DdsSubscriber<T>.
template<typename T>
class ISubscriber {
public:
    virtual ~ISubscriber() = default;

    /// Read the latest message. Returns true if a consistent read was obtained.
    /// @param out       Destination for the message payload.
    /// @param timestamp_ns  Optional output for the publisher timestamp.
    ///
    /// **Threading contract**: receive() must be called from a SINGLE
    /// CONSUMER THREAD per subscriber instance.  Implementations
    /// (ZenohSubscriber etc.) may use relaxed-memory-order self-suppression
    /// state (e.g. last_recorded_ts_) that assumes single-consumer access.
    /// Calling receive() concurrently from multiple threads on the same
    /// subscriber is undefined behaviour.  Issue #645 review fix (#644 P2).
    [[nodiscard]] virtual bool receive(T& out, uint64_t* timestamp_ns = nullptr) const = 0;

    /// Returns true if the subscriber is connected to its data source.
    [[nodiscard]] virtual bool is_connected() const = 0;

    /// Returns the topic/channel name this subscriber reads from.
    [[nodiscard]] virtual const std::string& topic_name() const = 0;

    /// Log latency summary if enough samples have been collected.
    /// Default: no-op (returns false). ZenohSubscriber overrides with real tracking.
    /// @return true if a summary was logged. Return value is informational
    ///         (fire-and-forget pattern) — not [[nodiscard]].
    virtual bool log_latency_if_due(size_t /*min_samples*/ = 100) const { return false; }
};

}  // namespace drone::ipc
