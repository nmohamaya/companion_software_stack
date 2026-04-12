// common/ipc/include/ipc/zenoh_subscriber.h
// Zenoh-backed subscriber — wraps zenoh::Subscriber behind ISubscriber<T>.
//
// Receives T from raw bytes via Zenoh callback, using a pluggable
// ISerializer<T> for wire-format decoding.  The Zenoh callback runs on
// an internal Zenoh thread; receive() is called from the process main
// loop.  Thread-safety is provided by atomics + mutex.
//
// The serializer defaults to RawSerializer<T> (byte-identical to the
// previous inline reinterpret_cast + std::copy), preserving full backward
// compatibility.
//
// Guarded by HAVE_ZENOH.
#pragma once


#include "ipc/iserializer.h"
#include "ipc/isubscriber.h"
#include "ipc/raw_serializer.h"
#include "ipc/zenoh_session.h"
#include "util/iclock.h"
#include "util/ilogger.h"
#include "util/latency_tracker.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include <zenoh.hxx>

namespace drone::ipc {

/// Zenoh-backed subscriber implementing ISubscriber<T>.
/// Maintains a latest-value cache updated by the Zenoh callback thread.
template<typename T>
class ZenohSubscriber final : public ISubscriber<T> {
public:
    /// Construct and declare a Zenoh subscriber on the given key expression.
    /// @param key_expr        Zenoh key expression (e.g. "drone/slam/pose").
    /// @param track_latency   Enable IPC latency tracking (default: true).
    /// @param serializer      Optional serializer (default: RawSerializer<T>).
    ///                        shared_ptr because MessageBus may share one
    ///                        instance across multiple subscribers for the
    ///                        same type.
    explicit ZenohSubscriber(
        const std::string& key_expr, bool track_latency = true,
        std::shared_ptr<const ISerializer<T>> serializer = std::make_shared<RawSerializer<T>>())
        : key_expr_(key_expr), track_latency_(track_latency), serializer_(std::move(serializer)) {
        try {
            auto& session = ZenohSession::instance().session();
            subscriber_.emplace(session.declare_subscriber(
                zenoh::KeyExpr(key_expr), [this](zenoh::Sample& sample) { on_sample(sample); },
                []() { /* on_drop — no-op */ }));
            DRONE_LOG_INFO("[ZenohSubscriber] Subscribed to '{}' (serializer={})", key_expr,
                           serializer_->name());
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[ZenohSubscriber] Failed to subscribe to '{}': {}", key_expr,
                            e.what());
        }
    }

    /// Read the latest message.
    /// @param out          Destination for the payload.
    /// @param timestamp_ns Optional output — the time (steady_clock, ns)
    ///                     at which the Zenoh callback delivered the sample.
    ///                     NOTE: this is the *callback arrival* time, not
    ///                     the publisher's send time.  ZenohPublisher does
    ///                     not embed a send timestamp, so the latency
    ///                     recorded here reflects callback→receive() delay
    ///                     (consumer polling lag), NOT wire latency.
    /// @return true if a message was available, false otherwise.
    [[nodiscard]] bool receive(T& out, uint64_t* timestamp_ns = nullptr) const override {
        if (!has_data_.load(std::memory_order_acquire)) return false;

        // Copy under lock (protects against concurrent callback)
        uint64_t msg_ts = 0;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            out    = latest_msg_;
            msg_ts = timestamp_ns_;
        }
        if (timestamp_ns) {
            *timestamp_ns = msg_ts;
        }
        if (track_latency_ && msg_ts > 0) {
            uint64_t now = drone::util::LatencyTracker::now_ns();
            if (now > msg_ts) {
                latency_tracker_.record(now - msg_ts);
            }
        }
        return true;
    }

    /// Returns true if the Zenoh subscriber was successfully declared.
    /// Unlike POSIX SHM (which requires the segment to exist), Zenoh
    /// subscriptions are valid immediately — data arrives asynchronously
    /// once a publisher appears.  Use receive() to check for actual data.
    [[nodiscard]] bool is_connected() const override { return subscriber_.has_value(); }

    [[nodiscard]] const std::string& topic_name() const override { return key_expr_; }

    /// Access the latency tracker for periodic reporting.
    [[nodiscard]] drone::util::LatencyTracker& latency_tracker() const { return latency_tracker_; }

    /// Log latency summary if enough samples have been collected.
    bool log_latency_if_due(size_t min_samples = 100) const override {
        if (!track_latency_) return false;
        return latency_tracker_.log_summary_if_due(key_expr_, min_samples);
    }

private:
    /// Zenoh callback — runs on Zenoh internal thread.
    /// Stamps timestamp_ns_ with the arrival time so that receive()
    /// can measure *callback→poll* delay (not true wire latency).
    /// Rejects messages that fail structural validation (Issues #179, #181, #185).
    void on_sample(zenoh::Sample& sample) {
        const auto& payload = sample.get_payload();
        auto        bytes   = payload.as_vector();

        // Validate into a heap-allocated temporary before committing to
        // latest_msg_, so a failed validation never overwrites a previously
        // good value.  Heap allocation avoids stack overflow for large types
        // (e.g. VideoFrame ~6.2 MB) on Zenoh's callback thread.
        if constexpr (has_validate<T>::value) {
            auto temp = std::make_unique<T>();
            if (!serializer_->deserialize(bytes.data(), bytes.size(), *temp)) {
                DRONE_LOG_WARN("[ZenohSubscriber] Deserialization failed on '{}': "
                               "expected {} got {}",
                               key_expr_, sizeof(T), bytes.size());
                return;
            }
            if (!temp->validate()) {
                DRONE_LOG_WARN("[ZenohSubscriber] Validation failed on '{}' — "
                               "dropping message",
                               key_expr_);
                return;
            }
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_msg_   = *temp;
            timestamp_ns_ = drone::util::get_clock().now_ns();
        } else {
            // For types without validate(), deserialize directly into latest_msg_
            // under lock.  A stack temporary is NOT safe here — large types like
            // VideoFrame (~6.2 MB) would overflow Zenoh's callback thread stack.
            // The serializer checks size first, so partial writes on failure are
            // acceptable (the old value was already overwritten by design).
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!serializer_->deserialize(bytes.data(), bytes.size(), latest_msg_)) {
                DRONE_LOG_WARN("[ZenohSubscriber] Size mismatch on '{}': "
                               "expected {} got {}",
                               key_expr_, sizeof(T), bytes.size());
                return;
            }
            timestamp_ns_ = drone::util::get_clock().now_ns();
        }

        seq_.fetch_add(1, std::memory_order_relaxed);
        has_data_.store(true, std::memory_order_release);
    }

    /// SFINAE helper to detect T::validate() at compile time.
    template<typename U, typename = void>
    struct has_validate : std::false_type {};

    template<typename U>
    struct has_validate<U, std::void_t<decltype(std::declval<const U&>().validate())>>
        : std::true_type {};

    std::string                            key_expr_;
    bool                                   track_latency_ = true;
    std::shared_ptr<const ISerializer<T>>  serializer_;
    std::optional<zenoh::Subscriber<void>> subscriber_;

    // Latest-value cache (protected by data_mutex_ + atomics)
    mutable std::mutex                  data_mutex_;
    T                                   latest_msg_{};
    uint64_t                            timestamp_ns_{0};
    std::atomic<uint64_t>               seq_{0};
    std::atomic<bool>                   has_data_{false};
    mutable drone::util::LatencyTracker latency_tracker_{1024};
};

}  // namespace drone::ipc
