// common/ipc/include/ipc/zenoh_subscriber.h
// Zenoh-backed subscriber — wraps zenoh::Subscriber behind ISubscriber<T>.
//
// Receives trivially-copyable T from raw bytes via Zenoh callback.
// The Zenoh callback runs on an internal Zenoh thread; receive() is called
// from the process main loop.  Thread-safety is provided by atomics.
//
// Guarded by HAVE_ZENOH.
#pragma once

#ifdef HAVE_ZENOH

#include "ipc/isubscriber.h"
#include "ipc/zenoh_session.h"

#include <zenoh.hxx>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>

#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Zenoh-backed subscriber implementing ISubscriber<T>.
/// Maintains a latest-value cache updated by the Zenoh callback thread.
template <typename T>
class ZenohSubscriber final : public ISubscriber<T> {
    static_assert(std::is_trivially_copyable_v<T>,
                  "ZenohSubscriber requires trivially copyable types");
public:
    /// Construct and declare a Zenoh subscriber on the given key expression.
    /// @param key_expr  Zenoh key expression (e.g. "drone/slam/pose").
    explicit ZenohSubscriber(const std::string& key_expr)
        : key_expr_(key_expr)
    {
        try {
            auto& session = ZenohSession::instance().session();
            subscriber_.emplace(
                session.declare_subscriber(
                    zenoh::KeyExpr(key_expr),
                    [this](zenoh::Sample& sample) {
                        on_sample(sample);
                    },
                    []() { /* on_drop — no-op */ }));
            spdlog::info("[ZenohSubscriber] Subscribed to '{}'", key_expr);
        } catch (const std::exception& e) {
            spdlog::error("[ZenohSubscriber] Failed to subscribe to '{}': {}",
                          key_expr, e.what());
        }
    }

    /// Read the latest message.
    /// @param out          Destination for the payload.
    /// @param timestamp_ns Optional output — the receive timestamp (ns).
    /// @return true if a message was available, false otherwise.
    bool receive(T& out, uint64_t* timestamp_ns = nullptr) const override {
        if (!has_data_.load(std::memory_order_acquire)) return false;

        // Copy under lock (protects against concurrent callback)
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::memcpy(&out, &latest_msg_, sizeof(T));
        if (timestamp_ns) {
            *timestamp_ns = timestamp_ns_;
        }
        return true;
    }

    /// Returns true once at least one sample has been received.
    bool is_connected() const override {
        return has_data_.load(std::memory_order_acquire);
    }

    const std::string& topic_name() const override { return key_expr_; }

private:
    /// Zenoh callback — runs on Zenoh internal thread.
    void on_sample(zenoh::Sample& sample) {
        const auto& payload = sample.get_payload();
        auto bytes = payload.as_vector();
        if (bytes.size() != sizeof(T)) {
            spdlog::warn("[ZenohSubscriber] Size mismatch on '{}': "
                         "expected {} got {}",
                         key_expr_, sizeof(T), bytes.size());
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        std::memcpy(&latest_msg_, bytes.data(), sizeof(T));
        timestamp_ns_ = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        seq_.fetch_add(1, std::memory_order_relaxed);
        has_data_.store(true, std::memory_order_release);
    }

    std::string key_expr_;
    std::optional<zenoh::Subscriber<void>> subscriber_;

    // Latest-value cache (protected by data_mutex_ + atomics)
    mutable std::mutex data_mutex_;
    T latest_msg_{};
    uint64_t timestamp_ns_{0};
    std::atomic<uint64_t> seq_{0};
    std::atomic<bool> has_data_{false};
};

}  // namespace drone::ipc

#endif  // HAVE_ZENOH
