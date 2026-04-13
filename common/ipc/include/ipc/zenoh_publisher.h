// common/ipc/include/ipc/zenoh_publisher.h
// Zenoh-backed publisher — wraps zenoh::Publisher behind IPublisher<T>.
//
// Publishes T via Zenoh using a pluggable ISerializer<T> for wire-format
// encoding.  For large messages (sizeof(T) > kShmPublishThreshold, e.g.
// video frames), the payload is written directly into a Zenoh SHM buffer
// — zero-copy for local subscribers.  Small messages use the regular
// Bytes path.
//
// The serializer defaults to RawSerializer<T> (byte-identical to the
// previous inline reinterpret_cast + std::copy), preserving full backward
// compatibility.  A future ProtobufSerializer<T> can be injected for
// cross-language interop without changing transport code.
//
// Guarded by HAVE_ZENOH.
#pragma once


#include "ipc/ipublisher.h"
#include "ipc/iserializer.h"
#include "ipc/raw_serializer.h"
#include "ipc/zenoh_session.h"
#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <zenoh.hxx>

namespace drone::ipc {

/// Zenoh-backed publisher implementing IPublisher<T>.
template<typename T>
class ZenohPublisher final : public IPublisher<T> {
public:
    /// Construct and declare a Zenoh publisher on the given key expression.
    /// @param key_expr    Zenoh key expression (e.g. "drone/slam/pose").
    /// @param serializer  Optional serializer (default: RawSerializer<T>).
    ///                    shared_ptr because MessageBus may share one instance
    ///                    across multiple publishers for the same type.
    explicit ZenohPublisher(
        const std::string&                    key_expr,
        std::shared_ptr<const ISerializer<T>> serializer = std::make_shared<RawSerializer<T>>())
        : key_expr_(key_expr)
        , serializer_(serializer ? std::move(serializer) : std::make_shared<RawSerializer<T>>()) {
        try {
            auto& session = ZenohSession::instance().session();
            publisher_.emplace(session.declare_publisher(zenoh::KeyExpr(key_expr)));
            ready_.store(true, std::memory_order_release);
            DRONE_LOG_INFO("[ZenohPublisher] Declared on '{}' "
                           "(size={}, shm={}, serializer={})",
                           key_expr, sizeof(T), sizeof(T) > kShmPublishThreshold ? "yes" : "no",
                           serializer_->name());
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[ZenohPublisher] Failed to declare on '{}': {}", key_expr, e.what());
            ready_.store(false, std::memory_order_release);
        }
    }

    void publish(const T& msg) override {
        if (!ready_.load(std::memory_order_acquire) || !publisher_.has_value()) return;

        if constexpr (sizeof(T) > kShmPublishThreshold) {
            publish_shm(msg);
            // relaxed: diagnostic-only counter, no dependent data
            shm_publishes_.fetch_add(1, std::memory_order_relaxed);
        } else {
            publish_bytes(msg);
            // relaxed: diagnostic-only counter, no dependent data
            bytes_publishes_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    /// Number of publishes that used the SHM zero-copy path.
    [[nodiscard]] uint64_t shm_publish_count() const {
        return shm_publishes_.load(std::memory_order_relaxed);
    }

    /// Number of publishes that used the standard bytes path.
    [[nodiscard]] uint64_t bytes_publish_count() const {
        return bytes_publishes_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] const std::string& topic_name() const override { return key_expr_; }

    [[nodiscard]] bool is_ready() const override { return ready_.load(std::memory_order_acquire); }

private:
    /// Small-message path: serialize to vector<uint8_t>.
    void publish_bytes(const T& msg) {
        auto buf = serializer_->serialize(msg);
        publisher_->put(zenoh::Bytes(std::move(buf)));
    }

    /// Large-message path: allocate SHM buffer, serialize into it, publish zero-copy.
    void publish_shm(const T& msg) {
        auto* provider = ZenohSession::instance().shm_provider();
        if (!provider) {
            // Fallback to bytes path if SHM provider is unavailable
            publish_bytes(msg);
            return;
        }

        const size_t needed = serializer_->serialized_size(msg);
        auto         result = provider->alloc_gc_defrag_blocking(needed);
        auto*        buf    = std::get_if<zenoh::ZShmMut>(&result);
        if (!buf) {
            DRONE_LOG_WARN("[ZenohPublisher] SHM alloc failed on '{}', "
                           "falling back to bytes",
                           key_expr_);
            publish_bytes(msg);
            return;
        }

        // Zero-copy write: serialize directly into shared memory
        const size_t written = serializer_->serialize(msg, buf->data(), needed);
        if (written == 0 || written != needed) {
            DRONE_LOG_WARN("[ZenohPublisher] SHM serialization produced {} bytes, expected {} "
                           "on '{}'; falling back to bytes",
                           written, needed, key_expr_);
            publish_bytes(msg);
            return;
        }

        // Move SHM buffer into Bytes and publish — only a descriptor
        // is sent over the transport; local subscribers receive a
        // direct SHM reference.
        publisher_->put(zenoh::Bytes(std::move(*buf)));
    }

    std::string                           key_expr_;
    std::shared_ptr<const ISerializer<T>> serializer_;
    std::optional<zenoh::Publisher>       publisher_;
    std::atomic<bool>                     ready_{false};
    std::atomic<uint64_t>                 shm_publishes_{0};
    std::atomic<uint64_t>                 bytes_publishes_{0};
};

}  // namespace drone::ipc
