// common/ipc/include/ipc/zenoh_publisher.h
// Zenoh-backed publisher — wraps zenoh::Publisher behind IPublisher<T>.
//
// Publishes trivially-copyable T via Zenoh.  For large messages
// (sizeof(T) > kShmPublishThreshold, e.g. video frames), the payload is
// written directly into a Zenoh SHM buffer — zero-copy for local
// subscribers.  Small messages use the regular Bytes path.
//
// Guarded by HAVE_ZENOH.
#pragma once


#include "ipc/ipublisher.h"
#include "ipc/zenoh_session.h"

#include <atomic>
#include <cstring>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>
#include <zenoh.hxx>

namespace drone::ipc {

/// Zenoh-backed publisher implementing IPublisher<T>.
template<typename T>
class ZenohPublisher final : public IPublisher<T> {
    static_assert(std::is_trivially_copyable_v<T>,
                  "ZenohPublisher requires trivially copyable types");

public:
    /// Construct and declare a Zenoh publisher on the given key expression.
    /// @param key_expr  Zenoh key expression (e.g. "drone/slam/pose").
    explicit ZenohPublisher(const std::string& key_expr) : key_expr_(key_expr) {
        try {
            auto& session = ZenohSession::instance().session();
            publisher_.emplace(session.declare_publisher(zenoh::KeyExpr(key_expr)));
            ready_ = true;
            spdlog::info("[ZenohPublisher] Declared on '{}' "
                         "(size={}, shm={})",
                         key_expr, sizeof(T), sizeof(T) > kShmPublishThreshold ? "yes" : "no");
        } catch (const std::exception& e) {
            spdlog::error("[ZenohPublisher] Failed to declare on '{}': {}", key_expr, e.what());
            ready_ = false;
        }
    }

    void publish(const T& msg) override {
        if (!ready_ || !publisher_.has_value()) return;

        if constexpr (sizeof(T) > kShmPublishThreshold) {
            publish_shm(msg);
            shm_publishes_.fetch_add(1, std::memory_order_relaxed);
        } else {
            publish_bytes(msg);
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

    [[nodiscard]] bool is_ready() const override { return ready_; }

private:
    /// Small-message path: serialize to vector<uint8_t>.
    void publish_bytes(const T& msg) {
        const auto*          ptr = reinterpret_cast<const uint8_t*>(&msg);
        std::vector<uint8_t> buf(ptr, ptr + sizeof(T));
        publisher_->put(zenoh::Bytes(std::move(buf)));
    }

    /// Large-message path: allocate SHM buffer, memcpy, publish zero-copy.
    void publish_shm(const T& msg) {
        auto* provider = ZenohSession::instance().shm_provider();
        if (!provider) {
            // Fallback to bytes path if SHM provider is unavailable
            publish_bytes(msg);
            return;
        }

        auto  result = provider->alloc_gc_defrag_blocking(sizeof(T));
        auto* buf    = std::get_if<zenoh::ZShmMut>(&result);
        if (!buf) {
            spdlog::warn("[ZenohPublisher] SHM alloc failed on '{}', "
                         "falling back to bytes",
                         key_expr_);
            publish_bytes(msg);
            return;
        }

        // Zero-copy write: memcpy directly into shared memory
        std::memcpy(buf->data(), &msg, sizeof(T));

        // Move SHM buffer into Bytes and publish — only a descriptor
        // is sent over the transport; local subscribers receive a
        // direct SHM reference.
        publisher_->put(zenoh::Bytes(std::move(*buf)));
    }

    std::string                     key_expr_;
    std::optional<zenoh::Publisher> publisher_;
    bool                            ready_ = false;
    std::atomic<uint64_t>           shm_publishes_{0};
    std::atomic<uint64_t>           bytes_publishes_{0};
};

}  // namespace drone::ipc
