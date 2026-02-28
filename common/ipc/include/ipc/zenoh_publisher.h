// common/ipc/include/ipc/zenoh_publisher.h
// Zenoh-backed publisher — wraps zenoh::Publisher behind IPublisher<T>.
//
// Publishes trivially-copyable T as raw bytes via Zenoh.
// When the Zenoh SHM provider is enabled, local subscribers receive
// a zero-copy SHM reference (no memcpy).  Remote subscribers get
// the bytes transported transparently over the network.
//
// Guarded by HAVE_ZENOH.
#pragma once

#ifdef HAVE_ZENOH

#include "ipc/ipublisher.h"
#include "ipc/zenoh_session.h"

#include <zenoh.hxx>

#include <cstring>
#include <string>
#include <type_traits>

#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Zenoh-backed publisher implementing IPublisher<T>.
template <typename T>
class ZenohPublisher final : public IPublisher<T> {
    static_assert(std::is_trivially_copyable_v<T>,
                  "ZenohPublisher requires trivially copyable types");
public:
    /// Construct and declare a Zenoh publisher on the given key expression.
    /// @param key_expr  Zenoh key expression (e.g. "drone/slam/pose").
    explicit ZenohPublisher(const std::string& key_expr)
        : key_expr_(key_expr)
    {
        try {
            auto& session = ZenohSession::instance().session();
            publisher_ = std::make_unique<zenoh::Publisher>(
                session.declare_publisher(
                    zenoh::KeyExpr(key_expr)));
            ready_ = true;
            spdlog::info("[ZenohPublisher] Declared on '{}'", key_expr);
        } catch (const std::exception& e) {
            spdlog::error("[ZenohPublisher] Failed to declare on '{}': {}",
                          key_expr, e.what());
            ready_ = false;
        }
    }

    void publish(const T& msg) override {
        if (!ready_ || !publisher_) return;
        // Serialize as raw bytes — trivially_copyable guarantee
        publisher_->put(
            zenoh::Bytes(
                reinterpret_cast<const uint8_t*>(&msg),
                sizeof(T)));
    }

    const std::string& topic_name() const override { return key_expr_; }

    bool is_ready() const override { return ready_; }

private:
    std::string key_expr_;
    std::unique_ptr<zenoh::Publisher> publisher_;
    bool ready_ = false;
};

}  // namespace drone::ipc

#endif  // HAVE_ZENOH
