// common/ipc/include/ipc/message_bus.h
// Transport-agnostic message bus — type-erased wrapper over concrete backends.
//
// Processes use this class to create publishers, subscribers, and service
// channels without knowing or caring which transport is active:
//
//   auto bus = drone::ipc::create_message_bus(cfg);
//   auto pub = bus.advertise<Pose>("drone/slam/pose");
//   auto sub = bus.subscribe<Pose>("drone/slam/pose");
//
// Adding a new transport backend (e.g. iceoryx) requires:
//   1. Implement IceoryxPublisher<T>, IceoryxSubscriber<T>, IceoryxMessageBus
//   2. Add one arm to BusVariant below
//   3. Add one case in create_message_bus() factory
//   Zero changes to process code.
//
// Architecture:
//   IPublisher<T>  ←── ZenohPublisher<T>   / (future IceoryxPublisher<T>)
//   ISubscriber<T> ←── ZenohSubscriber<T>  / (future IceoryxSubscriber<T>)
//   MessageBus     ←── wraps std::variant<ZenohMessageBus, ...>
//
#pragma once

#include "ipc/ipublisher.h"
#include "ipc/iservice_channel.h"
#include "ipc/isubscriber.h"
#include "ipc/zenoh_message_bus.h"
#include "util/ilogger.h"

#include <memory>
#include <string>
#include <type_traits>
#include <variant>

namespace drone::ipc {

// ─────────────────────────────────────────────────────────────
// BusVariant — internal variant of all compiled-in backends.
//
// To add a new backend (e.g. iceoryx):
//   1. #ifdef HAVE_ICEORYX
//      #include "ipc/iceoryx_message_bus.h"
//      #endif
//   2. Add ", std::unique_ptr<IceoryxMessageBus>" inside the variant
//   3. That's it — MessageBus methods use std::visit, so they
//      automatically dispatch to the new type.
// ─────────────────────────────────────────────────────────────
namespace detail {

using BusVariant = std::variant<std::unique_ptr<ZenohMessageBus>
                                // --- Add new backends here ---
                                // #ifdef HAVE_ICEORYX
                                // , std::unique_ptr<IceoryxMessageBus>
                                // #endif
                                >;

}  // namespace detail

/// Transport-agnostic message bus.
///
/// Wraps a concrete backend (Zenoh, future iceoryx/DDS) behind a
/// uniform API.  Template methods forward to the active backend via
/// std::visit — processes never see the variant.
///
/// Movable, not copyable.
class MessageBus {
public:
    /// Construct from any concrete bus backend.
    /// @tparam BusT  ZenohMessageBus, etc.
    template<typename BusT>
    explicit MessageBus(std::unique_ptr<BusT> bus) : impl_(std::move(bus)) {}

    MessageBus(MessageBus&&) noexcept            = default;
    MessageBus& operator=(MessageBus&&) noexcept = default;
    MessageBus(const MessageBus&)                = delete;
    MessageBus& operator=(const MessageBus&)     = delete;

    // ─── Pub/Sub ─────────────────────────────────────────────

    /// Create a publisher for the given topic.
    /// @tparam T  Trivially-copyable message type.
    /// @param topic  Topic name (Zenoh key expression).
    /// @return  Owning publisher — ready to call publish().
    template<typename T>
    [[nodiscard]] std::unique_ptr<IPublisher<T>> advertise(const std::string& topic) {
        return std::visit(
            [&](auto& b) -> std::unique_ptr<IPublisher<T>> {
                return b->template advertise<T>(topic);
            },
            impl_);
    }

    /// Create a subscriber for the given topic.
    /// @tparam T  Trivially-copyable message type.
    /// @param topic        Topic name.
    /// @param max_retries  Legacy (ignored by Zenoh).
    /// @param retry_ms     Legacy (ignored by Zenoh).
    template<typename T>
    [[nodiscard]] std::unique_ptr<ISubscriber<T>> subscribe(const std::string& topic,
                                                            int                max_retries = 50,
                                                            int                retry_ms    = 200) {
        return std::visit(
            [&](auto& b) -> std::unique_ptr<ISubscriber<T>> {
                return b->template subscribe<T>(topic, max_retries, retry_ms);
            },
            impl_);
    }

    /// Create an optional subscriber (single attempt, no retries).
    /// Use for channels where it's OK if the publisher doesn't exist yet.
    template<typename T>
    [[nodiscard]] std::unique_ptr<ISubscriber<T>> subscribe_optional(const std::string& topic) {
        return subscribe<T>(topic, 0, 0);
    }

    // ─── Service Channels ────────────────────────────────────

    /// Create a service client (request/reply pattern).
    template<typename Req, typename Resp>
    [[nodiscard]] std::unique_ptr<IServiceClient<Req, Resp>> create_client(
        const std::string& service, uint64_t timeout_ms = 5000) {
        return std::visit(
            [&](auto& b) -> std::unique_ptr<IServiceClient<Req, Resp>> {
                return b->template create_client<Req, Resp>(service, timeout_ms);
            },
            impl_);
    }

    /// Create a service server (request/reply pattern).
    template<typename Req, typename Resp>
    [[nodiscard]] std::unique_ptr<IServiceServer<Req, Resp>> create_server(
        const std::string& service) {
        return std::visit(
            [&](auto& b) -> std::unique_ptr<IServiceServer<Req, Resp>> {
                return b->template create_server<Req, Resp>(service);
            },
            impl_);
    }

    // ─── Introspection ───────────────────────────────────────

    /// Returns the name of the active transport backend ("zenoh", ...).
    [[nodiscard]] std::string backend_name() const {
        return std::visit(
            [](const auto& b) -> std::string {
                using BusType = std::decay_t<decltype(*b)>;
                if constexpr (std::is_same_v<BusType, ZenohMessageBus>) {
                    return "zenoh";
                } else {
                    return "unknown";
                }
            },
            impl_);
    }

    /// Returns true if this backend supports service channels (request/reply).
    [[nodiscard]] bool has_service_channels() const {
        return std::holds_alternative<std::unique_ptr<ZenohMessageBus>>(impl_);
    }

private:
    detail::BusVariant impl_;
};

}  // namespace drone::ipc
