// common/ipc/include/ipc/message_bus.h
// Transport-agnostic message bus — type-erased wrapper over concrete backends.
//
// Processes use this class to create publishers, subscribers, and service
// channels without knowing or caring which transport is active:
//
//   auto bus = drone::ipc::create_message_bus(cfg);
//   auto pub = bus.advertise<ShmPose>("drone/slam/pose");
//   auto sub = bus.subscribe<ShmPose>("drone/slam/pose");
//
// Adding a new transport backend (e.g. iceoryx) requires:
//   1. Implement IceoryxPublisher<T>, IceoryxSubscriber<T>, IceoryxMessageBus
//   2. Add one #ifdef arm to BusVariant below
//   3. Add one case in create_message_bus() factory
//   Zero changes to process code.
//
// Architecture:
//   IPublisher<T>  ←── ShmPublisher<T>   / ZenohPublisher<T>   / (future)
//   ISubscriber<T> ←── ShmSubscriber<T>  / ZenohSubscriber<T>  / (future)
//   MessageBus     ←── wraps std::variant<ShmMessageBus, ZenohMessageBus, ...>
//
#pragma once

#include "ipc/ipublisher.h"
#include "ipc/iservice_channel.h"
#include "ipc/isubscriber.h"
#include "ipc/shm_message_bus.h"

#ifdef HAVE_ZENOH
#include "ipc/zenoh_message_bus.h"
#endif

#include <memory>
#include <string>
#include <type_traits>
#include <variant>

#include <spdlog/spdlog.h>

namespace drone::ipc {

// ─────────────────────────────────────────────────────────────
// BusVariant — internal variant of all compiled-in backends.
//
// To add a new backend (e.g. iceoryx):
//   1. #ifdef HAVE_ICEORYX
//      #include "ipc/iceoryx_message_bus.h"
//      #endif
//   2. Add ", std::unique_ptr<IceoryxMessageBus>" inside the #ifdef below
//   3. That's it — MessageBus methods use std::visit, so they
//      automatically dispatch to the new type.
// ─────────────────────────────────────────────────────────────
namespace detail {

using BusVariant = std::variant<std::unique_ptr<ShmMessageBus>
#ifdef HAVE_ZENOH
                                ,
                                std::unique_ptr<ZenohMessageBus>
#endif
                                // --- Add new backends here ---
                                // #ifdef HAVE_ICEORYX
                                // , std::unique_ptr<IceoryxMessageBus>
                                // #endif
                                >;

}  // namespace detail

/// Transport-agnostic message bus.
///
/// Wraps a concrete backend (SHM, Zenoh, future iceoryx/DDS) behind a
/// uniform API.  Template methods forward to the active backend via
/// std::visit — processes never see the variant.
///
/// Movable, not copyable.
class MessageBus {
public:
    /// Construct from any concrete bus backend.
    /// @tparam BusT  ShmMessageBus, ZenohMessageBus, etc.
    template<typename BusT>
    explicit MessageBus(std::unique_ptr<BusT> bus) : impl_(std::move(bus)) {}

    MessageBus(MessageBus&&) noexcept            = default;
    MessageBus& operator=(MessageBus&&) noexcept = default;
    MessageBus(const MessageBus&)                = delete;
    MessageBus& operator=(const MessageBus&)     = delete;

    // ─── Pub/Sub ─────────────────────────────────────────────

    /// Create a publisher for the given topic.
    /// @tparam T  Trivially-copyable message type.
    /// @param topic  Topic name (SHM segment or Zenoh key expression).
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
    /// @param max_retries  SHM: number of connection retries. Zenoh: ignored.
    /// @param retry_ms     SHM: delay between retries. Zenoh: ignored.
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

    /// Create a service client.
    /// @note Only available on backends that support request/reply
    ///       (currently Zenoh). Returns nullptr on other backends.
    template<typename Req, typename Resp>
    [[nodiscard]] std::unique_ptr<IServiceClient<Req, Resp>> create_client(
        const std::string& service, uint64_t timeout_ms = 5000) {
        return std::visit(
            [&](auto& b) -> std::unique_ptr<IServiceClient<Req, Resp>> {
#ifdef HAVE_ZENOH
                using BusType = std::decay_t<decltype(*b)>;
                if constexpr (std::is_same_v<BusType, ZenohMessageBus>) {
                    return b->template create_client<Req, Resp>(service, timeout_ms);
                }
#endif
                (void)b;
                (void)service;
                (void)timeout_ms;
                spdlog::warn("[MessageBus] Service channels are not available "
                             "on the {} backend",
                             backend_name());
                return nullptr;
            },
            impl_);
    }

    /// Create a service server.
    /// @note Only available on backends that support request/reply
    ///       (currently Zenoh). Returns nullptr on other backends.
    template<typename Req, typename Resp>
    [[nodiscard]] std::unique_ptr<IServiceServer<Req, Resp>> create_server(
        const std::string& service) {
        return std::visit(
            [&](auto& b) -> std::unique_ptr<IServiceServer<Req, Resp>> {
#ifdef HAVE_ZENOH
                using BusType = std::decay_t<decltype(*b)>;
                if constexpr (std::is_same_v<BusType, ZenohMessageBus>) {
                    return b->template create_server<Req, Resp>(service);
                }
#endif
                (void)b;
                (void)service;
                spdlog::warn("[MessageBus] Service channels are not available "
                             "on the {} backend",
                             backend_name());
                return nullptr;
            },
            impl_);
    }

    // ─── Introspection ───────────────────────────────────────

    /// Returns the name of the active transport backend ("shm", "zenoh", ...).
    [[nodiscard]] std::string backend_name() const {
        return std::visit(
            [](const auto& b) -> std::string {
                using BusType = std::decay_t<decltype(*b)>;
                if constexpr (std::is_same_v<BusType, ShmMessageBus>) {
                    return "shm";
                }
#ifdef HAVE_ZENOH
                else if constexpr (std::is_same_v<BusType, ZenohMessageBus>) {
                    return "zenoh";
                }
#endif
                else {
                    return "unknown";
                }
            },
            impl_);
    }

    /// Returns true if this backend supports service channels (request/reply).
    [[nodiscard]] bool has_service_channels() const {
#ifdef HAVE_ZENOH
        return std::holds_alternative<std::unique_ptr<ZenohMessageBus>>(impl_);
#else
        return false;
#endif
    }

private:
    detail::BusVariant impl_;
};

}  // namespace drone::ipc
