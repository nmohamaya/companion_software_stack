// common/ipc/include/ipc/message_bus_factory.h
// IPC backend selection helper.
//
// Selects and constructs the appropriate message bus backend based on a
// simple string parameter:
//   - "shm"   → ShmMessageBus   (POSIX shared memory, always available)
//   - "zenoh" → ZenohMessageBus (requires -DENABLE_ZENOH=ON at build time)
//
// Usage:
//   #include "ipc/message_bus_factory.h"
//
//   // Create a bus variant using the desired backend (defaults to "shm"):
//   auto bus = drone::ipc::create_message_bus("shm");
//
//   // Example: advertise/subscribe through whichever backend is active.
//   auto pub = drone::ipc::bus_advertise<MyMsg>(bus, "telemetry");
//   auto sub = drone::ipc::bus_subscribe<MyMsg>(bus, "telemetry");
//
// Since ShmMessageBus and ZenohMessageBus are concrete (non-virtual)
// template factories, we can't return a single polymorphic pointer.
// Instead we return a std::variant<...> (MessageBusVariant) plus helpers.
#pragma once

#include "ipc/shm_message_bus.h"

#ifdef HAVE_ZENOH
#include "ipc/zenoh_message_bus.h"
#endif

#include <memory>
#include <string>
#include <variant>

#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Holds exactly one of the two message bus backends.
/// Processes use visit() or if-else to call advertise/subscribe.
using MessageBusVariant = std::variant<
    std::unique_ptr<ShmMessageBus>
#ifdef HAVE_ZENOH
    , std::unique_ptr<ZenohMessageBus>
#endif
>;

/// Create the IPC message bus based on config.
/// @param backend  "shm" (default) or "zenoh".
/// @return A variant holding the selected message bus.
inline MessageBusVariant create_message_bus(const std::string& backend = "shm") {
#ifdef HAVE_ZENOH
    if (backend == "zenoh") {
        spdlog::info("[MessageBusFactory] Selected backend: Zenoh");
        return std::make_unique<ZenohMessageBus>();
    }
#else
    if (backend == "zenoh") {
        spdlog::warn("[MessageBusFactory] Zenoh requested but not compiled in "
                     "(HAVE_ZENOH not defined) — falling back to SHM");
    }
#endif
    spdlog::info("[MessageBusFactory] Selected backend: POSIX SHM");
    return std::make_unique<ShmMessageBus>();
}

// ─────────────────────────────────────────────────────────────
// Helper: advertise / subscribe through the variant
// ─────────────────────────────────────────────────────────────

/// Create a publisher through whichever bus is active.
template <typename T>
std::unique_ptr<IPublisher<T>> bus_advertise(
    MessageBusVariant& bus, const std::string& topic)
{
    return std::visit([&](auto& b) -> std::unique_ptr<IPublisher<T>> {
        return b->template advertise<T>(topic);
    }, bus);
}

/// Create a subscriber through whichever bus is active.
template <typename T>
std::unique_ptr<ISubscriber<T>> bus_subscribe(
    MessageBusVariant& bus, const std::string& topic,
    int max_retries = 50, int retry_ms = 200)
{
    return std::visit([&](auto& b) -> std::unique_ptr<ISubscriber<T>> {
        return b->template subscribe<T>(topic, max_retries, retry_ms);
    }, bus);
}

/// Create an optional/lazy subscriber (single attempt, no retries).
/// Equivalent to subscribe() with max_retries=0.
/// For SHM: tries once to open the segment — is_connected() may return false
///          if the publisher hasn't created the shared-memory segment yet.
/// For Zenoh: the underlying Zenoh session always accepts the subscription
///            (is_connected() returns true) because pub/sub discovery is
///            asynchronous; messages will arrive once a publisher appears.
/// Use for optional channels where it's OK if no publisher exists yet.
template <typename T>
std::unique_ptr<ISubscriber<T>> bus_subscribe_optional(
    MessageBusVariant& bus, const std::string& topic)
{
    return bus_subscribe<T>(bus, topic, 0, 0);
}

}  // namespace drone::ipc
