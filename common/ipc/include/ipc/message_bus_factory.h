// common/ipc/include/ipc/message_bus_factory.h
// Config-driven IPC backend selection.
//
// Reads "ipc_backend" from the drone::Config and creates the
// appropriate message bus:
//   - "shm"   → ShmMessageBus   (POSIX shared memory, always available)
//   - "zenoh" → ZenohMessageBus (requires -DENABLE_ZENOH=ON at build time)
//
// Usage:
//   #include "ipc/message_bus_factory.h"
//   auto [shm_bus, zenoh_bus] = drone::ipc::create_message_bus(config);
//   // Use whichever is non-null — the other is nullptr.
//
// Since ShmMessageBus and ZenohMessageBus are concrete (non-virtual)
// template factories, we can't return a single polymorphic pointer.
// Instead we provide a variant-based helper.
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

}  // namespace drone::ipc
