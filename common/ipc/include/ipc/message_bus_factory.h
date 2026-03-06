// common/ipc/include/ipc/message_bus_factory.h
// IPC backend factory — creates a transport-agnostic MessageBus.
//
// Selects and constructs the appropriate message bus backend based on
// a config parameter or string:
//   - "shm"   → ShmMessageBus   (POSIX shared memory, always available)
//   - "zenoh" → ZenohMessageBus (requires -DENABLE_ZENOH=ON at build time)
//
// Usage (current — transport-agnostic):
//   #include "ipc/message_bus_factory.h"
//
//   auto bus = drone::ipc::create_message_bus(cfg);   // or ("shm") / ("zenoh")
//   auto pub = bus.advertise<MyMsg>("telemetry");
//   auto sub = bus.subscribe<MyMsg>("telemetry");
//
//   // Service channels (Zenoh only; returns nullptr on SHM):
//   auto client = bus.create_client<Req, Resp>("my_service");
//   auto server = bus.create_server<Req, Resp>("my_service");
//
// Adding a new backend (e.g. iceoryx):
//   1. Implement IceoryxPublisher<T>, IceoryxSubscriber<T>, IceoryxMessageBus
//   2. Add one #ifdef arm in message_bus.h BusVariant
//   3. Add one case in create_message_bus() below
//   Zero changes to process code.
#pragma once

#include "ipc/message_bus.h"

#ifdef HAVE_ZENOH
#include "ipc/zenoh_network_config.h"
#include "ipc/zenoh_session.h"
#endif

#include <memory>
#include <string>
#include <type_traits>

#include <spdlog/spdlog.h>

namespace drone::ipc {

// ─────────────────────────────────────────────────────────────
// Factory: create_message_bus
// ─────────────────────────────────────────────────────────────

/// Create the IPC message bus by backend name.
/// @param backend  "shm" (default) or "zenoh".
/// @param shm_pool_mb  Zenoh SHM pool size in MB (0 = use default 32 MB).
/// @param zenoh_config_json  Optional raw Zenoh JSON config string.
inline MessageBus create_message_bus(const std::string& backend           = "shm",
                                     std::size_t        shm_pool_mb       = 0,
                                     const std::string& zenoh_config_json = "") {
    (void)shm_pool_mb;
    (void)zenoh_config_json;
#ifdef HAVE_ZENOH
    if (backend == "zenoh") {
        if (!zenoh_config_json.empty()) {
            drone::ipc::ZenohSession::instance().configure(zenoh_config_json);
            drone::ipc::ZenohSession::instance().set_network_enabled(true);
        }
        if (shm_pool_mb > 0) {
            drone::ipc::ZenohSession::instance().configure_shm(shm_pool_mb * 1024 * 1024);
        }
        spdlog::info("[MessageBusFactory] Selected backend: Zenoh");
        return MessageBus(std::make_unique<ZenohMessageBus>());
    }
#else
    if (backend == "zenoh") {
        spdlog::warn("[MessageBusFactory] Zenoh requested but not compiled in "
                     "(HAVE_ZENOH not defined) — falling back to SHM");
    }
#endif
    spdlog::info("[MessageBusFactory] Selected backend: POSIX SHM");
    return MessageBus(std::make_unique<ShmMessageBus>());
}

/// Config-aware overload: reads ipc_backend, zenoh.shm_pool_size_mb, and
/// zenoh.network from the application config object.
///
/// @tparam ConfigT  Any type with get<T>(key, default) and section(key)
///                  methods (e.g. drone::Config).
template<typename ConfigT,
         typename = typename std::enable_if<!std::is_convertible<ConfigT, std::string>::value>::type>
MessageBus create_message_bus(const ConfigT& cfg) {
    const auto backend     = cfg.template get<std::string>("ipc_backend", "shm");
    const auto shm_pool_mb = cfg.template get<std::size_t>("zenoh.shm_pool_size_mb", 0);

    std::string zenoh_json;
#ifdef HAVE_ZENOH
    if (backend == "zenoh" && cfg.template get<bool>("zenoh.network.enabled", false)) {
        auto net_cfg = ZenohNetworkConfig::from_app_config(cfg.section("zenoh.network"));
        zenoh_json   = net_cfg.to_json();
    }
#endif

    return create_message_bus(backend, shm_pool_mb, zenoh_json);
}

// ─────────────────────────────────────────────────────────────
// Backward-compatible free functions (deprecated)
// These forward to MessageBus methods.  Prefer bus.advertise<T>() directly.
// ─────────────────────────────────────────────────────────────

/// @deprecated Use bus.advertise<T>(topic) instead.
template<typename T>
[[deprecated("Use bus.advertise<T>(topic) instead of bus_advertise<T>(bus, topic)")]]
std::unique_ptr<IPublisher<T>> bus_advertise(MessageBus& bus, const std::string& topic) {
    return bus.advertise<T>(topic);
}

/// @deprecated Use bus.subscribe<T>(topic, retries, ms) instead.
template<typename T>
[[deprecated("Use bus.subscribe<T>(topic) instead of bus_subscribe<T>(bus, topic)")]]
std::unique_ptr<ISubscriber<T>> bus_subscribe(MessageBus& bus, const std::string& topic,
                                              int max_retries = 50, int retry_ms = 200) {
    return bus.subscribe<T>(topic, max_retries, retry_ms);
}

/// @deprecated Use bus.subscribe_optional<T>(topic) instead.
template<typename T>
[[deprecated("Use bus.subscribe_optional<T>(topic) instead")]]
std::unique_ptr<ISubscriber<T>> bus_subscribe_optional(MessageBus& bus, const std::string& topic) {
    return bus.subscribe_optional<T>(topic);
}

/// @deprecated Use bus.create_client<Req,Resp>(service, timeout_ms) instead.
template<typename Req, typename Resp>
[[deprecated("Use bus.create_client<Req,Resp>(service) instead")]]
std::unique_ptr<IServiceClient<Req, Resp>> bus_create_client(MessageBus&        bus,
                                                             const std::string& service,
                                                             uint64_t           timeout_ms = 5000) {
    return bus.create_client<Req, Resp>(service, timeout_ms);
}

/// @deprecated Use bus.create_server<Req,Resp>(service) instead.
template<typename Req, typename Resp>
[[deprecated("Use bus.create_server<Req,Resp>(service) instead")]]
std::unique_ptr<IServiceServer<Req, Resp>> bus_create_server(MessageBus&        bus,
                                                             const std::string& service) {
    return bus.create_server<Req, Resp>(service);
}

}  // namespace drone::ipc
