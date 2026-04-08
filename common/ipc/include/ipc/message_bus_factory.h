// common/ipc/include/ipc/message_bus_factory.h
// IPC backend factory — creates a transport-agnostic MessageBus.
//
// The default (and currently only) backend is Zenoh.  The factory pattern
// is preserved so that adding a new backend (e.g. iceoryx, DDS) requires
// one new class + one case here — zero changes to process code.
//
// Usage:
//   #include "ipc/message_bus_factory.h"
//
//   auto bus = drone::ipc::create_message_bus(cfg);     // config-driven
//   auto bus = drone::ipc::create_message_bus("zenoh"); // explicit
//   auto pub = bus.advertise<MyMsg>("telemetry");
//   auto sub = bus.subscribe<MyMsg>("telemetry");
//
// Adding a new backend (e.g. iceoryx):
//   1. Implement IceoryxPublisher<T>, IceoryxSubscriber<T>, IceoryxMessageBus
//   2. Add one arm in message_bus.h BusVariant
//   3. Add one case in create_message_bus() below
//   Zero changes to process code.
#pragma once

#include "ipc/message_bus.h"
#include "ipc/zenoh_network_config.h"
#include "ipc/zenoh_session.h"
#include "util/ilogger.h"

#include <memory>
#include <string>
#include <type_traits>

namespace drone::ipc {

// ─────────────────────────────────────────────────────────────
// Factory: create_message_bus
// ─────────────────────────────────────────────────────────────

/// Create the IPC message bus by backend name.
/// @param backend  "zenoh" (default).  Unknown backends produce a fatal error.
/// @param shm_pool_mb  Zenoh SHM pool size in MB (0 = use default 32 MB).
/// @param zenoh_config_json  Optional raw Zenoh JSON config string.
inline MessageBus create_message_bus(const std::string& backend           = "zenoh",
                                     std::size_t        shm_pool_mb       = 0,
                                     const std::string& zenoh_config_json = "") {
    if (backend != "zenoh") {
        if (backend == "shm") {
            DRONE_LOG_ERROR("[MessageBusFactory] The 'shm' backend has been removed. "
                            "Please update config/default.json to use ipc_backend: \"zenoh\". "
                            "Falling back to Zenoh.");
        } else {
            DRONE_LOG_ERROR("[MessageBusFactory] Unknown backend '{}' — only 'zenoh' is "
                            "currently supported. Falling back to Zenoh.",
                            backend);
        }
    }

    if (!zenoh_config_json.empty()) {
        drone::ipc::ZenohSession::instance().configure(zenoh_config_json);
        drone::ipc::ZenohSession::instance().set_network_enabled(true);
    }
    if (shm_pool_mb > 0) {
        drone::ipc::ZenohSession::instance().configure_shm(shm_pool_mb * 1024 * 1024);
    }
    DRONE_LOG_INFO("[MessageBusFactory] Selected backend: Zenoh");
    return MessageBus(std::make_unique<ZenohMessageBus>());
}

/// Config-aware overload: reads ipc_backend, zenoh.shm_pool_size_mb, and
/// zenoh.network from the application config object.
///
/// @tparam ConfigT  Any type with get<T>(key, default) and section(key)
///                  methods (e.g. drone::Config).
template<typename ConfigT,
         typename = typename std::enable_if<!std::is_convertible<ConfigT, std::string>::value>::type>
MessageBus create_message_bus(const ConfigT& cfg) {
    const auto backend     = cfg.template get<std::string>("ipc_backend", "zenoh");
    const auto shm_pool_mb = cfg.template get<std::size_t>("zenoh.shm_pool_size_mb", 0);

    std::string zenoh_json;
    if (cfg.template get<bool>("zenoh.network.enabled", false)) {
        auto net_cfg = ZenohNetworkConfig::from_app_config(cfg.section("zenoh.network"));
        zenoh_json   = net_cfg.to_json();
    }

    return create_message_bus(backend, shm_pool_mb, zenoh_json);
}

}  // namespace drone::ipc
