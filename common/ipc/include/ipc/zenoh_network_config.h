// common/ipc/include/ipc/zenoh_network_config.h
// Builds a Zenoh JSON configuration string from the application's
// network settings, enabling drone↔GCS communication over TCP/UDP.
//
// Architecture:
//   Drone  → peer mode  + listener on 0.0.0.0:7447
//   GCS    → client mode + connect to drone_ip:7447
//
// The generated config is passed to ZenohSession::configure() before
// the session is opened.
//
// Guarded by HAVE_ZENOH (only meaningful with a Zenoh backend).
#pragma once


#include <cstdint>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Network transport configuration for the Zenoh session.
///
/// Generates a Zenoh JSON5 config string suitable for passing to
/// ZenohSession::configure().  Two primary modes:
///
///   Drone (peer + listener):
///     ZenohNetworkConfig cfg;
///     cfg.mode = "peer";
///     cfg.listen_endpoints = {"tcp/0.0.0.0:7447"};
///     cfg.multicast_scouting = true;
///     session.configure(cfg.to_json());
///
///   GCS (client + connect to drone):
///     ZenohNetworkConfig cfg;
///     cfg.mode = "client";
///     cfg.connect_endpoints = {"tcp/192.168.1.100:7447"};
///     cfg.multicast_scouting = false;
///     session.configure(cfg.to_json());
struct ZenohNetworkConfig {
    /// Zenoh session mode: "peer" (default), "client", or "router".
    std::string mode = "peer";

    /// Endpoints to listen on (drone side).
    /// Examples: "tcp/0.0.0.0:7447", "udp/0.0.0.0:7447"
    std::vector<std::string> listen_endpoints;

    /// Endpoints to connect to (GCS side).
    /// Examples: "tcp/192.168.1.100:7447", "udp/DRONE_IP:7447"
    std::vector<std::string> connect_endpoints;

    /// Enable multicast scouting for automatic peer discovery (LAN).
    bool multicast_scouting = true;

    /// Enable gossip scouting for peer discovery via routers.
    bool gossip_scouting = true;

    /// Default listen port (used by convenience helpers).
    uint16_t listen_port = 7447;

    /// Default listen address (used by convenience helpers).
    std::string listen_address = "0.0.0.0";

    /// Default protocol for convenience helpers.
    std::string protocol = "tcp";

    // ─── Convenience factory methods ──────────────────────────

    /// Create a drone-side config: peer mode, listening on all interfaces.
    static ZenohNetworkConfig make_drone(uint16_t           port    = 7447,
                                         const std::string& address = "0.0.0.0",
                                         const std::string& proto   = "tcp") {
        ZenohNetworkConfig cfg;
        cfg.mode           = "peer";
        cfg.listen_port    = port;
        cfg.listen_address = address;
        cfg.protocol       = proto;
        cfg.listen_endpoints.push_back(proto + "/" + address + ":" + std::to_string(port));
        cfg.multicast_scouting = true;
        return cfg;
    }

    /// Create a GCS-side config: client mode, connect to drone IP.
    static ZenohNetworkConfig make_gcs(const std::string& drone_ip, uint16_t port = 7447,
                                       const std::string& proto = "tcp") {
        ZenohNetworkConfig cfg;
        cfg.mode = "client";
        cfg.connect_endpoints.push_back(proto + "/" + drone_ip + ":" + std::to_string(port));
        cfg.multicast_scouting = false;
        cfg.gossip_scouting    = false;
        return cfg;
    }

    /// Create a local-only config: peer mode, no listeners, no network.
    /// Suitable for unit tests and single-machine operation.
    static ZenohNetworkConfig make_local() {
        ZenohNetworkConfig cfg;
        cfg.mode               = "peer";
        cfg.multicast_scouting = false;
        cfg.gossip_scouting    = false;
        return cfg;
    }

    // ─── Serialization ───────────────────────────────────────

    /// Generate a Zenoh JSON5 configuration string.
    ///
    /// The output is suitable for zenoh::Config::from_str().
    /// Only non-default fields are emitted to keep the config minimal.
    std::string to_json() const {
        std::string json = "{\n";
        json += "  \"mode\": \"" + mode + "\"";

        // Listen endpoints
        if (!listen_endpoints.empty()) {
            json += ",\n  \"listen\": {\n    \"endpoints\": [";
            for (size_t i = 0; i < listen_endpoints.size(); ++i) {
                if (i > 0) json += ", ";
                json += "\"" + listen_endpoints[i] + "\"";
            }
            json += "]\n  }";
        }

        // Connect endpoints
        if (!connect_endpoints.empty()) {
            json += ",\n  \"connect\": {\n    \"endpoints\": [";
            for (size_t i = 0; i < connect_endpoints.size(); ++i) {
                if (i > 0) json += ", ";
                json += "\"" + connect_endpoints[i] + "\"";
            }
            json += "]\n  }";
        }

        // Scouting
        json += ",\n  \"scouting\": {\n";
        json += "    \"multicast\": { \"enabled\": " +
                std::string(multicast_scouting ? "true" : "false") + " },\n";
        json += "    \"gossip\": { \"enabled\": " +
                std::string(gossip_scouting ? "true" : "false") + " }\n";
        json += "  }";

        json += "\n}";
        return json;
    }

    /// Parse network settings from the application config JSON.
    ///
    /// Expects a "zenoh.network" object in the app config:
    /// ```json
    /// {
    ///   "zenoh": {
    ///     "network": {
    ///       "enabled": true,
    ///       "mode": "peer",
    ///       "listen_port": 7447,
    ///       "listen_address": "0.0.0.0",
    ///       "protocol": "tcp",
    ///       "connect_endpoints": [],
    ///       "multicast_scouting": true,
    ///       "gossip_scouting": true
    ///     }
    ///   }
    /// }
    /// ```
    ///
    /// @param zenoh_network  The "zenoh.network" JSON object (nlohmann::json).
    ///                       Pass the raw object, not a string.
    ///
    /// Usage with nlohmann::json:
    /// ```cpp
    /// auto cfg = ZenohNetworkConfig::from_app_config(app_config["zenoh"]["network"]);
    /// ```
    ///
    /// This method is implemented as a template to avoid pulling in
    /// nlohmann/json.hpp into this header.
    template<typename JsonT>
    static ZenohNetworkConfig from_app_config(const JsonT& zenoh_network) {
        ZenohNetworkConfig cfg;

        if (zenoh_network.contains("mode")) {
            cfg.mode = zenoh_network["mode"].template get<std::string>();
        }
        if (zenoh_network.contains("listen_port")) {
            cfg.listen_port = zenoh_network["listen_port"].template get<uint16_t>();
        }
        if (zenoh_network.contains("listen_address")) {
            cfg.listen_address = zenoh_network["listen_address"].template get<std::string>();
        }
        if (zenoh_network.contains("protocol")) {
            cfg.protocol = zenoh_network["protocol"].template get<std::string>();
        }
        if (zenoh_network.contains("multicast_scouting")) {
            cfg.multicast_scouting = zenoh_network["multicast_scouting"].template get<bool>();
        }
        if (zenoh_network.contains("gossip_scouting")) {
            cfg.gossip_scouting = zenoh_network["gossip_scouting"].template get<bool>();
        }

        // Build listen endpoint from components
        if (zenoh_network.contains("listen_port")) {
            cfg.listen_endpoints.clear();
            cfg.listen_endpoints.push_back(cfg.protocol + "/" + cfg.listen_address + ":" +
                                           std::to_string(cfg.listen_port));
        }

        // Parse explicit connect endpoints
        if (zenoh_network.contains("connect_endpoints")) {
            for (const auto& ep : zenoh_network["connect_endpoints"]) {
                cfg.connect_endpoints.push_back(ep.template get<std::string>());
            }
        }

        spdlog::info("[ZenohNetworkConfig] Loaded from app config: "
                     "mode={}, listen={}, connect_eps={}, "
                     "multicast={}, gossip={}",
                     cfg.mode, cfg.listen_endpoints.empty() ? "none" : cfg.listen_endpoints[0],
                     cfg.connect_endpoints.size(), cfg.multicast_scouting, cfg.gossip_scouting);
        return cfg;
    }
};

}  // namespace drone::ipc
