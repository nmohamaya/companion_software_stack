// common/ipc/include/ipc/zenoh_message_bus.h
// ZenohMessageBus — factory for Zenoh-backed publishers and subscribers.
//
// Drop-in replacement for ShmMessageBus.  Same API surface:
//   - advertise<T>(topic)  → IPublisher<T>
//   - subscribe<T>(topic)  → ISubscriber<T>
//
// Topic name mapping:
//   Old SHM segment names (e.g. "/slam_pose") are automatically
//   converted to Zenoh key expressions (e.g. "drone/slam/pose").
//   Callers can also pass Zenoh key expressions directly.
//
// Guarded by HAVE_ZENOH.
#pragma once

#ifdef HAVE_ZENOH

#include "ipc/ipublisher.h"
#include "ipc/isubscriber.h"
#include "ipc/zenoh_publisher.h"
#include "ipc/zenoh_subscriber.h"

#include <memory>
#include <string>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace drone::ipc {

/// Factory that creates Zenoh-backed publishers and subscribers.
/// Mirrors the ShmMessageBus API so processes can switch backends
/// by changing a single type (or via the MessageBusFactory).
class ZenohMessageBus {
public:
    ZenohMessageBus() {
        spdlog::info("[ZenohMessageBus] Created (Zenoh IPC backend)");
    }

    ~ZenohMessageBus() = default;

    /// Create a publisher for the given topic.
    /// @param topic  SHM segment name (auto-mapped) or Zenoh key expression.
    template <typename T>
    std::unique_ptr<IPublisher<T>> advertise(const std::string& topic) {
        return std::make_unique<ZenohPublisher<T>>(to_key_expr(topic));
    }

    /// Create a subscriber for the given topic.
    /// @param topic        SHM segment name or Zenoh key expression.
    /// @param max_retries  Ignored (Zenoh connects asynchronously).
    /// @param retry_ms     Ignored.
    template <typename T>
    std::unique_ptr<ISubscriber<T>> subscribe(
        const std::string& topic,
        [[maybe_unused]] int max_retries = 50,
        [[maybe_unused]] int retry_ms = 200)
    {
        return std::make_unique<ZenohSubscriber<T>>(to_key_expr(topic));
    }

    /// Create a subscriber for the given topic (lazy variant).
    /// For Zenoh, this is equivalent to subscribe() — connections are
    /// always asynchronous, so there is no distinction between eager
    /// and lazy subscription.
    /// @param topic  SHM segment name or Zenoh key expression.
    template <typename T>
    std::unique_ptr<ZenohSubscriber<T>> subscribe_lazy(
        const std::string& topic)
    {
        return std::make_unique<ZenohSubscriber<T>>(to_key_expr(topic));
    }

    /// Convert a legacy SHM segment name to a Zenoh key expression.
    /// @param shm_name  E.g. "/drone_mission_cam" or "/slam_pose".
    /// @return          Zenoh key expression, e.g. "drone/video/frame".
    ///
    /// If the name is not in the mapping table, it's passed through
    /// with the leading '/' stripped and '_' replaced by '/'.
    static std::string to_key_expr(const std::string& name) {
        if (name.empty()) {
            spdlog::warn("[ZenohMessageBus] to_key_expr() called with empty name");
            return "";
        }

        // Lookup table: legacy SHM name → Zenoh key expression
        static const std::unordered_map<std::string, std::string> mapping = {
            {"/drone_mission_cam",  "drone/video/frame"},
            {"/drone_stereo_cam",   "drone/video/stereo_frame"},
            {"/detected_objects",   "drone/perception/detections"},
            {"/slam_pose",          "drone/slam/pose"},
            {"/mission_status",     "drone/mission/status"},
            {"/trajectory_cmd",     "drone/mission/trajectory"},
            {"/payload_commands",   "drone/mission/payload_command"},
            {"/fc_commands",        "drone/comms/fc_command"},
            {"/fc_state",           "drone/comms/fc_state"},
            {"/gcs_commands",       "drone/comms/gcs_command"},
            {"/payload_status",     "drone/payload/status"},
            {"/system_health",      "drone/monitor/health"},
        };

        auto it = mapping.find(name);
        if (it != mapping.end()) {
            return it->second;
        }

        // If already a Zenoh key expression (no leading '/'), pass through
        if (!name.empty() && name[0] != '/') {
            return name;
        }

        // Fallback: strip leading '/' and replace '_' with '/'
        std::string key = name.substr(1);
        for (auto& c : key) {
            if (c == '_') c = '/';
        }
        spdlog::debug("[ZenohMessageBus] Unmapped topic '{}' → '{}'",
                      name, key);
        return key;
    }
};

}  // namespace drone::ipc

#endif  // HAVE_ZENOH
