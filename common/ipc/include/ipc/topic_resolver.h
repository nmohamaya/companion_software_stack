// common/ipc/include/ipc/topic_resolver.h
// Resolves IPC topic names with an optional vehicle_id prefix.
//
// Multi-vehicle support: when vehicle_id is set, all topics are
// namespaced under "/<vehicle_id>/..." so that multiple vehicles
// on the same Zenoh network don't interfere.
//
// Empty vehicle_id (the default) produces IDENTICAL behavior to
// pre-TopicResolver code — no prefix, no behavioral change.
#pragma once

#include <string>

namespace drone::ipc {

/// Resolves IPC topic names with optional vehicle_id prefix.
///
/// Empty vehicle_id = no prefix = fully backward compatible.
/// Non-empty vehicle_id prepends "/<vehicle_id>" to every topic.
///
/// Example:
///   TopicResolver r("drone42");
///   r.resolve("/slam_pose")       -> "/drone42/slam_pose"
///   r.resolve("/fc_commands")     -> "/drone42/fc_commands"
///
///   TopicResolver r_default;      // empty vehicle_id
///   r_default.resolve("/slam_pose") -> "/slam_pose"  (unchanged)
class TopicResolver {
public:
    explicit TopicResolver(std::string vehicle_id = "") : vehicle_id_(std::move(vehicle_id)) {}

    /// Resolve a base topic name to a namespaced topic.
    /// @param base_topic  The base topic (e.g. "/slam_pose").
    /// @return  Namespaced topic, or base_topic unchanged if no vehicle_id.
    [[nodiscard]] std::string resolve(const std::string& base_topic) const {
        if (vehicle_id_.empty()) {
            return base_topic;
        }
        return "/" + vehicle_id_ + base_topic;
    }

    /// Returns the configured vehicle_id (empty string if none).
    [[nodiscard]] const std::string& vehicle_id() const { return vehicle_id_; }

    /// Returns true if a vehicle_id prefix is configured.
    [[nodiscard]] bool has_prefix() const { return !vehicle_id_.empty(); }

private:
    std::string vehicle_id_;
};

}  // namespace drone::ipc
