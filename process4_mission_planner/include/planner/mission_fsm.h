// process4_mission_planner/include/planner/mission_fsm.h
// Finite State Machine for mission control.
#pragma once
#include "ipc/ipc_types.h"

#include <string>

#include <spdlog/spdlog.h>

namespace drone::planner {

using MissionState = drone::ipc::MissionState;

inline const char* state_name(MissionState s) {
    switch (s) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::PREFLIGHT: return "PREFLIGHT";
        case MissionState::TAKEOFF: return "TAKEOFF";
        case MissionState::NAVIGATE: return "NAVIGATE";
        case MissionState::LOITER: return "LOITER";
        case MissionState::RTL: return "RTL";
        case MissionState::LAND: return "LAND";
        case MissionState::EMERGENCY: return "EMERGENCY";
        case MissionState::SURVEY: return "SURVEY";
        default: return "UNKNOWN";
    }
}

/// Waypoint for mission navigation.
struct Waypoint {
    float x, y, z;          // target position in world frame
    float yaw;              // target heading
    float radius;           // acceptance radius (m)
    float speed;            // cruise speed (m/s)
    bool  trigger_payload;  // trigger camera/gimbal at this waypoint
};

/// Mission planner FSM — manages the mission lifecycle.
class MissionFSM {
public:
    explicit MissionFSM(float overshoot_proximity_factor = 3.0f)
        : state_(MissionState::IDLE), overshoot_proximity_factor_(overshoot_proximity_factor) {}

    [[nodiscard]] MissionState state() const { return state_; }

    /// Process an event and transition state.
    void on_arm() { transition(MissionState::PREFLIGHT); }
    void on_takeoff() { transition(MissionState::TAKEOFF); }
    void on_survey() { transition(MissionState::SURVEY); }
    void on_navigate() { transition(MissionState::NAVIGATE); }
    void on_loiter() { transition(MissionState::LOITER); }
    void on_rtl() { transition(MissionState::RTL); }
    void on_land() { transition(MissionState::LAND); }
    void on_landed() {
        transition(MissionState::IDLE);
        fault_triggered_ = false;
    }
    void on_emergency() { transition(MissionState::EMERGENCY); }

    /// Check if waypoint is reached (within acceptance radius).
    [[nodiscard]] bool waypoint_reached(float px, float py, float pz, const Waypoint& wp) const {
        float dx = px - wp.x, dy = py - wp.y, dz = pz - wp.z;
        return (dx * dx + dy * dy + dz * dz) < (wp.radius * wp.radius);
    }

    /// Load a simple mission (list of waypoints).
    void load_mission(const std::vector<Waypoint>& waypoints) {
        waypoints_  = waypoints;
        current_wp_ = 0;
        spdlog::info("[FSM] Mission loaded: {} waypoints", waypoints_.size());
    }

    [[nodiscard]] const Waypoint* current_waypoint() const {
        if (current_wp_ < waypoints_.size()) return &waypoints_[current_wp_];
        return nullptr;
    }

    [[nodiscard]] const Waypoint* next_waypoint() const {
        if (current_wp_ + 1 < waypoints_.size()) return &waypoints_[current_wp_ + 1];
        return nullptr;
    }

    /// Check if the drone has passed the current waypoint along the approach
    /// vector toward the next waypoint (dot-product sign check).
    /// Only triggers when the drone is within `overshoot_proximity_factor` ×
    /// acceptance_radius of the waypoint — prevents premature advancement when
    /// the drone is far away and merely "ahead" in the dot-product sense.
    /// Returns false for the last waypoint — it always requires acceptance radius.
    [[nodiscard]] bool waypoint_overshot(float px, float py, float pz) const {
        const Waypoint* wp = current_waypoint();
        if (!wp || current_wp_ == 0) return false;               // No overshoot for first WP
        if (current_wp_ + 1 >= waypoints_.size()) return false;  // Last WP needs acceptance

        // Drone offset from current WP
        float dx = px - wp->x, dy = py - wp->y, dz = pz - wp->z;
        float dist_sq = dx * dx + dy * dy + dz * dz;

        // Must be within proximity zone to qualify (configurable via overshoot_proximity_factor)
        float proximity_r = wp->radius * overshoot_proximity_factor_;
        if (dist_sq > proximity_r * proximity_r) return false;

        // Approach vector: previous_wp → current_wp (direction drone was traveling)
        const auto& prev = waypoints_[current_wp_ - 1];
        float       ax = wp->x - prev.x, ay = wp->y - prev.y, az = wp->z - prev.z;

        // Positive dot product means drone is past WP along the approach direction
        return (dx * ax + dy * ay + dz * az) > 0.0f;
    }

    [[nodiscard]] bool advance_waypoint() {
        if (current_wp_ + 1 < waypoints_.size()) {
            ++current_wp_;
            spdlog::info("[FSM] Advanced to waypoint {}/{}", current_wp_ + 1, waypoints_.size());
            return true;
        }
        return false;  // mission complete
    }

    [[nodiscard]] size_t current_wp_index() const { return current_wp_; }
    [[nodiscard]] size_t total_waypoints() const { return waypoints_.size(); }

    /// Returns true if the FSM is in a fault-handling state (LOITER from
    /// fault, RTL, LAND, or EMERGENCY) and should not be overridden by
    /// normal mission logic.
    [[nodiscard]] bool is_in_fault_state() const {
        return fault_triggered_ &&
               (state_ == MissionState::LOITER || state_ == MissionState::RTL ||
                state_ == MissionState::LAND || state_ == MissionState::EMERGENCY);
    }

    /// Mark that the current state was caused by a fault (not a GCS
    /// command or normal mission completion).
    void set_fault_triggered(bool v) { fault_triggered_ = v; }

    /// Whether the current state was caused by a fault.
    [[nodiscard]] bool fault_triggered() const { return fault_triggered_; }

private:
    MissionState          state_;
    float                 overshoot_proximity_factor_{3.0f};
    std::vector<Waypoint> waypoints_;
    size_t                current_wp_      = 0;
    bool                  fault_triggered_ = false;

    void transition(MissionState new_state) {
        spdlog::info("[FSM] {} → {}", state_name(state_), state_name(new_state));
        state_ = new_state;
    }
};

}  // namespace drone::planner
