// process4_mission_planner/include/planner/mission_fsm.h
// Finite State Machine for mission control.
#pragma once
#include "ipc/shm_types.h"
#include <string>
#include <spdlog/spdlog.h>

namespace drone::planner {

using MissionState = drone::ipc::MissionState;

inline const char* state_name(MissionState s) {
    switch (s) {
        case MissionState::IDLE:       return "IDLE";
        case MissionState::PREFLIGHT:  return "PREFLIGHT";
        case MissionState::TAKEOFF:    return "TAKEOFF";
        case MissionState::NAVIGATE:   return "NAVIGATE";
        case MissionState::LOITER:     return "LOITER";
        case MissionState::RTL:        return "RTL";
        case MissionState::LAND:       return "LAND";
        case MissionState::EMERGENCY:  return "EMERGENCY";
        default:                       return "UNKNOWN";
    }
}

/// Waypoint for mission navigation.
struct Waypoint {
    float x, y, z;     // target position in world frame
    float yaw;         // target heading
    float radius;      // acceptance radius (m)
    float speed;       // cruise speed (m/s)
    bool  trigger_payload;  // trigger camera/gimbal at this waypoint
};

/// Mission planner FSM — manages the mission lifecycle.
class MissionFSM {
public:
    MissionFSM() : state_(MissionState::IDLE) {}

    MissionState state() const { return state_; }

    /// Process an event and transition state.
    void on_arm()       { transition(MissionState::PREFLIGHT); }
    void on_takeoff()   { transition(MissionState::TAKEOFF); }
    void on_navigate()  { transition(MissionState::NAVIGATE); }
    void on_loiter()    { transition(MissionState::LOITER); }
    void on_rtl()       { transition(MissionState::RTL); }
    void on_land()      { transition(MissionState::LAND); }
    void on_landed()    { transition(MissionState::IDLE); fault_triggered_ = false; }
    void on_emergency() { transition(MissionState::EMERGENCY); }

    /// Check if waypoint is reached (within acceptance radius).
    bool waypoint_reached(float px, float py, float pz,
                          const Waypoint& wp) const {
        float dx = px - wp.x, dy = py - wp.y, dz = pz - wp.z;
        return (dx*dx + dy*dy + dz*dz) < (wp.radius * wp.radius);
    }

    /// Load a simple mission (list of waypoints).
    void load_mission(const std::vector<Waypoint>& waypoints) {
        waypoints_ = waypoints;
        current_wp_ = 0;
        spdlog::info("[FSM] Mission loaded: {} waypoints", waypoints_.size());
    }

    const Waypoint* current_waypoint() const {
        if (current_wp_ < waypoints_.size()) return &waypoints_[current_wp_];
        return nullptr;
    }

    bool advance_waypoint() {
        if (current_wp_ + 1 < waypoints_.size()) {
            ++current_wp_;
            spdlog::info("[FSM] Advanced to waypoint {}/{}", current_wp_ + 1,
                         waypoints_.size());
            return true;
        }
        return false;  // mission complete
    }

    size_t current_wp_index() const { return current_wp_; }
    size_t total_waypoints() const  { return waypoints_.size(); }

    /// Returns true if the FSM is in a fault-handling state (LOITER from
    /// fault, RTL, LAND, or EMERGENCY) and should not be overridden by
    /// normal mission logic.
    bool is_in_fault_state() const {
        return fault_triggered_ && (
            state_ == MissionState::LOITER ||
            state_ == MissionState::RTL   ||
            state_ == MissionState::LAND  ||
            state_ == MissionState::EMERGENCY);
    }

    /// Mark that the current state was caused by a fault (not a GCS
    /// command or normal mission completion).
    void set_fault_triggered(bool v) { fault_triggered_ = v; }

    /// Whether the current state was caused by a fault.
    bool fault_triggered() const { return fault_triggered_; }

private:
    MissionState state_;
    std::vector<Waypoint> waypoints_;
    size_t current_wp_ = 0;
    bool fault_triggered_ = false;

    void transition(MissionState new_state) {
        spdlog::info("[FSM] {} → {}", state_name(state_), state_name(new_state));
        state_ = new_state;
    }
};

} // namespace drone::planner
