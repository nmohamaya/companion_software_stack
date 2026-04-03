// process4_mission_planner/include/planner/fault_response_executor.h
// Executes fault response actions (WARN, LOITER, RTL, EMERGENCY_LAND).
// Escalation-only: never downgrades from a previously applied action.
//
// Extracted from main.cpp as part of Issue #154.
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "planner/fault_manager.h"
#include "planner/gcs_command_handler.h"
#include "planner/mission_fsm.h"

#include <chrono>
#include <cstdint>

#include <spdlog/spdlog.h>

namespace drone::planner {

/// Executes fault response actions based on FaultManager evaluation.
/// Only applies escalated actions (never downgrades).
/// Skips non-airborne states (IDLE, PREFLIGHT).
class FaultResponseExecutor {
public:
    /// Execute the fault response if the action has escalated.
    void execute(const FaultState& fault, MissionFSM& fsm, const FCSendFn& send_fc,
                 drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& traj_pub,
                 SharedFlightState& flight_state, uint64_t now_ns) {
        if (fault.recommended_action <= last_fault_action_ ||
            fault.recommended_action <= FaultAction::NONE || fsm.state() == MissionState::IDLE ||
            fsm.state() == MissionState::PREFLIGHT)
            return;

        // During COLLISION_RECOVERY, only allow high-severity escalation (RTL, EMERGENCY_LAND).
        // Lower-severity actions (WARN, LOITER) are skipped to avoid disrupting the recovery.
        if (fsm.state() == MissionState::COLLISION_RECOVERY &&
            fault.recommended_action < FaultAction::RTL)
            return;

        spdlog::warn("[FaultMgr] Escalation: {} → {} (reason: {}) active_faults=[{}]",
                     fault_action_name(last_fault_action_),
                     fault_action_name(fault.recommended_action), fault.reason,
                     drone::ipc::fault_flags_string(fault.active_faults));

        switch (fault.recommended_action) {
            case FaultAction::WARN:
                // Log only — continue mission
                break;
            case FaultAction::LOITER:
                if (fsm.state() == MissionState::TAKEOFF || fsm.state() == MissionState::NAVIGATE) {
                    publish_stop_trajectory(traj_pub, now_ns);
                    fsm.on_loiter();
                    fsm.set_fault_triggered(true);
                }
                break;
            case FaultAction::RTL:
                send_fc(drone::ipc::FCCommandType::RTL, 0.0f);
                publish_stop_trajectory(traj_pub, now_ns);
                flight_state.rtl_start_time = std::chrono::steady_clock::now();
                flight_state.nav_was_armed  = true;
                fsm.on_rtl();
                fsm.set_fault_triggered(true);
                break;
            case FaultAction::EMERGENCY_LAND:
                send_fc(drone::ipc::FCCommandType::LAND, 0.0f);
                publish_stop_trajectory(traj_pub, now_ns);
                flight_state.land_sent = true;
                fsm.on_land();
                fsm.set_fault_triggered(true);
                break;
            default: break;
        }
        last_fault_action_ = fault.recommended_action;
    }

    /// Log when new fault flags appear (even without action-level change).
    void log_new_faults(uint32_t active_faults) {
        uint32_t new_flags = active_faults & ~last_active_faults_;
        if (new_flags != 0) {
            spdlog::warn("[FaultMgr] New faults: [{}] active_faults=[{}]",
                         drone::ipc::fault_flags_string(new_flags),
                         drone::ipc::fault_flags_string(active_faults));
        }
        last_active_faults_ = active_faults;
    }

    /// Reset after landing (for next mission).
    void reset() {
        last_fault_action_  = FaultAction::NONE;
        last_active_faults_ = 0;
    }

    /// Last applied fault action.
    [[nodiscard]] FaultAction last_action() const { return last_fault_action_; }

private:
    FaultAction last_fault_action_  = FaultAction::NONE;
    uint32_t    last_active_faults_ = 0;

    static void publish_stop_trajectory(drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& pub,
                                        uint64_t                                           now_ns) {
        drone::ipc::TrajectoryCmd stop{};
        stop.valid        = true;  // P5 skips valid=false — send zero-velocity to stop
        stop.timestamp_ns = now_ns;
        pub.publish(stop);
    }
};

}  // namespace drone::planner
