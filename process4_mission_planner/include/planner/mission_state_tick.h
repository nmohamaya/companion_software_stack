// process4_mission_planner/include/planner/mission_state_tick.h
// Per-tick state machine logic for the mission planner.
// Handles PREFLIGHT, TAKEOFF, NAVIGATE, RTL, LAND state transitions.
//
// Extracted from main.cpp as part of Issue #154.
// Updated in Issue #158: AStarPathPlanner* → IGridPlanner*.
#pragma once

#include "ipc/ipc_types.h"
#include "ipc/ipublisher.h"
#include "planner/gcs_command_handler.h"
#include "planner/grid_planner_base.h"
#include "planner/iobstacle_avoider.h"
#include "planner/ipath_planner.h"
#include "planner/mission_fsm.h"
#include "planner/static_obstacle_layer.h"
#include "util/diagnostic.h"
#include "util/scoped_timer.h"

#include <chrono>
#include <cmath>
#include <cstdint>

#include <spdlog/spdlog.h>

namespace drone::planner {

/// Configuration for MissionStateTick tunables.
struct StateTickConfig {
    float takeoff_alt_m{10.0f};
    float rtl_acceptance_m{1.5f};
    float landed_alt_m{0.5f};
    int   rtl_min_dwell_s{5};
};

/// Per-tick state machine logic for the mission planner.
/// Owns tracking variables and implements the FSM tick for each state.
class MissionStateTick {
public:
    explicit MissionStateTick(const StateTickConfig& config) : config_(config) {}

    /// Execute one tick of the state machine.
    void tick(MissionFSM& fsm, const drone::ipc::Pose& pose, const drone::ipc::FCState& fc_state,
              const drone::ipc::DetectedObjectList& objects, IPathPlanner& planner,
              IGridPlanner* grid_planner, IObstacleAvoider& avoider,
              StaticObstacleLayer&                                obstacle_layer,
              drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>&  traj_pub,
              drone::ipc::IPublisher<drone::ipc::PayloadCommand>& payload_pub,
              const FCSendFn& send_fc, uint64_t correlation_id,
              drone::util::FrameDiagnostics& diag) {
        // Record home position from the first real pose, regardless of state.
        try_record_home(pose);

        switch (fsm.state()) {
            case MissionState::PREFLIGHT: tick_preflight(fsm, fc_state, send_fc); break;
            case MissionState::TAKEOFF: tick_takeoff(fsm, pose, fc_state, send_fc); break;
            case MissionState::NAVIGATE:
                tick_navigate(fsm, pose, fc_state, objects, planner, grid_planner, avoider,
                              obstacle_layer, traj_pub, payload_pub, send_fc, correlation_id, diag);
                break;
            case MissionState::RTL: tick_rtl(fsm, pose, fc_state, send_fc); break;
            case MissionState::LAND: tick_land(fsm, fc_state); break;
            case MissionState::IDLE:
            case MissionState::EMERGENCY:
            default: break;
        }
    }

    /// Access shared flight state (for GCS handler + fault executor).
    [[nodiscard]] SharedFlightState&       flight_state() { return flight_state_; }
    [[nodiscard]] const SharedFlightState& flight_state() const { return flight_state_; }

    /// Whether a fault-state reset happened this tick (caller reads + clears).
    [[nodiscard]] bool consume_fault_reset() {
        bool v            = fault_exec_reset_;
        fault_exec_reset_ = false;
        return v;
    }

private:
    StateTickConfig   config_;
    SharedFlightState flight_state_;

    // Tracking variables
    bool     takeoff_sent_     = false;
    float    home_x_           = 0.0f;
    float    home_y_           = 0.0f;
    float    home_z_           = 0.0f;
    bool     home_recorded_    = false;
    bool     home_warn_logged_ = false;
    bool     fault_exec_reset_ = false;
    uint64_t debug_tick_       = 0;  // DEBUG(#234): periodic avoider comparison logging

    std::chrono::steady_clock::time_point last_arm_time_ = std::chrono::steady_clock::now() -
                                                           std::chrono::seconds(10);

    // ── PREFLIGHT: retry ARM until FC confirms ────────────────
    void tick_preflight(MissionFSM& fsm, const drone::ipc::FCState& fc_state,
                        const FCSendFn& send_fc) {
        auto now_arm = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now_arm - last_arm_time_).count() >=
            3) {
            spdlog::info("[Planner] Sending ARM command");
            send_fc(drone::ipc::FCCommandType::ARM, 0.0f);
            last_arm_time_ = now_arm;
        }
        if (fc_state.armed) {
            spdlog::info("[Planner] Vehicle armed — initiating takeoff");
            fsm.on_takeoff();
            takeoff_sent_ = false;
        }
    }

    // ── Record home from the first real pose (any state) ──────
    void try_record_home(const drone::ipc::Pose& pose) {
        if (home_recorded_) return;
        // Only accept poses with a non-zero timestamp (i.e. actually received
        // from the SLAM/VIO pipeline, not a default-constructed Pose{}).
        if (pose.timestamp_ns == 0) return;
        if (!std::isfinite(pose.translation[0]) || !std::isfinite(pose.translation[1])) return;

        home_x_        = static_cast<float>(pose.translation[0]);
        home_y_        = static_cast<float>(pose.translation[1]);
        home_z_        = 0.0f;
        home_recorded_ = true;
        spdlog::info("[Planner] Home position recorded: ({:.1f}, {:.1f}, {:.1f})", home_x_, home_y_,
                     home_z_);
    }

    // ── TAKEOFF: send TAKEOFF, wait for altitude + home ─────────
    void tick_takeoff(MissionFSM&                fsm, const drone::ipc::Pose& /*pose*/,
                      const drone::ipc::FCState& fc_state, const FCSendFn& send_fc) {
        if (!takeoff_sent_) {
            spdlog::info("[Planner] Sending TAKEOFF to {:.1f}m", config_.takeoff_alt_m);
            send_fc(drone::ipc::FCCommandType::TAKEOFF, config_.takeoff_alt_m);
            takeoff_sent_ = true;
        }
        if (fc_state.rel_alt >= config_.takeoff_alt_m * 0.9f) {
            if (!home_recorded_) {
                if (!home_warn_logged_) {
                    spdlog::warn("[Planner] Takeoff altitude reached but no valid pose "
                                 "received yet — deferring NAVIGATE until home is recorded");
                    home_warn_logged_ = true;
                }
                return;
            }
            spdlog::info("[Planner] Takeoff complete (alt={:.1f}m) — NAVIGATE", fc_state.rel_alt);
            fsm.on_navigate();
            spdlog::info("[FSM] EXECUTING — navigating {} waypoints", fsm.total_waypoints());
        }
    }

    // ── NAVIGATE: waypoint tracking, collision detect, plan+avoid ─
    void tick_navigate(MissionFSM& fsm, const drone::ipc::Pose& pose,
                       const drone::ipc::FCState&            fc_state,
                       const drone::ipc::DetectedObjectList& objects, IPathPlanner& planner,
                       IGridPlanner* grid_planner, IObstacleAvoider& avoider,
                       StaticObstacleLayer&                                obstacle_layer,
                       drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>&  traj_pub,
                       drone::ipc::IPublisher<drone::ipc::PayloadCommand>& payload_pub,
                       const FCSendFn& send_fc, uint64_t correlation_id,
                       drone::util::FrameDiagnostics& diag) {
        // Detect unexpected disarm mid-navigation
        if (flight_state_.nav_was_armed && !fc_state.armed) {
            spdlog::warn("[Planner] OBSTACLE COLLISION detected — vehicle unexpectedly "
                         "disarmed during navigation");
        }
        flight_state_.nav_was_armed = fc_state.armed;

        // Proximity collision detection
        {
            const float px = static_cast<float>(pose.translation[0]);
            const float py = static_cast<float>(pose.translation[1]);
            const float pz = static_cast<float>(pose.translation[2]);
            obstacle_layer.check_collision(px, py, pz, std::chrono::steady_clock::now());
        }

        // Update planner obstacle grid
        if (grid_planner) {
            drone::util::ScopedDiagTimer t(diag, "GridUpdate");
            grid_planner->update_obstacles(objects, pose);
        }

        // Warn about approaching unconfirmed obstacles
        {
            const float px = static_cast<float>(pose.translation[0]);
            const float py = static_cast<float>(pose.translation[1]);
            obstacle_layer.check_unconfirmed_approach(px, py, std::chrono::steady_clock::now());
        }

        const Waypoint* wp = fsm.current_waypoint();
        if (!wp) return;

        {
            auto planned = [&]() {
                drone::util::ScopedDiagTimer t(diag, "PathPlan");
                return planner.plan(pose, *wp);
            }();
            if (grid_planner && grid_planner->using_direct_fallback()) {
                diag.add_warning("PathPlan",
                                 "Planner fallback: no obstacle-free path — using direct line");
            }
            auto traj = [&]() {
                drone::util::ScopedDiagTimer t(diag, "ObstacleAvoid");
                return avoider.avoid(planned, pose, objects);
            }();

            // DEBUG(#234): Compare planner output vs avoider output to detect
            // whether the avoider is deflecting the drone away from the planned path.
            if (debug_tick_++ % 30 == 0) {
                float dvx  = traj.velocity_x - planned.velocity_x;
                float dvy  = traj.velocity_y - planned.velocity_y;
                float dvz  = traj.velocity_z - planned.velocity_z;
                float dmag = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
                spdlog::info(
                    "[DEBUG] plan_vel=({:.2f},{:.2f},{:.2f}) avoid_vel=({:.2f},{:.2f},{:.2f})"
                    " delta=({:.2f},{:.2f},{:.2f}) |d|={:.2f} wp={}/{}",
                    planned.velocity_x, planned.velocity_y, planned.velocity_z, traj.velocity_x,
                    traj.velocity_y, traj.velocity_z, dvx, dvy, dvz, dmag,
                    fsm.current_wp_index() + 1, fsm.total_waypoints());
            }

            traj_pub.publish(traj);

            const float px = static_cast<float>(pose.translation[0]);
            const float py = static_cast<float>(pose.translation[1]);
            const float pz = static_cast<float>(pose.translation[2]);
            if (fsm.waypoint_reached(px, py, pz, *wp) || fsm.waypoint_overshot(px, py, pz)) {
                spdlog::info("[Planner] Waypoint {} {}!", fsm.current_wp_index() + 1,
                             fsm.waypoint_overshot(px, py, pz) ? "overshot" : "reached");

                if (wp->trigger_payload) {
                    drone::ipc::PayloadCommand pay_cmd{};
                    pay_cmd.timestamp_ns   = traj.timestamp_ns;
                    pay_cmd.correlation_id = correlation_id;
                    pay_cmd.action         = drone::ipc::PayloadAction::CAMERA_CAPTURE;
                    pay_cmd.gimbal_pitch   = -90.0f;
                    pay_cmd.gimbal_yaw     = 0.0f;
                    pay_cmd.valid          = true;
                    payload_pub.publish(pay_cmd);
                }

                if (!fsm.advance_waypoint()) {
                    spdlog::info("[Planner] Mission complete — RTL");
                    send_fc(drone::ipc::FCCommandType::RTL, 0.0f);
                    {
                        drone::ipc::TrajectoryCmd stop{};
                        stop.valid        = false;
                        stop.timestamp_ns = static_cast<uint64_t>(
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count());
                        traj_pub.publish(stop);
                    }
                    flight_state_.rtl_start_time = std::chrono::steady_clock::now();
                    flight_state_.nav_was_armed  = true;
                    fsm.on_rtl();
                }
            }
        }
    }

    // ── RTL: detect disarm, monitor position for LAND ─────────
    void tick_rtl(MissionFSM& fsm, const drone::ipc::Pose& pose,
                  const drone::ipc::FCState& fc_state, const FCSendFn& send_fc) {
        if (flight_state_.nav_was_armed && !fc_state.armed) {
            spdlog::warn("[Planner] Vehicle disarmed during RTL — mission IDLE");
            fsm.on_landed();
            return;
        }
        flight_state_.nav_was_armed = fc_state.armed;

        if (home_recorded_) {
            auto rtl_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                   std::chrono::steady_clock::now() - flight_state_.rtl_start_time)
                                   .count();
            float dx         = static_cast<float>(pose.translation[0]) - home_x_;
            float dy         = static_cast<float>(pose.translation[1]) - home_y_;
            float horiz_dist = std::sqrt(dx * dx + dy * dy);
            if (rtl_elapsed >= config_.rtl_min_dwell_s && horiz_dist < config_.rtl_acceptance_m) {
                spdlog::info("[Planner] Near home ({:.1f}m, {}s in RTL) — sending LAND", horiz_dist,
                             rtl_elapsed);
                send_fc(drone::ipc::FCCommandType::LAND, 0.0f);
                flight_state_.land_sent = true;
                fsm.on_land();
            }
        }
    }

    // ── LAND: wait for touchdown ──────────────────────────────
    void tick_land(MissionFSM& fsm, const drone::ipc::FCState& fc_state) {
        if (fc_state.rel_alt < config_.landed_alt_m && flight_state_.land_sent) {
            spdlog::info("[Planner] Landed (alt={:.2f}m) — mission IDLE", fc_state.rel_alt);
            fsm.on_landed();
            fault_exec_reset_ = true;
        }
    }
};

}  // namespace drone::planner
