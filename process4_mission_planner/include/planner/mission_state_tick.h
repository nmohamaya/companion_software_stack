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
    float survey_duration_s{0.0f};  // Post-takeoff obstacle survey duration (0 = skip)
    float survey_yaw_rate{0.3f};    // Yaw rate during survey (rad/s, ~0.3 = full 360 in ~21s)

    // Collision recovery (Issue #226)
    bool  collision_recovery_enabled{true};
    float collision_climb_delta_m{3.0f};     // altitude gain during recovery climb
    float collision_hover_duration_s{2.0f};  // hover-in-place duration before climb
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

        // Pause promotion during RTL/LAND — the drone is descending to a
        // known-safe location and ground-feature detections would pollute the
        // static layer, blocking the landing approach (Issue #340).
        if (grid_planner != nullptr) {
            const bool landing = (fsm.state() == MissionState::RTL ||
                                  fsm.state() == MissionState::LAND);
            grid_planner->set_promotion_paused(landing);
        }

        switch (fsm.state()) {
            case MissionState::PREFLIGHT: tick_preflight(fsm, fc_state, send_fc); break;
            case MissionState::TAKEOFF: tick_takeoff(fsm, pose, fc_state, send_fc); break;
            case MissionState::SURVEY:
                tick_survey(fsm, pose, objects, grid_planner, traj_pub);
                break;
            case MissionState::NAVIGATE:
                tick_navigate(fsm, pose, fc_state, objects, planner, grid_planner, avoider,
                              obstacle_layer, traj_pub, payload_pub, send_fc, correlation_id, diag);
                break;
            case MissionState::COLLISION_RECOVERY:
                tick_collision_recovery(fsm, pose, fc_state, grid_planner, traj_pub);
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

    // Survey state
    bool                                  survey_started_ = false;
    std::chrono::steady_clock::time_point survey_start_time_{};
    float                                 survey_start_yaw_ = 0.0f;
    uint64_t                              survey_log_tick_  = 0;

    std::chrono::steady_clock::time_point last_arm_time_ = std::chrono::steady_clock::now() -
                                                           std::chrono::seconds(10);

    // Collision recovery state (Issue #226)
    enum class RecoveryPhase : uint8_t { HOVER = 0, CLIMB = 1, REPLAN = 2 };
    bool                                  recovery_started_ = false;
    RecoveryPhase                         recovery_phase_   = RecoveryPhase::HOVER;
    std::chrono::steady_clock::time_point recovery_start_time_{};
    float                                 recovery_target_alt_ = 0.0f;

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
            if (config_.survey_duration_s > 0.0f) {
                spdlog::info("[Planner] Takeoff complete (alt={:.1f}m) — SURVEY for {:.0f}s",
                             fc_state.rel_alt, config_.survey_duration_s);
                fsm.on_survey();
            } else {
                spdlog::info("[Planner] Takeoff complete (alt={:.1f}m) — NAVIGATE",
                             fc_state.rel_alt);
                fsm.on_navigate();
                spdlog::info("[FSM] EXECUTING — navigating {} waypoints", fsm.total_waypoints());
            }
        }
    }

    // ── SURVEY: hover + slow yaw rotation to detect obstacles ────
    void tick_survey(MissionFSM& fsm, const drone::ipc::Pose& pose,
                     const drone::ipc::DetectedObjectList& objects, IGridPlanner* grid_planner,
                     drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& traj_pub) {
        auto now = std::chrono::steady_clock::now();
        if (!survey_started_) {
            survey_start_time_ = now;
            survey_started_    = true;
            // Extract initial yaw from pose quaternion [w, x, y, z]
            double qw = pose.quaternion[0], qx = pose.quaternion[1];
            double qy = pose.quaternion[2], qz = pose.quaternion[3];
            survey_start_yaw_ = static_cast<float>(
                std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
            spdlog::info("[Survey] Start yaw={:.2f} rad, rotating at {:.2f} rad/s for {:.0f}s",
                         survey_start_yaw_, config_.survey_yaw_rate, config_.survey_duration_s);
        }

        // Feed detections into the occupancy grid during survey
        if (grid_planner) {
            grid_planner->update_obstacles(objects, pose);
        }

        float elapsed_s = std::chrono::duration<float>(now - survey_start_time_).count();

        // Publish hover command with incrementing yaw for slow rotation.
        // IFCLink::send_trajectory only accepts absolute yaw (no yaw_rate),
        // so we compute the target yaw from elapsed time × yaw rate.
        // Two-phase survey: first half CW, second half CCW — sees each obstacle
        // from opposite directions, improving radar angular coverage.
        const float half_dur = config_.survey_duration_s * 0.5f;
        float       target_yaw;
        if (elapsed_s <= half_dur) {
            // Phase 1: clockwise
            target_yaw = survey_start_yaw_ + config_.survey_yaw_rate * elapsed_s;
        } else {
            // Phase 2: counter-clockwise from where phase 1 ended
            const float phase1_end = survey_start_yaw_ + config_.survey_yaw_rate * half_dur;
            target_yaw             = phase1_end - config_.survey_yaw_rate * (elapsed_s - half_dur);
        }
        // Wrap to [-π, π] to avoid sending unbounded angles to the FC.
        constexpr float kPiF = 3.14159265358979323846f;
        target_yaw           = std::fmod(target_yaw + kPiF, 2.0f * kPiF);
        if (target_yaw < 0.0f) target_yaw += 2.0f * kPiF;
        target_yaw -= kPiF;

        drone::ipc::TrajectoryCmd cmd{};
        cmd.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        cmd.valid      = true;
        cmd.velocity_x = 0.0f;
        cmd.velocity_y = 0.0f;
        cmd.velocity_z = 0.0f;
        cmd.target_yaw = target_yaw;
        cmd.target_x   = static_cast<float>(pose.translation[0]);
        cmd.target_y   = static_cast<float>(pose.translation[1]);
        cmd.target_z   = static_cast<float>(pose.translation[2]);
        traj_pub.publish(cmd);

        // Access grid diagnostics via GridPlannerBase
        auto* base = dynamic_cast<GridPlannerBase*>(grid_planner);

        // Log progress periodically
        if (survey_log_tick_++ % 30 == 0) {
            int promoted = base ? base->grid().promoted_count() : 0;
            int static_n = base ? static_cast<int>(base->grid().static_count()) : 0;
            spdlog::info("[Survey] {:.0f}/{:.0f}s — {} static cells ({} promoted), yaw_rate={:.2f}",
                         elapsed_s, config_.survey_duration_s, static_n, promoted,
                         config_.survey_yaw_rate);
        }

        // Survey complete — transition to NAVIGATE
        if (elapsed_s >= config_.survey_duration_s) {
            int promoted = base ? base->grid().promoted_count() : 0;
            int static_n = base ? static_cast<int>(base->grid().static_count()) : 0;
            spdlog::info("[Planner] Survey complete ({:.0f}s) — {} static obstacles ({} promoted)"
                         " — NAVIGATE",
                         elapsed_s, static_n, promoted);
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
        // Detect unexpected disarm mid-navigation.
        // A disarmed vehicle cannot execute recovery commands (hover/climb), so
        // transition to IDLE rather than COLLISION_RECOVERY. Recovery is only
        // useful when triggered by proximity detection while still armed.
        if (flight_state_.nav_was_armed && !fc_state.armed) {
            spdlog::warn("[Planner] Vehicle unexpectedly disarmed during navigation — "
                         "cannot recover without motors, transitioning to IDLE");
            publish_stop_trajectory(traj_pub, correlation_id);
            flight_state_.nav_was_armed = fc_state.armed;
            fsm.on_landed();
            return;
        }
        flight_state_.nav_was_armed = fc_state.armed;

        // Proximity collision detection
        {
            const float px        = static_cast<float>(pose.translation[0]);
            const float py        = static_cast<float>(pose.translation[1]);
            const float pz        = static_cast<float>(pose.translation[2]);
            bool        collision = obstacle_layer.check_collision(px, py, pz,
                                                                   std::chrono::steady_clock::now());
            if (collision && config_.collision_recovery_enabled) {
                spdlog::info("[Planner] Proximity collision — entering COLLISION_RECOVERY");
                publish_stop_trajectory(traj_pub, correlation_id);
                recovery_started_ = false;
                fsm.on_collision_recovery();
                return;
            }
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

            // Diagnostic every 10 ticks (~1s at 10Hz) — gated by log level
            if (debug_tick_++ % 10 == 0) {
                const float dpx        = static_cast<float>(pose.translation[0]);
                const float dpy        = static_cast<float>(pose.translation[1]);
                const float dpz        = static_cast<float>(pose.translation[2]);
                float       dvx        = traj.velocity_x - planned.velocity_x;
                float       dvy        = traj.velocity_y - planned.velocity_y;
                float       dvz        = traj.velocity_z - planned.velocity_z;
                float       dmag       = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
                float       dist_to_wp = std::sqrt((dpx - wp->x) * (dpx - wp->x) +
                                                   (dpy - wp->y) * (dpy - wp->y) +
                                                   (dpz - wp->z) * (dpz - wp->z));
                auto*       base       = dynamic_cast<GridPlannerBase*>(grid_planner);
                int         occ  = base ? static_cast<int>(base->grid().occupied_count()) : -1;
                int         stat = base ? static_cast<int>(base->grid().static_count()) : -1;
                int         prom = base ? base->grid().promoted_count() : -1;
                bool        fb   = grid_planner && grid_planner->using_direct_fallback();
                spdlog::debug("[DIAG] pos=({:.1f},{:.1f},{:.1f}) wp{}/{}=({:.0f},{:.0f},{:.0f})"
                              " dist={:.1f}m plan_v=({:.2f},{:.2f},{:.2f})"
                              " avoid_v=({:.2f},{:.2f},{:.2f}) |delta|={:.2f}"
                              " grid: occ={} static={} promoted={} fallback={}",
                              dpx, dpy, dpz, fsm.current_wp_index() + 1, fsm.total_waypoints(),
                              wp->x, wp->y, wp->z, dist_to_wp, planned.velocity_x,
                              planned.velocity_y, planned.velocity_z, traj.velocity_x,
                              traj.velocity_y, traj.velocity_z, dmag, occ, stat, prom, fb);
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
                    publish_stop_trajectory(traj_pub, correlation_id);
                    flight_state_.rtl_start_time = std::chrono::steady_clock::now();
                    flight_state_.nav_was_armed  = true;
                    fsm.on_rtl();
                }
            }
        }
    }

    // ── COLLISION_RECOVERY: hover → climb → replan → NAVIGATE (Issue #226) ──
    void tick_collision_recovery(MissionFSM& fsm, const drone::ipc::Pose& pose,
                                 const drone::ipc::FCState& fc_state, IGridPlanner* grid_planner,
                                 drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& traj_pub) {
        // Safety: abort recovery if disarmed or FC disconnected
        if (!fc_state.armed || !fc_state.connected) {
            spdlog::warn("[Recovery] FC {} during recovery — aborting to IDLE",
                         !fc_state.armed ? "disarmed" : "disconnected");
            recovery_started_ = false;
            fsm.on_landed();
            return;
        }

        auto now = std::chrono::steady_clock::now();
        if (!recovery_started_) {
            recovery_start_time_ = now;
            recovery_phase_      = RecoveryPhase::HOVER;
            recovery_target_alt_ = static_cast<float>(pose.translation[2]) +
                                   config_.collision_climb_delta_m;
            recovery_started_ = true;
            spdlog::info("[Recovery] Phase HOVER — holding position for {:.1f}s, "
                         "then climbing to {:.1f}m",
                         config_.collision_hover_duration_s, recovery_target_alt_);
        }

        const float px        = static_cast<float>(pose.translation[0]);
        const float py        = static_cast<float>(pose.translation[1]);
        const float pz        = static_cast<float>(pose.translation[2]);
        float       elapsed_s = std::chrono::duration<float>(now - recovery_start_time_).count();

        // Extract current yaw from pose quaternion [w, x, y, z] to avoid yaw snap
        double qw = pose.quaternion[0], qx = pose.quaternion[1];
        double qy = pose.quaternion[2], qz = pose.quaternion[3];
        float  current_yaw = static_cast<float>(
            std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));

        switch (recovery_phase_) {
            case RecoveryPhase::HOVER: {
                // Publish hover-in-place command
                drone::ipc::TrajectoryCmd cmd{};
                cmd.timestamp_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                        .count());
                cmd.valid      = true;
                cmd.target_x   = px;
                cmd.target_y   = py;
                cmd.target_z   = pz;
                cmd.target_yaw = current_yaw;
                cmd.velocity_x = 0.0f;
                cmd.velocity_y = 0.0f;
                cmd.velocity_z = 0.0f;
                traj_pub.publish(cmd);

                if (elapsed_s >= config_.collision_hover_duration_s) {
                    recovery_phase_ = RecoveryPhase::CLIMB;
                    spdlog::info("[Recovery] Phase CLIMB — ascending {:.1f}m to {:.1f}m",
                                 config_.collision_climb_delta_m, recovery_target_alt_);
                }
                break;
            }
            case RecoveryPhase::CLIMB: {
                // Publish climb command — move to recovery_target_alt_
                drone::ipc::TrajectoryCmd cmd{};
                cmd.timestamp_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                        .count());
                cmd.valid      = true;
                cmd.target_x   = px;
                cmd.target_y   = py;
                cmd.target_z   = recovery_target_alt_;
                cmd.target_yaw = current_yaw;
                cmd.velocity_x = 0.0f;
                cmd.velocity_y = 0.0f;
                cmd.velocity_z = 1.0f;  // gentle climb rate
                traj_pub.publish(cmd);

                constexpr float kAltTolerance = 0.5f;
                if (pz >= recovery_target_alt_ - kAltTolerance) {
                    recovery_phase_ = RecoveryPhase::REPLAN;
                    spdlog::info("[Recovery] Phase REPLAN — altitude {:.1f}m reached, "
                                 "resuming navigation",
                                 pz);
                }
                break;
            }
            case RecoveryPhase::REPLAN: {
                // Invalidate cached path to force a full replan after collision
                if (grid_planner) {
                    grid_planner->invalidate_path();
                }
                spdlog::info("[Recovery] Complete — transitioning to NAVIGATE");
                recovery_started_ = false;
                fsm.on_recovery_complete();
                break;
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

    // ── Utility: publish an immediate zero-velocity stop trajectory ──
    // Sends a valid command with zero velocities so P5 forwards it to the FC.
    // (P5 skips commands with valid=false, so we must send valid=true to stop.)
    static void publish_stop_trajectory(drone::ipc::IPublisher<drone::ipc::TrajectoryCmd>& pub,
                                        uint64_t correlation_id = 0) {
        drone::ipc::TrajectoryCmd stop{};
        stop.valid          = true;
        stop.velocity_x     = 0.0f;
        stop.velocity_y     = 0.0f;
        stop.velocity_z     = 0.0f;
        stop.correlation_id = correlation_id;
        stop.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        pub.publish(stop);
    }
};

}  // namespace drone::planner
