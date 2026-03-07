// process4_mission_planner/src/main.cpp
// Process 4 — Mission Planner: FSM + path planning + obstacle avoidance.
// Reads SLAM pose and detected objects, outputs trajectory + payload commands.
// Sends FC commands (arm, takeoff, mode) to comms via ShmFCCommand.

#include "ipc/message_bus_factory.h"
#include "ipc/shm_types.h"
#include "ipc/zenoh_liveliness.h"
#include "planner/astar_planner.h"
#include "planner/fault_manager.h"
#include "planner/geofence.h"
#include "planner/iobstacle_avoider.h"
#include "planner/ipath_planner.h"
#include "planner/mission_fsm.h"
#include "planner/obstacle_avoider_3d.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/correlation.h"
#include "util/diagnostic.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"
#include "util/signal_handler.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include <spdlog/spdlog.h>

using namespace drone::planner;

static std::atomic<bool> g_running{true};

/// Monotonic sequence counter for FC commands (dedup in comms)
static uint64_t fc_cmd_seq = 0;

/// Publish an FC command to the FC command SHM channel.
/// Carries the current thread-local correlation ID.
static void send_fc_command(drone::ipc::IPublisher<drone::ipc::ShmFCCommand>& pub,
                            drone::ipc::FCCommandType cmd, float param1 = 0.0f) {
    drone::ipc::ShmFCCommand fc_cmd{};
    fc_cmd.timestamp_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    fc_cmd.correlation_id = drone::util::CorrelationContext::get();
    fc_cmd.command        = cmd;
    fc_cmd.param1         = param1;
    fc_cmd.sequence_id    = ++fc_cmd_seq;
    fc_cmd.valid          = true;
    pub.publish(fc_cmd);
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "mission_planner");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("mission_planner", LogConfig::resolve_log_dir(), args.log_level,
                    args.json_logs);

    drone::Config cfg;
    if (!cfg.load(args.config_path)) {
        spdlog::warn("Running with default configuration; failed to load '{}'", args.config_path);
    }

    spdlog::info("=== Mission Planner starting (PID {}) ===", getpid());

    // ── Create message bus (config-driven: shm or zenoh) ───
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("mission_planner");

    // ── Subscribe to inputs ─────────────────────────────────
    auto pose_sub = bus.subscribe<drone::ipc::ShmPose>(drone::ipc::shm_names::SLAM_POSE);
    if (!pose_sub->is_connected()) {
        spdlog::error("Cannot connect to SLAM pose");
        return 1;
    }

    auto obj_sub =
        bus.subscribe<drone::ipc::ShmDetectedObjectList>(drone::ipc::shm_names::DETECTED_OBJECTS);
    if (!obj_sub->is_connected()) {
        spdlog::error("Cannot connect to detected objects");
        return 1;
    }

    // FC state is *critical* for the FSM (armed check, altitude feedback).
    // Subscribe with retries first so we wait for comms to create the segment,
    // ensuring the SHM backend has the segment available before we proceed.
    auto fc_state_sub = bus.subscribe<drone::ipc::ShmFCState>(drone::ipc::shm_names::FC_STATE);
    if (!fc_state_sub->is_connected()) {
        spdlog::error("Cannot connect to FC state — comms may not be running");
        return 1;
    }

    // GCS commands are optional (may never be published).
    // Placed after FC-state so that the SHM backend's blocking retry above
    // gives comms time to initialise, improving GCS segment availability.
    auto gcs_sub =
        bus.subscribe_optional<drone::ipc::ShmGCSCommand>(drone::ipc::shm_names::GCS_COMMANDS);

    // Mission upload channel (optional — used for mid-flight waypoint upload)
    auto mission_upload_sub =
        bus.subscribe_optional<drone::ipc::ShmMissionUpload>(drone::ipc::shm_names::MISSION_UPLOAD);

    // System health from Process 7 (optional — monitor may not be running)
    auto health_sub =
        bus.subscribe_optional<drone::ipc::ShmSystemHealth>(drone::ipc::shm_names::SYSTEM_HEALTH);

    // ── Create publishers ───────────────────────────────────
    auto status_pub =
        bus.advertise<drone::ipc::ShmMissionStatus>(drone::ipc::shm_names::MISSION_STATUS);
    auto traj_pub =
        bus.advertise<drone::ipc::ShmTrajectoryCmd>(drone::ipc::shm_names::TRAJECTORY_CMD);
    auto payload_pub =
        bus.advertise<drone::ipc::ShmPayloadCommand>(drone::ipc::shm_names::PAYLOAD_COMMANDS);
    auto fc_cmd_pub = bus.advertise<drone::ipc::ShmFCCommand>(drone::ipc::shm_names::FC_COMMANDS);

    if (!status_pub->is_ready() || !traj_pub->is_ready() || !payload_pub->is_ready() ||
        !fc_cmd_pub->is_ready()) {
        spdlog::error("Failed to create mission planner publishers");
        return 1;
    }

    // ── Load mission from config or use defaults ────────────
    MissionFSM fsm;
    auto       wp_json = cfg.section("mission_planner.waypoints");
    if (wp_json.is_array() && !wp_json.empty()) {
        std::vector<Waypoint> waypoints;
        const float default_radius = cfg.get<float>("mission_planner.acceptance_radius_m", 2.0f);
        const float default_speed  = cfg.get<float>("mission_planner.cruise_speed_mps", 2.0f);
        for (const auto& w : wp_json) {
            waypoints.push_back({w.value("x", 0.0f), w.value("y", 0.0f), w.value("z", 5.0f),
                                 w.value("yaw", 0.0f), default_radius,
                                 w.value("speed", default_speed),
                                 w.value("payload_trigger", false)});
        }
        fsm.load_mission(waypoints);
    } else {
        fsm.load_mission({
            {10.0f, 0.0f, 5.0f, 0.0f, 2.0f, 3.0f, true},
            {10.0f, 10.0f, 5.0f, 1.57f, 2.0f, 3.0f, false},
            {0.0f, 10.0f, 5.0f, 3.14f, 2.0f, 3.0f, true},
            {0.0f, 0.0f, 5.0f, -1.57f, 2.0f, 3.0f, false},
        });
    }

    // Mission starts in IDLE → PREFLIGHT. Arm and takeoff are handled
    // by the state machine below, sending FC commands via SHM to comms.
    fsm.on_arm();  // IDLE → PREFLIGHT

    // ── Create path planner and obstacle avoider strategies ──
    const float takeoff_alt = cfg.get<float>("mission_planner.takeoff_altitude_m", 10.0f);
    const float influence_radius =
        cfg.get<float>("mission_planner.obstacle_avoidance.influence_radius_m", 5.0f);
    const float repulsive_gain = cfg.get<float>("mission_planner.obstacle_avoidance.repulsive_gain",
                                                2.0f);
    const int   update_ms      = cfg.get<int>("mission_planner.update_rate_hz", 10);
    const int   loop_sleep_ms  = std::max(1, update_ms > 0 ? 1000 / update_ms : 100);

    auto planner_backend = cfg.get<std::string>("mission_planner.path_planner.backend",
                                                "potential_field");
    auto path_planner    = drone::planner::create_path_planner(planner_backend);
    spdlog::info("Path planner: {}", path_planner->name());
    // Cache pointer for A* obstacle-grid updates (null if not A*)
    auto* astar_planner = dynamic_cast<drone::planner::AStarPathPlanner*>(path_planner.get());

    auto avoider_backend = cfg.get<std::string>("mission_planner.obstacle_avoider.backend",
                                                "potential_field");
    auto avoider = drone::planner::create_obstacle_avoider(avoider_backend, influence_radius,
                                                           repulsive_gain);
    spdlog::info("Obstacle avoider: {}", avoider->name());

    // ── Geofence setup ─────────────────────────────────────
    Geofence geofence;
    auto     fence_json = cfg.section("mission_planner.geofence.polygon");
    if (fence_json.is_array() && fence_json.size() >= 3) {
        std::vector<GeoVertex> vertices;
        for (const auto& v : fence_json) {
            vertices.push_back({v.value("x", 0.0f), v.value("y", 0.0f)});
        }
        geofence.set_polygon(vertices);
        float alt_floor   = cfg.get<float>("mission_planner.geofence.altitude_floor_m", 0.0f);
        float alt_ceiling = cfg.get<float>("mission_planner.geofence.altitude_ceiling_m", 120.0f);
        geofence.set_altitude_limits(alt_floor, alt_ceiling);
        geofence.set_warning_margin(
            cfg.get<float>("mission_planner.geofence.warning_margin_m", 5.0f));
        geofence.enable(true);
        spdlog::info("Geofence: {} vertices, alt [{:.0f}, {:.0f}]m, margin {:.0f}m",
                     vertices.size(), alt_floor, alt_ceiling,
                     cfg.get<float>("mission_planner.geofence.warning_margin_m", 5.0f));
    } else {
        spdlog::info("Geofence: disabled (no polygon configured)");
    }

    // RTL/landing tuning
    const float rtl_acceptance_m = cfg.get<float>("mission_planner.rtl_acceptance_radius_m", 1.5f);
    const float landed_alt_m     = cfg.get<float>("mission_planner.landed_altitude_m", 0.5f);
    const int   rtl_min_dwell_s  = cfg.get<int>("mission_planner.rtl_min_dwell_seconds", 5);

    // ── Create fault manager (config-driven thresholds) ────
    FaultManager fault_mgr(cfg);
    FaultAction  last_fault_action = FaultAction::NONE;

    spdlog::info("Mission Planner READY — {} waypoints loaded", fsm.total_waypoints());

    // ── Thread heartbeat + watchdog + health publisher ──────
    auto                        planning_hb = drone::util::ScopedHeartbeat("planning_loop", true);
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub = bus.advertise<drone::ipc::ShmThreadHealth>(
        drone::ipc::shm_names::THREAD_HEALTH_MISSION_PLANNER);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "mission_planner",
                                                        watchdog);
    uint32_t                           health_tick = 0;

    // Tracking variables for state machine
    bool                                  takeoff_sent = false;
    bool                                  land_sent    = false;
    float                                 home_x = 0.0f, home_y = 0.0f, home_z = 0.0f;
    bool                                  home_recorded = false;
    std::chrono::steady_clock::time_point rtl_start_time{};
    uint64_t last_gcs_timestamp    = 0;  // dedup GCS commands by timestamp
    uint64_t last_upload_timestamp = 0;  // dedup mission uploads by timestamp
    uint64_t active_correlation_id = 0;  // persisted GCS correlation ID for mission outputs
    auto     last_arm_time         = std::chrono::steady_clock::now() -
                         std::chrono::seconds(10);  // allow immediate first ARM

    // ── Main planning loop (10 Hz) ──────────────────────────
    // Pose staleness threshold (500ms = 5× planning period)
    constexpr uint64_t kPoseStaleThresholdNs = 500'000'000ULL;
    uint64_t           pose_stale_count      = 0;
    uint64_t           loop_tick             = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(planning_hb.handle());
        drone::util::FrameDiagnostics diag(loop_tick);
        drone::util::ScopedDiagTimer  loop_timer(diag, "PlannerLoop");

        // Read inputs
        drone::ipc::ShmPose pose{};
        pose_sub->receive(pose);

        drone::ipc::ShmDetectedObjectList objects{};
        obj_sub->receive(objects);

        // Read FC state (for arm/altitude feedback)
        drone::ipc::ShmFCState fc_state{};
        if (fc_state_sub->is_connected()) {
            fc_state_sub->receive(fc_state);
        }

        // Read system health from Process 7
        drone::ipc::ShmSystemHealth sys_health{};
        if (health_sub->is_connected()) {
            health_sub->receive(sys_health);
        }

        // ── Fault evaluation (before GCS / FSM logic) ───────
        auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::steady_clock::now().time_since_epoch())
                                                .count());

        // ── Pose staleness check (SAFETY CRITICAL) ──────────
        // If pose data is older than threshold, we may be flying
        // with stale position — warn loudly.
        if (pose.timestamp_ns > 0 && (now_ns - pose.timestamp_ns) > kPoseStaleThresholdNs) {
            ++pose_stale_count;
            double age_ms = static_cast<double>(now_ns - pose.timestamp_ns) / 1e6;
            diag.add_warning("PoseInput",
                             "Stale pose: " + std::to_string(static_cast<int>(age_ms)) +
                                 "ms old (threshold: " +
                                 std::to_string(kPoseStaleThresholdNs / 1'000'000) + "ms)");
            if (pose_stale_count == 1 || pose_stale_count % 50 == 0) {
                spdlog::warn("[Planner] STALE POSE detected "
                             "(#{}, age={:.0f}ms) — position may be unreliable!",
                             pose_stale_count, age_ms);
            }
        } else if (pose_stale_count > 0) {
            spdlog::info("[Planner] Pose freshness restored after {} stale readings",
                         pose_stale_count);
            pose_stale_count = 0;
        }

        auto fault = [&]() {
            drone::util::ScopedDiagTimer t(diag, "FaultEval");
            return fault_mgr.evaluate(sys_health, fc_state, pose.timestamp_ns, now_ns);
        }();

        // ── Geofence check (every tick, airborne only) ──────
        {
            drone::util::ScopedDiagTimer fence_timer(diag, "GeofenceCheck");
            if (geofence.is_enabled() && fsm.state() != MissionState::IDLE &&
                fsm.state() != MissionState::PREFLIGHT) {
                auto fence_result = geofence.check(static_cast<float>(pose.translation[0]),
                                                   static_cast<float>(pose.translation[1]),
                                                   fc_state.rel_alt);
                fault_mgr.set_geofence_violation(fence_result.violated);
                if (fence_result.violated) {
                    diag.add_warning("Geofence", fence_result.message);
                } else if (fence_result.margin_m < geofence.warning_margin() &&
                           fence_result.margin_m >= 0.0f) {
                    diag.add_warning("Geofence",
                                     "Approaching boundary — margin " +
                                         std::to_string(static_cast<int>(fence_result.margin_m)) +
                                         "m");
                }
            } else {
                fault_mgr.set_geofence_violation(false);
            }
        }  // GeofenceCheck timer scope

        // Apply fault action if escalated (only in airborne states)
        if (fault.recommended_action > last_fault_action &&
            fault.recommended_action > FaultAction::NONE && fsm.state() != MissionState::IDLE &&
            fsm.state() != MissionState::PREFLIGHT) {

            spdlog::warn("[FaultMgr] Escalation: {} → {} (reason: {})",
                         fault_action_name(last_fault_action),
                         fault_action_name(fault.recommended_action), fault.reason);

            switch (fault.recommended_action) {
                case FaultAction::WARN:
                    // Log only — continue mission
                    break;
                case FaultAction::LOITER:
                    if (fsm.state() == MissionState::TAKEOFF ||
                        fsm.state() == MissionState::NAVIGATE) {
                        // Stop sending trajectory commands
                        {
                            drone::ipc::ShmTrajectoryCmd stop{};
                            stop.valid        = false;
                            stop.timestamp_ns = now_ns;
                            traj_pub->publish(stop);
                        }
                        fsm.on_loiter();
                        fsm.set_fault_triggered(true);
                    }
                    break;
                case FaultAction::RTL:
                    send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::RTL);
                    {
                        drone::ipc::ShmTrajectoryCmd stop{};
                        stop.valid        = false;
                        stop.timestamp_ns = now_ns;
                        traj_pub->publish(stop);
                    }
                    rtl_start_time = std::chrono::steady_clock::now();
                    fsm.on_rtl();
                    fsm.set_fault_triggered(true);
                    break;
                case FaultAction::EMERGENCY_LAND:
                    send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::LAND);
                    {
                        drone::ipc::ShmTrajectoryCmd stop{};
                        stop.valid        = false;
                        stop.timestamp_ns = now_ns;
                        traj_pub->publish(stop);
                    }
                    land_sent = true;
                    fsm.on_land();
                    fsm.set_fault_triggered(true);
                    break;
                default: break;
            }
            last_fault_action = fault.recommended_action;
        }

        // Check GCS commands (dedup by timestamp to ignore stale values)
        drone::ipc::ShmGCSCommand gcs_cmd{};
        if (gcs_sub->is_connected() && gcs_sub->receive(gcs_cmd) && gcs_cmd.valid &&
            gcs_cmd.timestamp_ns > last_gcs_timestamp) {
            last_gcs_timestamp    = gcs_cmd.timestamp_ns;
            active_correlation_id = gcs_cmd.correlation_id;
            // Propagate GCS correlation ID into outgoing commands
            drone::util::ScopedCorrelation gcs_guard(gcs_cmd.correlation_id);
            switch (gcs_cmd.command) {
                case drone::ipc::GCSCommandType::RTL:
                    spdlog::info("[Planner] GCS command: RTL corr={:#x}", gcs_cmd.correlation_id);
                    send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::RTL);
                    // Publish invalid traj to stop comms forwarding stale velocity cmds
                    {
                        drone::ipc::ShmTrajectoryCmd stop{};
                        stop.valid          = false;
                        stop.correlation_id = gcs_cmd.correlation_id;
                        stop.timestamp_ns   = static_cast<uint64_t>(
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count());
                        traj_pub->publish(stop);
                    }
                    rtl_start_time = std::chrono::steady_clock::now();
                    fsm.on_rtl();
                    break;
                case drone::ipc::GCSCommandType::LAND:
                    spdlog::info("[Planner] GCS command: LAND corr={:#x}", gcs_cmd.correlation_id);
                    send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::LAND);
                    land_sent = true;
                    {
                        drone::ipc::ShmTrajectoryCmd stop{};
                        stop.valid          = false;
                        stop.correlation_id = gcs_cmd.correlation_id;
                        stop.timestamp_ns   = static_cast<uint64_t>(
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count());
                        traj_pub->publish(stop);
                    }
                    fsm.on_land();
                    break;
                case drone::ipc::GCSCommandType::MISSION_UPLOAD: {
                    // Read waypoints from the mission upload channel
                    drone::ipc::ShmMissionUpload upload{};
                    if (mission_upload_sub->is_connected() && mission_upload_sub->receive(upload) &&
                        upload.valid && upload.timestamp_ns > last_upload_timestamp &&
                        upload.num_waypoints > 0) {
                        last_upload_timestamp = upload.timestamp_ns;
                        std::vector<Waypoint> new_wps;
                        for (uint8_t i = 0;
                             i < upload.num_waypoints && i < drone::ipc::kMaxUploadWaypoints; ++i) {
                            const auto& sw = upload.waypoints[i];
                            new_wps.push_back({sw.x, sw.y, sw.z, sw.yaw, sw.radius, sw.speed,
                                               sw.trigger_payload});
                        }
                        fsm.load_mission(new_wps);
                        spdlog::info("[Planner] GCS MISSION_UPLOAD: {} waypoints loaded "
                                     "corr={:#x}",
                                     new_wps.size(), gcs_cmd.correlation_id);
                        diag.add_warning("MissionUpload", "Mid-flight waypoint upload: " +
                                                              std::to_string(new_wps.size()) +
                                                              " waypoints");
                        // If currently navigating, start the new mission immediately
                        if (fsm.state() == MissionState::NAVIGATE ||
                            fsm.state() == MissionState::LOITER) {
                            fsm.on_navigate();
                        }
                    } else {
                        spdlog::warn("[Planner] MISSION_UPLOAD command but no valid "
                                     "upload data available");
                    }
                    break;
                }
                default: break;
            }
        }

        // ── State machine ───────────────────────────────────
        switch (fsm.state()) {
            case MissionState::PREFLIGHT: {
                // Send ARM command periodically until FC confirms armed.
                // PX4 may deny initial attempts until MAVSDK heartbeats
                // establish a GCS link (requires ~3-5s after connection).
                auto now_arm = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now_arm - last_arm_time)
                        .count() >= 3) {
                    spdlog::info("[Planner] Sending ARM command");
                    send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::ARM);
                    last_arm_time = now_arm;
                }
                if (fc_state.armed) {
                    spdlog::info("[Planner] Vehicle armed — initiating takeoff");
                    fsm.on_takeoff();
                    takeoff_sent = false;
                }
                break;
            }

            case MissionState::TAKEOFF: {
                // Send TAKEOFF command, wait for target altitude
                if (!takeoff_sent) {
                    spdlog::info("[Planner] Sending TAKEOFF to {:.1f}m", takeoff_alt);
                    send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::TAKEOFF, takeoff_alt);
                    takeoff_sent = true;
                }
                // Record home position (where we took off from)
                if (!home_recorded && std::isfinite(pose.translation[0]) &&
                    std::isfinite(pose.translation[1])) {
                    home_x        = static_cast<float>(pose.translation[0]);
                    home_y        = static_cast<float>(pose.translation[1]);
                    home_z        = 0.0f;  // ground level
                    home_recorded = true;
                    spdlog::info("[Planner] Home position recorded: ({:.1f}, {:.1f}, {:.1f})",
                                 home_x, home_y, home_z);
                }
                // Check if close to target altitude (90% threshold)
                if (fc_state.rel_alt >= takeoff_alt * 0.9f) {
                    spdlog::info("[Planner] Takeoff complete (alt={:.1f}m) — NAVIGATE",
                                 fc_state.rel_alt);
                    fsm.on_navigate();
                }
                break;
            }

            case MissionState::NAVIGATE: {
                // Update A* obstacle grid if applicable
                if (astar_planner) {
                    drone::util::ScopedDiagTimer t(diag, "AStarGridUpdate");
                    astar_planner->update_obstacles(objects, pose);
                }
                const Waypoint* wp = fsm.current_waypoint();
                if (wp) {
                    // Plan trajectory via IPathPlanner
                    auto planned = [&]() {
                        drone::util::ScopedDiagTimer t(diag, "PathPlan");
                        return path_planner->plan(pose, *wp);
                    }();
                    // Flag if A* fell back to direct line (no obstacle-free path found)
                    if (astar_planner && astar_planner->using_direct_fallback()) {
                        diag.add_warning("PathPlan", "A* fallback: no obstacle-free path — using "
                                                     "direct line");
                    }
                    // Apply obstacle avoidance via IObstacleAvoider
                    auto traj = [&]() {
                        drone::util::ScopedDiagTimer t(diag, "ObstacleAvoid");
                        return avoider->avoid(planned, pose, objects);
                    }();
                    traj_pub->publish(traj);

                    // Check if waypoint reached
                    if (fsm.waypoint_reached(static_cast<float>(pose.translation[0]),
                                             static_cast<float>(pose.translation[1]),
                                             static_cast<float>(pose.translation[2]), *wp)) {
                        spdlog::info("[Planner] Waypoint {} reached!", fsm.current_wp_index() + 1);

                        if (wp->trigger_payload) {
                            drone::ipc::ShmPayloadCommand pay_cmd{};
                            pay_cmd.timestamp_ns   = traj.timestamp_ns;
                            pay_cmd.correlation_id = active_correlation_id;
                            pay_cmd.action         = drone::ipc::PayloadAction::CAMERA_CAPTURE;
                            pay_cmd.gimbal_pitch   = -90.0f;
                            pay_cmd.gimbal_yaw     = 0.0f;
                            pay_cmd.valid          = true;
                            payload_pub->publish(pay_cmd);
                        }

                        if (!fsm.advance_waypoint()) {
                            spdlog::info("[Planner] Mission complete — RTL");
                            send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::RTL);
                            // Stop trajectory commands so comms doesn't override RTL
                            {
                                drone::ipc::ShmTrajectoryCmd stop{};
                                stop.valid        = false;
                                stop.timestamp_ns = static_cast<uint64_t>(
                                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
                                traj_pub->publish(stop);
                            }
                            rtl_start_time = std::chrono::steady_clock::now();
                            fsm.on_rtl();
                        }
                    }
                }
                break;
            }

            case MissionState::RTL: {
                // Monitor position — when near home, send LAND to bypass
                // PX4's RTL loiter delay (RTL_LAND_DELAY).
                // Only override if we have a valid home fix; otherwise let
                // PX4 handle RTL/landing autonomously.
                // Wait at least rtl_min_dwell_s seconds so PX4 has time
                // to actually fly the drone back to the home position
                // before we intercept with LAND.
                if (home_recorded) {
                    auto rtl_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                           std::chrono::steady_clock::now() - rtl_start_time)
                                           .count();
                    float dx         = static_cast<float>(pose.translation[0]) - home_x;
                    float dy         = static_cast<float>(pose.translation[1]) - home_y;
                    float horiz_dist = std::sqrt(dx * dx + dy * dy);
                    if (rtl_elapsed >= rtl_min_dwell_s && horiz_dist < rtl_acceptance_m) {
                        spdlog::info("[Planner] Near home ({:.1f}m, {}s in RTL) — sending LAND",
                                     horiz_dist, rtl_elapsed);
                        send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::LAND);
                        land_sent = true;
                        fsm.on_land();
                    }
                }
                break;
            }

            case MissionState::LAND: {
                // Monitor altitude — transition to IDLE when on the ground
                if (fc_state.rel_alt < landed_alt_m && land_sent) {
                    spdlog::info("[Planner] Landed (alt={:.2f}m) — mission IDLE", fc_state.rel_alt);
                    fsm.on_landed();
                    // Reset fault state for next mission / flight
                    fault_mgr.reset();
                    last_fault_action = FaultAction::NONE;
                }
                break;
            }

            case MissionState::IDLE:
            case MissionState::EMERGENCY:
            default:
                // No trajectory commands
                break;
        }

        // Publish mission status
        drone::ipc::ShmMissionStatus status{};
        status.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
        status.correlation_id   = active_correlation_id;
        status.state            = fsm.state();
        status.current_waypoint = static_cast<uint32_t>(fsm.current_wp_index());
        status.total_waypoints  = static_cast<uint32_t>(fsm.total_waypoints());
        status.progress_percent = fsm.total_waypoints() > 0
                                      ? 100.0f * fsm.current_wp_index() / fsm.total_waypoints()
                                      : 0.0f;
        status.mission_active   = (fsm.state() != MissionState::IDLE);
        status.active_faults    = fault.active_faults;
        status.fault_action     = static_cast<uint8_t>(fault.recommended_action);
        status_pub->publish(status);

        // Publish thread health at ~1 Hz (every update_ms ticks)
        ++health_tick;
        if (health_tick % static_cast<uint32_t>(std::max(1, 1000 / loop_sleep_ms)) == 0) {
            health_publisher.publish_snapshot();
        }

        // Log diagnostics if issues detected this tick
        if (diag.has_errors() || diag.has_warnings()) {
            diag.log_summary("MissionPlanner");
        }

        ++loop_tick;
        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    spdlog::info("=== Mission Planner stopped ===");
    return 0;
}
