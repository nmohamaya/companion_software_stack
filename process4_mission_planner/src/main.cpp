// process4_mission_planner/src/main.cpp
// Process 4 — Mission Planner: FSM + path planning + obstacle avoidance.
// Reads SLAM pose and detected objects, outputs trajectory + payload commands.
// Sends FC commands (arm, takeoff, mode) to comms via FCCommand.

#include "ipc/ipc_types.h"
#include "ipc/message_bus_factory.h"
#include "ipc/zenoh_liveliness.h"
#include "planner/fault_manager.h"
#include "planner/fault_response_executor.h"
#include "planner/gcs_command_handler.h"
#include "planner/geofence.h"
#include "planner/grid_planner_base.h"
#include "planner/iobstacle_avoider.h"
#include "planner/ipath_planner.h"
#include "planner/mission_fsm.h"
#include "planner/mission_state_tick.h"
#include "planner/obstacle_avoider_3d.h"
#include "planner/planner_factory.h"
#include "planner/static_obstacle_layer.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/correlation.h"
#include "util/diagnostic.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"
#include "util/sd_notify.h"
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

/// Publish an FC command to the FC command channel.
/// Carries the current thread-local correlation ID.
static void send_fc_command(drone::ipc::IPublisher<drone::ipc::FCCommand>& pub,
                            drone::ipc::FCCommandType cmd, float param1 = 0.0f) {
    drone::ipc::FCCommand fc_cmd{};
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

    // ── Create message bus (config-driven) ───────────────────
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("mission_planner");

    // ── Subscribe to inputs ─────────────────────────────────
    auto pose_sub = bus.subscribe<drone::ipc::Pose>(drone::ipc::topics::SLAM_POSE);
    if (!pose_sub->is_connected()) {
        spdlog::error("Cannot connect to SLAM pose");
        return 1;
    }

    auto obj_sub =
        bus.subscribe<drone::ipc::DetectedObjectList>(drone::ipc::topics::DETECTED_OBJECTS);
    if (!obj_sub->is_connected()) {
        spdlog::error("Cannot connect to detected objects");
        return 1;
    }

    auto fc_state_sub = bus.subscribe<drone::ipc::FCState>(drone::ipc::topics::FC_STATE);
    if (!fc_state_sub->is_connected()) {
        spdlog::error("Cannot connect to FC state — comms may not be running");
        return 1;
    }

    auto gcs_sub = bus.subscribe_optional<drone::ipc::GCSCommand>(drone::ipc::topics::GCS_COMMANDS);
    auto mission_upload_sub =
        bus.subscribe_optional<drone::ipc::MissionUpload>(drone::ipc::topics::MISSION_UPLOAD);
    auto health_sub =
        bus.subscribe_optional<drone::ipc::SystemHealth>(drone::ipc::topics::SYSTEM_HEALTH);

    // ── Create publishers ───────────────────────────────────
    auto status_pub = bus.advertise<drone::ipc::MissionStatus>(drone::ipc::topics::MISSION_STATUS);
    auto traj_pub   = bus.advertise<drone::ipc::TrajectoryCmd>(drone::ipc::topics::TRAJECTORY_CMD);
    auto payload_pub =
        bus.advertise<drone::ipc::PayloadCommand>(drone::ipc::topics::PAYLOAD_COMMANDS);
    auto fc_cmd_pub = bus.advertise<drone::ipc::FCCommand>(drone::ipc::topics::FC_COMMANDS);

    if (!status_pub->is_ready() || !traj_pub->is_ready() || !payload_pub->is_ready() ||
        !fc_cmd_pub->is_ready()) {
        spdlog::error("Failed to create mission planner publishers");
        return 1;
    }

    // ── Load mission from config or use defaults ────────────
    const float overshoot_proximity = cfg.get<float>("mission_planner.overshoot_proximity_factor",
                                                     3.0f);
    MissionFSM  fsm(overshoot_proximity);
    auto        wp_json = cfg.section("mission_planner.waypoints");
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
    drone::planner::GridPlannerConfig planner_cfg;
    planner_cfg.resolution_m       = cfg.get<float>("mission_planner.path_planner.resolution_m",
                                                    planner_cfg.resolution_m);
    planner_cfg.grid_extent_m      = cfg.get<float>("mission_planner.path_planner.grid_extent_m",
                                                    planner_cfg.grid_extent_m);
    planner_cfg.inflation_radius_m = cfg.get<float>(
        "mission_planner.path_planner.inflation_radius_m", planner_cfg.inflation_radius_m);
    planner_cfg.replan_interval_s = cfg.get<float>("mission_planner.path_planner.replan_interval_s",
                                                   planner_cfg.replan_interval_s);
    planner_cfg.path_speed_mps    = cfg.get<float>("mission_planner.path_planner.path_speed_mps",
                                                   planner_cfg.path_speed_mps);
    planner_cfg.smoothing_alpha   = cfg.get<float>("mission_planner.path_planner.smoothing_alpha",
                                                   planner_cfg.smoothing_alpha);
    planner_cfg.max_iterations    = cfg.get<int>("mission_planner.path_planner.max_iterations",
                                                 planner_cfg.max_iterations);
    planner_cfg.max_search_time_ms = cfg.get<float>(
        "mission_planner.path_planner.max_search_time_ms", planner_cfg.max_search_time_ms);
    planner_cfg.ramp_dist_m        = cfg.get<float>("mission_planner.path_planner.ramp_dist_m",
                                                    planner_cfg.ramp_dist_m);
    planner_cfg.min_speed_mps      = cfg.get<float>("mission_planner.path_planner.min_speed_mps",
                                                    planner_cfg.min_speed_mps);
    planner_cfg.snap_search_radius = cfg.get<int>("mission_planner.path_planner.snap_search_radius",
                                                  planner_cfg.snap_search_radius);

    // Occupancy-grid-specific settings: scenario configs use occupancy_grid.* keys
    // to configure these fields instead of the path_planner.* defaults.
    planner_cfg.resolution_m       = cfg.get<float>("mission_planner.occupancy_grid.resolution_m",
                                                    planner_cfg.resolution_m);
    planner_cfg.inflation_radius_m = cfg.get<float>(
        "mission_planner.occupancy_grid.inflation_radius_m", planner_cfg.inflation_radius_m);
    planner_cfg.cell_ttl_s = cfg.get<float>("mission_planner.occupancy_grid.dynamic_obstacle_ttl_s",
                                            planner_cfg.cell_ttl_s);
    planner_cfg.min_confidence = cfg.get<float>("mission_planner.occupancy_grid.min_confidence",
                                                planner_cfg.min_confidence);
    planner_cfg.promotion_hits = cfg.get<int>("mission_planner.occupancy_grid.promotion_hits",
                                              planner_cfg.promotion_hits);
    planner_cfg.radar_promotion_hits =
        static_cast<uint32_t>(cfg.get<int>("mission_planner.occupancy_grid.radar_promotion_hits",
                                           static_cast<int>(planner_cfg.radar_promotion_hits)));
    // Prediction config — under occupancy_grid.* for consistency with other grid params
    planner_cfg.prediction_enabled = cfg.get<bool>(
        "mission_planner.occupancy_grid.prediction_enabled", planner_cfg.prediction_enabled);
    planner_cfg.prediction_dt_s = cfg.get<float>("mission_planner.occupancy_grid.prediction_dt_s",
                                                 planner_cfg.prediction_dt_s);
    planner_cfg.z_band_cells    = cfg.get<int>("mission_planner.path_planner.z_band_cells",
                                               planner_cfg.z_band_cells);
    planner_cfg.look_ahead_m    = cfg.get<float>("mission_planner.path_planner.look_ahead_m",
                                                 planner_cfg.look_ahead_m);

    auto path_planner = drone::planner::create_path_planner(planner_backend, planner_cfg);
    spdlog::info("Path planner: {}", path_planner->name());
    auto* grid_planner = dynamic_cast<drone::planner::IGridPlanner*>(path_planner.get());

    // ── HD-map static obstacles ─────────────────────────────
    StaticObstacleLayer obstacle_layer;
    obstacle_layer.load(cfg, grid_planner);

    auto avoider_backend = cfg.get<std::string>("mission_planner.obstacle_avoider.backend",
                                                "potential_field");
    auto avoider = drone::planner::create_obstacle_avoider(avoider_backend, influence_radius,
                                                           repulsive_gain, &cfg);
    spdlog::info("Obstacle avoider: {}", avoider->name());

    // ── Geofence setup ─────────────────────────────────────
    Geofence   geofence;
    const bool geofence_cfg_enabled = cfg.get<bool>("mission_planner.geofence.enabled", true);
    auto       fence_json           = cfg.section("mission_planner.geofence.polygon");
    if (geofence_cfg_enabled && fence_json.is_array() && fence_json.size() >= 3) {
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
        geofence.set_altitude_tolerance(
            cfg.get<float>("mission_planner.geofence.altitude_tolerance_m", 0.5f));
        geofence.enable(true);
        spdlog::info("Geofence: {} vertices, alt [{:.0f}, {:.0f}]m, margin {:.0f}m",
                     vertices.size(), alt_floor, alt_ceiling,
                     cfg.get<float>("mission_planner.geofence.warning_margin_m", 5.0f));
    } else {
        spdlog::info("Geofence: disabled ({})",
                     geofence_cfg_enabled ? "no polygon configured" : "disabled by config");
    }

    // ── Create extracted subsystems ─────────────────────────
    const float rtl_acceptance_m = cfg.get<float>("mission_planner.rtl_acceptance_radius_m", 1.5f);
    const float landed_alt_m     = cfg.get<float>("mission_planner.landed_altitude_m", 0.5f);
    const int   rtl_min_dwell_s  = cfg.get<int>("mission_planner.rtl_min_dwell_seconds", 5);
    const float survey_duration  = cfg.get<float>("mission_planner.survey_duration_s", 0.0f);
    const float survey_yaw_rate  = cfg.get<float>("mission_planner.survey_yaw_rate", 0.3f);

    MissionStateTick      state_tick({takeoff_alt, rtl_acceptance_m, landed_alt_m, rtl_min_dwell_s,
                                      survey_duration, survey_yaw_rate});
    FaultResponseExecutor fault_exec;
    GCSCommandHandler     gcs_handler;
    auto                  send_fc = [&](drone::ipc::FCCommandType cmd, float p) {
        send_fc_command(*fc_cmd_pub, cmd, p);
    };

    // ── Create fault manager (config-driven thresholds) ────
    FaultManager fault_mgr(cfg);

    spdlog::info("Mission Planner READY — {} waypoints loaded", fsm.total_waypoints());
    drone::systemd::notify_ready();

    // ── Thread heartbeat + watchdog + health publisher ──────
    auto                        planning_hb = drone::util::ScopedHeartbeat("planning_loop", true);
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub =
        bus.advertise<drone::ipc::ThreadHealth>(drone::ipc::topics::THREAD_HEALTH_MISSION_PLANNER);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "mission_planner",
                                                        watchdog);
    uint32_t                           health_tick = 0;

    // ── Main planning loop (10 Hz) ──────────────────────────
    // Execution order contract:
    //   1. Read inputs → 2. Pose staleness → 3. Obstacle cross-check →
    //   4. Geofence → 5. Fault evaluate → 6. Fault execute →
    //   7. GCS commands → 8. State tick → 9. Publish status
    constexpr uint64_t kPoseStaleThresholdNs = 500'000'000ULL;
    uint64_t           pose_stale_count      = 0;
    uint64_t           loop_tick             = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(planning_hb.handle());
        drone::systemd::notify_watchdog();
        drone::util::FrameDiagnostics diag(loop_tick);
        drone::util::ScopedDiagTimer  loop_timer(diag, "PlannerLoop");

        // ── 1. Read inputs ──────────────────────────────────
        drone::ipc::Pose pose{};
        pose_sub->receive(pose);

        drone::ipc::DetectedObjectList objects{};
        obj_sub->receive(objects);

        drone::ipc::FCState fc_state{};
        if (fc_state_sub->is_connected()) {
            fc_state_sub->receive(fc_state);
        }

        drone::ipc::SystemHealth sys_health{};
        if (health_sub->is_connected()) {
            health_sub->receive(sys_health);
        }

        auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::steady_clock::now().time_since_epoch())
                                                .count());

        // ── 2. Pose staleness check (SAFETY CRITICAL, inline) ──
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

        // ── 3. Obstacle cross-check ─────────────────────────
        obstacle_layer.cross_check(objects, pose.quality, now_ns);

        // ── 4. Geofence check (airborne only, skip TAKEOFF) ─
        {
            drone::util::ScopedDiagTimer fence_timer(diag, "GeofenceCheck");
            if (geofence.is_enabled() && fsm.state() != MissionState::IDLE &&
                fsm.state() != MissionState::PREFLIGHT && fsm.state() != MissionState::TAKEOFF) {
                auto fence_result = geofence.check(static_cast<float>(pose.translation[0]),
                                                   static_cast<float>(pose.translation[1]),
                                                   fc_state.rel_alt);
                fault_mgr.set_geofence_violation(fence_result.violated);
                if (fence_result.violated) {
                    spdlog::warn("Geofence: VIOLATED — {}", fence_result.message);
                    diag.add_warning("Geofence", fence_result.message);
                } else if (fence_result.margin_m < 0.0f &&
                           std::abs(fence_result.margin_m) < geofence.warning_margin()) {
                    diag.add_warning("Geofence", "Approaching boundary — margin " +
                                                     std::to_string(static_cast<int>(
                                                         std::abs(fence_result.margin_m))) +
                                                     "m");
                }
            } else {
                fault_mgr.set_geofence_violation(false);
            }
        }

        // ── 5. Fault evaluate ───────────────────────────────
        auto fault = [&]() {
            drone::util::ScopedDiagTimer t(diag, "FaultEval");
            return fault_mgr.evaluate(sys_health, fc_state, pose.timestamp_ns, now_ns,
                                      pose.quality);
        }();

        // ── 6. Fault execute ────────────────────────────────
        fault_exec.execute(fault, fsm, send_fc, *traj_pub, state_tick.flight_state(), now_ns);
        fault_exec.log_new_faults(fault.active_faults);

        // ── 7. GCS commands ─────────────────────────────────
        gcs_handler.process(*gcs_sub, *mission_upload_sub, fsm, send_fc, *traj_pub,
                            state_tick.flight_state(), diag);

        // ── 8. State tick ───────────────────────────────────
        state_tick.tick(fsm, pose, fc_state, objects, *path_planner, grid_planner, *avoider,
                        obstacle_layer, *traj_pub, *payload_pub, send_fc,
                        gcs_handler.active_correlation_id(), diag);

        // Handle fault reset on landing
        if (state_tick.consume_fault_reset()) {
            fault_mgr.reset();
            fault_exec.reset();
        }

        // ── 9. Publish status + health + diagnostics ────────
        drone::ipc::MissionStatus status{};
        status.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
        status.correlation_id   = gcs_handler.active_correlation_id();
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

        ++health_tick;
        if (health_tick % static_cast<uint32_t>(std::max(1, 1000 / loop_sleep_ms)) == 0) {
            health_publisher.publish_snapshot();
        }

        if (diag.has_errors() || diag.has_warnings()) {
            diag.log_summary("MissionPlanner");
        }

        ++loop_tick;
        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    drone::systemd::notify_stopping();
    spdlog::info("=== Mission Planner stopped ===");
    return 0;
}
