// process4_mission_planner/src/main.cpp
// Process 4 — Mission Planner: FSM + path planning + obstacle avoidance.
// Reads SLAM pose and detected objects, outputs trajectory + payload commands.
// Sends FC commands (arm, takeoff, mode) to comms via ShmFCCommand.

#include "planner/mission_fsm.h"
#include "planner/ipath_planner.h"
#include "planner/iobstacle_avoider.h"
#include "ipc/message_bus_factory.h"
#include "ipc/shm_types.h"
#include "util/signal_handler.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <spdlog/spdlog.h>

using namespace drone::planner;

static std::atomic<bool> g_running{true};

/// Monotonic sequence counter for FC commands (dedup in comms)
static uint64_t fc_cmd_seq = 0;

/// Publish an FC command to the FC command SHM channel.
static void send_fc_command(
    drone::ipc::IPublisher<drone::ipc::ShmFCCommand>& pub,
    drone::ipc::FCCommandType cmd,
    float param1 = 0.0f)
{
    drone::ipc::ShmFCCommand fc_cmd{};
    fc_cmd.timestamp_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    fc_cmd.command = cmd;
    fc_cmd.param1 = param1;
    fc_cmd.sequence_id = ++fc_cmd_seq;
    fc_cmd.valid = true;
    pub.publish(fc_cmd);
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "mission_planner");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("mission_planner", LogConfig::resolve_log_dir(), args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== Mission Planner starting (PID {}) ===", getpid());

    // ── Create message bus (config-driven: shm or zenoh) ───
    auto bus = drone::ipc::create_message_bus(
        cfg.get<std::string>("ipc_backend", "shm"));

    // ── Subscribe to inputs ─────────────────────────────────
    auto pose_sub = drone::ipc::bus_subscribe<drone::ipc::ShmPose>(
        bus, drone::ipc::shm_names::SLAM_POSE);
    if (!pose_sub->is_connected()) {
        spdlog::error("Cannot connect to SLAM pose");
        return 1;
    }

    auto obj_sub = drone::ipc::bus_subscribe<drone::ipc::ShmDetectedObjectList>(
        bus, drone::ipc::shm_names::DETECTED_OBJECTS);
    if (!obj_sub->is_connected()) {
        spdlog::error("Cannot connect to detected objects");
        return 1;
    }

    // FC state is *critical* for the FSM (armed check, altitude feedback).
    // Subscribe with retries first so we wait for comms to create the segment,
    // ensuring the SHM backend has the segment available before we proceed.
    auto fc_state_sub = drone::ipc::bus_subscribe<drone::ipc::ShmFCState>(
        bus, drone::ipc::shm_names::FC_STATE);
    if (!fc_state_sub->is_connected()) {
        spdlog::error("Cannot connect to FC state — comms may not be running");
        return 1;
    }

    // GCS commands are optional (may never be published).
    // Placed after FC-state so that the SHM backend's blocking retry above
    // gives comms time to initialise, improving GCS segment availability.
    auto gcs_sub = drone::ipc::bus_subscribe_optional<drone::ipc::ShmGCSCommand>(
        bus, drone::ipc::shm_names::GCS_COMMANDS);

    // ── Create publishers ───────────────────────────────────
    auto status_pub = drone::ipc::bus_advertise<drone::ipc::ShmMissionStatus>(
        bus, drone::ipc::shm_names::MISSION_STATUS);
    auto traj_pub = drone::ipc::bus_advertise<drone::ipc::ShmTrajectoryCmd>(
        bus, drone::ipc::shm_names::TRAJECTORY_CMD);
    auto payload_pub = drone::ipc::bus_advertise<drone::ipc::ShmPayloadCommand>(
        bus, drone::ipc::shm_names::PAYLOAD_COMMANDS);
    auto fc_cmd_pub = drone::ipc::bus_advertise<drone::ipc::ShmFCCommand>(
        bus, drone::ipc::shm_names::FC_COMMANDS);

    if (!status_pub->is_ready() || !traj_pub->is_ready() ||
        !payload_pub->is_ready() || !fc_cmd_pub->is_ready()) {
        spdlog::error("Failed to create mission planner publishers");
        return 1;
    }

    // ── Load mission from config or use defaults ────────────
    MissionFSM fsm;
    auto wp_json = cfg.section("mission_planner.waypoints");
    if (wp_json.is_array() && !wp_json.empty()) {
        std::vector<Waypoint> waypoints;
        const float default_radius = cfg.get<float>("mission_planner.acceptance_radius_m", 2.0f);
        const float default_speed  = cfg.get<float>("mission_planner.cruise_speed_mps", 2.0f);
        for (const auto& w : wp_json) {
            waypoints.push_back({
                w.value("x", 0.0f), w.value("y", 0.0f), w.value("z", 5.0f),
                w.value("yaw", 0.0f),
                default_radius,
                w.value("speed", default_speed),
                w.value("payload_trigger", false)
            });
        }
        fsm.load_mission(waypoints);
    } else {
        fsm.load_mission({
            {10.0f,  0.0f, 5.0f, 0.0f,    2.0f, 3.0f, true},
            {10.0f, 10.0f, 5.0f, 1.57f,   2.0f, 3.0f, false},
            { 0.0f, 10.0f, 5.0f, 3.14f,   2.0f, 3.0f, true},
            { 0.0f,  0.0f, 5.0f, -1.57f,  2.0f, 3.0f, false},
        });
    }

    // Mission starts in IDLE → PREFLIGHT. Arm and takeoff are handled
    // by the state machine below, sending FC commands via SHM to comms.
    fsm.on_arm();   // IDLE → PREFLIGHT

    // ── Create path planner and obstacle avoider strategies ──
    const float takeoff_alt = cfg.get<float>(
        "mission_planner.takeoff_altitude_m", 10.0f);
    const float influence_radius = cfg.get<float>(
        "mission_planner.obstacle_avoidance.influence_radius_m", 5.0f);
    const float repulsive_gain = cfg.get<float>(
        "mission_planner.obstacle_avoidance.repulsive_gain", 2.0f);
    const int update_ms = cfg.get<int>("mission_planner.update_rate_hz", 10);
    const int loop_sleep_ms = update_ms > 0 ? 1000 / update_ms : 100;

    auto planner_backend = cfg.get<std::string>(
        "mission_planner.path_planner.backend", "potential_field");
    auto path_planner = drone::planner::create_path_planner(planner_backend);
    spdlog::info("Path planner: {}", path_planner->name());

    auto avoider_backend = cfg.get<std::string>(
        "mission_planner.obstacle_avoider.backend", "potential_field");
    auto avoider = drone::planner::create_obstacle_avoider(
        avoider_backend, influence_radius, repulsive_gain);
    spdlog::info("Obstacle avoider: {}", avoider->name());

    // RTL/landing tuning
    const float rtl_acceptance_m = cfg.get<float>(
        "mission_planner.rtl_acceptance_radius_m", 1.5f);
    const float landed_alt_m = cfg.get<float>(
        "mission_planner.landed_altitude_m", 0.5f);
    const int rtl_min_dwell_s = cfg.get<int>(
        "mission_planner.rtl_min_dwell_seconds", 5);

    spdlog::info("Mission Planner READY — {} waypoints loaded",
                 fsm.total_waypoints());

    // Tracking variables for state machine
    bool takeoff_sent = false;
    bool land_sent = false;
    float home_x = 0.0f, home_y = 0.0f, home_z = 0.0f;
    bool home_recorded = false;
    std::chrono::steady_clock::time_point rtl_start_time{};
    uint64_t last_gcs_timestamp = 0;  // dedup GCS commands by timestamp
    auto last_arm_time = std::chrono::steady_clock::now() -
                         std::chrono::seconds(10);  // allow immediate first ARM

    // ── Main planning loop (10 Hz) ──────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        ScopedTimer timer("PlannerLoop", 120.0);

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

        // Check GCS commands (dedup by timestamp to ignore stale values)
        drone::ipc::ShmGCSCommand gcs_cmd{};
        if (gcs_sub->is_connected() && gcs_sub->receive(gcs_cmd) &&
            gcs_cmd.valid && gcs_cmd.timestamp_ns > last_gcs_timestamp) {
            last_gcs_timestamp = gcs_cmd.timestamp_ns;
            switch (gcs_cmd.command) {
                case drone::ipc::GCSCommandType::RTL:
                    spdlog::info("[Planner] GCS command: RTL");
                    send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::RTL);
                    // Publish invalid traj to stop comms forwarding stale velocity cmds
                    { drone::ipc::ShmTrajectoryCmd stop{}; stop.valid = false;
                      stop.timestamp_ns = static_cast<uint64_t>(
                          std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch()).count());
                      traj_pub->publish(stop); }
                    rtl_start_time = std::chrono::steady_clock::now();
                    fsm.on_rtl();
                    break;
                case drone::ipc::GCSCommandType::LAND:
                    spdlog::info("[Planner] GCS command: LAND");
                    send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::LAND);
                    land_sent = true;
                    { drone::ipc::ShmTrajectoryCmd stop{}; stop.valid = false;
                      stop.timestamp_ns = static_cast<uint64_t>(
                          std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch()).count());
                      traj_pub->publish(stop); }
                    fsm.on_land();
                    break;
                default:
                    break;
            }
        }

        // ── State machine ───────────────────────────────────
        switch (fsm.state()) {
            case MissionState::PREFLIGHT: {
                // Send ARM command periodically until FC confirms armed.
                // PX4 may deny initial attempts until MAVSDK heartbeats
                // establish a GCS link (requires ~3-5s after connection).
                auto now_arm = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(
                        now_arm - last_arm_time).count() >= 3) {
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
                    send_fc_command(*fc_cmd_pub,
                                   drone::ipc::FCCommandType::TAKEOFF, takeoff_alt);
                    takeoff_sent = true;
                }
                // Record home position (where we took off from)
                if (!home_recorded &&
                    std::isfinite(pose.translation[0]) &&
                    std::isfinite(pose.translation[1])) {
                    home_x = static_cast<float>(pose.translation[0]);
                    home_y = static_cast<float>(pose.translation[1]);
                    home_z = 0.0f;  // ground level
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
                const Waypoint* wp = fsm.current_waypoint();
                if (wp) {
                    // Plan trajectory via IPathPlanner
                    auto planned = path_planner->plan(pose, *wp);
                    // Apply obstacle avoidance via IObstacleAvoider
                    auto traj = avoider->avoid(planned, pose, objects);
                    traj_pub->publish(traj);

                    // Check if waypoint reached
                    if (fsm.waypoint_reached(
                            static_cast<float>(pose.translation[0]),
                            static_cast<float>(pose.translation[1]),
                            static_cast<float>(pose.translation[2]), *wp)) {
                        spdlog::info("[Planner] Waypoint {} reached!",
                                     fsm.current_wp_index() + 1);

                        if (wp->trigger_payload) {
                            drone::ipc::ShmPayloadCommand pay_cmd{};
                            pay_cmd.timestamp_ns = traj.timestamp_ns;
                            pay_cmd.action = drone::ipc::PayloadAction::CAMERA_CAPTURE;
                            pay_cmd.gimbal_pitch = -90.0f;
                            pay_cmd.gimbal_yaw = 0.0f;
                            pay_cmd.valid = true;
                            payload_pub->publish(pay_cmd);
                        }

                        if (!fsm.advance_waypoint()) {
                            spdlog::info("[Planner] Mission complete — RTL");
                            send_fc_command(*fc_cmd_pub, drone::ipc::FCCommandType::RTL);
                            // Stop trajectory commands so comms doesn't override RTL
                            { drone::ipc::ShmTrajectoryCmd stop{}; stop.valid = false;
                              stop.timestamp_ns = static_cast<uint64_t>(
                                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch()).count());
                              traj_pub->publish(stop); }
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
                        std::chrono::steady_clock::now() - rtl_start_time).count();
                    float dx = static_cast<float>(pose.translation[0]) - home_x;
                    float dy = static_cast<float>(pose.translation[1]) - home_y;
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
                    spdlog::info("[Planner] Landed (alt={:.2f}m) — mission IDLE",
                                 fc_state.rel_alt);
                    fsm.on_landed();
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
            std::chrono::steady_clock::now().time_since_epoch()).count();
        status.state = fsm.state();
        status.current_waypoint = static_cast<uint32_t>(fsm.current_wp_index());
        status.total_waypoints  = static_cast<uint32_t>(fsm.total_waypoints());
        status.progress_percent = fsm.total_waypoints() > 0
            ? 100.0f * fsm.current_wp_index() / fsm.total_waypoints() : 0.0f;
        status.mission_active = (fsm.state() != MissionState::IDLE);
        status_pub->publish(status);

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    spdlog::info("=== Mission Planner stopped ===");
    return 0;
}
