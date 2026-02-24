// process4_mission_planner/src/main.cpp
// Process 4 — Mission Planner: FSM + path planning + obstacle avoidance.
// Reads SLAM pose and detected objects, outputs trajectory + payload commands.

#include "planner/mission_fsm.h"
#include "planner/ipath_planner.h"
#include "planner/iobstacle_avoider.h"
#include "ipc/shm_message_bus.h"
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

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "mission_planner");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("mission_planner", "/tmp/drone_logs", args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== Mission Planner starting (PID {}) ===", getpid());

    // ── Create message bus ──────────────────────────────────
    drone::ipc::ShmMessageBus bus;

    // ── Subscribe to inputs ─────────────────────────────────
    auto pose_sub = bus.subscribe<drone::ipc::ShmPose>(
        drone::ipc::shm_names::SLAM_POSE);
    if (!pose_sub->is_connected()) {
        spdlog::error("Cannot connect to SLAM pose SHM");
        return 1;
    }

    auto obj_sub = bus.subscribe<drone::ipc::ShmDetectedObjectList>(
        drone::ipc::shm_names::DETECTED_OBJECTS);
    if (!obj_sub->is_connected()) {
        spdlog::error("Cannot connect to detected objects SHM");
        return 1;
    }

    auto gcs_sub = bus.subscribe_lazy<drone::ipc::ShmGCSCommand>();
    gcs_sub->connect(drone::ipc::shm_names::GCS_COMMANDS);

    // ── Create publishers ───────────────────────────────────
    auto status_pub = bus.advertise<drone::ipc::ShmMissionStatus>(
        drone::ipc::shm_names::MISSION_STATUS);
    auto traj_pub = bus.advertise<drone::ipc::ShmTrajectoryCmd>(
        drone::ipc::shm_names::TRAJECTORY_CMD);
    auto payload_pub = bus.advertise<drone::ipc::ShmPayloadCommand>(
        drone::ipc::shm_names::PAYLOAD_COMMANDS);

    if (!status_pub->is_ready() || !traj_pub->is_ready() ||
        !payload_pub->is_ready()) {
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

    // Auto-start mission in simulation
    fsm.on_arm();
    fsm.on_takeoff();
    fsm.on_navigate();

    // ── Create path planner and obstacle avoider strategies ──
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

    spdlog::info("Mission Planner READY — {} waypoints loaded",
                 fsm.total_waypoints());

    // ── Main planning loop (10 Hz) ──────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        ScopedTimer timer("PlannerLoop", 120.0);

        // Read inputs
        drone::ipc::ShmPose pose{};
        pose_sub->receive(pose);

        drone::ipc::ShmDetectedObjectList objects{};
        obj_sub->receive(objects);

        // Check GCS commands
        drone::ipc::ShmGCSCommand gcs_cmd{};
        if (gcs_sub->is_connected() && gcs_sub->receive(gcs_cmd) && gcs_cmd.valid) {
            switch (gcs_cmd.command) {
                case drone::ipc::GCSCommandType::RTL:
                    spdlog::info("[Planner] GCS command: RTL");
                    fsm.on_rtl();
                    break;
                case drone::ipc::GCSCommandType::LAND:
                    spdlog::info("[Planner] GCS command: LAND");
                    fsm.on_land();
                    break;
                default:
                    break;
            }
        }

        // Navigate if we have a target
        if (fsm.state() == MissionState::NAVIGATE) {
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
                        fsm.on_rtl();
                    }
                }
            }
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
