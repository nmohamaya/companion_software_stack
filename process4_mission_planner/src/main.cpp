// process4_mission_planner/src/main.cpp
// Process 4 — Mission Planner: FSM + path planning + obstacle avoidance.
// Reads SLAM pose and detected objects, outputs trajectory + payload commands.

#include "planner/mission_fsm.h"
#include "ipc/shm_reader.h"
#include "ipc/shm_writer.h"
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

// ── Obstacle avoidance (simple potential field) ─────────────
static drone::ipc::ShmTrajectoryCmd compute_trajectory(
    const drone::ipc::ShmPose& pose,
    const drone::ipc::ShmDetectedObjectList& objects,
    const Waypoint& target,
    float influence_radius,
    float repulsive_gain)
{
    drone::ipc::ShmTrajectoryCmd cmd{};
    cmd.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    cmd.valid = true;

    // Attractive force toward waypoint
    float dx = target.x - static_cast<float>(pose.translation[0]);
    float dy = target.y - static_cast<float>(pose.translation[1]);
    float dz = target.z - static_cast<float>(pose.translation[2]);
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist > 0.01f) {
        float speed = std::min(target.speed, dist);  // slow down near target
        cmd.velocity_x = (dx / dist) * speed;
        cmd.velocity_y = (dy / dist) * speed;
        cmd.velocity_z = (dz / dist) * speed;
    }

    // Repulsive force from obstacles
    for (uint32_t i = 0; i < objects.num_objects; ++i) {
        const auto& obj = objects.objects[i];
        float ox = obj.position_x - static_cast<float>(pose.translation[0]);
        float oy = obj.position_y - static_cast<float>(pose.translation[1]);
        float obj_dist = std::sqrt(ox*ox + oy*oy);

        if (obj_dist < influence_radius && obj_dist > 0.1f) {
            float repulsion = repulsive_gain / (obj_dist * obj_dist);
            cmd.velocity_x -= (ox / obj_dist) * repulsion;
            cmd.velocity_y -= (oy / obj_dist) * repulsion;
            spdlog::debug("[Planner] Obstacle avoidance: obj at {:.1f}m, "
                          "repulsion={:.2f}", obj_dist, repulsion);
        }
    }

    cmd.target_x = target.x;
    cmd.target_y = target.y;
    cmd.target_z = target.z;
    cmd.target_yaw = target.yaw;

    return cmd;
}

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

    // ── Open SHM inputs ─────────────────────────────────────
    ShmReader<drone::ipc::ShmPose> pose_reader;
    for (int i = 0; i < 50; ++i) {
        if (pose_reader.open(drone::ipc::shm_names::SLAM_POSE)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!pose_reader.is_open()) {
        spdlog::error("Cannot connect to SLAM pose SHM");
        return 1;
    }

    ShmReader<drone::ipc::ShmDetectedObjectList> obj_reader;
    for (int i = 0; i < 50; ++i) {
        if (obj_reader.open(drone::ipc::shm_names::DETECTED_OBJECTS)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!obj_reader.is_open()) {
        spdlog::error("Cannot connect to detected objects SHM");
        return 1;
    }

    ShmReader<drone::ipc::ShmGCSCommand> gcs_reader;
    // GCS commands may not be available yet, just try
    gcs_reader.open(drone::ipc::shm_names::GCS_COMMANDS);

    // ── Create SHM outputs ──────────────────────────────────
    ShmWriter<drone::ipc::ShmMissionStatus> status_writer;
    ShmWriter<drone::ipc::ShmTrajectoryCmd> traj_writer;
    ShmWriter<drone::ipc::ShmPayloadCommand> payload_writer;

    if (!status_writer.create(drone::ipc::shm_names::MISSION_STATUS) ||
        !traj_writer.create(drone::ipc::shm_names::TRAJECTORY_CMD) ||
        !payload_writer.create(drone::ipc::shm_names::PAYLOAD_COMMANDS)) {
        spdlog::error("Failed to create mission planner SHM outputs");
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

    // ── Obstacle avoidance config ────────────────────────────
    const float influence_radius = cfg.get<float>(
        "mission_planner.obstacle_avoidance.influence_radius_m", 5.0f);
    const float repulsive_gain = cfg.get<float>(
        "mission_planner.obstacle_avoidance.repulsive_gain", 2.0f);
    const int update_ms = cfg.get<int>("mission_planner.update_rate_hz", 10);
    const int loop_sleep_ms = update_ms > 0 ? 1000 / update_ms : 100;

    spdlog::info("Mission Planner READY — {} waypoints loaded",
                 fsm.total_waypoints());

    // ── Main planning loop (10 Hz) ──────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        ScopedTimer timer("PlannerLoop", 120.0);

        // Read inputs
        drone::ipc::ShmPose pose{};
        pose_reader.read(pose);

        drone::ipc::ShmDetectedObjectList objects{};
        obj_reader.read(objects);

        // Check GCS commands
        drone::ipc::ShmGCSCommand gcs_cmd{};
        if (gcs_reader.is_open() && gcs_reader.read(gcs_cmd) && gcs_cmd.valid) {
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
                auto traj = compute_trajectory(pose, objects, *wp,
                                                influence_radius, repulsive_gain);
                traj_writer.write(traj);

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
                        payload_writer.write(pay_cmd);
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
        status_writer.write(status);

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    spdlog::info("=== Mission Planner stopped ===");
    return 0;
}
