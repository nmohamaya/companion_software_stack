// process5_comms/src/main.cpp
// Process 5 — Comms: bridges companion computer ↔ flight controller (FC)
//                    and companion computer ↔ ground control station (GCS).
// Uses HAL interfaces — backends selected via config.
// Threads:
//   fc_rx    — receives FC state, publishes ShmFCState
//   fc_tx    — reads ShmTrajectoryCmd, sends to FC
//   gcs_rx   — polls GCS for commands, publishes ShmGCSCommand
//   gcs_tx   — reads mission status + pose, sends telemetry to GCS

#include "ipc/message_bus_factory.h"
#include "ipc/zenoh_liveliness.h"
#include "ipc/shm_types.h"
#include "util/signal_handler.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/log_config.h"
#include "util/realtime.h"
#include "hal/hal_factory.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <spdlog/spdlog.h>

static std::atomic<bool> g_running{true};

// ── FC receive thread (10 Hz) ───────────────────────────────
static void fc_rx_thread(drone::hal::IFCLink& fc,
                         drone::ipc::IPublisher<drone::ipc::ShmFCState>& pub) {
    set_thread_params("fc_rx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] fc_rx thread started using {}", fc.name());

    while (g_running.load(std::memory_order_relaxed)) {
        auto hb = fc.receive_state();

        drone::ipc::ShmFCState state{};
        state.timestamp_ns      = hb.timestamp_ns;
        state.battery_voltage   = hb.battery_voltage;
        state.battery_remaining = hb.battery_percent;
        state.rel_alt           = hb.altitude_rel;
        state.vx                = hb.ground_speed;
        state.satellites_visible = hb.satellites;
        state.flight_mode       = hb.flight_mode;
        state.armed             = hb.armed;
        state.connected         = true;
        pub.publish(state);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── FC transmit thread (20 Hz) ──────────────────────────────
// Forwards trajectory commands AND FC commands (arm, takeoff, mode)
// from the mission planner to the flight controller.
static void fc_tx_thread(drone::hal::IFCLink& fc,
                         drone::ipc::ISubscriber<drone::ipc::ShmTrajectoryCmd>& traj_sub,
                         drone::ipc::ISubscriber<drone::ipc::ShmFCCommand>& cmd_sub) {
    set_thread_params("fc_tx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] fc_tx thread started using {}", fc.name());

    uint64_t last_cmd_seq = 0;  // dedup FC commands by sequence_id
    uint64_t last_traj_ts = 0;  // dedup trajectory commands by timestamp

    while (g_running.load(std::memory_order_relaxed)) {
        // ── Handle FC commands (arm, takeoff, mode) ─────────
        drone::ipc::ShmFCCommand fc_cmd{};
        if (cmd_sub.is_connected() && cmd_sub.receive(fc_cmd) &&
            fc_cmd.valid && fc_cmd.sequence_id > last_cmd_seq) {
            last_cmd_seq = fc_cmd.sequence_id;

            switch (fc_cmd.command) {
                case drone::ipc::FCCommandType::ARM:
                    spdlog::info("[Comms] FC cmd: ARM");
                    fc.send_arm(true);
                    break;
                case drone::ipc::FCCommandType::DISARM:
                    spdlog::info("[Comms] FC cmd: DISARM");
                    fc.send_arm(false);
                    break;
                case drone::ipc::FCCommandType::TAKEOFF:
                    spdlog::info("[Comms] FC cmd: TAKEOFF to {:.1f}m",
                                 fc_cmd.param1);
                    fc.send_takeoff(fc_cmd.param1);
                    break;
                case drone::ipc::FCCommandType::SET_MODE:
                    spdlog::info("[Comms] FC cmd: SET_MODE {}",
                                 static_cast<int>(fc_cmd.param1));
                    fc.send_mode(static_cast<uint8_t>(fc_cmd.param1));
                    break;
                case drone::ipc::FCCommandType::RTL:
                    spdlog::info("[Comms] FC cmd: RTL");
                    fc.send_mode(3);  // 3 = RTL
                    last_traj_ts = UINT64_MAX;  // block stale trajectory from re-entering offboard
                    break;
                case drone::ipc::FCCommandType::LAND:
                    spdlog::info("[Comms] FC cmd: LAND");
                    fc.send_mode(2);  // 2 = AUTO (Hold/Land)
                    last_traj_ts = UINT64_MAX;  // block stale trajectory from re-entering offboard
                    break;
                default:
                    break;
            }
        }

        // ── Forward trajectory velocity commands (dedup by timestamp) ──
        drone::ipc::ShmTrajectoryCmd cmd{};
        if (traj_sub.receive(cmd) && cmd.valid &&
            cmd.timestamp_ns > last_traj_ts) {
            last_traj_ts = cmd.timestamp_ns;
            fc.send_trajectory(cmd.velocity_x, cmd.velocity_y,
                               cmd.velocity_z, cmd.target_yaw);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ── GCS receive thread (2 Hz) ───────────────────────────────
static void gcs_rx_thread(drone::hal::IGCSLink& gcs,
                          drone::ipc::IPublisher<drone::ipc::ShmGCSCommand>& pub) {
    set_thread_params("gcs_rx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] gcs_rx thread started using {}", gcs.name());

    while (g_running.load(std::memory_order_relaxed)) {
        auto msg = gcs.poll_command();
        if (msg.valid) {
            drone::ipc::ShmGCSCommand shm_cmd{};
            shm_cmd.timestamp_ns = msg.timestamp_ns;
            switch (msg.type) {
                case drone::hal::GCSCommandType::RTL:
                    shm_cmd.command = drone::ipc::GCSCommandType::RTL;
                    break;
                case drone::hal::GCSCommandType::LAND:
                    shm_cmd.command = drone::ipc::GCSCommandType::LAND;
                    break;
                default:
                    shm_cmd.command = drone::ipc::GCSCommandType::NONE;
                    break;
            }
            shm_cmd.valid = true;
            pub.publish(shm_cmd);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// ── GCS telemetry transmit (2 Hz) ───────────────────────────
static void gcs_tx_thread(drone::hal::IGCSLink& gcs,
                          drone::ipc::ISubscriber<drone::ipc::ShmPose>& pose_sub,
                          drone::ipc::ISubscriber<drone::ipc::ShmMissionStatus>& status_sub,
                          drone::ipc::ISubscriber<drone::ipc::ShmFCState>& fc_sub) {
    set_thread_params("gcs_tx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] gcs_tx thread started using {}", gcs.name());

    // Wait for subscribers to connect
    while (g_running.load(std::memory_order_relaxed)) {
        if (pose_sub.is_connected() && status_sub.is_connected() &&
            fc_sub.is_connected()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    while (g_running.load(std::memory_order_relaxed)) {
        drone::ipc::ShmPose pose{};
        drone::ipc::ShmMissionStatus mission{};
        drone::ipc::ShmFCState fc{};

        pose_sub.receive(pose);
        status_sub.receive(mission);
        fc_sub.receive(fc);

        gcs.send_telemetry(
            static_cast<float>(pose.translation[0]),
            static_cast<float>(pose.translation[1]),
            static_cast<float>(pose.translation[2]),
            fc.battery_remaining,
            static_cast<uint8_t>(mission.state));

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "comms");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("comms", LogConfig::resolve_log_dir(), args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== Comms process starting (PID {}) ===", getpid());

    // ── Create links via HAL factory ────────────────────────
    auto fc_link  = drone::hal::create_fc_link(cfg, "comms.mavlink");
    auto fc_backend = cfg.get<std::string>("comms.mavlink.backend", "simulated");
    if (fc_backend == "mavlink") {
        // MavlinkFCLink: "port" = connection URI, "baud" = timeout_ms
        fc_link->open(
            cfg.get<std::string>("comms.mavlink.uri", "udp://:14540"),
            cfg.get<int>("comms.mavlink.timeout_ms", 8000));
    } else {
        fc_link->open(
            cfg.get<std::string>("comms.mavlink.serial_port", "/dev/ttyTHS1"),
            cfg.get<int>("comms.mavlink.baud_rate", 921600));
    }

    auto gcs_link = drone::hal::create_gcs_link(cfg, "comms.gcs");
    gcs_link->open("0.0.0.0",
                   cfg.get<int>("comms.gcs.udp_port", 14550));

    spdlog::info("FC link: {}, GCS link: {}", fc_link->name(), gcs_link->name());

    // ── Create message bus (config-driven: shm or zenoh) ───
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("comms");

    // ── Publishers ──────────────────────────────────────────
    auto fc_pub = drone::ipc::bus_advertise<drone::ipc::ShmFCState>(
        bus, drone::ipc::shm_names::FC_STATE);
    auto gcs_cmd_pub = drone::ipc::bus_advertise<drone::ipc::ShmGCSCommand>(
        bus, drone::ipc::shm_names::GCS_COMMANDS);
    if (!fc_pub->is_ready() || !gcs_cmd_pub->is_ready()) {
        spdlog::error("Failed to create Comms publishers");
        return 1;
    }

    // ── Subscribers ─────────────────────────────────────────
    auto traj_sub = drone::ipc::bus_subscribe<drone::ipc::ShmTrajectoryCmd>(
        bus, drone::ipc::shm_names::TRAJECTORY_CMD);
    auto fc_cmd_sub = drone::ipc::bus_subscribe<drone::ipc::ShmFCCommand>(
        bus, drone::ipc::shm_names::FC_COMMANDS);
    auto pose_sub = drone::ipc::bus_subscribe<drone::ipc::ShmPose>(
        bus, drone::ipc::shm_names::SLAM_POSE);
    auto mission_sub = drone::ipc::bus_subscribe<drone::ipc::ShmMissionStatus>(
        bus, drone::ipc::shm_names::MISSION_STATUS);
    auto fc_sub = drone::ipc::bus_subscribe<drone::ipc::ShmFCState>(
        bus, drone::ipc::shm_names::FC_STATE);

    spdlog::info("Comms READY");

    // ── Launch threads ──────────────────────────────────────
    std::thread t1(fc_rx_thread,  std::ref(*fc_link),  std::ref(*fc_pub));
    std::thread t2(fc_tx_thread,  std::ref(*fc_link),  std::ref(*traj_sub),
                   std::ref(*fc_cmd_sub));
    std::thread t3(gcs_rx_thread, std::ref(*gcs_link), std::ref(*gcs_cmd_pub));
    std::thread t4(gcs_tx_thread, std::ref(*gcs_link),
                   std::ref(*pose_sub),
                   std::ref(*mission_sub),
                   std::ref(*fc_sub));

    t1.join(); t2.join(); t3.join(); t4.join();

    fc_link->close();
    gcs_link->close();

    spdlog::info("=== Comms stopped ===");
    return 0;
}
