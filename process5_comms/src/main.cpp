// process5_comms/src/main.cpp
// Process 5 — Comms: bridges companion computer ↔ flight controller (FC)
//                    and companion computer ↔ ground control station (GCS).
// Uses HAL interfaces — backends selected via config.
// Threads:
//   fc_rx    — receives FC state, publishes ShmFCState
//   fc_tx    — reads ShmTrajectoryCmd, sends to FC
//   gcs_rx   — polls GCS for commands, publishes ShmGCSCommand
//   gcs_tx   — reads mission status + pose, sends telemetry to GCS

#include "ipc/shm_reader.h"
#include "ipc/shm_writer.h"
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
                         ShmWriter<drone::ipc::ShmFCState>& writer) {
    set_thread_params("fc_rx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] fc_rx thread started using {}", fc.name());

    while (g_running.load(std::memory_order_relaxed)) {
        auto hb = fc.receive_state();

        drone::ipc::ShmFCState state{};
        state.timestamp_ns      = hb.timestamp_ns;
        state.battery_voltage   = hb.battery_voltage;
        state.battery_remaining = hb.battery_percent;
        state.rel_alt           = hb.altitude_msl;
        state.vx                = hb.ground_speed;
        state.satellites_visible = hb.satellites;
        state.flight_mode       = hb.flight_mode;
        state.armed             = hb.armed;
        state.connected         = true;
        writer.write(state);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── FC transmit thread (20 Hz) ──────────────────────────────
static void fc_tx_thread(drone::hal::IFCLink& fc,
                         ShmReader<drone::ipc::ShmTrajectoryCmd>& reader) {
    set_thread_params("fc_tx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] fc_tx thread started using {}", fc.name());

    // Wait for trajectory SHM
    while (g_running.load(std::memory_order_relaxed) && !reader.is_open()) {
        reader.open(drone::ipc::shm_names::TRAJECTORY_CMD);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    while (g_running.load(std::memory_order_relaxed)) {
        drone::ipc::ShmTrajectoryCmd cmd{};
        if (reader.read(cmd) && cmd.valid) {
            fc.send_trajectory(cmd.velocity_x, cmd.velocity_y,
                               cmd.velocity_z, cmd.target_yaw);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ── GCS receive thread (2 Hz) ───────────────────────────────
static void gcs_rx_thread(drone::hal::IGCSLink& gcs,
                          ShmWriter<drone::ipc::ShmGCSCommand>& writer) {
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
            writer.write(shm_cmd);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// ── GCS telemetry transmit (2 Hz) ───────────────────────────
static void gcs_tx_thread(drone::hal::IGCSLink& gcs,
                          ShmReader<drone::ipc::ShmPose>& pose_reader,
                          ShmReader<drone::ipc::ShmMissionStatus>& status_reader,
                          ShmReader<drone::ipc::ShmFCState>& fc_reader) {
    set_thread_params("gcs_tx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] gcs_tx thread started using {}", gcs.name());

    // Wait for SHM sources
    while (g_running.load(std::memory_order_relaxed)) {
        bool ok = true;
        if (!pose_reader.is_open())
            ok &= pose_reader.open(drone::ipc::shm_names::SLAM_POSE);
        if (!status_reader.is_open())
            ok &= status_reader.open(drone::ipc::shm_names::MISSION_STATUS);
        if (!fc_reader.is_open())
            ok &= fc_reader.open(drone::ipc::shm_names::FC_STATE);
        if (ok) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    while (g_running.load(std::memory_order_relaxed)) {
        drone::ipc::ShmPose pose{};
        drone::ipc::ShmMissionStatus mission{};
        drone::ipc::ShmFCState fc{};

        pose_reader.read(pose);
        status_reader.read(mission);
        fc_reader.read(fc);

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
    LogConfig::init("comms", "/tmp/drone_logs", args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== Comms process starting (PID {}) ===", getpid());

    // ── Create links via HAL factory ────────────────────────
    auto fc_link  = drone::hal::create_fc_link(cfg, "comms.mavlink");
    fc_link->open(
        cfg.get<std::string>("comms.mavlink.serial_port", "/dev/ttyTHS1"),
        cfg.get<int>("comms.mavlink.baud_rate", 921600));

    auto gcs_link = drone::hal::create_gcs_link(cfg, "comms.gcs");
    gcs_link->open("0.0.0.0",
                   cfg.get<int>("comms.gcs.udp_port", 14550));

    spdlog::info("FC link: {}, GCS link: {}", fc_link->name(), gcs_link->name());

    // ── SHM writers ─────────────────────────────────────────
    ShmWriter<drone::ipc::ShmFCState>   fc_writer;
    ShmWriter<drone::ipc::ShmGCSCommand> gcs_cmd_writer;
    if (!fc_writer.create(drone::ipc::shm_names::FC_STATE) ||
        !gcs_cmd_writer.create(drone::ipc::shm_names::GCS_COMMANDS)) {
        spdlog::error("Failed to create Comms SHM outputs");
        return 1;
    }

    // ── SHM readers ─────────────────────────────────────────
    ShmReader<drone::ipc::ShmTrajectoryCmd> traj_reader;
    ShmReader<drone::ipc::ShmPose>          pose_reader;
    ShmReader<drone::ipc::ShmMissionStatus> mission_reader;
    ShmReader<drone::ipc::ShmFCState>       fc_reader;

    spdlog::info("Comms READY");

    // ── Launch threads ──────────────────────────────────────
    std::thread t1(fc_rx_thread,  std::ref(*fc_link),  std::ref(fc_writer));
    std::thread t2(fc_tx_thread,  std::ref(*fc_link),  std::ref(traj_reader));
    std::thread t3(gcs_rx_thread, std::ref(*gcs_link), std::ref(gcs_cmd_writer));
    std::thread t4(gcs_tx_thread, std::ref(*gcs_link),
                   std::ref(pose_reader),
                   std::ref(mission_reader),
                   std::ref(fc_reader));

    t1.join(); t2.join(); t3.join(); t4.join();

    fc_link->close();
    gcs_link->close();

    spdlog::info("=== Comms stopped ===");
    return 0;
}
