// process5_comms/src/main.cpp
// Process 5 — Comms: bridges companion computer ↔ flight controller (FC)
//                    and companion computer ↔ ground control station (GCS).
// Uses HAL interfaces — backends selected via config.
// Threads:
//   fc_rx    — receives FC state, publishes ShmFCState
//   fc_tx    — reads ShmTrajectoryCmd, sends to FC
//   gcs_rx   — polls GCS for commands, publishes ShmGCSCommand
//   gcs_tx   — reads mission status + pose, sends telemetry to GCS

#include "hal/hal_factory.h"
#include "ipc/message_bus_factory.h"
#include "ipc/shm_reader.h"
#include "ipc/shm_types.h"
#include "ipc/zenoh_liveliness.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/correlation.h"
#include "util/diagnostic.h"
#include "util/log_config.h"
#include "util/realtime.h"
#include "util/signal_handler.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include <spdlog/spdlog.h>

static std::atomic<bool> g_running{true};

// ── FC receive thread (10 Hz) ───────────────────────────────
static void fc_rx_thread(drone::hal::IFCLink&                            fc,
                         drone::ipc::IPublisher<drone::ipc::ShmFCState>& pub) {
    set_thread_params("fc_rx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] fc_rx thread started using {}", fc.name());

    // Optional fault-injection override reader.
    ShmReader<drone::ipc::ShmFaultOverrides> override_reader;
    (void)override_reader.open(drone::ipc::shm_names::FAULT_OVERRIDES);  // may fail — that's fine

    // When FC link is overridden to disconnected, freeze the timestamp
    // so the FaultManager sees a stale heartbeat (simulates real link loss).
    uint64_t frozen_ts = 0;

    auto hb = drone::util::ScopedHeartbeat("fc_rx", true);

    while (g_running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        auto hb = fc.receive_state();

        drone::ipc::ShmFCState state{};
        state.timestamp_ns       = hb.timestamp_ns;
        state.battery_voltage    = hb.battery_voltage;
        state.battery_remaining  = hb.battery_percent;
        state.rel_alt            = hb.altitude_rel;
        state.vx                 = hb.ground_speed;
        state.satellites_visible = hb.satellites;
        state.flight_mode        = hb.flight_mode;
        state.armed              = hb.armed;
        state.connected          = true;

        // Apply fault-injection overrides (if any).
        drone::ipc::ShmFaultOverrides ovr{};
        if (!override_reader.is_open()) {
            // Retry open — the fault_injector may create it later.
            (void)override_reader.open(drone::ipc::shm_names::FAULT_OVERRIDES);
        }
        if (override_reader.is_open() && override_reader.read(ovr)) {
            if (ovr.battery_percent >= 0.0f) {
                state.battery_remaining = ovr.battery_percent;
                state.battery_voltage   = ovr.battery_voltage;
            }
            if (ovr.fc_connected >= 0) {
                if (ovr.fc_connected == 0) {
                    // Simulate real link loss: freeze the timestamp so
                    // the FaultManager sees stale heartbeat data.
                    if (frozen_ts == 0) frozen_ts = state.timestamp_ns;
                    state.timestamp_ns = frozen_ts;
                    state.connected    = false;
                } else {
                    frozen_ts       = 0;  // link restored — resume live timestamps
                    state.connected = true;
                }
            }
        }

        pub.publish(state);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── FC transmit thread (20 Hz) ──────────────────────────────
// Forwards trajectory commands AND FC commands (arm, takeoff, mode)
// from the mission planner to the flight controller.
static void fc_tx_thread(drone::hal::IFCLink&                                   fc,
                         drone::ipc::ISubscriber<drone::ipc::ShmTrajectoryCmd>& traj_sub,
                         drone::ipc::ISubscriber<drone::ipc::ShmFCCommand>&     cmd_sub) {
    set_thread_params("fc_tx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] fc_tx thread started using {}", fc.name());

    auto hb = drone::util::ScopedHeartbeat("fc_tx", true);

    uint64_t last_cmd_seq    = 0;  // dedup FC commands by sequence_id
    uint64_t last_traj_ts    = 0;  // dedup trajectory commands by timestamp
    uint64_t send_fail_count = 0;
    uint64_t traj_send_fail  = 0;

    while (g_running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        // ── Handle FC commands (arm, takeoff, mode) ─────────
        drone::ipc::ShmFCCommand fc_cmd{};
        if (cmd_sub.is_connected() && cmd_sub.receive(fc_cmd) && fc_cmd.valid &&
            fc_cmd.sequence_id > last_cmd_seq) {
            last_cmd_seq = fc_cmd.sequence_id;
            drone::util::ScopedCorrelation guard(fc_cmd.correlation_id);
            bool                           cmd_ok = true;

            switch (fc_cmd.command) {
                case drone::ipc::FCCommandType::ARM:
                    spdlog::info("[Comms] FC cmd: ARM corr={:#x}", fc_cmd.correlation_id);
                    cmd_ok = fc.send_arm(true);
                    break;
                case drone::ipc::FCCommandType::DISARM:
                    spdlog::info("[Comms] FC cmd: DISARM corr={:#x}", fc_cmd.correlation_id);
                    cmd_ok = fc.send_arm(false);
                    break;
                case drone::ipc::FCCommandType::TAKEOFF:
                    spdlog::info("[Comms] FC cmd: TAKEOFF to {:.1f}m corr={:#x}", fc_cmd.param1,
                                 fc_cmd.correlation_id);
                    cmd_ok = fc.send_takeoff(fc_cmd.param1);
                    break;
                case drone::ipc::FCCommandType::SET_MODE:
                    spdlog::info("[Comms] FC cmd: SET_MODE {} corr={:#x}",
                                 static_cast<int>(fc_cmd.param1), fc_cmd.correlation_id);
                    cmd_ok = fc.send_mode(static_cast<uint8_t>(fc_cmd.param1));
                    break;
                case drone::ipc::FCCommandType::RTL:
                    spdlog::info("[Comms] FC cmd: RTL corr={:#x}", fc_cmd.correlation_id);
                    cmd_ok       = fc.send_mode(3);  // 3 = RTL
                    last_traj_ts = UINT64_MAX;  // block stale trajectory from re-entering offboard
                    break;
                case drone::ipc::FCCommandType::LAND:
                    spdlog::info("[Comms] FC cmd: LAND corr={:#x}", fc_cmd.correlation_id);
                    cmd_ok       = fc.send_mode(2);  // 2 = AUTO (Hold/Land)
                    last_traj_ts = UINT64_MAX;  // block stale trajectory from re-entering offboard
                    break;
                default: break;
            }
            if (!cmd_ok) {
                ++send_fail_count;
                spdlog::warn("[Comms] FC command send FAILED: cmd={} seq={} "
                             "(total failures: {})",
                             static_cast<int>(fc_cmd.command), fc_cmd.sequence_id, send_fail_count);
            }
        }

        // ── Forward trajectory velocity commands (dedup by timestamp) ──
        drone::ipc::ShmTrajectoryCmd cmd{};
        if (traj_sub.receive(cmd) && cmd.valid && cmd.timestamp_ns > last_traj_ts) {
            last_traj_ts = cmd.timestamp_ns;
            // target_yaw is stored in radians (ShmTrajectoryCmd contract);
            // MavlinkFCLink::send_trajectory expects degrees (PX4 VelocityNedYaw).
            const float yaw_deg = cmd.target_yaw * (180.0f / static_cast<float>(M_PI));
            if (!fc.send_trajectory(cmd.velocity_x, cmd.velocity_y, cmd.velocity_z, yaw_deg)) {
                ++traj_send_fail;
                if (traj_send_fail == 1 || traj_send_fail % 100 == 0) {
                    spdlog::warn("[Comms] Trajectory send failed (#{}) — "
                                 "FC link may be degraded",
                                 traj_send_fail);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ── GCS receive thread (2 Hz) ───────────────────────────────
static void gcs_rx_thread(drone::hal::IGCSLink&                              gcs,
                          drone::ipc::IPublisher<drone::ipc::ShmGCSCommand>& pub) {
    set_thread_params("gcs_rx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] gcs_rx thread started using {}", gcs.name());

    auto hb = drone::util::ScopedHeartbeat("gcs_rx", false);

    while (g_running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        auto msg = gcs.poll_command();
        if (msg.valid) {
            drone::ipc::ShmGCSCommand shm_cmd{};
            shm_cmd.timestamp_ns   = msg.timestamp_ns;
            shm_cmd.correlation_id = drone::util::CorrelationContext::generate();
            switch (msg.type) {
                case drone::hal::GCSCommandType::RTL:
                    shm_cmd.command = drone::ipc::GCSCommandType::RTL;
                    break;
                case drone::hal::GCSCommandType::LAND:
                    shm_cmd.command = drone::ipc::GCSCommandType::LAND;
                    break;
                default: shm_cmd.command = drone::ipc::GCSCommandType::NONE; break;
            }
            shm_cmd.valid = true;
            spdlog::info("[Comms] GCS cmd received: {} corr={:#x}",
                         static_cast<int>(shm_cmd.command), shm_cmd.correlation_id);
            pub.publish(shm_cmd);
            drone::util::CorrelationContext::clear();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// ── GCS telemetry transmit (2 Hz) ───────────────────────────
static void gcs_tx_thread(drone::hal::IGCSLink&                                  gcs,
                          drone::ipc::ISubscriber<drone::ipc::ShmPose>&          pose_sub,
                          drone::ipc::ISubscriber<drone::ipc::ShmMissionStatus>& status_sub,
                          drone::ipc::ISubscriber<drone::ipc::ShmFCState>&       fc_sub) {
    set_thread_params("gcs_tx", 0, SCHED_OTHER, 0);
    spdlog::info("[Comms] gcs_tx thread started using {}", gcs.name());

    auto hb = drone::util::ScopedHeartbeat("gcs_tx", false);

    uint64_t telem_fail_count = 0;

    // Wait for subscribers to connect
    while (g_running.load(std::memory_order_relaxed)) {
        if (pose_sub.is_connected() && status_sub.is_connected() && fc_sub.is_connected()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    while (g_running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::ipc::ShmPose          pose{};
        drone::ipc::ShmMissionStatus mission{};
        drone::ipc::ShmFCState       fc{};

        pose_sub.receive(pose);
        status_sub.receive(mission);
        fc_sub.receive(fc);

        if (!gcs.send_telemetry(static_cast<float>(pose.translation[0]),
                                static_cast<float>(pose.translation[1]),
                                static_cast<float>(pose.translation[2]), fc.battery_remaining,
                                static_cast<uint8_t>(mission.state))) {
            ++telem_fail_count;
            if (telem_fail_count == 1 || telem_fail_count % 100 == 0) {
                spdlog::warn("[Comms] GCS telemetry send failed (#{}) — "
                             "GCS link may be down",
                             telem_fail_count);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "comms");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("comms", LogConfig::resolve_log_dir(), args.log_level, args.json_logs);

    drone::Config cfg;
    if (!cfg.load(args.config_path)) {
        spdlog::warn("Running with default configuration; failed to load '{}'", args.config_path);
    }

    spdlog::info("=== Comms process starting (PID {}) ===", getpid());

    // ── Create links via HAL factory ────────────────────────
    auto fc_link    = drone::hal::create_fc_link(cfg, "comms.mavlink");
    auto fc_backend = cfg.get<std::string>("comms.mavlink.backend", "simulated");
    bool fc_open_ok = false;
    if (fc_backend == "mavlink") {
        // MavlinkFCLink: "port" = connection URI, "baud" = timeout_ms
        fc_open_ok = fc_link->open(cfg.get<std::string>("comms.mavlink.uri", "udp://:14540"),
                                   cfg.get<int>("comms.mavlink.timeout_ms", 8000));
    } else {
        fc_open_ok =
            fc_link->open(cfg.get<std::string>("comms.mavlink.serial_port", "/dev/ttyTHS1"),
                          cfg.get<int>("comms.mavlink.baud_rate", 921600));
    }
    if (!fc_open_ok) {
        spdlog::error("Failed to open FC link (backend: {})", fc_backend);
        return 1;
    }

    auto gcs_link = drone::hal::create_gcs_link(cfg, "comms.gcs");
    if (!gcs_link->open("0.0.0.0", cfg.get<int>("comms.gcs.udp_port", 14550))) {
        spdlog::error("Failed to open GCS link");
        return 1;
    }

    spdlog::info("FC link: {}, GCS link: {}", fc_link->name(), gcs_link->name());

    // ── Create message bus (config-driven: shm or zenoh) ───
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("comms");

    // ── Publishers ──────────────────────────────────────────
    auto fc_pub = bus.advertise<drone::ipc::ShmFCState>(drone::ipc::shm_names::FC_STATE);
    auto gcs_cmd_pub =
        bus.advertise<drone::ipc::ShmGCSCommand>(drone::ipc::shm_names::GCS_COMMANDS);
    auto mission_upload_pub =
        bus.advertise<drone::ipc::ShmMissionUpload>(drone::ipc::shm_names::MISSION_UPLOAD);
    if (!fc_pub->is_ready() || !gcs_cmd_pub->is_ready() || !mission_upload_pub->is_ready()) {
        spdlog::error("Failed to create Comms publishers");
        return 1;
    }

    // ── Subscribers ─────────────────────────────────────────
    auto traj_sub =
        bus.subscribe<drone::ipc::ShmTrajectoryCmd>(drone::ipc::shm_names::TRAJECTORY_CMD);
    auto fc_cmd_sub = bus.subscribe<drone::ipc::ShmFCCommand>(drone::ipc::shm_names::FC_COMMANDS);
    auto pose_sub   = bus.subscribe<drone::ipc::ShmPose>(drone::ipc::shm_names::SLAM_POSE);
    auto mission_sub =
        bus.subscribe<drone::ipc::ShmMissionStatus>(drone::ipc::shm_names::MISSION_STATUS);
    auto fc_sub = bus.subscribe<drone::ipc::ShmFCState>(drone::ipc::shm_names::FC_STATE);

    spdlog::info("Comms READY");

    // ── Launch threads ──────────────────────────────────────
    std::thread t1(fc_rx_thread, std::ref(*fc_link), std::ref(*fc_pub));
    std::thread t2(fc_tx_thread, std::ref(*fc_link), std::ref(*traj_sub), std::ref(*fc_cmd_sub));
    std::thread t3(gcs_rx_thread, std::ref(*gcs_link), std::ref(*gcs_cmd_pub));
    std::thread t4(gcs_tx_thread, std::ref(*gcs_link), std::ref(*pose_sub), std::ref(*mission_sub),
                   std::ref(*fc_sub));

    // ── Thread watchdog + health publisher ──────────────────
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub =
        bus.advertise<drone::ipc::ShmThreadHealth>(drone::ipc::shm_names::THREAD_HEALTH_COMMS);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "comms", watchdog);

    // ── Main loop: health publishing (replaces bare join) ─
    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        health_publisher.publish_snapshot();
    }

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    fc_link->close();
    gcs_link->close();

    spdlog::info("=== Comms stopped ===");
    return 0;
}
