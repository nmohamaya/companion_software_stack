// tools/flight_replay/main.cpp
// Flight Replay Tool — reads a .flog file recorded by FlightRecorder
// and republishes messages to IPC at recorded timestamps.
//
// Usage:
//   flight_replay <logfile> [--speed 2.0] [--config path/to/config.json]

#include "ipc/ipc_types.h"
#include "ipc/message_bus_factory.h"
#include "ipc/wire_format.h"
#include "recorder/flight_recorder.h"
#include "recorder/replay_dispatch.h"
#include "util/config.h"
#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_running{true};

static void signal_handler(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
}

static void print_usage(const char* prog) {
    std::cerr
        << "Usage: " << prog << " <logfile.flog> [options]\n"
        << "\nOptions:\n"
        << "  --speed <multiplier>  Playback speed (default: 1.0, 0=as fast as possible)\n"
        << "  --config <path>       Config file for IPC backend (default: config/default.json)\n"
        << "  --help                Show this help\n";
}

int main(int argc, char* argv[]) {
    // ── Parse arguments ──────────────────────────────────────
    std::string logfile;
    float       speed       = 1.0f;
    std::string config_path = "config/default.json";

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (arg == "--speed" && i + 1 < argc) {
            speed = std::strtof(argv[++i], nullptr);
            continue;
        }
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
            continue;
        }
        if (logfile.empty() && arg[0] != '-') {
            logfile = arg;
            continue;
        }
        std::cerr << "Unknown argument: " << arg << "\n";
        print_usage(argv[0]);
        return 1;
    }

    if (logfile.empty()) {
        std::cerr << "Error: no log file specified\n";
        print_usage(argv[0]);
        return 1;
    }

    if (speed < 0.0f || std::isnan(speed)) {
        std::cerr << "Error: --speed must be >= 0 (got " << speed << ")\n";
        return 1;
    }

    // ── Signal handling ──────────────────────────────────────
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    spdlog::set_level(spdlog::level::info);
    DRONE_LOG_INFO("Flight Replay starting: file={} speed={:.1f}x", logfile, speed);

    // ── Read log file ────────────────────────────────────────
    auto log_result = drone::recorder::read_log_file(logfile);
    if (!log_result.success) {
        DRONE_LOG_ERROR("Failed to read log file: {}", logfile);
        return 1;
    }
    if (log_result.entries.empty()) {
        DRONE_LOG_WARN("Log file is empty: {}", logfile);
        return 0;
    }

    DRONE_LOG_INFO("Loaded {} records from {}", log_result.entries.size(), logfile);

    // ── Create message bus ───────────────────────────────────
    drone::Config cfg;
    if (!cfg.load(config_path)) {
        DRONE_LOG_WARN("Could not load config '{}' — using defaults", config_path);
    }
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Build per-topic typed publishers ────────────────────
    // We create typed publishers for each known IPC message type.
    // Routing is by WireMessageType (not topic name) because each message
    // type maps to exactly one canonical topic — the type is sufficient
    // to determine the correct publisher and ensures type-safe deserialization.

    using namespace drone::ipc;

    auto pub_slam     = bus.advertise<Pose>(topics::SLAM_POSE);
    auto pub_objects  = bus.advertise<DetectedObjectList>(topics::DETECTED_OBJECTS);
    auto pub_fc_state = bus.advertise<FCState>(topics::FC_STATE);
    auto pub_traj     = bus.advertise<TrajectoryCmd>(topics::TRAJECTORY_CMD);
    auto pub_health   = bus.advertise<SystemHealth>(topics::SYSTEM_HEALTH);
    auto pub_payload  = bus.advertise<PayloadStatus>(topics::PAYLOAD_STATUS);

    // Allow subscribers to connect
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ── Dispatch callbacks — forward to typed publishers ────
    struct ReplayCallbacks {
        IPublisher<Pose>*               slam;
        IPublisher<DetectedObjectList>* objects;
        IPublisher<FCState>*            fc_state;
        IPublisher<TrajectoryCmd>*      traj;
        IPublisher<SystemHealth>*       health;
        IPublisher<PayloadStatus>*      payload;

        void on_pose(const Pose& msg) { slam->publish(msg); }
        void on_detections(const DetectedObjectList& msg) { objects->publish(msg); }
        void on_fc_state(const FCState& msg) { fc_state->publish(msg); }
        void on_trajectory(const TrajectoryCmd& msg) { traj->publish(msg); }
        void on_health(const SystemHealth& msg) { health->publish(msg); }
        void on_payload(const PayloadStatus& msg) { payload->publish(msg); }
    };

    ReplayCallbacks callbacks{pub_slam.get(), pub_objects.get(), pub_fc_state.get(),
                              pub_traj.get(), pub_health.get(),  pub_payload.get()};

    // ── Replay loop ──────────────────────────────────────────
    const auto& entries   = log_result.entries;
    uint64_t    first_ts  = entries.front().header.wire_header.timestamp_ns;
    auto        wall_base = std::chrono::steady_clock::now();
    std::size_t replayed  = 0;

    for (std::size_t i = 0; i < entries.size() && g_running.load(std::memory_order_relaxed); ++i) {
        const auto& entry     = entries[i];
        const auto  record_ts = entry.header.wire_header.timestamp_ns;
        const auto  topic     = entry.topic_name;

        // ── Timing: wait until replay time ───────────────────
        if (speed > 0.0f && i > 0) {
            const auto delay = drone::recorder::calculate_replay_delay(record_ts, first_ts, speed);
            const auto target_time = wall_base + delay;
            const auto now         = std::chrono::steady_clock::now();
            if (target_time > now) {
                std::this_thread::sleep_until(target_time);
            }
        }

        // ── Validate wire version ────────────────────────────
        if (entry.header.wire_header.version != kWireVersion) {
            DRONE_LOG_WARN("Skipping record {} — wire version {} != current {}", i,
                           entry.header.wire_header.version, kWireVersion);
            continue;
        }

        // ── Dispatch record to typed publisher ───────────────
        auto result = drone::recorder::dispatch_record(entry, callbacks);

        if (result.dispatched) {
            ++replayed;
        } else {
            DRONE_LOG_DEBUG("Skipped record: topic='{}' type={} size={}", topic,
                            static_cast<uint16_t>(result.msg_type), entry.payload.size());
        }
    }

    DRONE_LOG_INFO("Replay complete: {}/{} records published", replayed, entries.size());
    return 0;
}
