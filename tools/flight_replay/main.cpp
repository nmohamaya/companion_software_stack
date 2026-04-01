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
#include "util/config.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>

#include <spdlog/spdlog.h>

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

    // ── Signal handling ──────────────────────────────────────
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    spdlog::set_level(spdlog::level::info);
    spdlog::info("Flight Replay starting: file={} speed={:.1f}x", logfile, speed);

    // ── Read log file ────────────────────────────────────────
    auto log_result = drone::recorder::read_log_file(logfile);
    if (!log_result.success) {
        spdlog::error("Failed to read log file: {}", logfile);
        return 1;
    }
    if (log_result.entries.empty()) {
        spdlog::warn("Log file is empty: {}", logfile);
        return 0;
    }

    spdlog::info("Loaded {} records from {}", log_result.entries.size(), logfile);

    // ── Create message bus ───────────────────────────────────
    drone::Config cfg;
    if (!cfg.load(config_path)) {
        spdlog::warn("Could not load config '{}' — using defaults", config_path);
    }
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Build per-topic raw publishers ───────────────────────
    // We use raw byte publishing: advertise with a dummy type and publish
    // the payload bytes directly. Since we are replaying, the subscriber
    // will receive the bytes and deserialize per its expected type.
    //
    // For simplicity, we publish each topic's raw payload as-is.
    // The IPC layer uses trivially-copyable types with matching sizes.
    struct TopicPub {
        std::unique_ptr<drone::ipc::IPublisher<uint8_t>> publisher;
    };

    // We'll publish raw bytes using the advertise_raw / publish_raw pattern.
    // Since the codebase uses typed pub/sub, we advertise with a matching
    // type for each known topic.

    // Map: topic name -> publisher that can publish raw bytes
    // We use a simple approach: create typed publishers for known topics.

    using namespace drone::ipc;

    auto pub_slam     = bus.advertise<Pose>(topics::SLAM_POSE);
    auto pub_objects  = bus.advertise<DetectedObjectList>(topics::DETECTED_OBJECTS);
    auto pub_fc_state = bus.advertise<FCState>(topics::FC_STATE);
    auto pub_traj     = bus.advertise<TrajectoryCmd>(topics::TRAJECTORY_CMD);
    auto pub_health   = bus.advertise<SystemHealth>(topics::SYSTEM_HEALTH);
    auto pub_payload  = bus.advertise<PayloadStatus>(topics::PAYLOAD_STATUS);

    // Allow subscribers to connect
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ── Replay loop ──────────────────────────────────────────
    const auto& entries   = log_result.entries;
    uint64_t    first_ts  = entries.front().header.wire_header.timestamp_ns;
    auto        wall_base = std::chrono::steady_clock::now();
    std::size_t replayed  = 0;

    for (std::size_t i = 0; i < entries.size() && g_running.load(std::memory_order_relaxed); ++i) {
        const auto& entry     = entries[i];
        const auto  record_ts = entry.header.wire_header.timestamp_ns;
        const auto  topic     = entry.topic_name;
        const auto  msg_type  = entry.header.wire_header.msg_type;

        // ── Timing: wait until replay time ───────────────────
        if (speed > 0.0f && i > 0) {
            const auto record_delta_ns = record_ts - first_ts;
            const auto scaled_delta    = std::chrono::nanoseconds(static_cast<int64_t>(
                static_cast<double>(record_delta_ns) / static_cast<double>(speed)));
            const auto target_time     = wall_base + scaled_delta;
            const auto now             = std::chrono::steady_clock::now();
            if (target_time > now) {
                std::this_thread::sleep_until(target_time);
            }
        }

        // ── Publish based on message type ────────────────────
        bool published = false;
        if (msg_type == WireMessageType::SLAM_POSE && entry.payload.size() == sizeof(Pose)) {
            Pose msg{};
            std::copy(entry.payload.begin(), entry.payload.end(), reinterpret_cast<uint8_t*>(&msg));
            pub_slam->publish(msg);
            published = true;
        } else if (msg_type == WireMessageType::DETECTIONS &&
                   entry.payload.size() == sizeof(DetectedObjectList)) {
            DetectedObjectList msg{};
            std::copy(entry.payload.begin(), entry.payload.end(), reinterpret_cast<uint8_t*>(&msg));
            pub_objects->publish(msg);
            published = true;
        } else if (msg_type == WireMessageType::FC_STATE &&
                   entry.payload.size() == sizeof(FCState)) {
            FCState msg{};
            std::copy(entry.payload.begin(), entry.payload.end(), reinterpret_cast<uint8_t*>(&msg));
            pub_fc_state->publish(msg);
            published = true;
        } else if (msg_type == WireMessageType::TRAJECTORY_CMD &&
                   entry.payload.size() == sizeof(TrajectoryCmd)) {
            TrajectoryCmd msg{};
            std::copy(entry.payload.begin(), entry.payload.end(), reinterpret_cast<uint8_t*>(&msg));
            pub_traj->publish(msg);
            published = true;
        } else if (msg_type == WireMessageType::SYSTEM_HEALTH &&
                   entry.payload.size() == sizeof(SystemHealth)) {
            SystemHealth msg{};
            std::copy(entry.payload.begin(), entry.payload.end(), reinterpret_cast<uint8_t*>(&msg));
            pub_health->publish(msg);
            published = true;
        } else if (msg_type == WireMessageType::PAYLOAD_STATUS &&
                   entry.payload.size() == sizeof(PayloadStatus)) {
            PayloadStatus msg{};
            std::copy(entry.payload.begin(), entry.payload.end(), reinterpret_cast<uint8_t*>(&msg));
            pub_payload->publish(msg);
            published = true;
        }

        if (published) {
            ++replayed;
        } else {
            spdlog::debug("Skipped record: topic='{}' type={} size={}", topic,
                          static_cast<uint16_t>(msg_type), entry.payload.size());
        }
    }

    spdlog::info("Replay complete: {}/{} records published", replayed, entries.size());
    return 0;
}
