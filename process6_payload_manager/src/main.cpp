// process6_payload_manager/src/main.cpp
// Process 6 — Payload Manager: controls gimbal + camera.
// Uses HAL IGimbal interface — backend selected via config.
// Reads ShmPayloadCommand from Mission Planner, publishes ShmPayloadStatus.

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

int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "payload_manager");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("payload_manager", "/tmp/drone_logs", args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== Payload Manager starting (PID {}) ===", getpid());

    // ── Init gimbal via HAL factory ─────────────────────────
    auto gimbal = drone::hal::create_gimbal(cfg, "payload_manager.gimbal");
    gimbal->init();
    spdlog::info("Gimbal: {}", gimbal->name());

    // ── SHM ─────────────────────────────────────────────────
    ShmReader<drone::ipc::ShmPayloadCommand> cmd_reader;
    for (int i = 0; i < 50 && g_running; ++i) {
        if (cmd_reader.open(drone::ipc::shm_names::PAYLOAD_COMMANDS)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!cmd_reader.is_open()) {
        spdlog::error("Cannot open payload commands SHM");
        return 1;
    }

    ShmWriter<drone::ipc::ShmPayloadStatus> status_writer;
    if (!status_writer.create(drone::ipc::shm_names::PAYLOAD_STATUS)) {
        spdlog::error("Failed to create payload status SHM");
        return 1;
    }

    spdlog::info("Payload Manager READY");
    uint64_t last_cmd_ts = 0;

    const int update_hz = cfg.get<int>("payload_manager.update_rate_hz", 50);
    const int loop_sleep_ms = update_hz > 0 ? 1000 / update_hz : 20;
    const float dt = loop_sleep_ms / 1000.0f;

    // ── Main loop (configurable control rate) ───────────────
    while (g_running.load(std::memory_order_relaxed)) {

        // Read commands
        drone::ipc::ShmPayloadCommand cmd{};
        if (cmd_reader.read(cmd) && cmd.valid &&
            cmd.timestamp_ns != last_cmd_ts) {
            last_cmd_ts = cmd.timestamp_ns;

            gimbal->set_target(cmd.gimbal_pitch, cmd.gimbal_yaw);

            switch (cmd.action) {
                case drone::ipc::PayloadAction::CAMERA_CAPTURE:
                    gimbal->capture_image();
                    break;
                case drone::ipc::PayloadAction::CAMERA_START_VIDEO:
                    gimbal->start_recording();
                    break;
                case drone::ipc::PayloadAction::CAMERA_STOP_VIDEO:
                    gimbal->stop_recording();
                    break;
                default:
                    break;
            }
        }

        // Update gimbal motion
        gimbal->update(dt);

        // Publish status
        auto g_state = gimbal->state();
        drone::ipc::ShmPayloadStatus status{};
        status.timestamp_ns = std::chrono::duration_cast<
            std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        status.gimbal_pitch      = g_state.pitch;
        status.gimbal_yaw        = g_state.yaw;
        status.gimbal_stabilized = g_state.stabilised;
        status.recording_video   = gimbal->is_recording();
        status_writer.write(status);

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    spdlog::info("=== Payload Manager stopped ===");
    return 0;
}
