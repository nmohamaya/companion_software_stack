// process6_payload_manager/src/main.cpp
// Process 6 — Payload Manager: controls gimbal + camera.
// Uses HAL IGimbal interface — backend selected via config.
// Reads ShmPayloadCommand from Mission Planner, publishes ShmPayloadStatus.

#include "hal/hal_factory.h"
#include "ipc/message_bus_factory.h"
#include "ipc/shm_types.h"
#include "ipc/zenoh_liveliness.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/diagnostic.h"
#include "util/log_config.h"
#include "util/realtime.h"
#include "util/signal_handler.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <atomic>
#include <chrono>
#include <thread>

#include <spdlog/spdlog.h>

static std::atomic<bool> g_running{true};

int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "payload_manager");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("payload_manager", LogConfig::resolve_log_dir(), args.log_level,
                    args.json_logs);

    drone::Config cfg;
    if (!cfg.load(args.config_path)) {
        spdlog::warn("Running with default configuration; failed to load '{}'", args.config_path);
    }

    spdlog::info("=== Payload Manager starting (PID {}) ===", getpid());

    // ── Init gimbal via HAL factory ─────────────────────────
    auto gimbal = drone::hal::create_gimbal(cfg, "payload_manager.gimbal");
    if (!gimbal->init()) {
        spdlog::error("Failed to initialise gimbal ({})", gimbal->name());
        return 1;
    }
    spdlog::info("Gimbal: {}", gimbal->name());

    // ── Create message bus (config-driven: shm or zenoh) ───
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("payload_manager");

    auto cmd_sub =
        bus.subscribe<drone::ipc::ShmPayloadCommand>(drone::ipc::shm_names::PAYLOAD_COMMANDS);
    if (!cmd_sub->is_connected()) {
        spdlog::error("Cannot open payload commands channel");
        return 1;
    }

    auto status_pub =
        bus.advertise<drone::ipc::ShmPayloadStatus>(drone::ipc::shm_names::PAYLOAD_STATUS);
    if (!status_pub->is_ready()) {
        spdlog::error("Failed to create payload status publisher");
        return 1;
    }

    spdlog::info("Payload Manager READY");
    uint64_t last_cmd_ts   = 0;
    uint64_t cycle_count   = 0;
    uint64_t capture_count = 0;
    uint64_t cmd_count     = 0;

    const int   update_hz     = cfg.get<int>("payload_manager.update_rate_hz", 50);
    const int   loop_sleep_ms = std::max(1, update_hz > 0 ? 1000 / update_hz : 20);
    const float dt            = loop_sleep_ms / 1000.0f;

    // ── Thread heartbeat + watchdog + health publisher ──────
    auto                        payload_hb = drone::util::ScopedHeartbeat("payload_loop", false);
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub = bus.advertise<drone::ipc::ShmThreadHealth>(
        drone::ipc::shm_names::THREAD_HEALTH_PAYLOAD_MANAGER);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "payload_manager",
                                                        watchdog);
    uint32_t                           health_tick = 0;

    // ── Main loop (configurable control rate) ───────────────
    while (g_running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(payload_hb.handle());
        drone::util::FrameDiagnostics diag(cycle_count);

        // Read commands
        drone::ipc::ShmPayloadCommand cmd{};
        if (cmd_sub->receive(cmd) && cmd.valid && cmd.timestamp_ns != last_cmd_ts) {
            last_cmd_ts = cmd.timestamp_ns;
            ++cmd_count;

            gimbal->set_target(cmd.gimbal_pitch, cmd.gimbal_yaw);
            diag.add_metric("Gimbal", "target_pitch", static_cast<double>(cmd.gimbal_pitch));
            diag.add_metric("Gimbal", "target_yaw", static_cast<double>(cmd.gimbal_yaw));

            switch (cmd.action) {
                case drone::ipc::PayloadAction::CAMERA_CAPTURE: {
                    auto img_id = gimbal->capture_image();
                    ++capture_count;
                    diag.add_metric("Camera", "capture_id", static_cast<double>(img_id));
                    spdlog::info("[Payload] Captured image #{} (id={})", capture_count, img_id);
                    break;
                }
                case drone::ipc::PayloadAction::CAMERA_START_VIDEO:
                    gimbal->start_recording();
                    spdlog::info("[Payload] Started video recording");
                    break;
                case drone::ipc::PayloadAction::CAMERA_STOP_VIDEO:
                    gimbal->stop_recording();
                    spdlog::info("[Payload] Stopped video recording");
                    break;
                default: break;
            }
        }

        // Update gimbal motion
        gimbal->update(dt);

        // Publish status
        auto                         g_state = gimbal->state();
        drone::ipc::ShmPayloadStatus status{};
        status.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
        status.gimbal_pitch      = g_state.pitch;
        status.gimbal_yaw        = g_state.yaw;
        status.gimbal_stabilized = g_state.stabilised;
        status.recording_video   = gimbal->is_recording();
        status_pub->publish(status);

        // Publish thread health at ~1 Hz
        ++health_tick;
        if (health_tick % static_cast<uint32_t>(std::max(1, 1000 / loop_sleep_ms)) == 0) {
            health_publisher.publish_snapshot();
        }

        if (diag.has_errors() || diag.has_warnings()) {
            diag.log_summary("PayloadManager");
        }

        ++cycle_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    spdlog::info("=== Payload Manager stopped ===");
    return 0;
}
