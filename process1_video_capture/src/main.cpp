// process1_video_capture/src/main.cpp
// Process 1 — Video Capture: camera capture + publish.
// Uses HAL ICamera interface — backend selected via config ("simulated" today; "v4l2" planned).
// Uses MessageBusFactory for publishing — backend selected at runtime via config.

#include "hal/hal_factory.h"
#include "ipc/message_bus_factory.h"
#include "ipc/shm_types.h"
#include "ipc/zenoh_liveliness.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"
#include "util/signal_handler.h"
#include "video/frame.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

#include <spdlog/spdlog.h>

static std::atomic<bool> g_running{true};

// ── Mission camera thread ───────────────────────────────────
static void mission_cam_thread(drone::hal::ICamera&                               camera,
                               drone::ipc::IPublisher<drone::ipc::ShmVideoFrame>& publisher,
                               std::atomic<bool>& running, int fps) {
    spdlog::info("[MissionCam] Thread started using {} @ {}Hz", camera.name(), fps);

    uint64_t  seq      = 0;
    const int sleep_ms = fps > 0 ? 1000 / fps : 33;

    while (running.load(std::memory_order_relaxed)) {
        ScopedTimer timer("MissionCam", 50.0);

        auto frame = camera.capture();
        if (!frame.valid) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }

        drone::ipc::ShmVideoFrame shm_frame{};
        shm_frame.timestamp_ns    = frame.timestamp_ns;
        shm_frame.sequence_number = frame.sequence;
        shm_frame.width           = frame.width;
        shm_frame.height          = frame.height;
        shm_frame.channels        = frame.channels;
        shm_frame.stride          = frame.stride;

        // Copy pixel data into SHM frame
        size_t copy_size = std::min(static_cast<size_t>(frame.height) * frame.stride,
                                    sizeof(shm_frame.pixel_data));
        if (frame.data) {
            std::memcpy(shm_frame.pixel_data, frame.data, copy_size);
        }

        publisher.publish(shm_frame);

        ++seq;
        if (seq % 300 == 0) {
            spdlog::info("[MissionCam] Published frame #{}", seq);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    spdlog::info("[MissionCam] Thread stopped after {} frames", seq);
}

// ── Stereo camera thread ────────────────────────────────────
static void stereo_cam_thread(drone::hal::ICamera& left_cam, drone::hal::ICamera& right_cam,
                              drone::ipc::IPublisher<drone::ipc::ShmStereoFrame>& publisher,
                              std::atomic<bool>& running, int fps) {
    spdlog::info("[StereoCam] Thread started using {} @ {}Hz", left_cam.name(), fps);

    uint64_t  seq      = 0;
    const int sleep_ms = fps > 0 ? 1000 / fps : 33;

    while (running.load(std::memory_order_relaxed)) {
        auto left_frame  = left_cam.capture();
        auto right_frame = right_cam.capture();

        // Ensure both frames are valid before publishing
        if (!left_frame.valid || !right_frame.valid) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }

        drone::ipc::ShmStereoFrame shm_frame{};
        shm_frame.timestamp_ns    = left_frame.timestamp_ns;
        shm_frame.sequence_number = left_frame.sequence;
        shm_frame.width           = left_frame.width;
        shm_frame.height          = left_frame.height;

        // Copy left and right data using height * stride for correct padding
        size_t copy_size_left = std::min(static_cast<size_t>(left_frame.height) * left_frame.stride,
                                         sizeof(shm_frame.left_data));
        size_t copy_size_right =
            std::min(static_cast<size_t>(right_frame.height) * right_frame.stride,
                     sizeof(shm_frame.right_data));
        if (left_frame.data) {
            std::memcpy(shm_frame.left_data, left_frame.data, copy_size_left);
        }
        if (right_frame.data) {
            std::memcpy(shm_frame.right_data, right_frame.data, copy_size_right);
        }

        publisher.publish(shm_frame);

        ++seq;
        if (seq % 300 == 0) {
            spdlog::info("[StereoCam] Published stereo pair #{}", seq);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    spdlog::info("[StereoCam] Thread stopped after {} frames", seq);
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "video_capture");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("video_capture", LogConfig::resolve_log_dir(), args.log_level);

    drone::Config cfg;
    if (!cfg.load(args.config_path)) {
        spdlog::warn("Running with default configuration; failed to load '{}'",
                     args.config_path);
    }

    spdlog::info("=== Video Capture process starting (PID {}) ===", getpid());

    // ── Create cameras via HAL factory ──────────────────────
    auto       mission_cam = drone::hal::create_camera(cfg, "video_capture.mission_cam");
    const auto m_w         = cfg.get<uint32_t>("video_capture.mission_cam.width", 1920);
    const auto m_h         = cfg.get<uint32_t>("video_capture.mission_cam.height", 1080);
    const auto m_fps       = cfg.get<int>("video_capture.mission_cam.fps", 30);
    mission_cam->open(m_w, m_h, m_fps);

    auto       stereo_left  = drone::hal::create_camera(cfg, "video_capture.stereo_cam");
    auto       stereo_right = drone::hal::create_camera(cfg, "video_capture.stereo_cam");
    const auto s_w          = cfg.get<uint32_t>("video_capture.stereo_cam.width", 640);
    const auto s_h          = cfg.get<uint32_t>("video_capture.stereo_cam.height", 480);
    const auto s_fps        = cfg.get<int>("video_capture.stereo_cam.fps", 30);
    stereo_left->open(s_w, s_h, s_fps);
    stereo_right->open(s_w, s_h, s_fps);

    spdlog::info("Cameras: mission={}, stereo_l={}, stereo_r={}", mission_cam->name(),
                 stereo_left->name(), stereo_right->name());

    // ── Create publishers via message bus factory ───────────
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("video_capture");

    auto mission_pub = drone::ipc::bus_advertise<drone::ipc::ShmVideoFrame>(
        bus, drone::ipc::shm_names::VIDEO_MISSION_CAM);
    if (!mission_pub->is_ready()) {
        spdlog::error("Failed to create publisher: {}", drone::ipc::shm_names::VIDEO_MISSION_CAM);
        return 1;
    }

    auto stereo_pub = drone::ipc::bus_advertise<drone::ipc::ShmStereoFrame>(
        bus, drone::ipc::shm_names::VIDEO_STEREO_CAM);
    if (!stereo_pub->is_ready()) {
        spdlog::error("Failed to create publisher: {}", drone::ipc::shm_names::VIDEO_STEREO_CAM);
        return 1;
    }

    // ── Launch threads ──────────────────────────────────────
    std::thread t_mission(mission_cam_thread, std::ref(*mission_cam), std::ref(*mission_pub),
                          std::ref(g_running), m_fps);
    std::thread t_stereo(stereo_cam_thread, std::ref(*stereo_left), std::ref(*stereo_right),
                         std::ref(*stereo_pub), std::ref(g_running), s_fps);

    spdlog::info("All threads started — video_capture is READY");

    // ── Main loop: periodic health log ──────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        spdlog::info("[HealthCheck] video_capture alive");
    }

    spdlog::info("Shutting down...");
    if (t_mission.joinable()) t_mission.join();
    if (t_stereo.joinable()) t_stereo.join();

    mission_cam->close();
    stereo_left->close();
    stereo_right->close();

    spdlog::info("=== Video Capture process stopped ===");
    return 0;
}
