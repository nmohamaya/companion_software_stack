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
#include "util/diagnostic.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"
#include "util/sd_notify.h"
#include "util/signal_handler.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"
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

    auto hb = drone::util::ScopedHeartbeat("mission_cam", true);

    uint64_t  seq             = 0;
    uint64_t  drop_count      = 0;
    uint64_t  null_data_count = 0;
    const int sleep_ms        = fps > 0 ? 1000 / fps : 33;

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::util::FrameDiagnostics diag(seq);

        auto frame = [&]() {
            drone::util::ScopedDiagTimer timer(diag, "Capture");
            return camera.capture();
        }();

        if (!frame.valid) {
            ++drop_count;
            diag.add_warning("Camera", "Invalid frame (drop #" + std::to_string(drop_count) + ")");
            // Rate-limited logging
            if (drop_count == 1 || drop_count % 100 == 0) {
                diag.log_summary("MissionCam");
            }
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
        } else {
            ++null_data_count;
            ++drop_count;
            diag.add_error("Camera", "Frame marked valid but data is nullptr (null #" +
                                         std::to_string(null_data_count) + ")");
            if (null_data_count == 1 || null_data_count % 100 == 0) {
                diag.log_summary("MissionCam");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }

        diag.add_metric("Camera", "copy_bytes", static_cast<double>(copy_size));
        publisher.publish(shm_frame);

        ++seq;
        if (diag.has_errors() || diag.has_warnings()) {
            diag.log_summary("MissionCam");
        } else if (seq % 300 == 0) {
            spdlog::info("[MissionCam] Published frame #{} ({} drops, {} null-data)", seq,
                         drop_count, null_data_count);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    spdlog::info("[MissionCam] Thread stopped — {} frames, {} drops, {} null-data", seq, drop_count,
                 null_data_count);
}

// ── Stereo camera thread ────────────────────────────────────
static void stereo_cam_thread(drone::hal::ICamera& left_cam, drone::hal::ICamera& right_cam,
                              drone::ipc::IPublisher<drone::ipc::ShmStereoFrame>& publisher,
                              std::atomic<bool>& running, int fps) {
    spdlog::info("[StereoCam] Thread started using {} + {} @ {}Hz", left_cam.name(),
                 right_cam.name(), fps);

    auto hb = drone::util::ScopedHeartbeat("stereo_cam", true);

    uint64_t  seq             = 0;
    uint64_t  drop_count      = 0;
    uint64_t  sync_warn_count = 0;
    const int sleep_ms        = fps > 0 ? 1000 / fps : 33;

    // Stereo sync threshold: if left/right timestamps differ by more than
    // this, the pair may be temporally misaligned.
    constexpr uint64_t kSyncThresholdNs = 5'000'000;  // 5 ms

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::util::FrameDiagnostics diag(seq);

        auto left_frame = [&]() {
            drone::util::ScopedDiagTimer timer(diag, "CaptureLeft");
            return left_cam.capture();
        }();
        auto right_frame = [&]() {
            drone::util::ScopedDiagTimer timer(diag, "CaptureRight");
            return right_cam.capture();
        }();

        // Ensure both frames are valid before publishing
        if (!left_frame.valid || !right_frame.valid) {
            ++drop_count;
            if (!left_frame.valid) diag.add_warning("CameraLeft", "Invalid frame");
            if (!right_frame.valid) diag.add_warning("CameraRight", "Invalid frame");
            if (drop_count == 1 || drop_count % 100 == 0) {
                diag.log_summary("StereoCam");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }

        // Check stereo pair temporal synchronization
        uint64_t ts_delta = (left_frame.timestamp_ns > right_frame.timestamp_ns)
                                ? (left_frame.timestamp_ns - right_frame.timestamp_ns)
                                : (right_frame.timestamp_ns - left_frame.timestamp_ns);
        if (ts_delta > kSyncThresholdNs) {
            ++sync_warn_count;
            diag.add_warning("StereoSync",
                             "L/R timestamp delta = " + std::to_string(ts_delta / 1'000'000.0) +
                                 "ms (threshold " + std::to_string(kSyncThresholdNs / 1'000'000.0) +
                                 "ms)");
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

        // Treat null data pointers as a dropped pair — don't publish corrupted
        // frames into SLAM.
        const bool left_has_data  = (left_frame.data != nullptr);
        const bool right_has_data = (right_frame.data != nullptr);
        if (!left_has_data) diag.add_error("CameraLeft", "Valid frame but null data pointer");
        if (!right_has_data) diag.add_error("CameraRight", "Valid frame but null data pointer");

        if (!left_has_data || !right_has_data) {
            ++drop_count;
            if (diag.has_errors() || diag.has_warnings()) {
                diag.log_summary("StereoCam");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }

        std::memcpy(shm_frame.left_data, left_frame.data, copy_size_left);
        std::memcpy(shm_frame.right_data, right_frame.data, copy_size_right);

        publisher.publish(shm_frame);

        ++seq;
        if (diag.has_errors() || diag.has_warnings()) {
            // Rate-limit: don't log every frame if sync warnings persist
            if (seq <= 1 || seq % 100 == 0) {
                diag.log_summary("StereoCam");
            }
        } else if (seq % 300 == 0) {
            spdlog::info("[StereoCam] Published stereo pair #{} "
                         "({} drops, {} sync warnings)",
                         seq, drop_count, sync_warn_count);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    spdlog::info("[StereoCam] Thread stopped — {} pairs, {} drops, {} sync warnings", seq,
                 drop_count, sync_warn_count);
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "video_capture");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("video_capture", LogConfig::resolve_log_dir(), args.log_level, args.json_logs);

    drone::Config cfg;
    if (!cfg.load(args.config_path)) {
        spdlog::warn("Running with default configuration; failed to load '{}'", args.config_path);
    }

    spdlog::info("=== Video Capture process starting (PID {}) ===", getpid());

    // ── Create cameras via HAL factory ──────────────────────
    auto       mission_cam = drone::hal::create_camera(cfg, "video_capture.mission_cam");
    const auto m_w         = cfg.get<uint32_t>("video_capture.mission_cam.width", 1920);
    const auto m_h         = cfg.get<uint32_t>("video_capture.mission_cam.height", 1080);
    const auto m_fps       = cfg.get<int>("video_capture.mission_cam.fps", 30);
    if (!mission_cam->open(m_w, m_h, m_fps)) {
        spdlog::error("Failed to open mission camera ({}x{} @ {}fps)", m_w, m_h, m_fps);
        return 1;
    }

    auto       stereo_left  = drone::hal::create_camera(cfg, "video_capture.stereo_cam");
    auto       stereo_right = drone::hal::create_camera(cfg, "video_capture.stereo_cam");
    const auto s_w          = cfg.get<uint32_t>("video_capture.stereo_cam.width", 640);
    const auto s_h          = cfg.get<uint32_t>("video_capture.stereo_cam.height", 480);
    const auto s_fps        = cfg.get<int>("video_capture.stereo_cam.fps", 30);
    if (!stereo_left->open(s_w, s_h, s_fps) || !stereo_right->open(s_w, s_h, s_fps)) {
        spdlog::error("Failed to open stereo cameras ({}x{} @ {}fps)", s_w, s_h, s_fps);
        return 1;
    }

    spdlog::info("Cameras: mission={}, stereo_l={}, stereo_r={}", mission_cam->name(),
                 stereo_left->name(), stereo_right->name());

    // ── Create publishers via message bus factory ───────────
    auto bus = drone::ipc::create_message_bus(cfg);

    // ── Declare liveliness token (auto-dropped on exit/crash) ──
    drone::ipc::LivelinessToken liveliness_token("video_capture");

    auto mission_pub =
        bus.advertise<drone::ipc::ShmVideoFrame>(drone::ipc::shm_names::VIDEO_MISSION_CAM);
    if (!mission_pub->is_ready()) {
        spdlog::error("Failed to create publisher: {}", drone::ipc::shm_names::VIDEO_MISSION_CAM);
        return 1;
    }

    auto stereo_pub =
        bus.advertise<drone::ipc::ShmStereoFrame>(drone::ipc::shm_names::VIDEO_STEREO_CAM);
    if (!stereo_pub->is_ready()) {
        spdlog::error("Failed to create publisher: {}", drone::ipc::shm_names::VIDEO_STEREO_CAM);
        return 1;
    }

    // ── Launch threads ──────────────────────────────────────
    std::thread t_mission(mission_cam_thread, std::ref(*mission_cam), std::ref(*mission_pub),
                          std::ref(g_running), m_fps);
    std::thread t_stereo(stereo_cam_thread, std::ref(*stereo_left), std::ref(*stereo_right),
                         std::ref(*stereo_pub), std::ref(g_running), s_fps);

    // ── Thread watchdog + health publisher ──────────────────
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub = bus.advertise<drone::ipc::ShmThreadHealth>(
        drone::ipc::shm_names::THREAD_HEALTH_VIDEO_CAPTURE);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "video_capture",
                                                        watchdog);

    spdlog::info("All threads started — video_capture is READY");
    drone::systemd::notify_ready();

    // ── Main loop: periodic health log ──────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        drone::systemd::notify_watchdog();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        health_publisher.publish_snapshot();
        spdlog::info("[HealthCheck] video_capture alive");
    }

    drone::systemd::notify_stopping();
    spdlog::info("Shutting down...");
    if (t_mission.joinable()) t_mission.join();
    if (t_stereo.joinable()) t_stereo.join();

    mission_cam->close();
    stereo_left->close();
    stereo_right->close();

    spdlog::info("=== Video Capture process stopped ===");
    return 0;
}
