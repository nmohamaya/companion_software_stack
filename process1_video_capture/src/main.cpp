// process1_video_capture/src/main.cpp
// Process 1 — Video Capture: camera capture + publish.
// Uses HAL ICamera interface — backend selected via config ("simulated" today; "v4l2" planned).
// Uses MessageBusFactory for publishing — backend selected at runtime via config.

#include "hal/hal_factory.h"
#include "ipc/ipc_types.h"
#include "util/config_keys.h"
#include "util/diagnostic.h"
#include "util/process_context.h"
#include "util/scoped_timer.h"
#include "util/sd_notify.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"
#include "video/frame.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

static std::atomic<bool> g_running{true};

// ── Mission camera thread ───────────────────────────────────
static void mission_cam_thread(drone::hal::ICamera&                            camera,
                               drone::ipc::IPublisher<drone::ipc::VideoFrame>& publisher,
                               std::atomic<bool>& running, int fps) {
    DRONE_LOG_INFO("[MissionCam] Thread started using {} @ {}Hz", camera.name(), fps);

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

        drone::ipc::VideoFrame ipc_frame{};
        ipc_frame.timestamp_ns    = frame.timestamp_ns;
        ipc_frame.sequence_number = frame.sequence;
        ipc_frame.width           = frame.width;
        ipc_frame.height          = frame.height;
        ipc_frame.channels        = frame.channels;
        ipc_frame.stride          = frame.stride;

        // Copy pixel data into IPC frame
        size_t copy_size = std::min(static_cast<size_t>(frame.height) * frame.stride,
                                    sizeof(ipc_frame.pixel_data));
        if (!frame.data.empty()) {
            std::copy(frame.data.data(), frame.data.data() + copy_size, ipc_frame.pixel_data);
        } else {
            ++null_data_count;
            ++drop_count;
            diag.add_error("Camera", "Frame marked valid but data is empty (null #" +
                                         std::to_string(null_data_count) + ")");
            if (null_data_count == 1 || null_data_count % 100 == 0) {
                diag.log_summary("MissionCam");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }

        diag.add_metric("Camera", "copy_bytes", static_cast<double>(copy_size));
        publisher.publish(ipc_frame);

        ++seq;
        if (diag.has_errors() || diag.has_warnings()) {
            diag.log_summary("MissionCam");
        } else if (seq % 300 == 0) {
            DRONE_LOG_INFO("[MissionCam] Published frame #{} ({} drops, {} null-data)", seq,
                           drop_count, null_data_count);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    DRONE_LOG_INFO("[MissionCam] Thread stopped — {} frames, {} drops, {} null-data", seq,
                   drop_count, null_data_count);
}

// ── Stereo camera thread ────────────────────────────────────
static void stereo_cam_thread(drone::hal::ICamera& left_cam, drone::hal::ICamera& right_cam,
                              drone::ipc::IPublisher<drone::ipc::StereoFrame>& publisher,
                              std::atomic<bool>& running, int fps) {
    DRONE_LOG_INFO("[StereoCam] Thread started using {} + {} @ {}Hz", left_cam.name(),
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

        drone::ipc::StereoFrame ipc_frame{};
        ipc_frame.timestamp_ns    = left_frame.timestamp_ns;
        ipc_frame.sequence_number = left_frame.sequence;
        ipc_frame.width           = left_frame.width;
        ipc_frame.height          = left_frame.height;

        // Copy left and right data using height * stride for correct padding
        size_t copy_size_left = std::min(static_cast<size_t>(left_frame.height) * left_frame.stride,
                                         sizeof(ipc_frame.left_data));
        size_t copy_size_right =
            std::min(static_cast<size_t>(right_frame.height) * right_frame.stride,
                     sizeof(ipc_frame.right_data));

        // Treat empty data as a dropped pair — don't publish corrupted
        // frames into SLAM.
        const bool left_has_data  = !left_frame.data.empty();
        const bool right_has_data = !right_frame.data.empty();
        if (!left_has_data) diag.add_error("CameraLeft", "Valid frame but empty data");
        if (!right_has_data) diag.add_error("CameraRight", "Valid frame but empty data");

        if (!left_has_data || !right_has_data) {
            ++drop_count;
            if (diag.has_errors() || diag.has_warnings()) {
                diag.log_summary("StereoCam");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }

        std::copy(left_frame.data.data(), left_frame.data.data() + copy_size_left,
                  ipc_frame.left_data);
        std::copy(right_frame.data.data(), right_frame.data.data() + copy_size_right,
                  ipc_frame.right_data);

        publisher.publish(ipc_frame);

        ++seq;
        if (diag.has_errors() || diag.has_warnings()) {
            // Rate-limit: don't log every frame if sync warnings persist
            if (seq <= 1 || seq % 100 == 0) {
                diag.log_summary("StereoCam");
            }
        } else if (seq % 300 == 0) {
            DRONE_LOG_INFO("[StereoCam] Published stereo pair #{} "
                           "({} drops, {} sync warnings)",
                           seq, drop_count, sync_warn_count);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    DRONE_LOG_INFO("[StereoCam] Thread stopped — {} pairs, {} drops, {} sync warnings", seq,
                   drop_count, sync_warn_count);
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    // ── Common boilerplate (args, signals, logging, config, bus) ──
    auto ctx_result = drone::util::init_process(argc, argv, "video_capture", g_running,
                                                drone::util::video_capture_schema());
    if (!ctx_result.is_ok()) return ctx_result.error();
    auto& ctx = ctx_result.value();

    // ── Create cameras via HAL factory ──────────────────────
    auto mission_cam =
        drone::hal::create_camera(ctx.cfg, drone::cfg_key::video_capture::mission_cam::SECTION);
    const auto m_w = ctx.cfg.get<uint32_t>(drone::cfg_key::video_capture::mission_cam::WIDTH, 1920);
    const auto m_h = ctx.cfg.get<uint32_t>(drone::cfg_key::video_capture::mission_cam::HEIGHT,
                                           1080);
    const auto m_fps = ctx.cfg.get<int>(drone::cfg_key::video_capture::mission_cam::FPS, 30);
    if (!mission_cam->open(m_w, m_h, m_fps)) {
        DRONE_LOG_ERROR("Failed to open mission camera ({}x{} @ {}fps)", m_w, m_h, m_fps);
        return 1;
    }

    auto stereo_left =
        drone::hal::create_camera(ctx.cfg, drone::cfg_key::video_capture::stereo_cam::SECTION);
    auto stereo_right =
        drone::hal::create_camera(ctx.cfg, drone::cfg_key::video_capture::stereo_cam::SECTION);
    const auto s_w = ctx.cfg.get<uint32_t>(drone::cfg_key::video_capture::stereo_cam::WIDTH, 640);
    const auto s_h = ctx.cfg.get<uint32_t>(drone::cfg_key::video_capture::stereo_cam::HEIGHT, 480);
    const auto s_fps = ctx.cfg.get<int>(drone::cfg_key::video_capture::stereo_cam::FPS, 30);
    if (!stereo_left->open(s_w, s_h, s_fps) || !stereo_right->open(s_w, s_h, s_fps)) {
        DRONE_LOG_ERROR("Failed to open stereo cameras ({}x{} @ {}fps)", s_w, s_h, s_fps);
        return 1;
    }

    DRONE_LOG_INFO("Cameras: mission={}, stereo_l={}, stereo_r={}", mission_cam->name(),
                   stereo_left->name(), stereo_right->name());

    // ── Create publishers ───────────────────────────────────
    auto mission_pub =
        ctx.bus.advertise<drone::ipc::VideoFrame>(drone::ipc::topics::VIDEO_MISSION_CAM);
    if (!mission_pub->is_ready()) {
        DRONE_LOG_ERROR("Failed to create publisher: {}", drone::ipc::topics::VIDEO_MISSION_CAM);
        return 1;
    }

    auto stereo_pub =
        ctx.bus.advertise<drone::ipc::StereoFrame>(drone::ipc::topics::VIDEO_STEREO_CAM);
    if (!stereo_pub->is_ready()) {
        DRONE_LOG_ERROR("Failed to create publisher: {}", drone::ipc::topics::VIDEO_STEREO_CAM);
        return 1;
    }

    // ── Launch threads ──────────────────────────────────────
    std::thread t_mission(mission_cam_thread, std::ref(*mission_cam), std::ref(*mission_pub),
                          std::ref(g_running), m_fps);
    std::thread t_stereo(stereo_cam_thread, std::ref(*stereo_left), std::ref(*stereo_right),
                         std::ref(*stereo_pub), std::ref(g_running), s_fps);

    // ── Thread watchdog + health publisher ──────────────────
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub = ctx.bus.advertise<drone::ipc::ThreadHealth>(
        drone::ipc::topics::THREAD_HEALTH_VIDEO_CAPTURE);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "video_capture",
                                                        watchdog);

    DRONE_LOG_INFO("All threads started — video_capture is READY");
    drone::systemd::notify_ready();

    // ── Main loop: periodic health log ──────────────────────
    while (g_running.load(std::memory_order_acquire)) {
        drone::systemd::notify_watchdog();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        health_publisher.publish_snapshot();
        DRONE_LOG_INFO("[HealthCheck] video_capture alive");
    }

    drone::systemd::notify_stopping();
    DRONE_LOG_INFO("Shutting down...");
    if (t_mission.joinable()) t_mission.join();
    if (t_stereo.joinable()) t_stereo.join();

    mission_cam->close();
    stereo_left->close();
    stereo_right->close();

    DRONE_LOG_INFO("=== Video Capture process stopped ===");
    return 0;
}
