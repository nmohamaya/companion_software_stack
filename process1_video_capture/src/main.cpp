// process1_video_capture/src/main.cpp
// Process 1 — Video Capture: simulated camera capture + SHM publish.
// In simulation mode, generates synthetic frames instead of V4L2/CSI.

#include "video/frame.h"
#include "ipc/shm_writer.h"
#include "ipc/shm_types.h"
#include "util/signal_handler.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <random>
#include <spdlog/spdlog.h>

static std::atomic<bool> g_running{true};

// ── Simulated mission camera thread ─────────────────────────
static void mission_cam_thread(
    ShmWriter<drone::ipc::ShmVideoFrame>& writer,
    std::atomic<bool>& stop_flag,
    uint32_t width, uint32_t height, int fps)
{
    spdlog::info("[MissionCam] Thread started (simulation mode) {}x{}@{}Hz",
                 width, height, fps);
    std::mt19937 rng(42);
    std::uniform_int_distribution<uint8_t> pixel_dist(0, 255);

    uint64_t seq = 0;
    const uint32_t W = width, H = height, C = 3;
    const int sleep_ms = fps > 0 ? 1000 / fps : 33;

    while (!stop_flag.load(std::memory_order_relaxed)) {
        ScopedTimer timer("MissionCam", 50.0);

        drone::ipc::ShmVideoFrame frame{};
        frame.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        frame.sequence_number = seq++;
        frame.width    = W;
        frame.height   = H;
        frame.channels = C;
        frame.stride   = W * C;

        // Generate synthetic gradient pattern (faster than random per pixel)
        for (uint32_t y = 0; y < H; y += 16) {   // sparse fill for speed
            for (uint32_t x = 0; x < W; x += 16) {
                size_t idx = (y * W + x) * C;
                if (idx + 2 < sizeof(frame.pixel_data)) {
                    frame.pixel_data[idx]     = static_cast<uint8_t>(x * 255 / W);
                    frame.pixel_data[idx + 1] = static_cast<uint8_t>(y * 255 / H);
                    frame.pixel_data[idx + 2] = static_cast<uint8_t>((seq * 10) % 256);
                }
            }
        }

        writer.write(frame);

        if (seq % 300 == 0) {
            spdlog::info("[MissionCam] Published frame #{}", seq);
        }

        // 30 Hz target
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    spdlog::info("[MissionCam] Thread stopped after {} frames", seq);
}

// ── Simulated stereo camera thread ──────────────────────────
static void stereo_cam_thread(
    ShmWriter<drone::ipc::ShmStereoFrame>& writer,
    std::atomic<bool>& stop_flag,
    uint32_t width, uint32_t height, int fps)
{
    spdlog::info("[StereoCam] Thread started (simulation mode) {}x{}@{}Hz",
                 width, height, fps);

    uint64_t seq = 0;
    const uint32_t W = width, H = height;
    const int sleep_ms = fps > 0 ? 1000 / fps : 33;

    while (!stop_flag.load(std::memory_order_relaxed)) {
        drone::ipc::ShmStereoFrame frame{};
        frame.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        frame.sequence_number = seq++;
        frame.width  = W;
        frame.height = H;

        // Synthetic stereo pattern: left and right with slight offset
        for (uint32_t y = 0; y < H; y += 8) {
            for (uint32_t x = 0; x < W; x += 8) {
                size_t idx = y * W + x;
                frame.left_data[idx]  = static_cast<uint8_t>((x + y) % 256);
                uint32_t x_shifted = (x + 5) % W;  // 5px stereo disparity
                frame.right_data[idx] = static_cast<uint8_t>((x_shifted + y) % 256);
            }
        }

        writer.write(frame);

        if (seq % 300 == 0) {
            spdlog::info("[StereoCam] Published stereo pair #{}", seq);
        }

        // Target fps
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
    LogConfig::init("video_capture", "/tmp/drone_logs", args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== Video Capture process starting (PID {}) ===", getpid());
    spdlog::info("Mode: SIMULATION (no hardware cameras)");

    // ── Create SHM writers ──────────────────────────────────
    ShmWriter<drone::ipc::ShmVideoFrame> mission_writer;
    if (!mission_writer.create(drone::ipc::shm_names::VIDEO_MISSION_CAM)) {
        spdlog::error("Failed to create SHM: {}", drone::ipc::shm_names::VIDEO_MISSION_CAM);
        return 1;
    }
    spdlog::info("SHM created: {}", drone::ipc::shm_names::VIDEO_MISSION_CAM);

    ShmWriter<drone::ipc::ShmStereoFrame> stereo_writer;
    if (!stereo_writer.create(drone::ipc::shm_names::VIDEO_STEREO_CAM)) {
        spdlog::error("Failed to create SHM: {}", drone::ipc::shm_names::VIDEO_STEREO_CAM);
        return 1;
    }
    spdlog::info("SHM created: {}", drone::ipc::shm_names::VIDEO_STEREO_CAM);

    // ── Launch threads ──────────────────────────────────────
    const auto m_w = cfg.get<uint32_t>("video_capture.mission_cam.width",  1920);
    const auto m_h = cfg.get<uint32_t>("video_capture.mission_cam.height", 1080);
    const auto m_fps = cfg.get<int>("video_capture.mission_cam.fps", 30);
    const auto s_w = cfg.get<uint32_t>("video_capture.stereo_cam.width",  640);
    const auto s_h = cfg.get<uint32_t>("video_capture.stereo_cam.height", 480);
    const auto s_fps = cfg.get<int>("video_capture.stereo_cam.fps", 30);

    std::thread t_mission(mission_cam_thread,
                          std::ref(mission_writer), std::ref(g_running),
                          m_w, m_h, m_fps);
    std::thread t_stereo(stereo_cam_thread,
                          std::ref(stereo_writer), std::ref(g_running),
                          s_w, s_h, s_fps);

    spdlog::info("All threads started — video_capture is READY");

    // ── Main loop: periodic health log ──────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        spdlog::info("[HealthCheck] video_capture alive");
    }

    spdlog::info("Shutting down...");
    if (t_mission.joinable()) t_mission.join();
    if (t_stereo.joinable())  t_stereo.join();

    spdlog::info("=== Video Capture process stopped ===");
    return 0;
}
