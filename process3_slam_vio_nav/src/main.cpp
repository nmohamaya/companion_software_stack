// process3_slam_vio_nav/src/main.cpp
// Process 3 — SLAM/VIO/Nav: visual-inertial odometry + pose estimation.
// Simulated: generates fake VIO pose estimates from stereo camera input.
// Uses MessageBusFactory — backend selected at runtime via config.

#include "slam/types.h"
#include "slam/ivisual_frontend.h"
#include "ipc/message_bus_factory.h"
#include "ipc/shm_types.h"
#include "util/signal_handler.h"
#include "util/arg_parser.h"
#include "util/config.h"
#include "util/log_config.h"
#include "util/scoped_timer.h"
#include "hal/hal_factory.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <random>
#include <mutex>
#include <spdlog/spdlog.h>

using namespace drone::slam;

// ── Double-buffer for thread-safe pose exchange ─────────────
// Eliminates the use-after-free race in the old raw-pointer atomic pattern.
class PoseDoubleBuffer {
public:
    void write(const Pose& pose) {
        int idx = write_idx_.load(std::memory_order_relaxed) ^ 1;
        buffers_[idx] = pose;
        write_idx_.store(idx, std::memory_order_release);
    }

    bool read(Pose& out) const {
        int idx = write_idx_.load(std::memory_order_acquire);
        if (!initialized_.load(std::memory_order_acquire)) return false;
        out = buffers_[idx];
        return true;
    }

    void mark_initialized() {
        initialized_.store(true, std::memory_order_release);
    }

private:
    Pose buffers_[2];
    std::atomic<int> write_idx_{0};
    std::atomic<bool> initialized_{false};
};

static std::atomic<bool> g_running{true};

// ── Visual Frontend thread (uses IVisualFrontend strategy) ──
// In real system: ORB/KLT feature detection + tracking on stereo frames
static void visual_frontend_thread(
    drone::ipc::ISubscriber<drone::ipc::ShmStereoFrame>& stereo_sub,
    drone::slam::IVisualFrontend& frontend,
    PoseDoubleBuffer& pose_buffer,
    std::atomic<bool>& running)
{
    spdlog::info("[VisualFrontend] Thread started using {}", frontend.name());

    uint64_t frame_count = 0;

    while (running.load(std::memory_order_relaxed)) {
        drone::ipc::ShmStereoFrame frame;
        (void)stereo_sub.receive(frame);

        auto p = frontend.process_frame(frame);

        pose_buffer.write(p);
        pose_buffer.mark_initialized();

        ++frame_count;
        if (frame_count % 300 == 0) {
            spdlog::info("[VisualFrontend] Frame {}: pos=({:.2f}, {:.2f}, {:.2f})",
                         frame_count, p.position.x(), p.position.y(),
                         p.position.z());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    spdlog::info("[VisualFrontend] Thread stopped after {} frames", frame_count);
}

// ── IMU reader thread (uses HAL IIMUSource) ─────────────────
static void imu_reader_thread(drone::hal::IIMUSource& imu,
                              std::atomic<bool>& running,
                              int imu_rate_hz) {
    spdlog::info("[IMUReader] Thread started using {} at {} Hz",
                 imu.name(), imu_rate_hz);
    const int sleep_us = imu_rate_hz > 0 ? 1000000 / imu_rate_hz : 2500;

    uint64_t count = 0;
    while (running.load(std::memory_order_relaxed)) {
        auto sample = imu.read();
        (void)sample;  // In real system: feed to VIO pre-integrator
        ++count;
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
    spdlog::info("[IMUReader] Thread stopped after {} samples", count);
}

// ── Pose publisher thread ───────────────────────────────────
static void pose_publisher_thread(
    drone::ipc::IPublisher<drone::ipc::ShmPose>& pose_pub,
    PoseDoubleBuffer& pose_buffer,
    std::atomic<bool>& running,
    int publish_rate_hz)
{
    spdlog::info("[PosePublisher] Thread started at {} Hz", publish_rate_hz);

    const int sleep_ms = publish_rate_hz > 0 ? 1000 / publish_rate_hz : 10;

    while (running.load(std::memory_order_relaxed)) {
        Pose p;
        if (pose_buffer.read(p)) {
            drone::ipc::ShmPose shm_pose{};
            shm_pose.timestamp_ns = static_cast<uint64_t>(p.timestamp * 1e9);
            shm_pose.translation[0] = p.position.x();
            shm_pose.translation[1] = p.position.y();
            shm_pose.translation[2] = p.position.z();
            shm_pose.quaternion[0] = p.orientation.w();
            shm_pose.quaternion[1] = p.orientation.x();
            shm_pose.quaternion[2] = p.orientation.y();
            shm_pose.quaternion[3] = p.orientation.z();
            shm_pose.velocity[0] = 0.0;
            shm_pose.velocity[1] = 0.0;
            shm_pose.velocity[2] = 0.0;
            for (int i = 0; i < 36; ++i) {
                shm_pose.covariance[i] = p.covariance.data()[i];
            }
            shm_pose.quality = p.quality;
            pose_pub.publish(shm_pose);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms)); // configurable
    }
    spdlog::info("[PosePublisher] Thread stopped");
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    auto args = parse_args(argc, argv, "slam_vio_nav");
    if (args.help) return 0;

    SignalHandler::install(g_running);
    LogConfig::init("slam_vio_nav", LogConfig::resolve_log_dir(), args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== SLAM/VIO/Nav process starting (PID {}) ===", getpid());

    // ── Create message bus ──────────────────────────────────
    const auto backend = cfg.get<std::string>("ipc_backend", "shm");
    const auto shm_pool_mb = cfg.get<std::size_t>("zenoh.shm_pool_size_mb", 0);
    auto bus = drone::ipc::create_message_bus(backend, shm_pool_mb);

    // Subscribe to stereo camera from Process 1
    auto stereo_sub = drone::ipc::bus_subscribe<drone::ipc::ShmStereoFrame>(
        bus, drone::ipc::shm_names::VIDEO_STEREO_CAM);
    // For SHM: is_connected() means the segment exists (publisher running).
    // For Zenoh: is_connected() only becomes true after first sample, so we
    // can't use it as a startup gate. Log a warning instead of exiting.
    if (backend == "shm" && !stereo_sub->is_connected()) {
        spdlog::error("Cannot connect to stereo channel — is video_capture running?");
        return 1;
    }
    if (!stereo_sub->is_connected()) {
        spdlog::warn("Stereo subscriber not yet connected "
                     "(normal for Zenoh — data will arrive when publisher starts)");
    }

    // Create pose output publisher
    auto pose_pub = drone::ipc::bus_advertise<drone::ipc::ShmPose>(
        bus, drone::ipc::shm_names::SLAM_POSE);
    if (!pose_pub->is_ready()) {
        spdlog::error("Failed to create pose publisher");
        return 1;
    }

    // Thread-safe double-buffer for pose exchange (replaces raw pointer pattern)
    PoseDoubleBuffer pose_buffer;

    const int imu_rate = cfg.get<int>("slam.imu_rate_hz", 400);
    const int vio_rate  = cfg.get<int>("slam.vio_rate_hz", 100);

    // Create IMU via HAL factory
    auto imu = drone::hal::create_imu_source(cfg, "slam.imu");
    imu->init(imu_rate);

    // Create visual frontend via strategy factory
    auto frontend_backend = cfg.get<std::string>(
        "slam.visual_frontend.backend", "simulated");
    auto frontend_gz_topic = cfg.get<std::string>(
        "slam.visual_frontend.gz_topic", "/model/x500_companion_0/odometry");
    auto frontend = drone::slam::create_visual_frontend(
        frontend_backend, frontend_gz_topic);
    spdlog::info("Visual frontend: {}", frontend->name());

    // Launch threads
    std::thread t_frontend(visual_frontend_thread,
        std::ref(*stereo_sub), std::ref(*frontend),
        std::ref(pose_buffer), std::ref(g_running));
    std::thread t_imu(imu_reader_thread, std::ref(*imu), std::ref(g_running), imu_rate);
    std::thread t_publisher(pose_publisher_thread,
        std::ref(*pose_pub), std::ref(pose_buffer), std::ref(g_running),
        vio_rate);

    spdlog::info("All SLAM threads started — READY");

    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        Pose p;
        if (pose_buffer.read(p)) {
            spdlog::info("[HealthCheck] SLAM pose: ({:.2f}, {:.2f}, {:.2f}) q={}",
                         p.position.x(), p.position.y(), p.position.z(),
                         p.quality);
        }
    }

    spdlog::info("Shutting down...");
    t_frontend.join();
    t_imu.join();
    t_publisher.join();

    spdlog::info("=== SLAM/VIO/Nav process stopped ===");
    return 0;
}
