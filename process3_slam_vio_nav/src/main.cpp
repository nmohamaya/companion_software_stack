// process3_slam_vio_nav/src/main.cpp
// Process 3 — SLAM/VIO/Nav: visual-inertial odometry + pose estimation.
// Simulated: generates fake VIO pose estimates from stereo camera SHM.

#include "slam/types.h"
#include "ipc/shm_reader.h"
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

// ── Simulated Visual Frontend thread ────────────────────────
// In real system: ORB/KLT feature detection + tracking on stereo frames
static void visual_frontend_thread(
    ShmReader<drone::ipc::ShmStereoFrame>& stereo_reader,
    PoseDoubleBuffer& pose_buffer,
    std::atomic<bool>& stop_flag)
{
    spdlog::info("[VisualFrontend] Thread started (simulation)");

    double t = 0.0;
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.01);
    uint64_t frame_count = 0;
    Pose p;  // stack-allocated, reused each iteration

    while (!stop_flag.load(std::memory_order_relaxed)) {
        drone::ipc::ShmStereoFrame frame;
        (void)stereo_reader.read(frame);

        // Simulate circular trajectory with noise
        t += 0.033;  // 30 Hz
        p.timestamp = t;
        p.position = Eigen::Vector3d(
            5.0 * std::cos(t * 0.5) + noise(rng),
            5.0 * std::sin(t * 0.5) + noise(rng),
            2.0 + 0.1 * std::sin(t) + noise(rng)   // ~2m altitude
        );
        double yaw = t * 0.5 + M_PI / 2.0;
        p.orientation = Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        p.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        p.quality = 2;  // good

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

// ── Simulated IMU reader thread ─────────────────────────────
static void imu_reader_thread(std::atomic<bool>& stop_flag, int imu_rate_hz) {
    spdlog::info("[IMUReader] Thread started (simulation) at {} Hz", imu_rate_hz);
    std::mt19937 rng(99);
    std::normal_distribution<double> accel_noise(0.0, 0.05);
    std::normal_distribution<double> gyro_noise(0.0, 0.002);
    const int sleep_us = imu_rate_hz > 0 ? 1000000 / imu_rate_hz : 2500;

    uint64_t count = 0;
    while (!stop_flag.load(std::memory_order_relaxed)) {
        ImuSample sample;
        sample.timestamp = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        sample.accel = Eigen::Vector3d(accel_noise(rng),
                                        accel_noise(rng),
                                        9.81 + accel_noise(rng));
        sample.gyro = Eigen::Vector3d(gyro_noise(rng),
                                       gyro_noise(rng),
                                       0.5 + gyro_noise(rng));
        ++count;
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
    spdlog::info("[IMUReader] Thread stopped after {} samples", count);
}

// ── Pose publisher thread ───────────────────────────────────
static void pose_publisher_thread(
    ShmWriter<drone::ipc::ShmPose>& pose_writer,
    PoseDoubleBuffer& pose_buffer,
    std::atomic<bool>& stop_flag,
    int publish_rate_hz)
{
    spdlog::info("[PosePublisher] Thread started at {} Hz", publish_rate_hz);

    const int sleep_ms = publish_rate_hz > 0 ? 1000 / publish_rate_hz : 10;

    while (!stop_flag.load(std::memory_order_relaxed)) {
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
            pose_writer.write(shm_pose);
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
    LogConfig::init("slam_vio_nav", "/tmp/drone_logs", args.log_level);

    drone::Config cfg;
    cfg.load(args.config_path);

    spdlog::info("=== SLAM/VIO/Nav process starting (PID {}) ===", getpid());

    // Open stereo camera SHM from Process 1
    ShmReader<drone::ipc::ShmStereoFrame> stereo_reader;
    for (int attempt = 0; attempt < 50; ++attempt) {
        if (stereo_reader.open(drone::ipc::shm_names::VIDEO_STEREO_CAM)) {
            spdlog::info("Connected to SHM: {}",
                         drone::ipc::shm_names::VIDEO_STEREO_CAM);
            break;
        }
        spdlog::warn("Waiting for stereo SHM (attempt {}/50)...", attempt + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!stereo_reader.is_open()) {
        spdlog::error("Cannot connect to stereo SHM — is video_capture running?");
        return 1;
    }

    // Create pose output SHM
    ShmWriter<drone::ipc::ShmPose> pose_writer;
    if (!pose_writer.create(drone::ipc::shm_names::SLAM_POSE)) {
        spdlog::error("Failed to create SHM: {}",
                       drone::ipc::shm_names::SLAM_POSE);
        return 1;
    }

    // Thread-safe double-buffer for pose exchange (replaces raw pointer pattern)
    PoseDoubleBuffer pose_buffer;

    const int imu_rate = cfg.get<int>("slam.imu_rate_hz", 400);
    const int vio_rate  = cfg.get<int>("slam.vio_rate_hz", 100);

    // Launch threads
    std::thread t_frontend(visual_frontend_thread,
        std::ref(stereo_reader), std::ref(pose_buffer), std::ref(g_running));
    std::thread t_imu(imu_reader_thread, std::ref(g_running), imu_rate);
    std::thread t_publisher(pose_publisher_thread,
        std::ref(pose_writer), std::ref(pose_buffer), std::ref(g_running),
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
