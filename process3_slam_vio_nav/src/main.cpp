// process3_slam_vio_nav/src/main.cpp
// Process 3 — SLAM/VIO/Nav: visual-inertial odometry + pose estimation.
//
// Architecture (Phase 2A):
//   IMU reader thread  →  ImuRingBuffer  →  VIO pipeline thread
//   Stereo subscriber  →─────────────────→  VIO pipeline thread
//   VIO pipeline thread →  PoseDoubleBuffer  →  Pose publisher thread
//
// The VIO pipeline (IVIOBackend) processes stereo frames fused with
// buffered IMU data.  Every component returns VIOResult<T> with
// structured diagnostics so that simulation failures are immediately
// actionable.
//
// Uses MessageBusFactory — IPC backend selected at runtime via config.

#include "hal/hal_factory.h"
#include "ipc/ipc_types.h"
#include "slam/ivio_backend.h"
#include "slam/types.h"
#include "slam/vio_types.h"
#include "util/config_keys.h"
#include "util/diagnostic.h"
#include "util/process_context.h"
#include "util/rate_clamp.h"
#include "util/scoped_timer.h"
#include "util/sd_notify.h"
#include "util/thread_health_publisher.h"
#include "util/thread_heartbeat.h"
#include "util/thread_watchdog.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

using namespace drone::slam;

// ── Double-buffer for thread-safe pose exchange ─────────────
// Eliminates the use-after-free race in the old raw-pointer atomic pattern.
class PoseDoubleBuffer {
public:
    void write(const Pose& pose) {
        int idx       = write_idx_.load(std::memory_order_relaxed) ^ 1;
        buffers_[idx] = pose;
        write_idx_.store(idx, std::memory_order_release);
    }

    bool read(Pose& out) const {
        int idx = write_idx_.load(std::memory_order_acquire);
        if (!initialized_.load(std::memory_order_acquire)) return false;
        out = buffers_[idx];
        return true;
    }

    void mark_initialized() { initialized_.store(true, std::memory_order_release); }

private:
    Pose              buffers_[2];
    std::atomic<int>  write_idx_{0};
    std::atomic<bool> initialized_{false};
};

// ── Thread-safe IMU ring buffer ─────────────────────────────
// The IMU reader thread pushes samples; the VIO thread drains them.
// Bounded capacity prevents unbounded growth if VIO stalls.
class ImuRingBuffer {
public:
    explicit ImuRingBuffer(size_t capacity = 2048) : capacity_(capacity) {}

    /// Push a sample (IMU reader thread).
    void push(const ImuSample& s) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buffer_.size() >= capacity_) {
            ++drop_count_;
            buffer_.pop_front();  // O(1) with std::deque
        }
        buffer_.push_back(s);
        ++total_pushed_;
    }

    /// Drain all buffered samples into `out` (VIO thread).
    /// Returns the number of samples drained.
    size_t drain(std::vector<ImuSample>& out) {
        std::lock_guard<std::mutex> lk(mtx_);
        size_t                      n = buffer_.size();
        out.insert(out.end(), buffer_.begin(), buffer_.end());
        buffer_.clear();
        return n;
    }

    /// Total samples pushed since construction.
    uint64_t total_pushed() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return total_pushed_;
    }

    /// Total samples dropped due to buffer overflow.
    uint64_t drop_count() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return drop_count_;
    }

private:
    mutable std::mutex    mtx_;
    std::deque<ImuSample> buffer_;
    size_t                capacity_;
    uint64_t              total_pushed_ = 0;
    uint64_t              drop_count_   = 0;
};

static std::atomic<bool> g_running{true};

// ── VIO pipeline thread ─────────────────────────────────────
// Replaces the old visual_frontend_thread.  Now runs the full
// VIO backend: feature extraction → stereo matching → IMU
// pre-integration → pose output.
static void vio_pipeline_thread(drone::ipc::ISubscriber<drone::ipc::StereoFrame>& stereo_sub,
                                drone::slam::IVIOBackend& backend, ImuRingBuffer& imu_buffer,
                                PoseDoubleBuffer& pose_buffer, std::atomic<bool>& running) {
    DRONE_LOG_INFO("[VIOPipeline] Thread started using {}", backend.name());

    auto hb = drone::util::ScopedHeartbeat("vio_pipeline", true);

    uint64_t frame_count     = 0;
    uint64_t error_count     = 0;
    uint64_t no_frame_count  = 0;
    uint64_t last_drop_count = 0;
    uint64_t vio_loop_tick   = 0;

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        drone::util::FrameDiagnostics diag(vio_loop_tick);
        ++vio_loop_tick;

        // ── Receive stereo frame ────────────────────────────
        drone::ipc::StereoFrame frame;
        bool                    got_stereo = stereo_sub.receive(frame);
        if (!got_stereo) {
            ++no_frame_count;
            if (no_frame_count == 1 || no_frame_count % 300 == 0) {
                DRONE_LOG_WARN("[VIOPipeline] No stereo frame received "
                               "(#{} consecutive)",
                               no_frame_count);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }
        // Reset no-frame counter on successful receive
        if (no_frame_count > 0) {
            DRONE_LOG_INFO("[VIOPipeline] Stereo frames resumed after {} misses", no_frame_count);
            no_frame_count = 0;
        }

        // ── Drain IMU buffer ────────────────────────────────
        std::vector<ImuSample> imu_samples;
        imu_samples.reserve(64);
        imu_buffer.drain(imu_samples);
        diag.add_metric("IMU", "n_samples", static_cast<double>(imu_samples.size()));

        // Check for IMU buffer overflows (samples were dropped)
        uint64_t drops = imu_buffer.drop_count();
        if (drops > last_drop_count) {
            diag.add_warning("IMU", "Buffer overflow: " + std::to_string(drops - last_drop_count) +
                                        " samples dropped (total " + std::to_string(drops) + ")");
            last_drop_count = drops;
        }

        // ── Run VIO backend ─────────────────────────────────
        auto result = [&]() {
            drone::util::ScopedDiagTimer timer(diag, "VIOProcess");
            return backend.process_frame(frame, imu_samples);
        }();

        if (result.is_ok()) {
            auto& output = result.value();
            pose_buffer.write(output.pose);
            pose_buffer.mark_initialized();

            diag.add_metric("VIO", "features", static_cast<double>(output.num_features));
            diag.add_metric("VIO", "stereo_matches",
                            static_cast<double>(output.num_stereo_matches));
            diag.add_metric("VIO", "imu_used", static_cast<double>(output.imu_samples_used));

            ++frame_count;
            if (diag.has_warnings()) {
                diag.log_summary("VIOPipeline");
            } else if (frame_count % 300 == 0) {
                DRONE_LOG_INFO("[VIOPipeline] Frame {}: pos=({:.2f}, {:.2f}, {:.2f}) "
                               "health={} feat={} matches={} imu={}",
                               frame_count, output.pose.position.x(), output.pose.position.y(),
                               output.pose.position.z(), vio_health_name(output.health),
                               output.num_features, output.num_stereo_matches,
                               output.imu_samples_used);
            }
        } else {
            ++error_count;
            auto& err = result.error();
            diag.add_error("VIO", err.to_string());
            diag.log_summary("VIOPipeline");

            // Rate-limit repeated error logging
            if (error_count % 100 == 1) {
                DRONE_LOG_ERROR("[VIOPipeline] Total errors so far: {} "
                                "(last: {})",
                                error_count, err.to_string());
            }
        }

        // Log IPC latency periodically
        stereo_sub.log_latency_if_due();

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    DRONE_LOG_INFO("[VIOPipeline] Thread stopped — {} frames processed, {} errors", frame_count,
                   error_count);
}

// ── IMU reader thread (uses HAL IIMUSource) ─────────────────
// Now pushes samples into the shared ImuRingBuffer instead of discarding.
static void imu_reader_thread(drone::hal::IIMUSource& imu, ImuRingBuffer& imu_buffer,
                              std::atomic<bool>& running, int imu_rate_hz) {
    DRONE_LOG_INFO("[IMUReader] Thread started using {} at {} Hz", imu.name(), imu_rate_hz);
    const int sleep_us = imu_rate_hz > 0 ? 1000000 / imu_rate_hz : 2500;

    auto hb = drone::util::ScopedHeartbeat("imu_reader", true);

    uint64_t count         = 0;
    uint64_t invalid_count = 0;

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        auto reading = imu.read();

        if (reading.valid) {
            ImuSample sample;
            sample.timestamp = reading.timestamp;
            sample.accel     = reading.accel;
            sample.gyro      = reading.gyro;
            imu_buffer.push(sample);
            ++count;
        } else {
            ++invalid_count;
            // Rate-limited warning for invalid readings
            if (invalid_count == 1 || invalid_count % 1000 == 0) {
                DRONE_LOG_WARN("[IMUReader] Invalid IMU reading #{} — "
                               "sensor may not be ready",
                               invalid_count);
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
    DRONE_LOG_INFO("[IMUReader] Thread stopped — {} valid samples, {} invalid", count,
                   invalid_count);
}

// ── Pose publisher thread ───────────────────────────────────
static void pose_publisher_thread(drone::ipc::IPublisher<drone::ipc::Pose>& pose_pub,
                                  PoseDoubleBuffer& pose_buffer, std::atomic<bool>& running,
                                  int publish_rate_hz,
                                  drone::ipc::ISubscriber<drone::ipc::FaultOverrides>& fault_sub) {
    DRONE_LOG_INFO("[PosePublisher] Thread started at {} Hz", publish_rate_hz);

    auto hb = drone::util::ScopedHeartbeat("pose_publisher", true);

    const int sleep_ms = publish_rate_hz > 0 ? 1000 / publish_rate_hz : 10;

    while (running.load(std::memory_order_relaxed)) {
        drone::util::ThreadHeartbeatRegistry::instance().touch(hb.handle());
        Pose p;
        if (pose_buffer.read(p)) {
            drone::ipc::Pose shm_pose{};
            shm_pose.timestamp_ns   = static_cast<uint64_t>(p.timestamp * 1e9);
            shm_pose.translation[0] = p.position.x();
            shm_pose.translation[1] = p.position.y();
            shm_pose.translation[2] = p.position.z();
            shm_pose.quaternion[0]  = p.orientation.w();
            shm_pose.quaternion[1]  = p.orientation.x();
            shm_pose.quaternion[2]  = p.orientation.y();
            shm_pose.quaternion[3]  = p.orientation.z();
            shm_pose.velocity[0]    = 0.0;
            shm_pose.velocity[1]    = 0.0;
            shm_pose.velocity[2]    = 0.0;
            for (int i = 0; i < 36; ++i) {
                shm_pose.covariance[i] = p.covariance.data()[i];
            }
            shm_pose.quality = p.quality;

            // Apply VIO quality override from fault injector (if active)
            drone::ipc::FaultOverrides ovr{};
            if (fault_sub.receive(ovr) && ovr.vio_quality >= 0) {
                shm_pose.quality =
                    static_cast<uint32_t>(std::clamp(ovr.vio_quality, int32_t{0}, int32_t{3}));
            }

            pose_pub.publish(shm_pose);
        }

        // Log IPC latency from the thread that owns fault_sub.receive()
        fault_sub.log_latency_if_due();

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));  // configurable
    }
    DRONE_LOG_INFO("[PosePublisher] Thread stopped");
}

// ═══════════════════════════════════════════════════════════
// main()
// ═══════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    // ── Common boilerplate (args, signals, logging, config, bus) ──
    auto ctx_result = drone::util::init_process(argc, argv, "slam_vio_nav", g_running,
                                                drone::util::slam_schema());
    if (!ctx_result.is_ok()) return ctx_result.error();
    auto& ctx = ctx_result.value();

    // Subscribe to stereo camera from Process 1
    auto stereo_sub =
        ctx.bus.subscribe<drone::ipc::StereoFrame>(drone::ipc::topics::VIDEO_STEREO_CAM);
    // With Zenoh, is_connected() only becomes true after the first sample
    // arrives; we cannot use it as a startup gate.  Log a warning and continue —
    // data will arrive once video_capture starts publishing.
    const auto ipc_backend = ctx.cfg.get<std::string>(drone::cfg_key::IPC_BACKEND, "zenoh");
    if (!stereo_sub->is_connected()) {
        if (ipc_backend == "shm") {
            // Should never happen — shm backend was removed, factory falls back
            // to Zenoh.  Treat as a warning, not a fatal error.
            DRONE_LOG_WARN("ipc_backend=shm is no longer supported; using Zenoh.");
        }
        DRONE_LOG_WARN("Stereo subscriber not yet connected "
                       "(normal for Zenoh — data will arrive when publisher starts)");
    }

    // Create pose output publisher
    auto pose_pub = ctx.bus.advertise<drone::ipc::Pose>(drone::ipc::topics::SLAM_POSE);
    if (!pose_pub->is_ready()) {
        DRONE_LOG_ERROR("Failed to create pose publisher");
        return 1;
    }

    // Thread-safe double-buffer for pose exchange
    PoseDoubleBuffer pose_buffer;

    // Thread-safe IMU ring buffer (IMU reader → VIO pipeline)
    ImuRingBuffer imu_ring_buffer(2048);

    // ── Rate clamping (safety: prevent runaway loops or undersampling) ──
    const int imu_rate =
        drone::util::clamp_imu_rate(ctx.cfg.get<int>(drone::cfg_key::slam::IMU_RATE_HZ, 400));
    const int vio_rate =
        drone::util::clamp_vio_rate(ctx.cfg.get<int>(drone::cfg_key::slam::VIO_RATE_HZ, 100));

    // Create IMU via HAL factory
    auto imu = drone::hal::create_imu_source(ctx.cfg, drone::cfg_key::slam::imu::SECTION);
    if (!imu->init(imu_rate)) {
        DRONE_LOG_ERROR("Failed to initialise IMU source — check config");
        return 1;
    }
    DRONE_LOG_INFO("IMU source: {} at {} Hz", imu->name(), imu_rate);

    // Create VIO backend via factory
    auto vio_backend_name = ctx.cfg.get<std::string>(drone::cfg_key::slam::vio::BACKEND,
                                                     "simulated");
    auto vio_gz_topic     = ctx.cfg.get<std::string>(drone::cfg_key::slam::vio::GZ_TOPIC,
                                                     "/model/x500_companion_0/odometry");
    StereoCalibration calib;
    calib.fx       = ctx.cfg.get<double>(drone::cfg_key::slam::stereo::FX, 350.0);
    calib.fy       = ctx.cfg.get<double>(drone::cfg_key::slam::stereo::FY, 350.0);
    calib.cx       = ctx.cfg.get<double>(drone::cfg_key::slam::stereo::CX, 320.0);
    calib.cy       = ctx.cfg.get<double>(drone::cfg_key::slam::stereo::CY, 240.0);
    calib.baseline = ctx.cfg.get<double>(drone::cfg_key::slam::stereo::BASELINE, 0.12);

    ImuNoiseParams imu_params;
    imu_params.gyro_noise_density =
        ctx.cfg.get<double>(drone::cfg_key::slam::imu::GYRO_NOISE_DENSITY, 0.004);
    imu_params.gyro_random_walk = ctx.cfg.get<double>(drone::cfg_key::slam::imu::GYRO_RANDOM_WALK,
                                                      2.2e-5);
    imu_params.accel_noise_density =
        ctx.cfg.get<double>(drone::cfg_key::slam::imu::ACCEL_NOISE_DENSITY, 0.012);
    imu_params.accel_random_walk = ctx.cfg.get<double>(drone::cfg_key::slam::imu::ACCEL_RANDOM_WALK,
                                                       8.0e-5);

    const float  sim_speed_mps = ctx.cfg.get<float>(drone::cfg_key::slam::vio::SIM_SPEED_MPS, 3.0f);
    const double good_trace_max = ctx.cfg.get<double>(drone::cfg_key::slam::vio::GOOD_TRACE_MAX,
                                                      0.1);
    const double degraded_trace_max =
        ctx.cfg.get<double>(drone::cfg_key::slam::vio::DEGRADED_TRACE_MAX, 1.0);
    // For "cosys_airsim" VIO backend, reuse the shared CosysRpcClient from the HAL factory
    // (creating a second client would open a second RPC connection — wasteful).
    std::shared_ptr<drone::hal::CosysRpcClient> cosys_client;
    std::string                                 cosys_vehicle = "Drone0";
#ifdef HAVE_COSYS_AIRSIM
    if (vio_backend_name == "cosys_airsim") {
        cosys_client = drone::hal::detail::get_shared_cosys_client(ctx.cfg);
        cosys_vehicle = ctx.cfg.get<std::string>(
            std::string(drone::cfg_key::cosys_airsim::VEHICLE_NAME), "Drone0");
    }
#endif
    auto vio = drone::slam::create_vio_backend(vio_backend_name, calib, imu_params, vio_gz_topic,
                                               sim_speed_mps, good_trace_max, degraded_trace_max,
                                               cosys_client, cosys_vehicle);
    DRONE_LOG_INFO("VIO backend: {} (sim_speed={:.1f} m/s, quality: good<={:.2f} degraded<={:.2f})",
                   vio->name(), sim_speed_mps, good_trace_max, degraded_trace_max);
    DRONE_LOG_INFO("Stereo calib: fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} baseline={:.3f}m",
                   calib.fx, calib.fy, calib.cx, calib.cy, calib.baseline);

    // Subscribe to trajectory commands for simulated VIO navigation.
    // Only the simulated backend uses trajectory targets — Gazebo/real backends
    // get pose from actual sensors and ignore set_trajectory_target().
    std::unique_ptr<drone::ipc::ISubscriber<drone::ipc::TrajectoryCmd>> traj_sub;
    if (vio_backend_name == "simulated") {
        traj_sub = ctx.bus.subscribe<drone::ipc::TrajectoryCmd>(drone::ipc::topics::TRAJECTORY_CMD);
        DRONE_LOG_INFO("Subscribed to {} for simulated VIO target tracking",
                       drone::ipc::topics::TRAJECTORY_CMD);
    }

    // Subscribe to fault overrides for VIO quality injection
    auto fault_sub =
        ctx.bus.subscribe_optional<drone::ipc::FaultOverrides>(drone::ipc::topics::FAULT_OVERRIDES);

    // Launch threads
    std::thread t_vio(vio_pipeline_thread, std::ref(*stereo_sub), std::ref(*vio),
                      std::ref(imu_ring_buffer), std::ref(pose_buffer), std::ref(g_running));
    std::thread t_imu(imu_reader_thread, std::ref(*imu), std::ref(imu_ring_buffer),
                      std::ref(g_running), imu_rate);
    std::thread t_publisher(pose_publisher_thread, std::ref(*pose_pub), std::ref(pose_buffer),
                            std::ref(g_running), vio_rate, std::ref(*fault_sub));

    // ── Thread watchdog + health publisher ──────────────────
    drone::util::ThreadWatchdog watchdog;
    auto                        thread_health_pub =
        ctx.bus.advertise<drone::ipc::ThreadHealth>(drone::ipc::topics::THREAD_HEALTH_SLAM_VIO_NAV);
    drone::util::ThreadHealthPublisher health_publisher(*thread_health_pub, "slam_vio_nav",
                                                        watchdog);

    DRONE_LOG_INFO("All SLAM/VIO threads started — READY");
    drone::systemd::notify_ready();

    while (g_running.load(std::memory_order_acquire)) {
        drone::systemd::notify_watchdog();

        // ── Forward trajectory targets to VIO backend ────────
        // Poll at ~100 Hz so the simulated VIO tracks waypoint targets promptly.
        // Only active for simulated backend — Gazebo/real backends ignore targets.
        if (traj_sub) {
            for (int i = 0; i < 100 && g_running.load(std::memory_order_acquire); ++i) {
                drone::ipc::TrajectoryCmd traj_cmd{};
                if (traj_sub->receive(traj_cmd) && traj_cmd.valid) {
                    vio->set_trajectory_target(traj_cmd.target_x, traj_cmd.target_y,
                                               traj_cmd.target_z, traj_cmd.target_yaw);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } else {
            // Non-simulated backend — just sleep for the main loop period
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        health_publisher.publish_snapshot();

        // ── Periodic health report ──────────────────────────
        Pose p;
        if (pose_buffer.read(p)) {
            DRONE_LOG_INFO("[HealthCheck] VIO pose: ({:.2f}, {:.2f}, {:.2f}) q={} health={}",
                           p.position.x(), p.position.y(), p.position.z(), p.quality,
                           vio_health_name(vio->health()));
        }

        // Log IPC latency (traj_sub is received in this thread; fault_sub is
        // logged from pose_publisher_thread which owns its receive() calls)
        if (traj_sub) traj_sub->log_latency_if_due();

        // Report IMU buffer stats
        uint64_t drops = imu_ring_buffer.drop_count();
        if (drops > 0) {
            DRONE_LOG_WARN("[HealthCheck] IMU buffer drops: {} total", drops);
        }
    }

    drone::systemd::notify_stopping();
    DRONE_LOG_INFO("Shutting down...");
    t_vio.join();
    t_imu.join();
    t_publisher.join();

    DRONE_LOG_INFO("=== SLAM/VIO/Nav process stopped ===");
    return 0;
}
