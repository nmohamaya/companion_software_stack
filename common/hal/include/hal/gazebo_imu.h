// common/hal/include/hal/gazebo_imu.h
// HAL backend: IMU data from Gazebo transport (gz::msgs::IMU).
//
// Subscribes to a gz-transport IMU topic and caches the latest reading.
// Thread-safe: uses a std::mutex to guard access to the cached ImuReading.
// The struct is small (~56 bytes), so copying it under the lock is cheap.
//
// Compile guard: only available when HAVE_GAZEBO is defined by CMake.
#pragma once

#ifdef HAVE_GAZEBO

#include "hal/iimu_source.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>
#include <spdlog/spdlog.h>

namespace drone::hal {

/// GazeboIMUBackend — receives accelerometer + gyroscope data from
/// a Gazebo simulation via gz-transport topic subscription.
///
/// Usage:
///   GazeboIMUBackend imu("/imu");
///   imu.init(200);           // subscribe at 200 Hz (informational)
///   auto sample = imu.read(); // returns latest cached ImuReading
class GazeboIMUBackend : public IIMUSource {
public:
    /// @param gz_topic  Gazebo transport topic (e.g. "/imu")
    explicit GazeboIMUBackend(std::string gz_topic) : gz_topic_(std::move(gz_topic)) {}

    ~GazeboIMUBackend() override { shutdown(); }

    /// Explicitly shut down: prevents callbacks from racing the destructor.
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(reading_mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            active_.store(false, std::memory_order_release);
        }
        node_.Unsubscribe(gz_topic_);
        spdlog::info("[GazeboIMU] Shut down — unsubscribed from '{}'", gz_topic_);
    }

    // Non-copyable, non-movable (owns gz::transport::Node)
    GazeboIMUBackend(const GazeboIMUBackend&)            = delete;
    GazeboIMUBackend& operator=(const GazeboIMUBackend&) = delete;
    GazeboIMUBackend(GazeboIMUBackend&&)                 = delete;
    GazeboIMUBackend& operator=(GazeboIMUBackend&&)      = delete;

    /// Initialise — subscribe to the Gazebo IMU topic.
    /// @param rate_hz  Informational; actual rate is driven by Gazebo sensor.
    /// @return true on successful subscription
    bool init(int rate_hz) override {
        if (active_.load(std::memory_order_acquire)) {
            spdlog::warn("[GazeboIMU] Already initialised on '{}'", gz_topic_);
            return false;
        }

        rate_hz_ = rate_hz;

        bool subscribed = node_.Subscribe(gz_topic_, &GazeboIMUBackend::on_imu_msg, this);

        if (!subscribed) {
            spdlog::error("[GazeboIMU] Failed to subscribe to '{}'", gz_topic_);
            return false;
        }

        active_.store(true, std::memory_order_release);
        spdlog::info("[GazeboIMU] Subscribed to '{}' (rate_hz={} informational)", gz_topic_,
                     rate_hz_);
        return true;
    }

    /// Read the latest cached IMU sample (non-blocking).
    /// Returns an invalid reading if no data has arrived yet.
    ImuReading read() override {
        std::lock_guard<std::mutex> lock(reading_mutex_);
        return cached_reading_;
    }

    /// Returns true after init() succeeds and at least one message arrives.
    bool is_active() const override {
        return active_.load(std::memory_order_acquire) &&
               msg_count_.load(std::memory_order_acquire) > 0;
    }

    /// Human-readable name including the topic.
    std::string name() const override { return "GazeboIMU(" + gz_topic_ + ")"; }

    /// Number of messages received (useful for diagnostics).
    uint64_t message_count() const { return msg_count_.load(std::memory_order_acquire); }

private:
    // ── gz-transport callback (called on transport thread) ──────────
    void on_imu_msg(const gz::msgs::IMU& msg) {
        // Early-out if shutdown is in progress (prevents use-after-free)
        if (!active_.load(std::memory_order_acquire)) return;

        ImuReading r;

        // Extract linear acceleration (m/s²)
        bool has_accel = msg.has_linear_acceleration();
        if (has_accel) {
            const auto& la = msg.linear_acceleration();
            r.accel        = Eigen::Vector3d(la.x(), la.y(), la.z());
        }

        // Extract angular velocity (rad/s)
        bool has_gyro = msg.has_angular_velocity();
        if (has_gyro) {
            const auto& av = msg.angular_velocity();
            r.gyro         = Eigen::Vector3d(av.x(), av.y(), av.z());
        }

        // Mark valid only when both required fields are present
        r.valid = has_accel && has_gyro;
        if (!r.valid) return;  // discard incomplete messages

        // Timestamp: use steady_clock for monotonic time
        r.timestamp =
            std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
                .count();

        // Store under lock (fast — ImuReading is ~56 bytes)
        {
            std::lock_guard<std::mutex> lock(reading_mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            cached_reading_ = r;
        }

        uint64_t count = msg_count_.fetch_add(1, std::memory_order_acq_rel) + 1;
        if (count == 1) {
            spdlog::info("[GazeboIMU] First IMU message from '{}': "
                         "accel=({:.3f}, {:.3f}, {:.3f}) "
                         "gyro=({:.4f}, {:.4f}, {:.4f})",
                         gz_topic_, r.accel.x(), r.accel.y(), r.accel.z(), r.gyro.x(), r.gyro.y(),
                         r.gyro.z());
        }
    }

    // ── Members ────────────────────────────────────────────────────
    std::string           gz_topic_;        ///< Gazebo transport topic
    int                   rate_hz_{200};    ///< Informational sample rate
    gz::transport::Node   node_;            ///< Transport node (owns sub)
    std::atomic<bool>     active_{false};   ///< True after init() succeeds
    std::atomic<uint64_t> msg_count_{0};    ///< Messages received
    mutable std::mutex    reading_mutex_;   ///< Guards cached_reading_
    ImuReading            cached_reading_;  ///< Latest reading
};

}  // namespace drone::hal

#endif  // HAVE_GAZEBO
