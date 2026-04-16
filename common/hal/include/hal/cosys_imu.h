// common/hal/include/hal/cosys_imu.h
// Cosys-AirSim IMU backend — receives IMU data via AirSim RPC API.
// Implements IIMUSource — connects to Cosys-AirSim getImuData() endpoint.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Polling thread runs at the requested rate (default 200 Hz) and converts
// AirSim ImuBase::Output to our ImuReading format.
//
// AirSim IMU provides angular_velocity (rad/s) and linear_acceleration (m/s^2)
// in NED frame, which maps directly to our FRD convention (no negation needed).
//
// Thread-safe: uses a std::mutex to guard access to the cached ImuReading.
// The struct is small (~56 bytes), so copying it under the lock is cheap.
//
// Compile guard: only available when HAVE_COSYS_AIRSIM is defined by CMake.
//
// Issue: #434, #462
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"
#include "hal/iimu_source.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// CosysIMUBackend — receives accelerometer + gyroscope data from
/// a Cosys-AirSim simulation via the getImuData() RPC endpoint.
///
/// The polling thread runs at rate_hz (default 200 Hz). Each poll
/// retrieves the latest ImuBase::Output from AirSim and caches it
/// for non-blocking read() calls.
///
/// Usage:
///   drone::Config cfg;
///   cfg.load("config/cosys_airsim.json");
///   CosysIMUBackend imu(client, cfg, "slam.imu");
///   imu.init(200);
///   auto sample = imu.read();
class CosysIMUBackend : public IIMUSource {
public:
    /// @param client   Shared RPC client (manages connection lifecycle)
    /// @param cfg      Loaded configuration
    /// @param section  Config path prefix (e.g. "slam.imu")
    explicit CosysIMUBackend(std::shared_ptr<CosysRpcClient> client, const drone::Config& cfg,
                             const std::string& section)
        : client_(std::move(client))
        , imu_name_(
              cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::IMU_NAME), "imu"))
        , vehicle_name_(cfg.get<std::string>(
              std::string(drone::cfg_key::cosys_airsim::VEHICLE_NAME), "Drone0")) {
        (void)section;  // section reserved for future per-IMU config
        DRONE_LOG_INFO("[CosysIMU] Created for {} imu='{}' vehicle='{}'", client_->endpoint(),
                       imu_name_, vehicle_name_);
    }

    ~CosysIMUBackend() override { shutdown(); }

    /// Explicitly shut down: stops polling thread and prevents races.
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(reading_mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            active_.store(false, std::memory_order_release);
        }

        // Join polling thread outside the lock to avoid deadlock
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }

        DRONE_LOG_INFO("[CosysIMU] Shut down imu='{}'", imu_name_);
    }

    // Non-copyable, non-movable (owns polling thread)
    CosysIMUBackend(const CosysIMUBackend&)            = delete;
    CosysIMUBackend& operator=(const CosysIMUBackend&) = delete;
    CosysIMUBackend(CosysIMUBackend&&)                 = delete;
    CosysIMUBackend& operator=(CosysIMUBackend&&)      = delete;

    /// Initialise — verify RPC connection and start polling thread.
    /// @param rate_hz  Polling rate; actual data rate depends on AirSim sensor config.
    /// @return true on successful startup
    bool init(int rate_hz) override {
        std::lock_guard<std::mutex> lock(reading_mutex_);
        if (active_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysIMU] Already initialised on imu='{}'", imu_name_);
            return false;
        }

        if (!client_->is_connected()) {
            DRONE_LOG_ERROR("[CosysIMU] RPC client not connected — cannot init IMU");
            return false;
        }

        // Clamp rate_hz to [1, 1000] — non-positive creates hot-loop
        rate_hz_ = std::clamp(rate_hz, 1, 1000);
        if (rate_hz != rate_hz_) {
            DRONE_LOG_WARN("[CosysIMU] rate_hz clamped from {} to {}", rate_hz, rate_hz_);
        }
        active_.store(true, std::memory_order_release);

        // Start polling thread at the requested rate
        poll_thread_ = std::thread(&CosysIMUBackend::poll_loop, this);

        DRONE_LOG_INFO("[CosysIMU] Initialised imu='{}' on {} (rate_hz={})", imu_name_,
                       client_->endpoint(), rate_hz_);
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

    /// Human-readable name including the sensor and host.
    std::string name() const override {
        return "CosysIMU(" + imu_name_ + "@" + client_->endpoint() + ")";
    }

    /// Number of messages received (useful for diagnostics).
    uint64_t message_count() const { return msg_count_.load(std::memory_order_acquire); }

private:
    /// IMU polling loop — runs at rate_hz_, retrieves getImuData() from AirSim
    /// and converts to ImuReading format.
    void poll_loop() {
        // Clamp to minimum 1ms to prevent hot-loop on misconfigured rate > 1000
        const auto poll_interval =
            std::chrono::milliseconds(std::max(1, rate_hz_ > 0 ? 1000 / rate_hz_ : 5));

        DRONE_LOG_INFO("[CosysIMU] Polling thread started (interval={}ms)", poll_interval.count());

        while (active_.load(std::memory_order_acquire)) {
            try {
                // Use with_client() to prevent TOCTOU race on disconnect
                msr::airlib::ImuBase::Output imu_data;
                bool                         got_data = client_->with_client(
                    [&](auto& rpc) { imu_data = rpc.getImuData(imu_name_, vehicle_name_); });
                if (!got_data) {
                    DRONE_LOG_WARN("[CosysIMU] RPC disconnected — skipping sample");
                    std::this_thread::sleep_for(poll_interval);
                    continue;
                }

                ImuReading r;

                // AirSim NED maps directly to our FRD convention
                // (X=forward, Y=right, Z=down — same axes, no negation)
                r.accel = Eigen::Vector3d(imu_data.linear_acceleration.x(),
                                          imu_data.linear_acceleration.y(),
                                          imu_data.linear_acceleration.z());

                r.gyro = Eigen::Vector3d(imu_data.angular_velocity.x(),
                                         imu_data.angular_velocity.y(),
                                         imu_data.angular_velocity.z());

                // Use steady_clock for monotonic timestamp (matching Gazebo pattern)
                r.timestamp = std::chrono::duration<double>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
                r.valid = true;

                // Store under lock (fast — ImuReading is ~56 bytes)
                {
                    std::lock_guard<std::mutex> lock(reading_mutex_);
                    if (!active_.load(std::memory_order_acquire)) break;
                    cached_reading_ = r;
                }

                const uint64_t count = msg_count_.fetch_add(1, std::memory_order_acq_rel) + 1;
                if (count == 1) {
                    DRONE_LOG_INFO("[CosysIMU] First IMU sample from '{}': "
                                   "accel=({:.3f}, {:.3f}, {:.3f}) "
                                   "gyro=({:.4f}, {:.4f}, {:.4f})",
                                   imu_name_, r.accel.x(), r.accel.y(), r.accel.z(), r.gyro.x(),
                                   r.gyro.y(), r.gyro.z());
                }

            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysIMU] RPC error in polling thread: {}", e.what());
            }

            std::this_thread::sleep_for(poll_interval);
        }

        DRONE_LOG_INFO("[CosysIMU] Polling thread exiting");
    }

    // ── Shared RPC client (shared_ptr: shared across 4 HAL backends) ──
    std::shared_ptr<CosysRpcClient> client_;

    // ── Config ────────────────────────────────────────────────
    std::string imu_name_;      ///< AirSim IMU sensor name
    std::string vehicle_name_;  ///< AirSim vehicle name
    int         rate_hz_{200};  ///< Polling rate

    // ── State ─────────────────────────────────────────────────
    std::atomic<bool>     active_{false};   ///< True after init() succeeds
    std::atomic<uint64_t> msg_count_{0};    ///< Messages received
    mutable std::mutex    reading_mutex_;   ///< Guards cached_reading_
    ImuReading            cached_reading_;  ///< Latest reading

    // ── Polling thread ────────────────────────────────────────
    std::thread poll_thread_;  ///< Polls getImuData() at rate_hz_
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
