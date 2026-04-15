// common/hal/include/hal/cosys_imu.h
// Cosys-AirSim IMU backend — receives IMU data via AirSim RPC API.
// Implements IIMUSource — connects to Cosys-AirSim getImuData() endpoint.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Thread-safe: uses a std::mutex to guard access to the cached ImuReading.
// The struct is small (~56 bytes), so copying it under the lock is cheap.
//
// Compile guard: only available when HAVE_COSYS_AIRSIM is defined by CMake.
//
// Issue: #434
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"
#include "hal/iimu_source.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace drone::hal {

/// CosysIMUBackend — receives accelerometer + gyroscope data from
/// a Cosys-AirSim simulation via the getImuData() RPC endpoint.
///
/// Usage:
///   drone::Config cfg;
///   cfg.load("config/cosys_airsim.json");
///   CosysIMUBackend imu(cfg, "slam.imu");
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

    /// Explicitly shut down: prevents callbacks from racing the destructor.
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(reading_mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            active_.store(false, std::memory_order_release);
        }
        // TODO(#462): Stop polling thread.
        // Connection lifecycle is managed by the shared CosysRpcClient.
        DRONE_LOG_INFO("[CosysIMU] Shut down imu='{}'", imu_name_);
    }

    // Non-copyable, non-movable (owns RPC connection state)
    CosysIMUBackend(const CosysIMUBackend&)            = delete;
    CosysIMUBackend& operator=(const CosysIMUBackend&) = delete;
    CosysIMUBackend(CosysIMUBackend&&)                 = delete;
    CosysIMUBackend& operator=(CosysIMUBackend&&)      = delete;

    /// Initialise — connect to Cosys-AirSim IMU endpoint.
    /// @param rate_hz  Informational; actual rate is driven by AirSim sensor.
    /// @return true on successful connection
    bool init(int rate_hz) override {
        if (active_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysIMU] Already initialised on imu='{}'", imu_name_);
            return false;
        }

        rate_hz_ = rate_hz;

        // TODO(#462): Verify IMU sensor exists via client_ using
        // getImuData(imu_name_, vehicle_name_).
        // Start polling thread for periodic IMU data retrieval.

        active_.store(true, std::memory_order_release);
        DRONE_LOG_INFO("[CosysIMU] Initialised imu='{}' on {} (rate_hz={} informational)",
                       imu_name_, client_->endpoint(), rate_hz_);
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
    // ── Shared RPC client (shared_ptr: shared across 4 HAL backends) ──
    std::shared_ptr<CosysRpcClient> client_;

    // ── Config ────────────────────────────────────────────────
    std::string imu_name_;      ///< AirSim IMU sensor name
    std::string vehicle_name_;  ///< AirSim vehicle name
    int         rate_hz_{200};  ///< Informational sample rate

    // ── State ─────────────────────────────────────────────────
    std::atomic<bool>     active_{false};   ///< True after init() succeeds
    std::atomic<uint64_t> msg_count_{0};    ///< Messages received
    mutable std::mutex    reading_mutex_;   ///< Guards cached_reading_
    ImuReading            cached_reading_;  ///< Latest reading
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
