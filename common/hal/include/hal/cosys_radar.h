// common/hal/include/hal/cosys_radar.h
// Cosys-AirSim radar backend — receives radar detections via AirSim RPC API.
// Implements IRadar — connects to Cosys-AirSim getRadarData() endpoint.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Cosys-AirSim provides native radar sensor support (unlike stock AirSim),
// returning range, azimuth, elevation, and velocity for each detection.
//
// Polling thread runs at 20 Hz and converts AirSim RadarData to our
// RadarDetectionList format. Ground filtering skips returns with
// elevation < -30 degrees (looking at the ground).
//
// Thread-safe: uses std::mutex to guard cached RadarDetectionList.
// Compile guard: only available when HAVE_COSYS_AIRSIM is defined by CMake.
//
// Issue: #434, #462
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_rpc_client.h"
#include "hal/iradar.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <thread>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// CosysRadarBackend — emulates radar using LiDAR point cloud data
/// from Cosys-AirSim via the getLidarData() RPC endpoint.
///
/// The polling thread runs at a fixed 20 Hz (kPollInterval = 50 ms)
/// and converts LiDAR point cloud (flat x,y,z triples) to our
/// RadarDetectionList wire format. Ground returns with steep downward
/// elevation (> +30° in NED) are filtered.
///
/// AirSim uses NED body frame which maps directly to our FRD convention:
/// X=forward, Y=right, Z=down — no coordinate negation needed.
///
/// Usage:
///   drone::Config cfg;
///   cfg.load("config/cosys_airsim.json");
///   CosysRadarBackend radar(client, cfg, "perception.radar");
///   radar.init();
///   auto dets = radar.read();
class CosysRadarBackend : public IRadar {
public:
    /// @param client   Shared RPC client (manages connection lifecycle)
    /// @param cfg      Loaded configuration
    /// @param section  Config path prefix (e.g. "perception.radar")
    explicit CosysRadarBackend(std::shared_ptr<CosysRpcClient> client, const drone::Config& cfg,
                               const std::string& section)
        : client_(std::move(client))
        , radar_name_(
              cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::RADAR_NAME), "radar"))
        , vehicle_name_(cfg.get<std::string>(
              std::string(drone::cfg_key::cosys_airsim::VEHICLE_NAME), "Drone0"))
        , max_range_m_(cfg.get<float>(section + drone::cfg_key::hal::MAX_RANGE_M, 100.0f)) {
        DRONE_LOG_INFO("[CosysRadar] Created for {} radar='{}' vehicle='{}' max_range={}m",
                       client_->endpoint(), radar_name_, vehicle_name_, max_range_m_);
    }

    ~CosysRadarBackend() override { shutdown(); }

    // Non-copyable, non-movable (owns polling thread)
    CosysRadarBackend(const CosysRadarBackend&)            = delete;
    CosysRadarBackend& operator=(const CosysRadarBackend&) = delete;
    CosysRadarBackend(CosysRadarBackend&&)                 = delete;
    CosysRadarBackend& operator=(CosysRadarBackend&&)      = delete;

    bool init() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysRadar] Already initialised");
            return false;
        }

        if (!client_->is_connected()) {
            DRONE_LOG_ERROR("[CosysRadar] RPC client not connected — cannot init radar");
            return false;
        }

        active_.store(true, std::memory_order_release);

        // Start 20 Hz polling thread for radar data retrieval
        poll_thread_ = std::thread(&CosysRadarBackend::poll_loop, this);

        DRONE_LOG_INFO("[CosysRadar] Initialised radar='{}' on {}", radar_name_,
                       client_->endpoint());
        return true;
    }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            active_.store(false, std::memory_order_release);
        }

        // Join polling thread outside the lock to avoid deadlock
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }

        DRONE_LOG_INFO("[CosysRadar] Shut down radar='{}'", radar_name_);
    }

    drone::ipc::RadarDetectionList read() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return cached_detections_;
    }

    bool is_active() const override {
        return active_.load(std::memory_order_acquire) &&
               scan_count_.load(std::memory_order_acquire) > 0;
    }

    std::string name() const override {
        return "CosysRadar(" + radar_name_ + "@" + client_->endpoint() + ")";
    }

    /// Number of scan messages received (useful for diagnostics).
    uint64_t scan_message_count() const { return scan_count_.load(std::memory_order_acquire); }

private:
    /// Radar polling loop — runs at 20 Hz, converts AirSim radar data
    /// to our RadarDetectionList format with ground filtering.
    void poll_loop() {
        constexpr auto kPollInterval = std::chrono::milliseconds(50);  // 20 Hz
        // Ground filter: skip returns with steep downward elevation in NED frame.
        // In NED (Z=down), ground points below the drone have positive elevation
        // from asin(z/range). Threshold +30° filters steep ground returns.
        constexpr float kGroundElevationThreshold = 30.0f * std::numbers::pi_v<float> / 180.0f;
        // Simplified radar equation: SNR = ref_snr - path_loss * log10(range)
        constexpr float kReferenceSNRdB    = 30.0f;  // SNR at 1m range
        constexpr float kSNRPathLossFactor = 20.0f;  // Free-space path loss exponent

        DRONE_LOG_INFO("[CosysRadar] Polling thread started (interval={}ms)",
                       kPollInterval.count());

        while (active_.load(std::memory_order_acquire)) {
            try {
                // Use with_client() to prevent TOCTOU race on disconnect
                msr::airlib::RadarData radar_data;
                bool                   got_data = client_->with_client(
                    [&](auto& rpc) { radar_data = rpc.getRadarData(radar_name_, vehicle_name_); });
                if (!got_data) {
                    DRONE_LOG_WARN("[CosysRadar] RPC disconnected — skipping scan");
                    std::this_thread::sleep_for(kPollInterval);
                    continue;
                }

                drone::ipc::RadarDetectionList list{};
                const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count();
                list.timestamp_ns = static_cast<uint64_t>(std::max(decltype(now_ns){0}, now_ns));

                // Convert each AirSim radar point to our RadarDetection format.
                // AirSim RadarData contains point_cloud (x,y,z triples) and
                // additional per-point metadata.
                for (const auto& point : radar_data.point_cloud) {
                    if (list.num_detections >= drone::ipc::MAX_RADAR_DETECTIONS) break;

                    // AirSim radar point_cloud entries are (x, y, z) in NED body frame.
                    // Convert Cartesian to spherical: range, azimuth, elevation.
                    const float x = point.x;
                    const float y = point.y;
                    const float z = point.z;

                    // Guard against NaN/Inf in radar point cloud data
                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                    const float range = std::sqrt(x * x + y * y + z * z);

                    // Skip zero-range or out-of-range returns (before trig ops)
                    if (range < 0.1f || range > max_range_m_) continue;

                    const float azimuth   = std::atan2(y, x);
                    const float elevation = std::asin(std::clamp(z / range, -1.0f, 1.0f));

                    // Ground filter: in NED (Z=down), ground below has positive
                    // elevation. Skip returns with steep downward angle.
                    if (elevation > kGroundElevationThreshold) continue;

                    drone::ipc::RadarDetection det{};
                    det.timestamp_ns        = list.timestamp_ns;
                    det.range_m             = range;
                    det.azimuth_rad         = azimuth;
                    det.elevation_rad       = elevation;
                    det.radial_velocity_mps = point.velocity;
                    // SNR model: inversely proportional to range (simplified radar equation)
                    det.snr_db     = std::max(0.0f,
                                              kReferenceSNRdB - kSNRPathLossFactor *
                                                                    std::log10(std::max(1.0f, range)));
                    det.confidence = std::clamp(det.snr_db / kReferenceSNRdB, 0.0f, 1.0f);
                    det.rcs_dbsm   = 0.0f;  // Not provided by AirSim
                    det.track_id   = list.num_detections + 1;

                    list.detections[list.num_detections++] = det;
                }

                // Store under lock
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!active_.load(std::memory_order_acquire)) break;
                    cached_detections_ = list;
                }

                const uint64_t count = scan_count_.fetch_add(1, std::memory_order_acq_rel) + 1;
                if (count == 1) {
                    DRONE_LOG_INFO("[CosysRadar] First scan: {} detections from '{}'",
                                   list.num_detections, radar_name_);
                }

            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysRadar] RPC error in polling thread: {}", e.what());
            }

            std::this_thread::sleep_for(kPollInterval);
        }

        DRONE_LOG_INFO("[CosysRadar] Polling thread exiting");
    }

    // ── Shared RPC client (shared_ptr: shared across 4 HAL backends) ──
    std::shared_ptr<CosysRpcClient> client_;

    // ── Config ────────────────────────────────────────────────
    std::string radar_name_;    ///< AirSim LiDAR sensor name (used as radar proxy)
    std::string vehicle_name_;  ///< AirSim vehicle name
    float       max_range_m_;   ///< Maximum detection range

    // ── State ─────────────────────────────────────────────────
    std::atomic<bool>     active_{false};
    std::atomic<uint64_t> scan_count_{0};

    // ── Cached detections (guarded by mutex_) ─────────────────
    mutable std::mutex             mutex_;
    drone::ipc::RadarDetectionList cached_detections_{};

    // ── Polling thread ────────────────────────────────────────
    std::thread poll_thread_;  ///< Polls getLidarData() at 20 Hz (LiDAR as radar proxy)
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
