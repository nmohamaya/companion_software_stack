// common/hal/include/hal/cosys_radar.h
// Cosys-AirSim radar backend — receives radar detections via AirSim RPC API.
// Implements IRadar — connects to Cosys-AirSim getRadarData() endpoint.
// Guarded by HAVE_COSYS_AIRSIM (set by CMake when AirSim SDK is found).
//
// Cosys-AirSim provides native radar sensor support (unlike stock AirSim),
// returning range, azimuth, elevation, and velocity for each detection.
//
// Thread-safe: uses std::mutex to guard cached RadarDetectionList.
// Compile guard: only available when HAVE_COSYS_AIRSIM is defined by CMake.
//
// Issue: #434
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/iradar.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

namespace drone::hal {

/// CosysRadarBackend — retrieves radar detections from Cosys-AirSim
/// via the getRadarData() RPC endpoint.
///
/// Usage:
///   drone::Config cfg;
///   cfg.load("config/cosys_airsim.json");
///   CosysRadarBackend radar(cfg, "perception.radar");
///   radar.init();
///   auto dets = radar.read();
class CosysRadarBackend : public IRadar {
public:
    /// @param cfg      Loaded configuration
    /// @param section  Config path prefix (e.g. "perception.radar")
    explicit CosysRadarBackend(const drone::Config& cfg, const std::string& section)
        : host_(cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::HOST), "127.0.0.1"))
        , port_(cfg.get<int>(std::string(drone::cfg_key::cosys_airsim::PORT), 41451))
        , radar_name_(cfg.get<std::string>(std::string(drone::cfg_key::cosys_airsim::RADAR_NAME),
                                           "front_radar"))
        , vehicle_name_(cfg.get<std::string>(
              std::string(drone::cfg_key::cosys_airsim::VEHICLE_NAME), "Drone0"))
        , max_range_m_(cfg.get<float>(section + drone::cfg_key::hal::MAX_RANGE_M, 100.0f)) {
        DRONE_LOG_INFO("[CosysRadar] Created for {}:{} radar='{}' vehicle='{}' max_range={}m",
                       host_, port_, radar_name_, vehicle_name_, max_range_m_);
    }

    ~CosysRadarBackend() override { shutdown(); }

    // Non-copyable, non-movable (owns RPC connection state)
    CosysRadarBackend(const CosysRadarBackend&)            = delete;
    CosysRadarBackend& operator=(const CosysRadarBackend&) = delete;
    CosysRadarBackend(CosysRadarBackend&&)                 = delete;
    CosysRadarBackend& operator=(CosysRadarBackend&&)      = delete;

    bool init() override {
        if (active_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysRadar] Already initialised");
            return false;
        }

        // TODO(#434): Connect to Cosys-AirSim RPC endpoint at host_:port_
        // and verify radar sensor exists using getRadarData(radar_name_, vehicle_name_).
        // Start polling thread for periodic radar data retrieval.

        active_.store(true, std::memory_order_release);
        DRONE_LOG_INFO("[CosysRadar] Initialised radar='{}' on {}:{}", radar_name_, host_, port_);
        return true;
    }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            active_.store(false, std::memory_order_release);
        }
        // TODO(#434): Disconnect from Cosys-AirSim RPC endpoint
        // and stop polling thread.
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
        return "CosysRadar(" + radar_name_ + "@" + host_ + ":" + std::to_string(port_) + ")";
    }

    /// Number of scan messages received (useful for diagnostics).
    uint64_t scan_message_count() const { return scan_count_.load(std::memory_order_acquire); }

private:
    // ── Config ────────────────────────────────────────────────
    std::string host_;          ///< Cosys-AirSim RPC host
    int         port_;          ///< Cosys-AirSim RPC port
    std::string radar_name_;    ///< AirSim radar sensor name
    std::string vehicle_name_;  ///< AirSim vehicle name
    float       max_range_m_;   ///< Maximum detection range

    // ── State ─────────────────────────────────────────────────
    std::atomic<bool>     active_{false};
    std::atomic<uint64_t> scan_count_{0};

    // ── Cached detections (guarded by mutex_) ─────────────────
    mutable std::mutex             mutex_;
    drone::ipc::RadarDetectionList cached_detections_{};
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
