// common/hal/include/hal/simulated_radar.h
// Simulated radar backend: generates noisy synthetic radar detections.
// Config-driven via drone::Config — all tunables read from the given section.
// Implements Issue #209 — SimulatedRadar backend.
#pragma once

#include "hal/iradar.h"
#include "util/config.h"
#include "util/ilogger.h"

#include <algorithm>
#include <chrono>
#include <random>

namespace drone::hal {

class SimulatedRadar : public IRadar {
public:
    /// Construct with config; all tunables read from the given section.
    explicit SimulatedRadar(const drone::Config& cfg, const std::string& section)
        : max_range_m_(cfg.get<float>(section + ".max_range_m", 100.0f))
        , fov_azimuth_rad_(cfg.get<float>(section + ".fov_azimuth_rad", 1.047f))
        , fov_elevation_rad_(cfg.get<float>(section + ".fov_elevation_rad", 0.262f))
        , false_alarm_rate_(cfg.get<float>(section + ".false_alarm_rate", 0.02f))
        , num_targets_(cfg.get<int>(section + ".num_targets", 3))
        , range_noise_(0.0f, cfg.get<float>(section + ".noise.range_std_m", 0.3f))
        , azimuth_noise_(0.0f, cfg.get<float>(section + ".noise.azimuth_std_rad", 0.026f))
        , elevation_noise_(0.0f, cfg.get<float>(section + ".noise.elevation_std_rad", 0.026f))
        , velocity_noise_(0.0f, cfg.get<float>(section + ".noise.velocity_std_mps", 0.1f)) {}

    bool init() override {
        active_ = true;
        DRONE_LOG_INFO(
            "[SimulatedRadar] Initialised (max_range={:.1f}m, targets={}, FoV={:.1f}x{:.1f} "
            "rad)",
            max_range_m_, num_targets_, fov_azimuth_rad_, fov_elevation_rad_);
        return true;
    }

    drone::ipc::RadarDetectionList read() override {
        drone::ipc::RadarDetectionList list{};
        if (!active_) return list;

        const auto now    = std::chrono::steady_clock::now().time_since_epoch();
        list.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());

        // Clamp num_targets to max detections
        const uint32_t target_count = static_cast<uint32_t>(
            std::min(num_targets_, static_cast<int>(drone::ipc::MAX_RADAR_DETECTIONS)));

        const float min_range_m = 0.5f;

        for (uint32_t i = 0; i < target_count; ++i) {
            drone::ipc::RadarDetection det{};
            det.timestamp_ns = list.timestamp_ns;

            // Generate base target position within FoV
            const float base_range = range_dist_(rng_) * (max_range_m_ - min_range_m) + min_range_m;
            const float base_az = azimuth_dist_(rng_) * fov_azimuth_rad_ - fov_azimuth_rad_ / 2.0f;
            const float base_el = elevation_dist_(rng_) * fov_elevation_rad_ -
                                  fov_elevation_rad_ / 2.0f;

            // Add noise
            det.range_m       = std::clamp(base_range + range_noise_(rng_), 0.0f, max_range_m_);
            det.azimuth_rad   = std::clamp(base_az + azimuth_noise_(rng_), -fov_azimuth_rad_ / 2.0f,
                                           fov_azimuth_rad_ / 2.0f);
            det.elevation_rad = std::clamp(base_el + elevation_noise_(rng_),
                                           -fov_elevation_rad_ / 2.0f, fov_elevation_rad_ / 2.0f);

            // Doppler velocity: simulated target radial velocity + noise
            det.radial_velocity_mps = velocity_base_dist_(rng_) + velocity_noise_(rng_);

            // Signal quality: SNR inversely proportional to range, RCS random
            det.rcs_dbsm = rcs_dist_(rng_);
            det.snr_db   = std::max(0.0f, 30.0f - 20.0f * std::log10(std::max(1.0f, det.range_m)));
            det.confidence = std::clamp(det.snr_db / 30.0f, 0.0f, 1.0f);
            det.track_id   = next_track_id_++;

            list.detections[list.num_detections++] = det;
        }

        // False alarm detections
        if (false_alarm_dist_(rng_) < false_alarm_rate_ &&
            list.num_detections < drone::ipc::MAX_RADAR_DETECTIONS) {
            drone::ipc::RadarDetection fa{};
            fa.timestamp_ns  = list.timestamp_ns;
            fa.range_m       = range_dist_(rng_) * max_range_m_;
            fa.azimuth_rad   = azimuth_dist_(rng_) * fov_azimuth_rad_ - fov_azimuth_rad_ / 2.0f;
            fa.elevation_rad = elevation_dist_(rng_) * fov_elevation_rad_ -
                               fov_elevation_rad_ / 2.0f;
            fa.radial_velocity_mps                 = velocity_noise_(rng_);
            fa.rcs_dbsm                            = -10.0f;
            fa.snr_db                              = 3.0f;
            fa.confidence                          = 0.1f;
            fa.track_id                            = next_track_id_++;
            list.detections[list.num_detections++] = fa;
        }

        return list;
    }

    bool is_active() const override { return active_; }

    std::string name() const override { return "SimulatedRadar"; }

private:
    // Config-driven parameters
    float max_range_m_{100.0f};
    float fov_azimuth_rad_{1.047f};
    float fov_elevation_rad_{0.262f};
    float false_alarm_rate_{0.02f};
    int   num_targets_{3};

    // State
    bool     active_{false};
    uint32_t next_track_id_{1};

    // RNG
    std::mt19937                          rng_{42};
    std::uniform_real_distribution<float> range_dist_{0.0f, 1.0f};
    std::uniform_real_distribution<float> azimuth_dist_{0.0f, 1.0f};
    std::uniform_real_distribution<float> elevation_dist_{0.0f, 1.0f};
    std::uniform_real_distribution<float> velocity_base_dist_{-5.0f, 5.0f};
    std::uniform_real_distribution<float> false_alarm_dist_{0.0f, 1.0f};
    std::uniform_real_distribution<float> rcs_dist_{-5.0f, 20.0f};
    std::normal_distribution<float>       range_noise_;
    std::normal_distribution<float>       azimuth_noise_;
    std::normal_distribution<float>       elevation_noise_;
    std::normal_distribution<float>       velocity_noise_;
};

}  // namespace drone::hal
