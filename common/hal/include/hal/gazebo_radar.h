// common/hal/include/hal/gazebo_radar.h
// HAL backend: radar data from Gazebo transport via gpu_lidar + odometry.
//
// Gazebo has no native radar sensor. This backend repurposes gpu_lidar for
// range/bearing geometry and odometry for body velocity, then synthesises
// radar-like detections (range, azimuth, elevation, Doppler radial velocity).
//
// Why HAL backend instead of a custom Gazebo plugin:
//   - No .so to compile against Gazebo internal API (no version coupling)
//   - No SDF <plugin> XML or GZ_SIM_SYSTEM_PLUGIN_PATH required
//   - All domain logic (noise, Doppler, false alarms) lives in our codebase
//   - Same IRadar interface as SimulatedRadar — testable without Gazebo
//
// Subscribes to:
//   1. gz::msgs::LaserScan — per-ray range + bearing from gpu_lidar sensor
//   2. gz::msgs::Odometry  — body velocity for Doppler projection
//
// Thread-safe: uses std::mutex to guard cached RadarDetectionList.
// Compile guard: only available when HAVE_GAZEBO is defined by CMake.
//
// Issue #212 — Gazebo radar sensor for SITL testing.
#pragma once

#ifdef HAVE_GAZEBO

#include "hal/iradar.h"
#include "util/config.h"
#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <random>
#include <string>

#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>

namespace drone::hal {

/// GazeboRadarBackend — synthesises radar detections from Gazebo gpu_lidar +
/// odometry topics. Converts lidar rays to radar measurement space and adds
/// configurable noise + false alarms.
///
/// Usage:
///   drone::Config cfg;
///   cfg.load("config/gazebo_sitl.json");
///   GazeboRadarBackend radar(cfg, "perception.radar");
///   radar.init();
///   auto dets = radar.read();
class GazeboRadarBackend : public IRadar {
public:
    /// @param cfg      Loaded configuration
    /// @param section  Config path prefix (e.g. "perception.radar")
    explicit GazeboRadarBackend(const drone::Config& cfg, const std::string& section)
        : max_range_m_(cfg.get<float>(section + ".max_range_m", 100.0f))
        , fov_azimuth_rad_(cfg.get<float>(section + ".fov_azimuth_rad", 1.047f))
        , fov_elevation_rad_(cfg.get<float>(section + ".fov_elevation_rad", 0.698f))
        , ground_filter_alt_m_(cfg.get<float>(section + ".ground_filter_alt_m", 0.5f))
        , false_alarm_rate_(cfg.get<float>(section + ".false_alarm_rate", 0.02f))
        , scan_topic_(cfg.get<std::string>(section + ".gz_scan_topic", "/radar_lidar/scan"))
        , odom_topic_(
              cfg.get<std::string>(section + ".gz_odom_topic", "/model/x500_companion_0/odometry"))
        , range_noise_(0.0f, cfg.get<float>(section + ".noise.range_std_m", 0.3f))
        , azimuth_noise_(0.0f, cfg.get<float>(section + ".noise.azimuth_std_rad", 0.026f))
        , elevation_noise_(0.0f, cfg.get<float>(section + ".noise.elevation_std_rad", 0.026f))
        , velocity_noise_(0.0f, cfg.get<float>(section + ".noise.velocity_std_mps", 0.1f)) {}

    ~GazeboRadarBackend() override { shutdown(); }

    // Non-copyable, non-movable (owns gz::transport::Node)
    GazeboRadarBackend(const GazeboRadarBackend&)            = delete;
    GazeboRadarBackend& operator=(const GazeboRadarBackend&) = delete;
    GazeboRadarBackend(GazeboRadarBackend&&)                 = delete;
    GazeboRadarBackend& operator=(GazeboRadarBackend&&)      = delete;

    bool init() override {
        if (active_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[GazeboRadar] Already initialised");
            return false;
        }

        bool scan_ok = node_.Subscribe(scan_topic_, &GazeboRadarBackend::on_scan, this);
        if (!scan_ok) {
            DRONE_LOG_ERROR("[GazeboRadar] Failed to subscribe to scan topic '{}'", scan_topic_);
            return false;
        }

        bool odom_ok = node_.Subscribe(odom_topic_, &GazeboRadarBackend::on_odom, this);
        if (!odom_ok) {
            DRONE_LOG_ERROR("[GazeboRadar] Failed to subscribe to odom topic '{}'", odom_topic_);
            node_.Unsubscribe(scan_topic_);
            return false;
        }

        active_.store(true, std::memory_order_release);
        DRONE_LOG_INFO("[GazeboRadar] Subscribed to scan='{}', odom='{}'", scan_topic_,
                       odom_topic_);
        return true;
    }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            active_.store(false, std::memory_order_release);
        }
        node_.Unsubscribe(scan_topic_);
        node_.Unsubscribe(odom_topic_);
        DRONE_LOG_INFO("[GazeboRadar] Shut down — unsubscribed from '{}' and '{}'", scan_topic_,
                       odom_topic_);
    }

    drone::ipc::RadarDetectionList read() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return cached_detections_;
    }

    bool is_active() const override {
        return active_.load(std::memory_order_acquire) &&
               scan_count_.load(std::memory_order_acquire) > 0;
    }

    std::string name() const override { return "GazeboRadar(" + scan_topic_ + ")"; }

    /// Number of scan messages received (useful for diagnostics).
    uint64_t scan_message_count() const { return scan_count_.load(std::memory_order_acquire); }

    /// Number of odometry messages received.
    uint64_t odom_message_count() const { return odom_count_.load(std::memory_order_acquire); }

    // ── Public static helpers (exposed for unit testing) ──────

    /// Compute radar detection from a single lidar ray + body velocity.
    /// @param range     Ray range in metres
    /// @param azimuth   Ray azimuth angle in radians
    /// @param elevation Ray elevation angle in radians
    /// @param vx, vy, vz  Body-frame velocity components (m/s)
    /// @return RadarDetection with range, azimuth, elevation, radial_velocity, SNR, confidence
    static drone::ipc::RadarDetection ray_to_detection(float range, float azimuth, float elevation,
                                                       float vx, float vy, float vz) {
        drone::ipc::RadarDetection det{};
        det.range_m       = range;
        det.azimuth_rad   = azimuth;
        det.elevation_rad = elevation;

        // Doppler: project body velocity onto radial direction
        // Radial unit vector from spherical coords:
        //   rx = cos(el) * cos(az), ry = cos(el) * sin(az), rz = sin(el)
        const float cos_el      = std::cos(elevation);
        const float rx          = cos_el * std::cos(azimuth);
        const float ry          = cos_el * std::sin(azimuth);
        const float rz          = std::sin(elevation);
        det.radial_velocity_mps = vx * rx + vy * ry + vz * rz;

        // SNR: inversely proportional to range (radar equation ~1/R^4,
        // simplified to 30 - 20*log10(R) dB for simulation)
        det.snr_db     = std::max(0.0f, 30.0f - 20.0f * std::log10(std::max(1.0f, range)));
        det.confidence = std::clamp(det.snr_db / 30.0f, 0.0f, 1.0f);
        det.rcs_dbsm   = 0.0f;  // Not modelled from lidar

        return det;
    }

    /// Convert LaserScan ray index to azimuth/elevation angles.
    /// @param h_idx  Horizontal ray index
    /// @param v_idx  Vertical ray index
    /// @param h_count  Total horizontal rays
    /// @param v_count  Total vertical rays
    /// @param h_min, h_max  Horizontal angle range (radians)
    /// @param v_min, v_max  Vertical angle range (radians)
    static std::pair<float, float> ray_index_to_angles(int h_idx, int v_idx, int h_count,
                                                       int v_count, float h_min, float h_max,
                                                       float v_min, float v_max) {
        float azimuth   = (h_count > 1) ? h_min + static_cast<float>(h_idx) * (h_max - h_min) /
                                                    static_cast<float>(h_count - 1)
                                        : 0.5f * (h_min + h_max);
        float elevation = (v_count > 1) ? v_min + static_cast<float>(v_idx) * (v_max - v_min) /
                                                      static_cast<float>(v_count - 1)
                                        : 0.5f * (v_min + v_max);
        return {azimuth, elevation};
    }

private:
    // ── gz-transport callbacks ────────────────────────────────

    void on_scan(const gz::msgs::LaserScan& msg) {
        if (!active_.load(std::memory_order_acquire)) return;

        drone::ipc::RadarDetectionList list{};
        const auto                     now = std::chrono::steady_clock::now().time_since_epoch();
        list.timestamp_ns                  = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());

        // Extract scan geometry
        const int   h_count = static_cast<int>(msg.count());
        const int   v_count = static_cast<int>(msg.vertical_count());
        const float h_min   = static_cast<float>(msg.angle_min());
        const float h_max   = static_cast<float>(msg.angle_max());
        const float v_min   = static_cast<float>(msg.vertical_angle_min());
        const float v_max   = static_cast<float>(msg.vertical_angle_max());
        const float rng_min = static_cast<float>(msg.range_min());
        const float rng_max = std::min(static_cast<float>(msg.range_max()), max_range_m_);

        // Get current body velocity (under lock)
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            vx = body_vx_;
            vy = body_vy_;
            vz = body_vz_;
        }

        const int total_rays    = h_count * std::max(v_count, 1);
        const int ray_data_size = msg.ranges_size();

        // Lock RNG + track ID for thread safety (gz-transport callbacks may be
        // dispatched from different threads across subscription reconnects)
        std::lock_guard<std::mutex> rng_lock(rng_mutex_);
        uint32_t                    next_tid = next_track_id_;

        for (int i = 0; i < std::min(total_rays, ray_data_size); ++i) {
            if (list.num_detections >= drone::ipc::MAX_RADAR_DETECTIONS) break;

            const float range = static_cast<float>(msg.ranges(i));

            // Skip invalid rays (inf, NaN, out of range)
            if (!std::isfinite(range) || range < rng_min || range > rng_max) continue;

            // Compute ray angles from index
            const int h_idx = i % h_count;
            const int v_idx = (v_count > 1) ? (i / h_count) : 0;
            auto [az, el] = ray_index_to_angles(h_idx, v_idx, h_count, std::max(v_count, 1), h_min,
                                                h_max, v_min, v_max);

            // Ground rejection: compute ray's world-frame altitude and skip
            // returns below ground_filter_alt_m_ (Issue #229).
            // object_alt = drone_alt + range * sin(elevation)
            // Skip filter if no odometry altitude received yet — avoids dropping
            // all detections when the vehicle spawns above ground.
            if (has_altitude_.load(std::memory_order_acquire)) {
                const float object_alt = drone_altitude_m_.load(std::memory_order_acquire) +
                                         range * std::sin(el);
                if (object_alt < ground_filter_alt_m_) continue;
            }

            // Build detection from ray geometry + body velocity
            auto det = ray_to_detection(range, az, el, vx, vy, vz);

            // Add noise — clamp range to [rng_min, rng_max] to avoid
            // physically invalid sub-minimum-range detections
            det.range_m       = std::clamp(det.range_m + range_noise_(rng_), rng_min, rng_max);
            det.azimuth_rad   = std::clamp(det.azimuth_rad + azimuth_noise_(rng_),
                                           -fov_azimuth_rad_ / 2.0f, fov_azimuth_rad_ / 2.0f);
            det.elevation_rad = std::clamp(det.elevation_rad + elevation_noise_(rng_),
                                           -fov_elevation_rad_ / 2.0f, fov_elevation_rad_ / 2.0f);
            det.radial_velocity_mps += velocity_noise_(rng_);

            det.timestamp_ns = list.timestamp_ns;
            det.track_id     = next_tid++;

            list.detections[list.num_detections++] = det;
        }

        // False alarm injection
        if (list.num_detections < drone::ipc::MAX_RADAR_DETECTIONS &&
            false_alarm_dist_(rng_) < false_alarm_rate_) {
            drone::ipc::RadarDetection fa{};
            fa.timestamp_ns  = list.timestamp_ns;
            fa.range_m       = uniform_dist_(rng_) * max_range_m_;
            fa.azimuth_rad   = uniform_dist_(rng_) * fov_azimuth_rad_ - fov_azimuth_rad_ / 2.0f;
            fa.elevation_rad = uniform_dist_(rng_) * fov_elevation_rad_ - fov_elevation_rad_ / 2.0f;
            fa.radial_velocity_mps                 = velocity_noise_(rng_);
            fa.rcs_dbsm                            = -10.0f;
            fa.snr_db                              = 3.0f;
            fa.confidence                          = 0.1f;
            fa.track_id                            = next_tid++;
            list.detections[list.num_detections++] = fa;
        }

        next_track_id_ = next_tid;

        // Store under lock
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            cached_detections_ = list;
        }

        uint64_t count = scan_count_.fetch_add(1, std::memory_order_acq_rel) + 1;
        if (count == 1) {
            DRONE_LOG_INFO("[GazeboRadar] First scan: {} rays, {} detections from '{}'", total_rays,
                           list.num_detections, scan_topic_);
        }
    }

    void on_odom(const gz::msgs::Odometry& msg) {
        if (!active_.load(std::memory_order_acquire)) return;

        if (msg.has_twist()) {
            const auto&                 linear = msg.twist().linear();
            std::lock_guard<std::mutex> lock(odom_mutex_);
            body_vx_ = static_cast<float>(linear.x());
            body_vy_ = static_cast<float>(linear.y());
            body_vz_ = static_cast<float>(linear.z());
        }

        // Extract altitude from odometry pose for HAL-level ground filtering.
        // Gazebo Odometry pose.position.z is the drone's world-frame altitude.
        if (msg.has_pose()) {
            drone_altitude_m_.store(static_cast<float>(msg.pose().position().z()),
                                    std::memory_order_release);
            has_altitude_.store(true, std::memory_order_release);
        }

        odom_count_.fetch_add(1, std::memory_order_acq_rel);
    }

    // ── Config ────────────────────────────────────────────────
    float       max_range_m_;
    float       fov_azimuth_rad_;
    float       fov_elevation_rad_;
    float       ground_filter_alt_m_;  // reject rays resolving below this altitude AGL
    float       false_alarm_rate_;
    std::string scan_topic_;
    std::string odom_topic_;

    // ── gz-transport ──────────────────────────────────────────
    gz::transport::Node   node_;
    std::atomic<bool>     active_{false};
    std::atomic<uint64_t> scan_count_{0};
    std::atomic<uint64_t> odom_count_{0};

    // ── Cached detections (guarded by mutex_) ─────────────────
    mutable std::mutex             mutex_;
    drone::ipc::RadarDetectionList cached_detections_{};

    // ── Body velocity (guarded by odom_mutex_) ────────────────
    mutable std::mutex odom_mutex_;
    float              body_vx_{0.0f};
    float              body_vy_{0.0f};
    float              body_vz_{0.0f};

    // ── Drone altitude for HAL-level ground filtering ────────
    std::atomic<float> drone_altitude_m_{0.0f};
    std::atomic<bool>  has_altitude_{false};

    // ── RNG for noise injection (guarded by rng_mutex_) ───────
    mutable std::mutex                            rng_mutex_;
    mutable std::mt19937                          rng_{42};
    mutable std::normal_distribution<float>       range_noise_;
    mutable std::normal_distribution<float>       azimuth_noise_;
    mutable std::normal_distribution<float>       elevation_noise_;
    mutable std::normal_distribution<float>       velocity_noise_;
    mutable std::uniform_real_distribution<float> uniform_dist_{0.0f, 1.0f};
    mutable std::uniform_real_distribution<float> false_alarm_dist_{0.0f, 1.0f};

    // ── Track ID counter ──────────────────────────────────────
    uint32_t next_track_id_{1};
};

}  // namespace drone::hal

#endif  // HAVE_GAZEBO
