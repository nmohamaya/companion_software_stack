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
#include "util/config_keys.h"
#include "util/ilogger.h"
#include "util/sensor_geometry.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <random>
#include <string>
#include <tuple>

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
        : max_range_m_(cfg.get<float>(section + drone::cfg_key::hal::MAX_RANGE_M, 100.0f))
        , fov_azimuth_rad_(cfg.get<float>(section + drone::cfg_key::hal::FOV_AZIMUTH_RAD, 1.047f))
        , fov_elevation_rad_(
              cfg.get<float>(section + drone::cfg_key::hal::FOV_ELEVATION_RAD, 0.698f))
        , ground_filter_alt_m_(
              cfg.get<float>(section + drone::cfg_key::hal::GROUND_FILTER_ALT_M, 0.5f))
        , false_alarm_rate_(cfg.get<float>(section + drone::cfg_key::hal::FALSE_ALARM_RATE, 0.02f))
        , scan_topic_(cfg.get<std::string>(section + drone::cfg_key::hal::GZ_SCAN_TOPIC,
                                           "/radar_lidar/scan"))
        , odom_topic_(cfg.get<std::string>(section + drone::cfg_key::hal::GZ_ODOM_TOPIC,
                                           "/model/x500_companion_0/odometry"))
        , range_noise_(0.0f, cfg.get<float>(section + drone::cfg_key::hal::NOISE_RANGE_STD_M, 0.3f))
        , azimuth_noise_(
              0.0f, cfg.get<float>(section + drone::cfg_key::hal::NOISE_AZIMUTH_STD_RAD, 0.026f))
        , elevation_noise_(
              0.0f, cfg.get<float>(section + drone::cfg_key::hal::NOISE_ELEVATION_STD_RAD, 0.026f))
        , velocity_noise_(
              0.0f, cfg.get<float>(section + drone::cfg_key::hal::NOISE_VELOCITY_STD_MPS, 0.1f)) {
        // Issue #816 PR2 — the HAL owns the sensor→body mount rotation: emitted
        // az/el are BODY-frame (mount-compensated), so no downstream consumer
        // needs to know the extrinsics.  x500_companion lidar: pitch −0.087.
        //
        // PR #819 review (Minor 3) — sanity-clamp the mount extrinsics to
        // [-π, π] and WARN, mirroring the attitude-margin clamp below.  Unlike
        // the margin (bounded [0,0.35]) these were UNVALIDATED config, yet a
        // mis-entered mount angle silently mis-projects EVERY ray for the whole
        // run: a "-8.7" typo for "-0.087" rotates returns by ~500° instead of
        // −5°, corrupting the body-frame az/el the entire downstream stack
        // trusts.  |angle|>π is never a real physical mount (rotations wrap), so
        // clamping there converts a silent corruption into a loud WARN.  Bias:
        // keep the clamped value rather than fail hard — a typo degrades to a
        // bounded mis-projection the WARN surfaces, not a crash mid-flight.
        const auto clamp_mount_rad = [](const char* name, float raw) -> float {
            const float clamped = clamp_mount_angle_rad(raw);
            if (clamped != raw) {
                DRONE_LOG_WARN("[GazeboRadar] {}={} rad out of range [-pi,pi] - likely a typo "
                               "(e.g. -8.7 for -0.087); clamping to {}. Every ray is "
                               "mis-projected until corrected.",
                               name, raw, clamped);
            }
            return clamped;
        };
        const float mr = clamp_mount_rad(
            "mount_roll_rad", cfg.get<float>(section + drone::cfg_key::hal::MOUNT_ROLL_RAD, 0.0f));
        const float mp = clamp_mount_rad(
            "mount_pitch_rad",
            cfg.get<float>(section + drone::cfg_key::hal::MOUNT_PITCH_RAD, -0.087f));
        const float my = clamp_mount_rad(
            "mount_yaw_rad", cfg.get<float>(section + drone::cfg_key::hal::MOUNT_YAW_RAD, 0.0f));
        sensor_to_body_q_ = drone::util::quat_from_rpy(mr, mp, my);
        // Widen the post-noise angle clamps by the mount's TOTAL rotation
        // angle (PR #819 review): a general 3D mount couples roll/pitch/yaw
        // into BOTH body az and el (e.g. a pitch mount shifts off-boresight
        // azimuths via atan2), so per-axis sums under-cover.  The rotation
        // angle is a conservative upper bound on any bearing shift; clamping
        // wider than the FOV is harmless (the clamp only guards against noise
        // pushing angles unphysically far).
        const float mount_angle_rad = Eigen::AngleAxisf(sensor_to_body_q_).angle();
        clamp_az_rad_               = fov_azimuth_rad_ / 2.0f + mount_angle_rad;
        clamp_el_rad_               = fov_elevation_rad_ / 2.0f + mount_angle_rad;
        // Fail-safe margin for the attitude-aware ground gate (same semantics
        // as the fusion gate's key): reject as ground only if below the floor
        // even after allowing this much attitude error.  Clamped [0, 0.35] —
        // negative would bias toward dropping obstacles (the unsafe direction).
        const float raw_margin =
            cfg.get<float>(section + drone::cfg_key::hal::ATTITUDE_UNCERTAINTY_RAD, 0.02f);
        attitude_uncertainty_rad_ = std::clamp(raw_margin, 0.0f, 0.35f);
        if (attitude_uncertainty_rad_ != raw_margin) {
            DRONE_LOG_WARN("[GazeboRadar] attitude_uncertainty_rad={} out of range [0,0.35], "
                           "clamping to {}",
                           raw_margin, attitude_uncertainty_rad_);
        }
    }

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

    /// Issue #816 PR2 — rotate a sensor-frame ray bearing into the BODY frame
    /// through the mount extrinsics, returning (azimuth, elevation) in body
    /// coordinates.  Emitted detections use these, so downstream consumers
    /// (fusion ground gate, UKF cartesian conversion) need no mount knowledge.
    static std::pair<float, float> sensor_to_body_angles(float azimuth, float elevation,
                                                         const Eigen::Quaternionf& sensor_to_body) {
        const float           cos_el = std::cos(elevation);
        const Eigen::Vector3f dir_s(cos_el * std::cos(azimuth),  // forward
                                    cos_el * std::sin(azimuth),  // left (FLU)
                                    std::sin(elevation));        // up
        const Eigen::Vector3f dir_b = sensor_to_body * dir_s;
        const float           az_b  = std::atan2(dir_b.y(), dir_b.x());
        const float           el_b  = std::asin(std::clamp(dir_b.z(), -1.0f, 1.0f));
        return {az_b, el_b};
    }

    /// Issue #816 PR #819 review (Minor 3) — sanity-clamp a mount extrinsic
    /// angle to the physical range [-π, π].  |angle| > π is never a real mount
    /// (rotations wrap) and indicates a config typo (e.g. -8.7 for -0.087),
    /// which would silently mis-project EVERY emitted ray.  Pure + static so the
    /// clamp is directly unit-testable (same pattern as ground_gate_should_reject);
    /// the constructor pairs it with a WARN when the value was out of range.
    [[nodiscard]] static float clamp_mount_angle_rad(float raw) {
        constexpr float kPiRad = 3.14159265358979323846f;
        return std::clamp(raw, -kPiRad, kPiRad);
    }

    /// Issue #816 — attitude-aware HAL ground gate (shares the math with the
    /// fusion gate via util/sensor_geometry).  Returns true iff the return is
    /// confidently BELOW floor_m: requires BOTH altitude and attitude (an
    /// unknown attitude must never suppress — fail-safe), and biases
    /// false-accept by margin_rad of allowed attitude error.
    /// Public + static so the safety property is directly unit-testable
    /// (same pattern as ray_to_detection).
    static bool ground_gate_should_reject(bool has_altitude, bool has_attitude, float drone_alt_m,
                                          float range_m, float az_body_rad, float el_body_rad,
                                          const Eigen::Quaternionf& body_to_world, float floor_m,
                                          float margin_rad) {
        if (!has_altitude || !has_attitude) return false;  // fail-safe: keep
        const float alt    = drone::util::return_world_altitude_m(drone_alt_m, range_m, az_body_rad,
                                                                  el_body_rad, body_to_world);
        const float margin = range_m * std::sin(std::max(0.0f, margin_rad));
        return (alt + margin) < floor_m;
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

        // Get current body velocity + attitude snapshot (under lock).
        // Issue #816 PR2: one attitude per scan — every ray of this scan is
        // projected with the same body→world rotation.  has_attitude_ is read
        // INSIDE the same critical section as the quaternion (PR #819 review):
        // reading it after unlock could pair a stale quaternion with a newer
        // `true` flag — an inconsistent snapshot that changes reject/keep
        // decisions for the whole scan.
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
        bool  has_att = false;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            vx      = body_vx_;
            vy      = body_vy_;
            vz      = body_vz_;
            qw      = att_w_;
            qx      = att_x_;
            qy      = att_y_;
            qz      = att_z_;
            has_att = has_attitude_.load(std::memory_order_acquire);
        }
        const Eigen::Quaternionf body_to_world = drone::util::quat_from_wxyz(qw, qx, qy, qz);

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

            // Compute ray angles from index (raw SENSOR frame)
            const int h_idx   = i % h_count;
            const int v_idx   = (v_count > 1) ? (i / h_count) : 0;
            auto [az_s, el_s] = ray_index_to_angles(h_idx, v_idx, h_count, std::max(v_count, 1),
                                                    h_min, h_max, v_min, v_max);

            // Issue #816 PR2 — rotate through the mount extrinsics so the
            // emitted az/el are BODY-frame (the wire contract for
            // /radar_detections; consumers need no mount knowledge).
            auto [az, el] = sensor_to_body_angles(az_s, el_s, sensor_to_body_q_);

            // Ground rejection (Issue #229, attitude-aware since #816):
            // world altitude via the full body→world rotation, fail-safe —
            // rejects only with BOTH altitude and attitude present, and only
            // when confidently below the floor (margin biases false-accept).
            // Previously `drone_alt + range·sin(raw_el)`, which ignored drone
            // pitch/roll and the −5° mount: it admitted ground at 14–24 m as
            // ghosts (#815) and dropped real obstacles under nose-up pitch.
            if (ground_gate_should_reject(has_altitude_.load(std::memory_order_acquire), has_att,
                                          drone_altitude_m_.load(std::memory_order_acquire), range,
                                          az, el, body_to_world, ground_filter_alt_m_,
                                          attitude_uncertainty_rad_)) {
                continue;
            }

            // Build detection from ray geometry + body velocity (body-frame
            // angles ⇒ the Doppler projection now uses the true body ray).
            auto det = ray_to_detection(range, az, el, vx, vy, vz);

            // Add noise — clamp range to [rng_min, rng_max] to avoid
            // physically invalid sub-minimum-range detections.  Angle clamps
            // are the sensor FOV widened by the mount magnitude (body-frame
            // angles are shifted by the mount — see ctor).
            det.range_m       = std::clamp(det.range_m + range_noise_(rng_), rng_min, rng_max);
            det.azimuth_rad   = std::clamp(det.azimuth_rad + azimuth_noise_(rng_), -clamp_az_rad_,
                                           clamp_az_rad_);
            det.elevation_rad = std::clamp(det.elevation_rad + elevation_noise_(rng_),
                                           -clamp_el_rad_, clamp_el_rad_);
            det.radial_velocity_mps += velocity_noise_(rng_);

            det.timestamp_ns = list.timestamp_ns;
            det.track_id     = next_tid++;

            list.detections[list.num_detections++] = det;
        }

        // False alarm injection
        if (list.num_detections < drone::ipc::MAX_RADAR_DETECTIONS &&
            false_alarm_dist_(rng_) < false_alarm_rate_) {
            drone::ipc::RadarDetection fa{};
            fa.timestamp_ns = list.timestamp_ns;
            fa.range_m      = uniform_dist_(rng_) * max_range_m_;
            // False alarms are receiver artifacts in the SENSOR frame — convert
            // to body like real rays so the wire contract stays uniform (#816),
            // and clamp to the same widened body-frame bounds as real rays
            // (PR #819 review — downstream consumers assume those limits).
            const float fa_az_s = uniform_dist_(rng_) * fov_azimuth_rad_ - fov_azimuth_rad_ / 2.0f;
            const float fa_el_s = uniform_dist_(rng_) * fov_elevation_rad_ -
                                  fov_elevation_rad_ / 2.0f;
            std::tie(fa.azimuth_rad, fa.elevation_rad) = sensor_to_body_angles(fa_az_s, fa_el_s,
                                                                               sensor_to_body_q_);
            fa.azimuth_rad         = std::clamp(fa.azimuth_rad, -clamp_az_rad_, clamp_az_rad_);
            fa.elevation_rad       = std::clamp(fa.elevation_rad, -clamp_el_rad_, clamp_el_rad_);
            fa.radial_velocity_mps = velocity_noise_(rng_);
            fa.rcs_dbsm            = -10.0f;
            fa.snr_db              = 3.0f;
            fa.confidence          = 0.1f;
            fa.track_id            = next_tid++;
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

            // Issue #816 PR2 — body→world attitude for the attitude-aware
            // ground gate.  Multi-field → mutex (same rule as velocities).
            // PR #819 review hardening: VALIDATE at write time (finite, non-
            // degenerate norm) and CLEAR the flag when orientation is absent
            // or invalid — a suppression gate must treat "stale/garbage
            // attitude" exactly like "no attitude" (never reject), not run on
            // a latched flag with an identity fallback quaternion.
            bool valid = false;
            if (msg.pose().has_orientation()) {
                const auto& o  = msg.pose().orientation();
                const auto  w  = static_cast<float>(o.w());
                const auto  x  = static_cast<float>(o.x());
                const auto  y  = static_cast<float>(o.y());
                const auto  z  = static_cast<float>(o.z());
                const float n2 = w * w + x * x + y * y + z * z;
                if (std::isfinite(n2) && n2 > 1e-6f) {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    att_w_ = w;
                    att_x_ = x;
                    att_y_ = y;
                    att_z_ = z;
                    has_attitude_.store(true, std::memory_order_release);
                    valid = true;
                }
            }
            if (!valid) {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                has_attitude_.store(false, std::memory_order_release);
            }
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

    // ── Body velocity + attitude (guarded by odom_mutex_) ─────
    mutable std::mutex odom_mutex_;
    float              body_vx_{0.0f};
    float              body_vy_{0.0f};
    float              body_vz_{0.0f};
    // Issue #816 PR2 — body→world attitude (Pose order w,x,y,z) for the
    // attitude-aware ground gate.  Multi-field ⇒ mutex; identity until the
    // first odometry message (has_attitude_ gates any rejection).
    float att_w_{1.0f};
    float att_x_{0.0f};
    float att_y_{0.0f};
    float att_z_{0.0f};

    // ── Drone altitude for HAL-level ground filtering ────────
    std::atomic<float> drone_altitude_m_{0.0f};
    std::atomic<bool>  has_altitude_{false};
    std::atomic<bool>  has_attitude_{false};

    // ── Issue #816 PR2 — mount extrinsics + gate margin (set in ctor,
    //    immutable afterwards; read from the scan callback without a lock) ──
    Eigen::Quaternionf sensor_to_body_q_{Eigen::Quaternionf::Identity()};
    float              clamp_az_rad_{0.5235f};
    float              clamp_el_rad_{0.349f};
    float              attitude_uncertainty_rad_{0.02f};

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
