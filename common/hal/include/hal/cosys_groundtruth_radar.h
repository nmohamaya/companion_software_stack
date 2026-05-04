// common/hal/include/hal/cosys_groundtruth_radar.h
// Cosys-AirSim ground-truth radar backend.
//
// Implements IRadar by querying simListInstanceSegmentationObjects() +
// simListInstanceSegmentationPoses() and simGetGroundTruthKinematics()
// every scan, transforming each visible scene object's world pose into the
// drone body frame, applying a forward-cone FOV gate, and emitting one
// RadarDetection per visible obstacle with the ground-truth (range,
// azimuth, elevation, radial-velocity).  No ray casts, no clutter, no
// multipath — every detection is a real obstacle and only real obstacles.
//
// Sim-only counterpart to CosysRadarBackend (which does lidar emulation
// with cluster binning).  Use this for sim test scenarios where you want
// to validate the planner / avoider / cross-veto pipeline without radar
// noise contaminating the experiment.
//
// Issue: #698 — paired with CosysSegmentationBackend + CosysDepthBackend
// to give end-to-end ground-truth perception.
//
// NOT thread-safe for read() observation; the polling thread populates
// cached_detections_ under mutex.
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_name_resolver.h"
#include "hal/cosys_rpc_client.h"
#include "hal/iradar.h"
#include "ipc/ipc_types.h"
#include "util/config.h"
#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// Cosys-AirSim ground-truth radar — emits one RadarDetection per visible
/// obstacle by querying object poses directly from the simulator.
///
/// Config keys (with precedence: `<section>.<subkey>` → top-level → default):
///   `<section>.vehicle_name`        → `cosys_airsim.vehicle_name` (default "Drone0")
///   `<section>.max_range_m`         default 100 m
///   `<section>.min_range_m`         default 0.5 m
///   `<section>.fov_azimuth_rad`     default ±60° (forward cone half-angle)
///   `<section>.fov_elevation_rad`   default ±15°
///   `<section>.update_rate_hz`      default 20 Hz (matches CosysRadarBackend)
///   `<section>.include_substrings`  optional allowlist (e.g. "TemplateCube,Cube,Wall,Pillar")
///   `<section>.exclude_substrings`  blocklist for background (default "Ground,Sky,SkyDome,Floor")
class CosysGroundTruthRadarBackend : public IRadar {
public:
    static constexpr float kPi       = 3.14159265358979323846f;
    static constexpr float kDegToRad = kPi / 180.0f;
    static constexpr float kRadToDeg = 180.0f / kPi;

    explicit CosysGroundTruthRadarBackend(std::shared_ptr<CosysRpcClient> client,
                                          const drone::Config& cfg, const std::string& section)
        : client_(std::move(client))
        , vehicle_name_(drone::hal::resolve_vehicle_name(cfg, section))
        , max_range_m_(cfg.get<float>(section + ".max_range_m", 100.0f))
        , min_range_m_(std::max(0.1f, cfg.get<float>(section + ".min_range_m", 0.5f)))
        , fov_azimuth_rad_(cfg.get<float>(section + ".fov_azimuth_rad", 60.0f * kDegToRad))
        , fov_elevation_rad_(cfg.get<float>(section + ".fov_elevation_rad", 15.0f * kDegToRad))
        , update_rate_hz_(std::max(1, cfg.get<int>(section + ".update_rate_hz", 20))) {
        include_substrings_ = parse_csv(
            cfg.get<std::string>(section + ".include_substrings", std::string{}));
        exclude_substrings_ = parse_csv(
            cfg.get<std::string>(section + ".exclude_substrings", "Ground,Sky,SkyDome,Floor"));
        exclude_substrings_.push_back(vehicle_name_);

        DRONE_LOG_INFO(
            "[CosysGroundTruthRadar] Created for {} vehicle='{}' range=[{:.1f},{:.1f}]m "
            "FOV=±{:.0f}°×±{:.0f}° rate={}Hz mode={}",
            client_->endpoint(), vehicle_name_, min_range_m_, max_range_m_,
            fov_azimuth_rad_ * kRadToDeg, fov_elevation_rad_ * kRadToDeg, update_rate_hz_,
            include_substrings_.empty() ? "blocklist" : "allowlist");
    }

    ~CosysGroundTruthRadarBackend() override { shutdown(); }

    CosysGroundTruthRadarBackend(const CosysGroundTruthRadarBackend&)            = delete;
    CosysGroundTruthRadarBackend& operator=(const CosysGroundTruthRadarBackend&) = delete;
    CosysGroundTruthRadarBackend(CosysGroundTruthRadarBackend&&)                 = delete;
    CosysGroundTruthRadarBackend& operator=(CosysGroundTruthRadarBackend&&)      = delete;

    bool init() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysGroundTruthRadar] Already initialised");
            return false;
        }
        if (!client_->is_connected()) {
            DRONE_LOG_ERROR("[CosysGroundTruthRadar] RPC client not connected");
            return false;
        }

        // Cache the object-name list once.  simListInstanceSegmentationPoses
        // returns poses *parallel* to the names list, so we need them in the
        // same order on every read.
        try {
            const bool got = client_->with_client(
                [&](auto& rpc) { object_names_ = rpc.simListInstanceSegmentationObjects(); });
            if (!got) {
                DRONE_LOG_ERROR("[CosysGroundTruthRadar] init() — RPC client not connected");
                return false;
            }
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[CosysGroundTruthRadar] init() — RPC error: {}", e.what());
            return false;
        }

        // Pre-compute include/exclude flags per object index so the per-scan
        // hot path doesn't have to do string-search × N every tick.
        keep_index_.assign(object_names_.size(), 0);
        size_t n_kept = 0;
        for (size_t i = 0; i < object_names_.size(); ++i) {
            if (!is_excluded(object_names_[i])) {
                keep_index_[i] = 1;
                ++n_kept;
            }
        }
        DRONE_LOG_INFO(
            "[CosysGroundTruthRadar] Cached {} scene objects ({} kept after include/exclude)",
            object_names_.size(), n_kept);

        active_.store(true, std::memory_order_release);
        poll_thread_ = std::thread(&CosysGroundTruthRadarBackend::poll_loop, this);
        DRONE_LOG_INFO("[CosysGroundTruthRadar] Initialised on {} vehicle='{}'",
                       client_->endpoint(), vehicle_name_);
        return true;
    }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_.load(std::memory_order_acquire)) return;
            active_.store(false, std::memory_order_release);
        }
        if (poll_thread_.joinable()) poll_thread_.join();
        DRONE_LOG_INFO("[CosysGroundTruthRadar] Shut down");
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
        return "CosysGroundTruthRadar(" + vehicle_name_ + "@" + client_->endpoint() + ")";
    }

private:
    void poll_loop() {
        const auto poll_interval = std::chrono::milliseconds(1000 / update_rate_hz_);
        DRONE_LOG_INFO("[CosysGroundTruthRadar] Polling thread started (interval={}ms)",
                       poll_interval.count());

        while (active_.load(std::memory_order_acquire)) {
            try {
                using namespace msr::airlib;
                std::vector<Pose>           poses;
                Kinematics::State           drone_kin{};
                bool                        got = client_->with_client([&](auto& rpc) {
                    // only_visible=true means we get a vector with the SAME length as
                    // simListInstanceSegmentationObjects() (cached in object_names_),
                    // but objects not currently visible have zero/identity pose entries
                    // we must filter out.
                    poses     = rpc.simListInstanceSegmentationPoses(/* ned */ true,
                                                                     /* only_visible */ true);
                    drone_kin = rpc.simGetGroundTruthKinematics(vehicle_name_);
                });
                if (!got) {
                    std::this_thread::sleep_for(poll_interval);
                    continue;
                }

                drone::ipc::RadarDetectionList list{};
                const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count();
                list.timestamp_ns = static_cast<uint64_t>(std::max(decltype(now_ns){0}, now_ns));

                // Drone pose in NED (Cosys/AirSim convention).
                const float drone_n = static_cast<float>(drone_kin.pose.position.x());
                const float drone_e = static_cast<float>(drone_kin.pose.position.y());
                const float drone_d = static_cast<float>(drone_kin.pose.position.z());

                // Drone heading (yaw) in NED — extract yaw from body→world quaternion.
                // yaw_NED = atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))
                const float qw = static_cast<float>(drone_kin.pose.orientation.w());
                const float qx = static_cast<float>(drone_kin.pose.orientation.x());
                const float qy = static_cast<float>(drone_kin.pose.orientation.y());
                const float qz = static_cast<float>(drone_kin.pose.orientation.z());
                const float yaw_ned =
                    std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
                // World→body rotation about +Z (down).  body_x = forward, body_y = right
                // (FRD).  See body-frame computation below.
                const float cos_yaw = std::cos(yaw_ned);
                const float sin_yaw = std::sin(yaw_ned);

                // Drone linear velocity in NED for radial-velocity computation.
                const float vN = static_cast<float>(drone_kin.twist.linear.x());
                const float vE = static_cast<float>(drone_kin.twist.linear.y());
                const float vD = static_cast<float>(drone_kin.twist.linear.z());

                size_t in_fov = 0, out_fov = 0, kept = 0, missing_pose = 0;
                const size_t n = std::min(poses.size(), object_names_.size());
                for (size_t i = 0; i < n; ++i) {
                    if (!keep_index_[i]) continue;

                    // simListInstanceSegmentationPoses(only_visible=true) returns
                    // identity (NaN/0) entries for non-visible objects — filter
                    // those by checking for finite + non-zero position.
                    const float wn = static_cast<float>(poses[i].position.x());
                    const float we = static_cast<float>(poses[i].position.y());
                    const float wd = static_cast<float>(poses[i].position.z());
                    if (!std::isfinite(wn) || !std::isfinite(we) || !std::isfinite(wd)) {
                        ++missing_pose;
                        continue;
                    }
                    if (wn == 0.0f && we == 0.0f && wd == 0.0f) {
                        ++missing_pose;
                        continue;
                    }

                    // Translate object into drone-relative world coords (NED).
                    const float dx_world = wn - drone_n;
                    const float dy_world = we - drone_e;
                    const float dz_world = wd - drone_d;
                    const float range    = std::sqrt(dx_world * dx_world + dy_world * dy_world +
                                                  dz_world * dz_world);
                    if (range < min_range_m_ || range > max_range_m_) continue;

                    // Rotate world-relative vector (NED) into drone body frame (FRD):
                    //   body_x (forward) =  dN·cos(yaw) + dE·sin(yaw)
                    //   body_y (right)   = -dN·sin(yaw) + dE·cos(yaw)
                    //   body_z (down)    =  dD          (NED-Z axis preserved in FRD)
                    // Using the cos_yaw/sin_yaw computed above.
                    const float body_fx =  dx_world * cos_yaw + dy_world * sin_yaw;
                    const float body_fy = -dx_world * sin_yaw + dy_world * cos_yaw;
                    const float body_fz =  dz_world;

                    const float azimuth   = std::atan2(body_fy, body_fx);
                    const float elevation = std::asin(std::clamp(body_fz / range, -1.0f, 1.0f));

                    if (std::abs(azimuth) > fov_azimuth_rad_ ||
                        std::abs(elevation) > fov_elevation_rad_) {
                        ++out_fov;
                        continue;
                    }
                    ++in_fov;

                    // Radial velocity along the look vector.  Positive = receding
                    // (matches automotive-radar convention).
                    const float look_n = dx_world / range;
                    const float look_e = dy_world / range;
                    const float look_d = dz_world / range;
                    // Object is stationary in the scene; drone motion contributes
                    // the entire relative-velocity component along the look vector.
                    // Radial-velocity SIGN convention: closing rate is negative,
                    // receding is positive.
                    const float radial_vel = -(vN * look_n + vE * look_e + vD * look_d);

                    if (list.num_detections >= drone::ipc::MAX_RADAR_DETECTIONS) break;
                    drone::ipc::RadarDetection det{};
                    det.timestamp_ns        = list.timestamp_ns;
                    det.range_m             = range;
                    det.azimuth_rad         = azimuth;
                    det.elevation_rad       = elevation;
                    det.radial_velocity_mps = radial_vel;
                    det.snr_db              = 30.0f;  // ground truth = max-confidence return
                    det.confidence          = 1.0f;
                    det.rcs_dbsm            = 0.0f;
                    det.track_id            = list.num_detections + 1;
                    list.detections[list.num_detections++] = det;
                    ++kept;
                }

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!active_.load(std::memory_order_acquire)) break;
                    cached_detections_ = list;
                }

                const uint64_t scan = scan_count_.fetch_add(1, std::memory_order_acq_rel) + 1;
                if (scan == 1) {
                    DRONE_LOG_INFO(
                        "[CosysGroundTruthRadar] First scan: {} kept-objects → {} in-FOV, "
                        "{} out-of-FOV, {} non-visible (missing pose), {} emitted detections",
                        n, in_fov, out_fov, missing_pose, kept);
                } else if (scan % 200 == 0) {
                    DRONE_LOG_INFO(
                        "[CosysGroundTruthRadar] scan #{}: {} in-FOV / {} out / {} hidden / "
                        "{} emitted",
                        scan, in_fov, out_fov, missing_pose, kept);
                }
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysGroundTruthRadar] RPC error: {}", e.what());
            }
            std::this_thread::sleep_for(poll_interval);
        }
        DRONE_LOG_INFO("[CosysGroundTruthRadar] Polling thread stopped");
    }

    [[nodiscard]] bool is_excluded(const std::string& obj_name) const {
        if (!include_substrings_.empty()) {
            if (obj_name.empty()) return true;
            for (const auto& sub : include_substrings_) {
                if (!sub.empty() && obj_name.find(sub) != std::string::npos) return false;
            }
            return true;
        }
        if (obj_name.empty()) return false;
        for (const auto& sub : exclude_substrings_) {
            if (!sub.empty() && obj_name.find(sub) != std::string::npos) return true;
        }
        return false;
    }

    [[nodiscard]] static std::vector<std::string> parse_csv(const std::string& raw) {
        std::vector<std::string> out;
        size_t                   start = 0;
        while (start < raw.size()) {
            const size_t comma = raw.find(',', start);
            const size_t end   = (comma == std::string::npos) ? raw.size() : comma;
            std::string  tok   = raw.substr(start, end - start);
            while (!tok.empty() && (tok.front() == ' ' || tok.front() == '\t')) tok.erase(0, 1);
            while (!tok.empty() && (tok.back() == ' ' || tok.back() == '\t')) tok.pop_back();
            if (!tok.empty()) out.push_back(std::move(tok));
            if (comma == std::string::npos) break;
            start = comma + 1;
        }
        return out;
    }

    std::shared_ptr<CosysRpcClient> client_;
    std::string                     vehicle_name_;
    float                           max_range_m_;
    float                           min_range_m_;
    float                           fov_azimuth_rad_;
    float                           fov_elevation_rad_;
    int                             update_rate_hz_;
    std::vector<std::string>        include_substrings_;
    std::vector<std::string>        exclude_substrings_;

    std::vector<std::string>        object_names_;   ///< Cached at init(), parallel to keep_index_
    std::vector<uint8_t>            keep_index_;     ///< 1 if object should appear as obstacle

    mutable std::mutex              mutex_;
    drone::ipc::RadarDetectionList  cached_detections_{};
    std::atomic<bool>               active_{false};
    std::atomic<uint64_t>           scan_count_{0};
    std::thread                     poll_thread_;
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
