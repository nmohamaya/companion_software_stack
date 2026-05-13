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
// Threading: same contract as CosysEchoBackend — init/shutdown serialized
// by std::call_once, read() concurrent with poll thread under mutex,
// poll thread touches a heartbeat token each iteration.
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_name_filter.h"
#include "hal/cosys_name_resolver.h"
#include "hal/cosys_rpc_client.h"
#include "hal/iradar.h"
#include "ipc/ipc_types.h"
#include "util/config.h"
#include "util/ilogger.h"
#include "util/thread_heartbeat.h"

#include <Eigen/Geometry>

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
    explicit CosysGroundTruthRadarBackend(std::shared_ptr<CosysRpcClient> client,
                                          const drone::Config& cfg, const std::string& section)
        : client_(std::move(client))
        , vehicle_name_(drone::hal::resolve_vehicle_name(cfg, section))
        , max_range_m_(cfg.get<float>(section + ".max_range_m", 100.0f))
        , min_range_m_(std::max(0.1f, cfg.get<float>(section + ".min_range_m", 0.5f)))
        , fov_azimuth_rad_(cfg.get<float>(section + ".fov_azimuth_rad",
                                          60.0f * static_cast<float>(M_PI) / 180.0f))
        , fov_elevation_rad_(cfg.get<float>(section + ".fov_elevation_rad",
                                            15.0f * static_cast<float>(M_PI) / 180.0f))
        , update_rate_hz_(std::max(1, cfg.get<int>(section + ".update_rate_hz", 20)))
        , filter_(cfg, section,
                  /*default_excludes=*/"Ground,Sky,SkyDome,Floor",
                  /*vehicle_name=*/vehicle_name_,
                  /*unknown_action=*/CosysNameFilterUnknown::Keep) {
        const float rad_to_deg = 180.0f / static_cast<float>(M_PI);
        DRONE_LOG_INFO(
            "[CosysGroundTruthRadar] Created for {} vehicle='{}' range=[{:.1f},{:.1f}]m "
            "FOV=±{:.0f}°×±{:.0f}° rate={}Hz mode={}",
            client_->endpoint(), vehicle_name_, min_range_m_, max_range_m_,
            fov_azimuth_rad_ * rad_to_deg, fov_elevation_rad_ * rad_to_deg, update_rate_hz_,
            filter_.has_allowlist() ? "allowlist" : "blocklist");
    }

    ~CosysGroundTruthRadarBackend() override { shutdown(); }

    CosysGroundTruthRadarBackend(const CosysGroundTruthRadarBackend&)            = delete;
    CosysGroundTruthRadarBackend& operator=(const CosysGroundTruthRadarBackend&) = delete;
    CosysGroundTruthRadarBackend(CosysGroundTruthRadarBackend&&)                 = delete;
    CosysGroundTruthRadarBackend& operator=(CosysGroundTruthRadarBackend&&)      = delete;

    [[nodiscard]] bool init() override {
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
            if (!filter_.is_excluded(object_names_[i])) {
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
        std::call_once(shutdown_once_, [this] {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!active_.load(std::memory_order_acquire)) return;
                active_.store(false, std::memory_order_release);
            }
            if (poll_thread_.joinable()) poll_thread_.join();
            DRONE_LOG_INFO("[CosysGroundTruthRadar] Shut down");
        });
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
        // Watchdog token — see CosysEchoBackend for rationale.
        drone::util::ScopedHeartbeat heartbeat("cosys_gt_radar_poll", /*critical=*/false);
        DRONE_LOG_INFO("[CosysGroundTruthRadar] Polling thread started (interval={}ms)",
                       poll_interval.count());

        while (active_.load(std::memory_order_acquire)) {
            heartbeat.touch();
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

                // Defensive: if the simulator dynamically spawned/pruned objects
                // between init() and this poll, poses.size() will diverge from
                // object_names_.size().  Without this guard the parallel
                // (poses[i] ↔ object_names_[i]) invariant breaks and we get
                // phantom detections at attacker-chosen positions (PR #704
                // security review).  Drop the scan entirely and warn once.
                if (poses.size() != object_names_.size()) {
                    if (!size_mismatch_warned_) {
                        DRONE_LOG_WARN(
                            "[CosysGroundTruthRadar] pose/name list size mismatch ({} vs {}) — "
                            "scene was modified after init().  Dropping scans until matched.  "
                            "Logging once.",
                            poses.size(), object_names_.size());
                        size_mismatch_warned_ = true;
                    }
                    std::this_thread::sleep_for(poll_interval);
                    continue;
                }
                size_mismatch_warned_ = false;  // reset if we recover

                // Drone pose in NED (Cosys/AirSim convention).  Use the full
                // quaternion (not yaw-only) to rotate the world→body delta.
                // PR #704 — the prior yaw-only matrix was wrong during pitch /
                // roll: takeoff, braking, aggressive turns put the radar body
                // frame off-level and azimuth/elevation became meaningless.
                const Eigen::Vector3f drone_pos_w(static_cast<float>(drone_kin.pose.position.x()),
                                                  static_cast<float>(drone_kin.pose.position.y()),
                                                  static_cast<float>(drone_kin.pose.position.z()));
                const Eigen::Quaternionf q_body_world(
                    static_cast<float>(drone_kin.pose.orientation.w()),
                    static_cast<float>(drone_kin.pose.orientation.x()),
                    static_cast<float>(drone_kin.pose.orientation.y()),
                    static_cast<float>(drone_kin.pose.orientation.z()));
                const Eigen::Matrix3f R_body_from_world = q_body_world.conjugate().toRotationMatrix();

                // Drone linear velocity in NED for radial-velocity computation.
                const float vN = static_cast<float>(drone_kin.twist.linear.x());
                const float vE = static_cast<float>(drone_kin.twist.linear.y());
                const float vD = static_cast<float>(drone_kin.twist.linear.z());

                size_t in_fov = 0, out_fov = 0, kept = 0, missing_pose = 0;
                const size_t n = poses.size();
                for (size_t i = 0; i < n; ++i) {
                    if (!keep_index_[i]) continue;

                    // simListInstanceSegmentationPoses(only_visible=true) returns
                    // non-finite pose entries for hidden objects — filter on the
                    // finite check.  We do NOT additionally treat (0,0,0) as a
                    // sentinel: a real scene object placed at the world origin
                    // would otherwise be invisible to the radar (Copilot review
                    // on PR #704).
                    const float wn = static_cast<float>(poses[i].position.x());
                    const float we = static_cast<float>(poses[i].position.y());
                    const float wd = static_cast<float>(poses[i].position.z());
                    if (!std::isfinite(wn) || !std::isfinite(we) || !std::isfinite(wd)) {
                        ++missing_pose;
                        continue;
                    }

                    // World-relative vector (NED).
                    const Eigen::Vector3f delta_world(wn - drone_pos_w.x(), we - drone_pos_w.y(),
                                                      wd - drone_pos_w.z());
                    const float range = delta_world.norm();
                    if (range < min_range_m_ || range > max_range_m_) continue;

                    // Full 3D rotation: body-frame delta = R_body_from_world * delta_world.
                    // Body convention (FRD): +X forward, +Y right, +Z down.
                    const Eigen::Vector3f delta_body = R_body_from_world * delta_world;
                    const float           body_fx   = delta_body.x();
                    const float           body_fy   = delta_body.y();
                    const float           body_fz   = delta_body.z();

                    const float azimuth   = std::atan2(body_fy, body_fx);
                    const float elevation = std::asin(std::clamp(body_fz / range, -1.0f, 1.0f));

                    if (std::abs(azimuth) > fov_azimuth_rad_ ||
                        std::abs(elevation) > fov_elevation_rad_) {
                        ++out_fov;
                        continue;
                    }
                    ++in_fov;

                    // Radial velocity along the look vector.  Positive = receding
                    // (matches automotive-radar convention).  Look vector in
                    // world frame (NED) — drone twist is also NED so no
                    // rotation needed for the dot product.
                    const float look_n = delta_world.x() / range;
                    const float look_e = delta_world.y() / range;
                    const float look_d = delta_world.z() / range;
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

    std::shared_ptr<CosysRpcClient> client_;
    std::string                     vehicle_name_;
    float                           max_range_m_;
    float                           min_range_m_;
    float                           fov_azimuth_rad_;
    float                           fov_elevation_rad_;
    int                             update_rate_hz_;
    CosysNameFilter                 filter_;

    std::vector<std::string>        object_names_;   ///< Cached at init(), parallel to keep_index_
    std::vector<uint8_t>            keep_index_;     ///< 1 if object should appear as obstacle

    mutable std::mutex              mutex_;
    drone::ipc::RadarDetectionList  cached_detections_{};
    std::atomic<bool>               active_{false};
    std::atomic<uint64_t>           scan_count_{0};
    bool                            size_mismatch_warned_ = false;  ///< poll-thread only
    std::thread                     poll_thread_;
    std::once_flag                  shutdown_once_;
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
