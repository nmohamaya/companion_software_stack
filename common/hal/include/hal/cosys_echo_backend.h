// common/hal/include/hal/cosys_echo_backend.h
// Cosys-AirSim Echo radar backend.
//
// Implements IRadar by polling Cosys-Lab's EchoSimple sensor (sensor type 7),
// which is a physical FMCW-style radar/sonar simulator built into the
// Cosys-AirSim plugin.  Each Echo scan ray-casts from the sensor, accumulates
// reflections (with attenuation per distance + per reflection, plus a
// reflection limit), and returns a flat point cloud where each detection is
// (x, y, z, attenuation, distance) — Unreal coords.
//
// Replaces CosysRadarBackend (lidar emulation, Issue #702) for the proper
// "radar physics" role.  CosysGroundTruthRadarBackend stays available as a
// validation oracle (one detection per visible object, no clutter).
//
// Issue: #705 (parent: #698 perception epic, supersedes #702)
//
// Threading contract:
//   - init() / shutdown() are NOT reentrant — call them from a single thread.
//     Both internally serialise via std::call_once so a dtor racing with an
//     explicit shutdown is safe, but this is belt-and-braces.
//   - read() may be called concurrently with the polling thread (mutex
//     protects cached_detections_).
//   - The polling thread touches a ThreadHeartbeat token at the top of each
//     iteration so the watchdog detects an Echo RPC stall.
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_name_filter.h"
#include "hal/cosys_name_resolver.h"
#include "hal/cosys_rpc_client.h"
#include "hal/iradar.h"
#include "ipc/ipc_types.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"
#include "util/thread_heartbeat.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// Cosys-AirSim Echo radar — physical reflection simulator.
///
/// Config keys (with precedence: `<section>.<subkey>` → top-level → default):
///   `<section>.echo_name`           → `cosys_airsim.echo_name`    (default "echo")
///   `<section>.vehicle_name`        → `cosys_airsim.vehicle_name` (default "Drone0")
///   `<section>.max_range_m`         default 50 m  (mirrors EchoSimpleParams.distance_limit)
///   `<section>.min_range_m`         default 0.5 m
///   `<section>.fov_azimuth_rad`     default ±60° — used for our extra HAL-side gate
///                                    (Echo applies its own SensorLowerAzimuthLimit/
///                                    SensorUpperAzimuthLimit at the sim level)
///   `<section>.fov_elevation_rad`   default ±15°
///   `<section>.update_rate_hz`      default 20 Hz (must match settings.json's
///                                    MeasurementFrequency or you'll get duplicate
///                                    poll-equals-data scans)
///   `<section>.cluster_range_m`     default 1.0 m   — bin Echo's many per-object
///   `<section>.cluster_az_rad`      default 5° (rad) returns into one detection
///   `<section>.cluster_el_rad`      default 5° (rad) per ~obstacle
///   `<section>.include_substrings`  optional allowlist (e.g. "TemplateCube,Cube")
///   `<section>.exclude_substrings`  default "Ground,Sky,SkyDome,Floor"
class CosysEchoBackend : public IRadar {
public:
    static constexpr float kMinClusterBinM   = 1e-3f;
    static constexpr float kMinClusterBinRad = 1e-6f;

    /// Echo's per-point packing: (x_unreal, y_unreal, z_unreal, attenuation_dB, distance_m).
    /// See `third_party/cosys-airsim/PythonClient/car/echo_test.py` for the
    /// authoritative reshape.
    static constexpr int kFloatsPerEchoPoint = 5;

    explicit CosysEchoBackend(std::shared_ptr<CosysRpcClient> client, const drone::Config& cfg,
                              const std::string& section)
        : client_(std::move(client))
        , echo_name_(drone::hal::resolve_echo_name(cfg, section))
        , vehicle_name_(drone::hal::resolve_vehicle_name(cfg, section))
        , max_range_m_(cfg.get<float>(section + ".max_range_m", 50.0f))
        , min_range_m_(std::max(0.1f, cfg.get<float>(section + ".min_range_m", 0.5f)))
        , fov_azimuth_rad_(cfg.get<float>(section + ".fov_azimuth_rad",
                                          60.0f * static_cast<float>(M_PI) / 180.0f))
        , fov_elevation_rad_(cfg.get<float>(section + ".fov_elevation_rad",
                                            15.0f * static_cast<float>(M_PI) / 180.0f))
        , update_rate_hz_(std::max(1, cfg.get<int>(section + ".update_rate_hz", 20)))
        , cluster_range_m_(std::max(kMinClusterBinM,
                                    cfg.get<float>(section + ".cluster_range_m", 1.0f)))
        , cluster_az_rad_(std::max(kMinClusterBinRad,
                                   cfg.get<float>(section + ".cluster_az_rad",
                                                  5.0f * static_cast<float>(M_PI) / 180.0f)))
        , cluster_el_rad_(std::max(kMinClusterBinRad,
                                   cfg.get<float>(section + ".cluster_el_rad",
                                                  5.0f * static_cast<float>(M_PI) / 180.0f)))
        , filter_(cfg, section,
                  /*default_excludes=*/"Ground,Sky,SkyDome,Floor",
                  /*vehicle_name=*/vehicle_name_,
                  /*unknown_action=*/CosysNameFilterUnknown::Keep) {
        const float rad_to_deg = 180.0f / static_cast<float>(M_PI);
        DRONE_LOG_INFO(
            "[CosysEcho] Created for {} echo='{}' vehicle='{}' range=[{:.1f},{:.1f}]m "
            "FOV=±{:.0f}°×±{:.0f}° rate={}Hz cluster={:.1f}m×{:.0f}°×{:.0f}° mode={}",
            client_->endpoint(), echo_name_, vehicle_name_, min_range_m_, max_range_m_,
            fov_azimuth_rad_ * rad_to_deg, fov_elevation_rad_ * rad_to_deg, update_rate_hz_,
            cluster_range_m_, cluster_az_rad_ * rad_to_deg, cluster_el_rad_ * rad_to_deg,
            filter_.has_allowlist() ? "allowlist" : "blocklist");
    }

    ~CosysEchoBackend() override { shutdown(); }

    CosysEchoBackend(const CosysEchoBackend&)            = delete;
    CosysEchoBackend& operator=(const CosysEchoBackend&) = delete;
    CosysEchoBackend(CosysEchoBackend&&)                 = delete;
    CosysEchoBackend& operator=(CosysEchoBackend&&)      = delete;

    [[nodiscard]] bool init() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysEcho] Already initialised");
            return false;
        }
        if (!client_->is_connected()) {
            DRONE_LOG_ERROR("[CosysEcho] RPC client not connected");
            return false;
        }
        active_.store(true, std::memory_order_release);
        poll_thread_ = std::thread(&CosysEchoBackend::poll_loop, this);
        DRONE_LOG_INFO("[CosysEcho] Initialised echo='{}' on {}", echo_name_, client_->endpoint());
        return true;
    }

    void shutdown() {
        // call_once guarantees idempotent + race-free shutdown even if dtor
        // and an explicit shutdown() fire from different threads.  Inner
        // body still uses the mutex to publish active_=false to read().
        std::call_once(shutdown_once_, [this] {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!active_.load(std::memory_order_acquire)) return;
                active_.store(false, std::memory_order_release);
            }
            if (poll_thread_.joinable()) poll_thread_.join();
            DRONE_LOG_INFO("[CosysEcho] Shut down");
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
        return "CosysEcho(" + echo_name_ + "@" + client_->endpoint() + ")";
    }

private:
    void poll_loop() {
        const auto poll_interval = std::chrono::milliseconds(1000 / update_rate_hz_);
        // Watchdog token: an Echo RPC stall would otherwise leave the
        // thread silently spinning forever, with cached_detections_ growing
        // stale and the cross-veto gate's `radar_stale` flag firing
        // indefinitely with no upstream signal.  Touching the heartbeat
        // each iteration lets ThreadWatchdog detect the hang.
        drone::util::ScopedHeartbeat heartbeat("cosys_echo_poll", /*critical=*/false);
        DRONE_LOG_INFO("[CosysEcho] Polling thread started (interval={}ms)",
                       poll_interval.count());

        // Per-bin accumulator for clustering.  Echo can emit hundreds of returns
        // per real obstacle (each ray that hits + each multipath reflection);
        // bin them into one detection per ~obstacle so downstream UKF doesn't
        // have to deal with hundreds of correlated tracks per frame.
        struct ClusterAcc {
            float    sum_range = 0.0f;
            float    sum_az    = 0.0f;
            float    sum_el    = 0.0f;
            float    sum_atten = 0.0f;
            uint32_t count     = 0;
            // Most-frequent groundtruth name for this cluster (best-effort —
            // ties broken by first-seen).  Used for include/exclude filtering
            // post-cluster so a single ground-bounce ray within an obstacle
            // cluster doesn't kill the whole detection.  Flat vector beats
            // unordered_map at the typical N=1-2 unique names per bin and
            // costs zero heap allocations per scan when reused (PR #704
            // perf review P1).
            std::vector<std::pair<std::string, uint32_t>> name_votes;
        };
        std::unordered_map<uint64_t, ClusterAcc> bins;

        auto bin_key = [](int rb, int ab, int eb) -> uint64_t {
            return (static_cast<uint64_t>(static_cast<uint32_t>(rb) & 0xFFFFu) << 32) |
                   (static_cast<uint64_t>(static_cast<uint32_t>(ab) & 0xFFFFu) << 16) |
                   static_cast<uint64_t>(static_cast<uint32_t>(eb) & 0xFFFFu);
        };

        while (active_.load(std::memory_order_acquire)) {
            heartbeat.touch();
            try {
                using namespace msr::airlib;
                EchoData data;
                bool     got = client_->with_client([&](auto& rpc) {
                    data = rpc.getEchoData(echo_name_, vehicle_name_);
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

                bins.clear();
                size_t raw = 0, in_fov = 0, dropped_range = 0, dropped_nonfinite = 0;

                // Each Echo point is 5 floats: (x, y, z, attenuation_dB, distance_m)
                // in Unreal coords.  Apply [1, -1, -1] flip to match NED body
                // (Cosys's own echo_test.py reference does the same flip).
                const auto&  pc       = data.point_cloud;
                const size_t n_points = pc.size() / kFloatsPerEchoPoint;
                if (pc.size() % kFloatsPerEchoPoint != 0 && !partial_point_warned_) {
                    DRONE_LOG_WARN(
                        "[CosysEcho] Echo returned partial point ({} floats; expected multiple "
                        "of {}). Truncating last point. Logging once.",
                        pc.size(), kFloatsPerEchoPoint);
                    partial_point_warned_ = true;
                }
                for (size_t i = 0; i < n_points; ++i) {
                    ++raw;
                    const float x = pc[i * kFloatsPerEchoPoint + 0];
                    const float y = -pc[i * kFloatsPerEchoPoint + 1];
                    const float z = -pc[i * kFloatsPerEchoPoint + 2];
                    const float attenuation_dB = pc[i * kFloatsPerEchoPoint + 3];
                    // pc[i*5 + 4] is total path distance (including multipath);
                    // we recompute straight-line range from xyz instead, since
                    // that's what the UKF expects.
                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                        !std::isfinite(attenuation_dB)) {
                        ++dropped_nonfinite;
                        continue;
                    }

                    const float range = std::sqrt(x * x + y * y + z * z);
                    if (range < min_range_m_ || range > max_range_m_) {
                        ++dropped_range;
                        continue;
                    }
                    const float azimuth   = std::atan2(y, x);
                    const float elevation = std::asin(std::clamp(z / range, -1.0f, 1.0f));

                    if (std::abs(azimuth) > fov_azimuth_rad_) continue;
                    if (std::abs(elevation) > fov_elevation_rad_) continue;
                    ++in_fov;

                    // Per-point name from parallel groundtruth vector.  May be
                    // empty if Echo didn't tag this return (e.g. multipath
                    // reflection beyond the first bounce).  string_view to
                    // avoid per-point heap allocation.
                    std::string_view gt_name;
                    if (i < data.groundtruth.size()) gt_name = data.groundtruth[i];

                    const int rb = static_cast<int>(std::floor(range / cluster_range_m_));
                    const int ab = static_cast<int>(std::floor(azimuth / cluster_az_rad_));
                    const int eb = static_cast<int>(std::floor(elevation / cluster_el_rad_));
                    if (rb < kBinIndexMin || rb > kBinIndexMax || ab < kBinIndexMin ||
                        ab > kBinIndexMax || eb < kBinIndexMin || eb > kBinIndexMax) {
                        continue;
                    }
                    auto& acc = bins[bin_key(rb, ab, eb)];
                    acc.sum_range += range;
                    acc.sum_az    += azimuth;
                    acc.sum_el    += elevation;
                    acc.sum_atten += attenuation_dB;
                    ++acc.count;
                    if (!gt_name.empty()) {
                        // name_votes is a small flat vector — usually 1-2
                        // unique names per bin, so linear scan is faster
                        // than the unordered_map allocation per bin per scan
                        // (PR #704 perf review P1).
                        bool found = false;
                        for (auto& [n, v] : acc.name_votes) {
                            if (n == gt_name) {
                                ++v;
                                found = true;
                                break;
                            }
                        }
                        if (!found) acc.name_votes.emplace_back(std::string(gt_name), 1u);
                    }
                }

                // Emit one detection per non-empty bin (centroid).  Skip bins
                // whose dominant name matches the exclude list.
                size_t skipped_filter = 0;
                for (const auto& [key, acc] : bins) {
                    if (list.num_detections >= drone::ipc::MAX_RADAR_DETECTIONS) break;
                    if (acc.count == 0) continue;

                    // Pick dominant groundtruth name (most votes).
                    std::string_view dominant_name;
                    uint32_t         best_votes = 0;
                    for (const auto& [n, v] : acc.name_votes) {
                        if (v > best_votes) {
                            best_votes    = v;
                            dominant_name = n;
                        }
                    }
                    if (filter_.is_excluded(dominant_name)) {
                        ++skipped_filter;
                        continue;
                    }

                    const float inv = 1.0f / static_cast<float>(acc.count);
                    drone::ipc::RadarDetection det{};
                    det.timestamp_ns        = list.timestamp_ns;
                    det.range_m             = acc.sum_range * inv;
                    det.azimuth_rad         = acc.sum_az * inv;
                    det.elevation_rad       = acc.sum_el * inv;
                    det.radial_velocity_mps = 0.0f;  // Echo doesn't provide Doppler
                    // SNR derived from attenuation: less attenuation = stronger return.
                    // attenuation_dB is negative or zero in Cosys; SNR = -mean_attenuation
                    // gives a positive signal-strength-style metric.  isfinite guard
                    // is belt-and-braces — per-point NaN/Inf already filters above
                    // (PR #704 security review).
                    const float mean_atten = acc.sum_atten * inv;
                    const float snr_db     = std::isfinite(mean_atten)
                                                 ? std::max(0.0f, -mean_atten)
                                                 : 0.0f;
                    det.snr_db              = snr_db;
                    det.confidence          = std::clamp(snr_db / 30.0f, 0.0f, 1.0f);
                    det.rcs_dbsm            = 0.0f;
                    det.track_id            = list.num_detections + 1;
                    list.detections[list.num_detections++] = det;
                }

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!active_.load(std::memory_order_acquire)) break;
                    cached_detections_ = list;
                }

                const uint64_t scan = scan_count_.fetch_add(1, std::memory_order_release) + 1;
                // Two summary cadences: a one-shot loud log on the first
                // non-zero emission (so the operator can see exactly when
                // Echo starts producing returns vs. silence on takeoff
                // pose), and a periodic 1 Hz summary thereafter.
                if (scan == 1) {
                    DRONE_LOG_INFO(
                        "[CosysEcho] First scan: {} raw → {} in FOV ({} dropped on range, "
                        "{} non-finite), {} unique bins → {} emitted ({} skipped by name filter)",
                        raw, in_fov, dropped_range, dropped_nonfinite, bins.size(),
                        list.num_detections, skipped_filter);
                } else if (raw > 0 && !first_nonzero_logged_) {
                    DRONE_LOG_INFO(
                        "[CosysEcho] First NON-ZERO scan #{}: {} raw → {} in FOV → "
                        "{} bins → {} emitted ({} skipped)",
                        scan, raw, in_fov, bins.size(), list.num_detections, skipped_filter);
                    first_nonzero_logged_ = true;
                } else if (scan % 20 == 0) {
                    DRONE_LOG_INFO(
                        "[CosysEcho] scan #{}: {} raw → {} in FOV → {} bins → {} emitted "
                        "({} skipped)",
                        scan, raw, in_fov, bins.size(), list.num_detections, skipped_filter);
                }
            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysEcho] RPC error: {}", e.what());
            }
            std::this_thread::sleep_for(poll_interval);
        }
        DRONE_LOG_INFO("[CosysEcho] Polling thread stopped");
    }

    static constexpr int kBinIndexMin = -32768;
    static constexpr int kBinIndexMax = 32767;

    std::shared_ptr<CosysRpcClient> client_;
    std::string                     echo_name_;
    std::string                     vehicle_name_;
    float                           max_range_m_;
    float                           min_range_m_;
    float                           fov_azimuth_rad_;
    float                           fov_elevation_rad_;
    int                             update_rate_hz_;
    float                           cluster_range_m_;
    float                           cluster_az_rad_;
    float                           cluster_el_rad_;
    CosysNameFilter                 filter_;

    mutable std::mutex             mutex_;
    drone::ipc::RadarDetectionList cached_detections_{};
    std::atomic<bool>              active_{false};
    std::atomic<uint64_t>          scan_count_{0};
    /// poll thread only — no synchronisation needed.
    bool                           first_nonzero_logged_ = false;
    bool                           partial_point_warned_ = false;
    std::thread                    poll_thread_;
    std::once_flag                 shutdown_once_;
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
