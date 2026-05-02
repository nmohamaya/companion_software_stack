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
// elevation > +30 degrees in NED (positive elevation = looking down at the
// ground; the body-frame Z-axis points down, so asin(z/range) is positive
// for ground returns).  PR #679 Copilot review aligned this with the code.
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
#include <cstdint>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <thread>
#include <unordered_map>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// CosysRadarBackend — emulates radar using LiDAR point cloud data
/// from Cosys-AirSim via the getLidarData() RPC endpoint.
///
/// The polling thread runs at a fixed 20 Hz (kPollInterval = 50 ms)
/// and reduces a 1000+-point LiDAR scan to ~10–30 RadarDetections via
/// a four-stage pipeline:
///
///   1. **FOV gate** — keep only points within ±fov_azimuth and
///      ±fov_elevation (forward cone, automotive-radar style;
///      defaults ±60° × ±15°).
///   2. **Range gate** — drop |r| < min_range_m or > max_range_m.
///   3. **Ground filter** — drop steep downward returns
///      (elevation > +30° in NED, computed from `asin(z / range)`).
///   4. **Clustering** — bin (range, azimuth, elevation) into
///      cluster_range × cluster_az × cluster_el cells; emit one
///      RadarDetection per non-empty cell at the centroid.  Bin
///      indices are clamped into a 16-bit signed range to fit the
///      packed 64-bit key (`bin_key`); points outside that range are
///      dropped (radar realism: only the central cone matters).
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
    // PR #636 P2 review: lift the magic π out of the call sites
    // (was repeated 8 times across the ctor / log lines) and give it
    // typed constants for radians ↔ degrees conversion.
    static constexpr float kPi       = 3.14159265358979323846f;
    static constexpr float kDegToRad = kPi / 180.0f;
    static constexpr float kRadToDeg = 180.0f / kPi;

    // PR #636 P2 review: positive-floor on cluster bin sizes — a config
    // value of 0 (typo, tampering, or future "disable clustering" flag
    // that landed without guard) would trigger div-by-zero inside the
    // per-point binning loop.  1 mm range / 1 µrad angular is well
    // below any realistic radar-bin granularity yet keeps the floor
    // away from denormals.
    static constexpr float kMinClusterBinM   = 1e-3f;
    static constexpr float kMinClusterBinRad = 1e-6f;

    // PR #636 P2 review: cluster bin index range is clamped into a
    // 16-bit signed window to fit the packed 64-bit `bin_key`.  At the
    // default 1 m range bin × 5° angular bin this covers ±32 km range
    // and ±163° azimuth — well past any realistic radar gate.  Returns
    // outside this window are dropped.
    static constexpr int kBinIndexMin = -32768;
    static constexpr int kBinIndexMax = 32767;

    // PR #636 P2 review: bin accumulator hoisted out of the polling-
    // loop body (was a function-local struct, harder to test/inspect
    // and fights template / debugger names).
    struct ClusterAcc {
        float    sum_range = 0.0f;
        float    sum_az    = 0.0f;
        float    sum_el    = 0.0f;
        uint32_t count     = 0;
    };


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
        , max_range_m_(cfg.get<float>(section + drone::cfg_key::hal::MAX_RANGE_M, 100.0f))
        // Issue #635 — radar realism knobs.  Defaults model a forward-looking
        // automotive-style radar (±60° azimuth, ±15° elevation, 0.5–50 m
        // range).  The previous behaviour (full-360° lidar passing through
        // unchanged) was clipped to the first 128 raw points by
        // MAX_RADAR_DETECTIONS, dropping ~87 % of returns and producing a
        // narrow contiguous arc that didn't cover real obstacles.
        // PR #679 Copilot review: clamp `min_range_m_` to a small positive
        // epsilon.  A LiDAR point at the origin (0, 0, 0) yields
        // `range == 0`; if min_range_m_ is also 0 (or negative) the
        // gate `range < min_range_m_` lets it through, then `z / range`
        // → NaN propagates into asin/atan2 and bin-index `floor(NaN/...)`
        // is UB.  10 cm is well below any realistic radar near-field
        // and prevents the degenerate config from taking down P2.
        , min_range_m_(
              std::max(0.1f, cfg.get<float>(section + drone::cfg_key::hal::MIN_RANGE_M, 0.5f)))
        , fov_azimuth_rad_(
              cfg.get<float>(section + drone::cfg_key::hal::FOV_AZIMUTH_RAD, 60.0f * kDegToRad))
        , fov_elevation_rad_(
              cfg.get<float>(section + drone::cfg_key::hal::FOV_ELEVATION_RAD, 15.0f * kDegToRad))
        // Cluster bin sizes — collapse nearby raw lidar points to one
        // detection per ~obstacle.  1 m × 5° × 5° gives ~1.7 m angular
        // separation at 20 m range; matches scenario-33 cube/pillar size.
        // PR #636 P2 review: clamp configured bin sizes to a positive
        // floor so a tampered config with `cluster_range_m: 0` cannot
        // div-by-zero in the per-point binning loop.
        , cluster_range_m_(
              std::max(kMinClusterBinM,
                       cfg.get<float>(section + drone::cfg_key::hal::CLUSTER_RANGE_M, 1.0f)))
        , cluster_az_rad_(std::max(
              kMinClusterBinRad,
              cfg.get<float>(section + drone::cfg_key::hal::CLUSTER_AZIMUTH_RAD, 5.0f * kDegToRad)))
        , cluster_el_rad_(
              std::max(kMinClusterBinRad,
                       cfg.get<float>(section + drone::cfg_key::hal::CLUSTER_ELEVATION_RAD,
                                      5.0f * kDegToRad))) {
        DRONE_LOG_INFO("[CosysRadar] Created for {} radar='{}' vehicle='{}' "
                       "range=[{:.1f},{:.1f}]m FOV=±{:.0f}°×±{:.0f}° "
                       "cluster={:.1f}m×{:.0f}°×{:.0f}°",
                       client_->endpoint(), radar_name_, vehicle_name_, min_range_m_, max_range_m_,
                       fov_azimuth_rad_ * kRadToDeg, fov_elevation_rad_ * kRadToDeg,
                       cluster_range_m_, cluster_az_rad_ * kRadToDeg, cluster_el_rad_ * kRadToDeg);
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
        // PR #636 P2 review: π pulled from class-level kDegToRad constant.
        constexpr float kGroundElevationThreshold = 30.0f * kDegToRad;
        // Simplified radar equation: SNR = ref_snr - path_loss * log10(range)
        constexpr float kReferenceSNRdB    = 30.0f;  // SNR at 1m range
        constexpr float kSNRPathLossFactor = 20.0f;  // Free-space path loss exponent

        DRONE_LOG_INFO("[CosysRadar] Polling thread started (interval={}ms)",
                       kPollInterval.count());

        while (active_.load(std::memory_order_acquire)) {
            try {
                // Use with_client() to prevent TOCTOU race on disconnect
                // Note: AirSim does not have a native RadarData type, so we emulate
                // radar from LiDAR point cloud (vehicle frame, NED convention).
                msr::airlib::LidarData lidar_data;
                bool                   got_data = client_->with_client([&](auto& rpc) {
                    lidar_data =
                        rpc.getLidarData(radar_name_,  // sensor name (typically "Radar" or "Lidar")
                                                           vehicle_name_);
                });
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

                // Issue #635 — radar realism pass.
                // Old behaviour (#434): treat every lidar point as a radar
                // detection, take the first MAX_RADAR_DETECTIONS in scan
                // order, drop the rest.  With Cosys lidar at 1000 pts/scan
                // this kept ~13 % of returns in a contiguous angular slice
                // and hid real obstacles outside that arc (scenario-33 run
                // 2026-04-27_135747: tracks clustered at one azimuth
                // direction, missed pillars at (8,12), (15,10), (22,14)).
                //
                // New pipeline:
                //   1. FOV gate — keep only points within ±fov_azimuth and
                //      ±fov_elevation (forward cone, automotive-radar
                //      style; defaults ±60° × ±15°).
                //   2. Range gate — drop |r| < min_range or > max_range.
                //   3. Ground filter — drop steep-downward returns.
                //   4. Cluster — bin (range, azimuth, elevation) into
                //      cluster_range × cluster_az × cluster_el cells; emit
                //      one RadarDetection per non-empty cell at the
                //      centroid.  Collapses ~1000 raw points to ~10–30
                //      tracks, each representing one physical obstacle.
                //
                // PR #636 P2 review: ClusterAcc lifted to class scope —
                // see `struct ClusterAcc` above.  Bin index validity
                // checked against [kBinIndexMin, kBinIndexMax] before
                // packing so the 16-bit truncation can't silently
                // collide bins from far-apart points.
                auto bin_key = [](int rb, int ab, int eb) -> uint64_t {
                    return (static_cast<uint64_t>(static_cast<uint32_t>(rb) & 0xFFFFu) << 32) |
                           (static_cast<uint64_t>(static_cast<uint32_t>(ab) & 0xFFFFu) << 16) |
                           static_cast<uint64_t>(static_cast<uint32_t>(eb) & 0xFFFFu);
                };
                std::unordered_map<uint64_t, ClusterAcc> bins;

                size_t raw    = 0;
                size_t in_fov = 0;

                for (size_t i = 0; i + 2 < lidar_data.point_cloud.size(); i += 3) {
                    ++raw;
                    const float x = lidar_data.point_cloud[i];
                    const float y = lidar_data.point_cloud[i + 1];
                    const float z = lidar_data.point_cloud[i + 2];
                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                    const float range = std::sqrt(x * x + y * y + z * z);
                    // PR #679 Copilot review: belt-and-braces — even with
                    // the construction-time `min_range_m_` floor, a
                    // literal range==0 (LiDAR point at vehicle origin)
                    // would make `z / range` produce NaN in asin below.
                    // Explicit gate so the NaN-propagation path is closed.
                    if (range <= 0.0f || range < min_range_m_ || range > max_range_m_) continue;
                    const float azimuth   = std::atan2(y, x);
                    const float elevation = std::asin(std::clamp(z / range, -1.0f, 1.0f));

                    // Ground filter: in NED (Z=down), ground returns have
                    // positive elevation from the drone's perspective.
                    if (elevation > kGroundElevationThreshold) continue;

                    // Forward-cone FOV gate.  azimuth ∈ [-π, π] from
                    // atan2; |az| > fov_azimuth_rad_ → outside cone.
                    if (std::abs(azimuth) > fov_azimuth_rad_) continue;
                    if (std::abs(elevation) > fov_elevation_rad_) continue;
                    ++in_fov;

                    // Bin into cluster cell (signed indices keep the
                    // origin sensible for negative az/el values).
                    // PR #636 P2 review: divisor floor (kMinClusterBinM /
                    // kMinClusterBinRad) enforced at construction
                    // — no div-by-zero possible here.
                    const int rb = static_cast<int>(std::floor(range / cluster_range_m_));
                    const int ab = static_cast<int>(std::floor(azimuth / cluster_az_rad_));
                    const int eb = static_cast<int>(std::floor(elevation / cluster_el_rad_));
                    // PR #636 P2 review: drop returns whose bin index
                    // falls outside the 16-bit signed window the
                    // packed `bin_key` represents — otherwise far-apart
                    // points would silently collide into the same bin
                    // via wraparound truncation.
                    if (rb < kBinIndexMin || rb > kBinIndexMax || ab < kBinIndexMin ||
                        ab > kBinIndexMax || eb < kBinIndexMin || eb > kBinIndexMax) {
                        continue;
                    }
                    auto& acc = bins[bin_key(rb, ab, eb)];
                    acc.sum_range += range;
                    acc.sum_az += azimuth;
                    acc.sum_el += elevation;
                    // PR #636 P2 review: SNR computation deferred to
                    // the per-bin emit phase.  std::log10 here was
                    // called O(in_fov) times — hundreds per scan; once
                    // per emitted bin (~10–30/scan) is enough since
                    // the centroid range is what we report anyway.
                    ++acc.count;
                }

                // Emit one RadarDetection per non-empty bin (centroid).
                for (const auto& [key, acc] : bins) {
                    if (list.num_detections >= drone::ipc::MAX_RADAR_DETECTIONS) break;
                    if (acc.count == 0) continue;
                    const float inv = 1.0f / static_cast<float>(acc.count);

                    drone::ipc::RadarDetection det{};
                    det.timestamp_ns  = list.timestamp_ns;
                    det.range_m       = acc.sum_range * inv;
                    det.azimuth_rad   = acc.sum_az * inv;
                    det.elevation_rad = acc.sum_el * inv;
                    // PR #636 P2 review: SNR computed once per emitted
                    // detection (was per per-FOV-filtered point) using
                    // the centroid range — same value, far fewer
                    // log10 calls.
                    const float snr_db = std::max(
                        0.0f, kReferenceSNRdB -
                                  kSNRPathLossFactor * std::log10(std::max(1.0f, det.range_m)));
                    det.radial_velocity_mps = 0.0f;  // lidar gives no doppler
                    det.snr_db              = snr_db;
                    det.confidence          = std::clamp(snr_db / kReferenceSNRdB, 0.0f, 1.0f);
                    det.rcs_dbsm            = 0.0f;
                    det.track_id            = list.num_detections + 1;

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
                    DRONE_LOG_INFO("[CosysRadar] First scan: {} raw → {} in FOV → {} clusters", raw,
                                   in_fov, list.num_detections);
                }
                // Periodic diagnostic so a future cap-saturation / FOV-too-
                // narrow regression surfaces in logs without needing a debug
                // session — the lack of this exact diagnostic is what hid
                // the pre-#635 87 %-drop bug.
                if (count > 0 && count % 200 == 0) {
                    DRONE_LOG_INFO("[CosysRadar] scan #{}: {} raw → {} in FOV → {} clusters", count,
                                   raw, in_fov, list.num_detections);
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
    // PR #636 P2 review: default member initializers added.  Members
    // are still set in the ctor initialiser list from config — these
    // defaults guarantee a valid value if a future ctor path forgets
    // a field, and document the "expected order of magnitude" inline.
    std::string radar_name_;           ///< AirSim LiDAR sensor name (used as radar proxy)
    std::string vehicle_name_;         ///< AirSim vehicle name
    float       max_range_m_{100.0f};  ///< Maximum detection range (m)
    // Issue #635 — radar FOV + clustering knobs.  See ctor for defaults
    // and the "behave like a radar" rationale (forward cone + clustered
    // detections instead of raw 360° lidar dump).
    float min_range_m_{0.5f};                     ///< Minimum range (m); drops near returns
    float fov_azimuth_rad_{60.0f * kDegToRad};    ///< Half-FOV in azimuth (kept |az| <= this)
    float fov_elevation_rad_{15.0f * kDegToRad};  ///< Half-FOV in elevation (kept |el| <= this)
    float cluster_range_m_{1.0f};                 ///< Range bin size for clustering (m)
    float cluster_az_rad_{5.0f * kDegToRad};      ///< Azimuth bin size for clustering (rad)
    float cluster_el_rad_{5.0f * kDegToRad};      ///< Elevation bin size for clustering (rad)

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
