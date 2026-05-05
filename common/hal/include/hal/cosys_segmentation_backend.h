// common/hal/include/hal/cosys_segmentation_backend.h
// Cosys-AirSim ground-truth instance-segmentation backend.
// Implements IInferenceBackend by retrieving a per-pixel object-ID image
// from the simulator (ImageType::Segmentation) and emitting one
// InferenceDetection per visible object.  Each detection's pixel mask is
// pixel-perfect because it comes directly from the renderer — no ML
// inference, no SAM mega-mask collapse, no over-/under-segmentation.
//
// This is the sim-only counterpart to FastSamInferenceBackend.  Used for
// scenarios where we want to validate the rest of the perception/planning
// stack without SAM mask-quality variance contaminating the result.
//
// Issue: #698 — diagnostic backend that replaces SAM with simulator
// ground truth, paired with cosys_depth backend for end-to-end
// ground-truth PATH A.
//
// Threading contract:
//   - infer() must be called from a single thread (SAM thread in P2).
//   - init() may be called from a different thread; `initialized_` is
//     `std::atomic<bool>` with release/acquire ordering so the SAM
//     thread observes a fully-published color_to_name_ map.
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_name_filter.h"
#include "hal/cosys_name_resolver.h"
#include "hal/cosys_rpc_client.h"
#include "hal/iinference_backend.h"
#include "util/config.h"
#include "util/ilogger.h"
#include "util/result.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
STRICT_MODE_ON

namespace drone::hal {

/// Cosys-AirSim instance-segmentation backend.
///
/// On init(): caches the simulator's color→object table via
/// simListInstanceSegmentationObjects() + simGetInstanceSegmentationColorMap().
/// On infer(): pulls one Segmentation image, scans pixels, groups by RGB
/// color, and emits one InferenceDetection per distinct color encountered.
/// Each detection's `mask` is the per-pixel binary occupancy of that object
/// (255 = pixel belongs to object, 0 = doesn't), `mask_width/_height` match
/// the segmentation image, and `bbox` is the tight pixel bounding box.
///
/// Filtering: object names matching any of `exclude_substrings_` (default:
/// "Ground", "SkyDome", "Sky", "Floor", and the drone vehicle name) are
/// suppressed so they don't promote into the obstacle layer.
///
/// Class-ID assignment: every emitted detection gets `class_id =
/// obstacle_class_id_` (default 0 = GEOMETRIC_OBSTACLE).  Downstream
/// MaskDepthProjector treats this as the semantic label.
///
/// Config keys (with precedence: `<section>.<subkey>` → top-level → default):
///   `<section>.camera_name`        → `cosys_airsim.camera_name`  (default "front_center")
///   `<section>.vehicle_name`       → `cosys_airsim.vehicle_name` (default "Drone0")
///   `<section>.obstacle_class_id`  default 0 (GEOMETRIC_OBSTACLE)
///   `<section>.exclude_substrings` default ["Ground","Sky","SkyDome","Floor"]
class CosysSegmentationBackend : public IInferenceBackend {
public:
    explicit CosysSegmentationBackend(std::shared_ptr<CosysRpcClient> client,
                                      const drone::Config& cfg, const std::string& section)
        : client_(std::move(client))
        , camera_name_(drone::hal::resolve_camera_name(cfg, section))
        , vehicle_name_(drone::hal::resolve_vehicle_name(cfg, section))
        , obstacle_class_id_(cfg.get<int>(section + ".obstacle_class_id", 0))
        , min_pixels_(cfg.get<int>(section + ".min_pixels", 25))
        , filter_(cfg, section,
                  /*default_excludes=*/"Ground,Sky,SkyDome,Floor",
                  /*vehicle_name=*/vehicle_name_,
                  /*unknown_action=*/CosysNameFilterUnknown::Drop) {
        // Filter modes (priority: include > exclude):
        //   - allowlist (`include_substrings`): keep only matching names.
        //     Unknown-color pixels are dropped — they cannot satisfy any
        //     include.
        //   - blocklist (`exclude_substrings`): drop named background only.
        //     Unknown-color pixels are ALSO dropped here (Drop policy).
        //     This is the safe-by-default behaviour — a partial color map
        //     would otherwise paint every unmapped pixel as an obstacle,
        //     producing false-positive flight-critical hits (Copilot
        //     review on PR #704).  The vehicle name is always added to
        //     the blocklist.
        DRONE_LOG_INFO(
            "[CosysSegmentation] Created for {} camera='{}' vehicle='{}' "
            "obstacle_class_id={} min_pixels={} mode={} (include={} exclude={})",
            client_->endpoint(), camera_name_, vehicle_name_, obstacle_class_id_, min_pixels_,
            filter_.has_allowlist() ? "allowlist" : "blocklist",
            filter_.include_size(), filter_.exclude_size());
    }

    // Non-copyable, non-movable (owns RPC client reference and ID cache)
    CosysSegmentationBackend(const CosysSegmentationBackend&)            = delete;
    CosysSegmentationBackend& operator=(const CosysSegmentationBackend&) = delete;
    CosysSegmentationBackend(CosysSegmentationBackend&&)                 = delete;
    CosysSegmentationBackend& operator=(CosysSegmentationBackend&&)      = delete;

    /// Build the color→object table by querying the simulator once.
    /// `model_path` and `input_size` are accepted for interface compatibility
    /// and ignored — there is no model to load.
    [[nodiscard]] bool init(const std::string& /*model_path*/, int /*input_size*/) override {
        if (initialized_.load(std::memory_order_acquire)) {
            DRONE_LOG_WARN("[CosysSegmentation] init() called twice — second call is a no-op");
            return false;
        }
        try {
            std::vector<std::string>            names;
            std::vector<msr::airlib::Vector3r>  colors;
            const bool got = client_->with_client([&](auto& rpc) {
                names  = rpc.simListInstanceSegmentationObjects();
                colors = rpc.simGetInstanceSegmentationColorMap();
            });
            if (!got) {
                DRONE_LOG_ERROR("[CosysSegmentation] init() — RPC client not connected");
                return false;
            }
            if (names.size() != colors.size()) {
                DRONE_LOG_WARN(
                    "[CosysSegmentation] init() — name/color size mismatch ({} vs {}); "
                    "table will be partial",
                    names.size(), colors.size());
            }
            const size_t n = std::min(names.size(), colors.size());
            color_to_name_.clear();
            color_to_name_.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                // Cosys returns colors as Vector3r (R, G, B in [0, 255] float).
                // Pack to a 24-bit key; clamp to handle slight float drift.
                const uint32_t r = clamp_u8(colors[i].x());
                const uint32_t g = clamp_u8(colors[i].y());
                const uint32_t b = clamp_u8(colors[i].z());
                const uint32_t key = (r << 16) | (g << 8) | b;
                color_to_name_.emplace(key, names[i]);
            }
            DRONE_LOG_INFO("[CosysSegmentation] Cached {} object↔color mappings from simulator",
                           color_to_name_.size());
            // Release-store publishes color_to_name_ contents to any
            // thread that later acquire-loads `initialized_` in infer().
            initialized_.store(true, std::memory_order_release);
            return true;
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[CosysSegmentation] init() — RPC error: {}", e.what());
            return false;
        }
    }

    [[nodiscard]] drone::util::Result<InferenceOutput, std::string> infer(
        const uint8_t* /*frame_data*/, uint32_t /*width*/, uint32_t /*height*/,
        uint32_t /*channels*/, uint32_t /*stride*/) override {
        // P2 passes a Scene image as `frame_data`; we ignore it and fetch a
        // fresh Segmentation image via RPC.  The two come from different
        // simGetImages calls (RGB published by P1, segmentation here), so
        // they may correspond to different simulator ticks.  Frame-perfect
        // alignment would require P1 to publish both Scene + Segmentation
        // in one simGetImages call (deferred — tracked in IMPROVEMENTS.md).
        // The output is timestamped from the segmentation response so
        // downstream knows when the masks were captured (Copilot review
        // on PR #704).
        using R = drone::util::Result<InferenceOutput, std::string>;
        if (!initialized_.load(std::memory_order_acquire)) {
            return R::err("CosysSegmentation: backend not initialised — call init() first");
        }
        try {
            using namespace msr::airlib;
            std::vector<ImageCaptureBase::ImageResponse> responses;
            const bool got = client_->with_client([&](auto& rpc) {
                std::vector<ImageCaptureBase::ImageRequest> requests = {ImageCaptureBase::ImageRequest(
                    camera_name_, ImageCaptureBase::ImageType::Segmentation,
                    /* pixels_as_float */ false, /* compress */ false)};
                responses = rpc.simGetImages(requests, vehicle_name_);
            });
            if (!got) return R::err("CosysSegmentation: RPC client not connected");
            if (responses.empty()) return R::err("CosysSegmentation: empty image response");

            const auto& resp = responses[0];
            if (resp.image_data_uint8.empty() || resp.width <= 0 || resp.height <= 0) {
                return R::err("CosysSegmentation: invalid segmentation image");
            }

            const uint32_t img_w = static_cast<uint32_t>(resp.width);
            const uint32_t img_h = static_cast<uint32_t>(resp.height);
            const size_t   total = static_cast<size_t>(img_w) * img_h;
            // Cosys delivers compress=false RGB as 3 bytes per pixel.
            if (resp.image_data_uint8.size() < total * 3) {
                return R::err("CosysSegmentation: pixel buffer shorter than W*H*3 (got " +
                              std::to_string(resp.image_data_uint8.size()) + ", expected " +
                              std::to_string(total * 3) + ")");
            }
            const uint8_t* px = resp.image_data_uint8.data();

            // ── Pass 1: scan, group pixels by color, build per-color mask + bbox ──
            // by_color_pool_ is a class member: clear() preserves the bucket
            // array (avoids per-frame allocation of the unordered_map's
            // internal storage — PR #704 perf review P3).  The ObjAccum
            // mask buffers are moved out into the InferenceOutput at the
            // bottom and don't survive across frames; reusing them across
            // frames would require a separate mask_pool_, deferred to a
            // follow-up perf PR.
            by_color_pool_.clear();

            for (uint32_t y = 0; y < img_h; ++y) {
                for (uint32_t x = 0; x < img_w; ++x) {
                    const size_t  i   = (static_cast<size_t>(y) * img_w + x) * 3;
                    const uint8_t r   = px[i + 0];
                    const uint8_t g   = px[i + 1];
                    const uint8_t b   = px[i + 2];
                    const uint32_t key = (static_cast<uint32_t>(r) << 16) |
                                         (static_cast<uint32_t>(g) << 8) | b;

                    auto it = by_color_pool_.find(key);
                    if (it == by_color_pool_.end()) {
                        ObjAccum acc{};
                        const auto name_it = color_to_name_.find(key);
                        if (name_it != color_to_name_.end()) acc.name = name_it->second;
                        acc.excluded = filter_.is_excluded(acc.name);
                        if (!acc.excluded) acc.mask.assign(total, 0);
                        it = by_color_pool_.emplace(key, std::move(acc)).first;
                    }
                    auto& acc = it->second;
                    if (acc.excluded) continue;

                    const size_t pix = static_cast<size_t>(y) * img_w + x;
                    acc.mask[pix]    = 255;
                    if (x < acc.min_x) acc.min_x = x;
                    if (x > acc.max_x) acc.max_x = x;
                    if (y < acc.min_y) acc.min_y = y;
                    if (y > acc.max_y) acc.max_y = y;
                    ++acc.pixel_count;
                }
            }

            // ── Pass 2: emit one InferenceDetection per non-excluded object ──
            InferenceOutput output;
            // Timestamp clamp: cast through max(int64{0}, …) to guard against
            // negative steady_clock readings on platforms where the rep is
            // signed.  Mirrors the radar-backend pattern (PR #704 memory-
            // safety review).
            const auto now_ns_signed =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
            output.timestamp_ns =
                static_cast<uint64_t>(std::max(decltype(now_ns_signed){0}, now_ns_signed));
            output.detections.reserve(by_color_pool_.size());
            for (auto& [key, acc] : by_color_pool_) {
                if (acc.excluded) continue;
                if (acc.pixel_count < static_cast<uint32_t>(min_pixels_)) continue;

                InferenceDetection det{};
                det.bbox.x      = static_cast<float>(acc.min_x);
                det.bbox.y      = static_cast<float>(acc.min_y);
                det.bbox.w      = static_cast<float>(acc.max_x - acc.min_x + 1);
                det.bbox.h      = static_cast<float>(acc.max_y - acc.min_y + 1);
                det.class_id    = obstacle_class_id_;
                det.confidence  = 1.0f;  // ground truth
                det.mask        = std::move(acc.mask);
                det.mask_width  = img_w;
                det.mask_height = img_h;
                output.detections.push_back(std::move(det));
            }

            // Diagnostics: first frame logs full breakdown so the user can
            // see which colors actually appeared and which got filtered.
            // Subsequent frames log only a one-liner every 100 calls.
            ++infer_count_;
            if (infer_count_ == 1) {
                size_t excluded_n = 0, unknown_n = 0;
                for (const auto& [k, a] : by_color_pool_) {
                    if (a.excluded) ++excluded_n;
                    if (a.name.empty()) ++unknown_n;
                }
                DRONE_LOG_INFO(
                    "[CosysSegmentation] First infer: image {}x{}, {} unique colors "
                    "(named={}, unknown={}) → {} detections kept, {} excluded "
                    "(min_pixels gate={})",
                    img_w, img_h, by_color_pool_.size(), by_color_pool_.size() - unknown_n, unknown_n,
                    output.detections.size(), excluded_n, min_pixels_);
                // First frame: list the up-to-12 largest detections by name + pixel count
                // so we can verify the include/exclude filtering hit the right objects.
                std::vector<std::pair<std::string, uint32_t>> kept;
                for (const auto& [k, a] : by_color_pool_) {
                    if (!a.excluded && a.pixel_count >= static_cast<uint32_t>(min_pixels_)) {
                        kept.emplace_back(a.name.empty() ? "<unknown>" : a.name, a.pixel_count);
                    }
                }
                std::sort(kept.begin(), kept.end(),
                          [](const auto& a, const auto& b) { return a.second > b.second; });
                const size_t show = std::min<size_t>(12, kept.size());
                for (size_t i = 0; i < show; ++i) {
                    DRONE_LOG_INFO("[CosysSegmentation]   #{} '{}' — {} px", i + 1, kept[i].first,
                                   kept[i].second);
                }
            } else if (infer_count_ % 100 == 0) {
                DRONE_LOG_INFO("[CosysSegmentation] infer #{}: {} unique colors → {} detections",
                               infer_count_, by_color_pool_.size(), output.detections.size());
            }
            return R::ok(std::move(output));
        } catch (const std::exception& e) {
            return drone::util::Result<InferenceOutput, std::string>::err(
                std::string("CosysSegmentation: RPC error — ") + e.what());
        }
    }

    [[nodiscard]] std::string name() const override {
        return "CosysSegmentation(" + camera_name_ + "@" + client_->endpoint() + ")";
    }

private:
    static uint32_t clamp_u8(float v) {
        const float clamped = std::clamp(v, 0.0f, 255.0f);
        return static_cast<uint32_t>(clamped + 0.5f);  // round to nearest
    }

    struct ObjAccum {
        std::vector<uint8_t> mask;          // size W*H, 255 where pixel matches
        uint32_t             min_x = UINT32_MAX, min_y = UINT32_MAX;
        uint32_t             max_x = 0, max_y = 0;
        uint32_t             pixel_count = 0;
        bool                 excluded    = false;
        std::string          name;
    };

    std::shared_ptr<CosysRpcClient> client_;
    std::string                     camera_name_;
    std::string                     vehicle_name_;
    int                             obstacle_class_id_;
    int                             min_pixels_;
    CosysNameFilter                 filter_;
    std::unordered_map<uint32_t, std::string> color_to_name_;  ///< RGB-key → object name
    /// Pooled across calls — clear() preserves the bucket array.
    std::unordered_map<uint32_t, ObjAccum> by_color_pool_;
    std::atomic<bool>                      initialized_{false};
    uint64_t                               infer_count_ = 0;
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
