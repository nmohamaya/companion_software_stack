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
// NOT thread-safe for infer() — call from a single thread (SAM thread in P2).
#pragma once
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_name_resolver.h"
#include "hal/cosys_rpc_client.h"
#include "hal/iinference_backend.h"
#include "util/config.h"
#include "util/ilogger.h"
#include "util/result.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
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
        , min_pixels_(cfg.get<int>(section + ".min_pixels", 25)) {
        // Two filtering modes (priority: include > exclude):
        //   - include_substrings: allowlist.  If non-empty, ONLY objects
        //     whose name contains one of these substrings are emitted as
        //     obstacles.  Use this when the scenario knows exactly what
        //     to detect (e.g. "TemplateCube,Cube" for scenario 33's
        //     default Blocks scene).  Unknown-color pixels (not in the
        //     cached map) are skipped — won't satisfy any include match.
        //   - exclude_substrings: blocklist (only consulted when
        //     include_substrings is empty).  Drops named background.
        //     Unknown-color pixels are KEPT as obstacles by default so
        //     a partial color map doesn't silently zero out PATH A.
        //     The vehicle name is always added to the blocklist.
        include_substrings_ = parse_csv(
            cfg.get<std::string>(section + ".include_substrings", std::string{}));
        exclude_substrings_ = parse_csv(
            cfg.get<std::string>(section + ".exclude_substrings", "Ground,Sky,SkyDome,Floor"));
        exclude_substrings_.push_back(vehicle_name_);

        DRONE_LOG_INFO(
            "[CosysSegmentation] Created for {} camera='{}' vehicle='{}' "
            "obstacle_class_id={} min_pixels={} mode={} (include={} exclude={})",
            client_->endpoint(), camera_name_, vehicle_name_, obstacle_class_id_, min_pixels_,
            include_substrings_.empty() ? "blocklist" : "allowlist",
            include_substrings_.size(), exclude_substrings_.size());
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
            initialized_ = true;
            return true;
        } catch (const std::exception& e) {
            DRONE_LOG_ERROR("[CosysSegmentation] init() — RPC error: {}", e.what());
            return false;
        }
    }

    [[nodiscard]] drone::util::Result<InferenceOutput, std::string> infer(
        const uint8_t* /*frame_data*/, uint32_t /*width*/, uint32_t /*height*/,
        uint32_t /*channels*/, uint32_t /*stride*/) override {
        using R = drone::util::Result<InferenceOutput, std::string>;
        if (!initialized_) {
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
            struct ObjAccum {
                std::vector<uint8_t> mask;          // size W*H, 255 where pixel matches
                uint32_t             min_x = UINT32_MAX, min_y = UINT32_MAX;
                uint32_t             max_x = 0, max_y = 0;
                uint32_t             pixel_count = 0;
                bool                 excluded    = false;
                std::string          name;
            };
            std::unordered_map<uint32_t, ObjAccum> by_color;

            for (uint32_t y = 0; y < img_h; ++y) {
                for (uint32_t x = 0; x < img_w; ++x) {
                    const size_t  i   = (static_cast<size_t>(y) * img_w + x) * 3;
                    const uint8_t r   = px[i + 0];
                    const uint8_t g   = px[i + 1];
                    const uint8_t b   = px[i + 2];
                    const uint32_t key = (static_cast<uint32_t>(r) << 16) |
                                         (static_cast<uint32_t>(g) << 8) | b;

                    auto it = by_color.find(key);
                    if (it == by_color.end()) {
                        ObjAccum acc{};
                        const auto name_it = color_to_name_.find(key);
                        acc.name           = (name_it != color_to_name_.end()) ? name_it->second
                                                                               : std::string{};
                        acc.excluded       = is_excluded(acc.name);
                        if (!acc.excluded) {
                            acc.mask.assign(total, 0);
                        }
                        it = by_color.emplace(key, std::move(acc)).first;
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
            output.timestamp_ns = static_cast<uint64_t>(
                std::chrono::steady_clock::now().time_since_epoch().count());
            output.detections.reserve(by_color.size());
            for (auto& [key, acc] : by_color) {
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
                for (const auto& [k, a] : by_color) {
                    if (a.excluded) ++excluded_n;
                    if (a.name.empty()) ++unknown_n;
                }
                DRONE_LOG_INFO(
                    "[CosysSegmentation] First infer: image {}x{}, {} unique colors "
                    "(named={}, unknown={}) → {} detections kept, {} excluded "
                    "(min_pixels gate={})",
                    img_w, img_h, by_color.size(), by_color.size() - unknown_n, unknown_n,
                    output.detections.size(), excluded_n, min_pixels_);
                // First frame: list the up-to-12 largest detections by name + pixel count
                // so we can verify the include/exclude filtering hit the right objects.
                std::vector<std::pair<std::string, uint32_t>> kept;
                for (const auto& [k, a] : by_color) {
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
                               infer_count_, by_color.size(), output.detections.size());
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

    /// Decide whether this object name should be treated as an obstacle.
    /// Returns true ⇒ skip (excluded).  See constructor for filtering modes.
    [[nodiscard]] bool is_excluded(const std::string& obj_name) const {
        if (!include_substrings_.empty()) {
            // Allowlist mode: keep only names matching an include substring.
            // Empty name (unknown color) cannot match → exclude.
            if (obj_name.empty()) return true;
            for (const auto& sub : include_substrings_) {
                if (!sub.empty() && obj_name.find(sub) != std::string::npos) return false;
            }
            return true;
        }
        // Blocklist mode: drop named background only.  Unknown colors stay
        // in (better to over-detect than silently zero out PATH A).
        if (obj_name.empty()) return false;
        for (const auto& sub : exclude_substrings_) {
            if (!sub.empty() && obj_name.find(sub) != std::string::npos) return true;
        }
        return false;
    }

    /// Parse a comma-separated list, trimming whitespace, dropping empty tokens.
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
    std::string                     camera_name_;
    std::string                     vehicle_name_;
    int                             obstacle_class_id_;
    int                             min_pixels_;
    std::vector<std::string>        include_substrings_;
    std::vector<std::string>        exclude_substrings_;
    std::unordered_map<uint32_t, std::string> color_to_name_;  ///< RGB-key → object name
    bool                                      initialized_  = false;
    uint64_t                                  infer_count_  = 0;
};

}  // namespace drone::hal

#endif  // HAVE_COSYS_AIRSIM
