// process2_perception/include/perception/color_contour_detector.h
// Real detector: HSV color segmentation + connected-component labeling.
// Processes actual RGB camera frames — no external CV libraries.
//
// Optimisations (Issue #128):
//   1. Single-pass color classification: one RGB→HSV scan classifies all N color
//      ranges simultaneously, eliminating (N-1)/N of per-pixel HSV conversions.
//   2. Spatial subsampling: configurable stride (default 2) reduces pixel work by
//      subsample^2; bbox coordinates are scaled back to full resolution on output.
//   3. max_fps cap: optional sleep throttle to limit CPU use in non-critical paths.
#pragma once

#include "perception/detector_interface.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// HSV Color Range for a single target color
// ═══════════════════════════════════════════════════════════
struct HsvRange {
    float       h_min, h_max;  // Hue [0, 360)
    float       s_min, s_max;  // Saturation [0, 1]
    float       v_min, v_max;  // Value [0, 1]
    ObjectClass class_id;      // What class this color maps to
    std::string label;         // Human-readable name (for logging)

    bool contains(float h, float s, float v) const {
        if (s < s_min || s > s_max) return false;
        if (v < v_min || v > v_max) return false;
        // Handle hue wrap-around (e.g., red spans 350-10)
        if (h_min <= h_max) {
            return h >= h_min && h <= h_max;
        } else {
            return h >= h_min || h <= h_max;
        }
    }
};

// ═══════════════════════════════════════════════════════════
// RGB → HSV conversion (single pixel)
// ═══════════════════════════════════════════════════════════
struct HsvPixel {
    float h = 0.0f;  // [0, 360)
    float s = 0.0f;  // [0, 1]
    float v = 0.0f;  // [0, 1]
};

inline HsvPixel rgb_to_hsv(uint8_t r8, uint8_t g8, uint8_t b8) {
    float r = r8 / 255.0f;
    float g = g8 / 255.0f;
    float b = b8 / 255.0f;

    float cmax  = std::max({r, g, b});
    float cmin  = std::min({r, g, b});
    float delta = cmax - cmin;

    HsvPixel hsv;
    hsv.v = cmax;
    hsv.s = (cmax > 1e-6f) ? (delta / cmax) : 0.0f;

    if (delta < 1e-6f) {
        hsv.h = 0.0f;
    } else if (cmax == r) {
        hsv.h = 60.0f * std::fmod((g - b) / delta + 6.0f, 6.0f);
    } else if (cmax == g) {
        hsv.h = 60.0f * ((b - r) / delta + 2.0f);
    } else {
        hsv.h = 60.0f * ((r - g) / delta + 4.0f);
    }
    return hsv;
}

// ═══════════════════════════════════════════════════════════
// Connected-component labeling (union-find)
// ═══════════════════════════════════════════════════════════
class UnionFind {
public:
    explicit UnionFind(int n) : parent_(n), rank_(n, 0) {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    int find(int x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];  // path compression
            x          = parent_[x];
        }
        return x;
    }

    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (rank_[a] < rank_[b]) std::swap(a, b);
        parent_[b] = a;
        if (rank_[a] == rank_[b]) ++rank_[a];
    }

private:
    std::vector<int> parent_;
    std::vector<int> rank_;
};

// ═══════════════════════════════════════════════════════════
// Bounding box from connected component
// ═══════════════════════════════════════════════════════════
struct ComponentBBox {
    int x_min = 0, y_min = 0, x_max = 0, y_max = 0;
    int pixel_count = 0;

    int width() const { return x_max - x_min + 1; }
    int height() const { return y_max - y_min + 1; }
    int area() const { return pixel_count; }
};

// ═══════════════════════════════════════════════════════════
// ColorContourDetector
// ═══════════════════════════════════════════════════════════
class ColorContourDetector : public IDetector {
public:
    /// Sentinel stored in color_map_: pixel matched no color range.
    static constexpr uint8_t kNoColor = 0xFF;

    /// Construct with default color ranges (tuned for Gazebo obstacle colors).
    ColorContourDetector() { init_default_colors(); }

    /// Construct from config — reads HSV ranges, min_area, subsample and max_fps.
    explicit ColorContourDetector(const drone::Config& cfg) {
        min_contour_area_ = cfg.get<int>(drone::cfg_key::perception::detector::MIN_CONTOUR_AREA,
                                         80);
        max_detections_   = cfg.get<int>(drone::cfg_key::perception::detector::MAX_DETECTIONS, 20);
        subsample_ = std::max(1, cfg.get<int>(drone::cfg_key::perception::detector::SUBSAMPLE, 2));
        max_fps_   = cfg.get<int>(drone::cfg_key::perception::detector::MAX_FPS, 0);
        confidence_max_ =
            std::clamp(cfg.get<float>(drone::cfg_key::perception::detector::CONFIDENCE_MAX, 0.95f),
                       0.0f, 1.0f);
        confidence_base_ =
            std::clamp(cfg.get<float>(drone::cfg_key::perception::detector::CONFIDENCE_BASE, 0.5f),
                       0.0f, 1.0f);
        confidence_area_scale_ = std::max(
            0.0f,
            cfg.get<float>(drone::cfg_key::perception::detector::CONFIDENCE_AREA_SCALE, 50.0f));
        min_bbox_height_px_ =
            std::max(0, cfg.get<int>(drone::cfg_key::perception::detector::MIN_BBOX_HEIGHT_PX,
                                     min_bbox_height_px_));
        max_aspect_ratio_ =
            std::max(0.0f, cfg.get<float>(drone::cfg_key::perception::detector::MAX_ASPECT_RATIO,
                                          max_aspect_ratio_));

        // Try to read custom color ranges from config
        if (cfg.has(drone::cfg_key::perception::detector::COLORS)) {
            auto colors_json = cfg.section(drone::cfg_key::perception::detector::COLORS);
            for (const auto& [key, val] : colors_json.items()) {
                HsvRange range;
                range.h_min    = val.value("h_min", 0.0f);
                range.h_max    = val.value("h_max", 360.0f);
                range.s_min    = val.value("s_min", 0.3f);
                range.s_max    = val.value("s_max", 1.0f);
                range.v_min    = val.value("v_min", 0.3f);
                range.v_max    = val.value("v_max", 1.0f);
                range.class_id = static_cast<ObjectClass>(val.value("class_id", 0));
                range.label    = key;
                color_ranges_.push_back(range);
            }
        }

        // Fall back to defaults if no colors configured
        if (color_ranges_.empty()) {
            init_default_colors();
        }

        DRONE_LOG_INFO("[ColorContourDetector] {} color ranges, min_area={}, max_dets={}, "
                       "subsample={}, max_fps={}",
                       color_ranges_.size(), min_contour_area_, max_detections_, subsample_,
                       max_fps_);
    }

    std::vector<Detection2D> detect(const uint8_t* frame_data, uint32_t width, uint32_t height,
                                    uint32_t channels) override {
        if (!frame_data || width == 0 || height == 0 || channels < 3) {
            return {};
        }

        // Capture start time for optional max_fps throttle applied at the end of detect().
        const auto detect_start = std::chrono::steady_clock::now();

        const int w      = static_cast<int>(width);
        const int h      = static_cast<int>(height);
        const int stride = static_cast<int>(channels);
        // Subsampled grid dimensions (ceiling division).
        const int ws = (w + subsample_ - 1) / subsample_;
        const int hs = (h + subsample_ - 1) / subsample_;

        // Single pass: classify every subsampled pixel into a color bucket.
        build_color_map(frame_data, w, h, stride, ws, hs);

        auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::steady_clock::now().time_since_epoch())
                                                .count());

        std::vector<Detection2D> all_detections;

        // Cap at kNoColor (0xFF is the sentinel) so color indices always fit in uint8_t.
        const auto n_colors =
            static_cast<uint8_t>(std::min(color_ranges_.size(), static_cast<size_t>(kNoColor)));
        for (uint8_t ci = 0; ci < n_colors; ++ci) {
            auto bboxes = extract_bboxes(ws, hs, ci);
            for (const auto& bbox : bboxes) {
                if (bbox.area() < min_contour_area_) continue;

                // Clamp to image bounds: ceiling-division of ws/hs can push the last
                // subsampled cell's right/bottom edge beyond the actual frame extent.
                const int x_min_c = std::max(0, bbox.x_min);
                const int y_min_c = std::max(0, bbox.y_min);
                const int x_max_c = std::min(bbox.x_max, w - 1);
                const int y_max_c = std::min(bbox.y_max, h - 1);

                const int bbox_w = x_max_c - x_min_c + 1;
                const int bbox_h = y_max_c - y_min_c + 1;

                // Ground-feature rejection (Issue #345):
                // Reject detections that are too short — unreliable at distance.
                if (min_bbox_height_px_ > 0 && bbox_h < min_bbox_height_px_) continue;
                // Reject flat/wide detections — ground textures seen from above.
                if (max_aspect_ratio_ > 0.0f && bbox_h > 0 &&
                    static_cast<float>(bbox_w) / static_cast<float>(bbox_h) > max_aspect_ratio_) {
                    continue;
                }

                Detection2D det;
                det.x              = static_cast<float>(x_min_c);
                det.y              = static_cast<float>(y_min_c);
                det.w              = static_cast<float>(x_max_c - x_min_c + 1);
                det.h              = static_cast<float>(y_max_c - y_min_c + 1);
                det.class_id       = color_ranges_[ci].class_id;
                det.timestamp_ns   = now_ns;
                det.frame_sequence = 0;

                // Confidence based on area: larger objects → higher confidence.
                // Cast each dimension separately to float before multiplying to avoid
                // uint32_t overflow on large frames (e.g. > ~4K resolution).
                float area_ratio = static_cast<float>(bbox.area()) /
                                   (static_cast<float>(width) * static_cast<float>(height));
                det.confidence = std::min(confidence_max_,
                                          confidence_base_ + area_ratio * confidence_area_scale_);

                all_detections.push_back(det);
            }
        }

        // Sort by confidence (descending) and cap at max_detections.
        std::sort(
            all_detections.begin(), all_detections.end(),
            [](const Detection2D& a, const Detection2D& b) { return a.confidence > b.confidence; });
        if (static_cast<int>(all_detections.size()) > max_detections_) {
            all_detections.resize(max_detections_);
        }

        // Optional: sleep the remainder of the frame interval so the per-call CPU budget
        // is bounded.  Sleeping *after* detection ensures the elapsed detection time is
        // accounted for, so the effective call rate never exceeds max_fps.
        if (max_fps_ > 0) {
            const int64_t min_interval_ns = 1'000'000'000LL / max_fps_;
            const int64_t elapsed_ns      = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           std::chrono::steady_clock::now() - detect_start)
                                           .count();
            if (elapsed_ns < min_interval_ns) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(min_interval_ns - elapsed_ns));
            }
        }

        return all_detections;
    }

    std::string name() const override { return "ColorContourDetector"; }

    /// Expose color ranges for testing.
    const std::vector<HsvRange>& color_ranges() const { return color_ranges_; }

    /// Expose current subsample factor for testing.
    int subsample() const { return subsample_; }

private:
    std::vector<HsvRange> color_ranges_;
    int                   min_contour_area_      = 80;
    int                   max_detections_        = 20;
    int                   subsample_             = 2;
    int                   max_fps_               = 0;
    float                 confidence_max_        = 0.95f;  // ceiling for confidence score
    float                 confidence_base_       = 0.5f;   // base confidence before area scaling
    float                 confidence_area_scale_ = 50.0f;  // area_ratio multiplier for confidence
    int                   min_bbox_height_px_    = 15;  // reject detections shorter than this (px)
    float                 max_aspect_ratio_      = 3.0f;  // reject if w/h > this (ground features)

    // Reusable per-frame buffers (avoid heap churn on every detect() call).
    // color_map_: subsampled grid; each cell holds a color index or kNoColor.
    std::vector<uint8_t> color_map_;
    std::vector<int>     parent_buf_;
    std::vector<int>     rank_buf_;
    // Reusable components map for extract_bboxes — avoids per-call heap
    // allocation of an unordered_map (was ~180 heap ops/sec at 30 fps ×
    // 6 color ranges, flagged P2 in #503 review).  Cleared at entry
    // instead of default-constructed.
    std::unordered_map<int, ComponentBBox> components_buf_;

    void init_default_colors() {
        color_ranges_.clear();
        // Red (hue wraps around 0/360) — ends at 15 to avoid overlap with orange.
        color_ranges_.push_back(
            {340.0f, 15.0f, 0.3f, 1.0f, 0.3f, 1.0f, ObjectClass::PERSON, "red"});
        // Blue
        color_ranges_.push_back(
            {200.0f, 260.0f, 0.3f, 1.0f, 0.2f, 1.0f, ObjectClass::VEHICLE_CAR, "blue"});
        // Yellow
        color_ranges_.push_back(
            {40.0f, 70.0f, 0.4f, 1.0f, 0.5f, 1.0f, ObjectClass::VEHICLE_TRUCK, "yellow"});
        // Green
        color_ranges_.push_back(
            {80.0f, 160.0f, 0.3f, 1.0f, 0.2f, 1.0f, ObjectClass::DRONE, "green"});
        // Orange
        color_ranges_.push_back(
            {15.0f, 40.0f, 0.5f, 1.0f, 0.5f, 1.0f, ObjectClass::ANIMAL, "orange"});
        // Magenta/Purple
        color_ranges_.push_back(
            {270.0f, 330.0f, 0.3f, 1.0f, 0.2f, 1.0f, ObjectClass::BUILDING, "magenta"});
    }

    /// Single-pass: for each subsampled pixel, convert to HSV once and record the
    /// first matching color range index in color_map_. Pixels matching no range
    /// are set to kNoColor. Ranges are expected to be non-overlapping in hue;
    /// the first match wins when overlap exists.
    void build_color_map(const uint8_t* frame, int w, int h, int stride, int ws, int hs) {
        const size_t npix_sub = static_cast<size_t>(ws) * static_cast<size_t>(hs);
        color_map_.assign(npix_sub, kNoColor);

        // Cap at kNoColor (0xFF = sentinel) so stored indices always fit in uint8_t.
        const auto n_colors =
            static_cast<uint8_t>(std::min(color_ranges_.size(), static_cast<size_t>(kNoColor)));
        for (int y = 0; y < h; y += subsample_) {
            for (int x = 0; x < w; x += subsample_) {
                const int  px  = (y * w + x) * stride;
                const auto hsv = rgb_to_hsv(frame[px], frame[px + 1], frame[px + 2]);
                for (uint8_t ci = 0; ci < n_colors; ++ci) {
                    if (color_ranges_[ci].contains(hsv.h, hsv.s, hsv.v)) {
                        color_map_[(y / subsample_) * ws + (x / subsample_)] = ci;
                        break;
                    }
                }
            }
        }
    }

    /// Run union-find on color_map_ for color index ci, then collect bounding
    /// boxes. Returned coordinates and pixel_count are scaled back to full
    /// resolution (multiply by subsample_).
    std::vector<ComponentBBox> extract_bboxes(int ws, int hs, uint8_t color_idx) {
        const size_t npix = static_cast<size_t>(ws) * static_cast<size_t>(hs);

        parent_buf_.resize(npix);
        std::iota(parent_buf_.begin(), parent_buf_.end(), 0);
        rank_buf_.assign(npix, 0);

        auto uf_find = [this](int x) -> int {
            while (parent_buf_[x] != x) {
                parent_buf_[x] = parent_buf_[parent_buf_[x]];  // path compression
                x              = parent_buf_[x];
            }
            return x;
        };
        auto uf_unite = [&](int a, int b) {
            a = uf_find(a);
            b = uf_find(b);
            if (a == b) return;
            if (rank_buf_[a] < rank_buf_[b]) std::swap(a, b);
            parent_buf_[b] = a;
            if (rank_buf_[a] == rank_buf_[b]) ++rank_buf_[a];
        };

        for (int y = 0; y < hs; ++y) {
            for (int x = 0; x < ws; ++x) {
                if (color_map_[y * ws + x] != color_idx) continue;
                if (x > 0 && color_map_[y * ws + (x - 1)] == color_idx)
                    uf_unite(y * ws + x, y * ws + (x - 1));
                if (y > 0 && color_map_[(y - 1) * ws + x] == color_idx)
                    uf_unite(y * ws + x, (y - 1) * ws + x);
            }
        }

        // Reuse the member buffer — clear() preserves bucket capacity across
        // calls, sparing the per-call allocation that an unordered_map
        // constructor would perform.
        components_buf_.clear();
        for (int y = 0; y < hs; ++y) {
            for (int x = 0; x < ws; ++x) {
                if (color_map_[y * ws + x] != color_idx) continue;
                int  root = uf_find(y * ws + x);
                auto it   = components_buf_.find(root);
                if (it == components_buf_.end()) {
                    components_buf_[root] = {x, y, x, y, 1};
                } else {
                    auto& bb = it->second;
                    bb.x_min = std::min(bb.x_min, x);
                    bb.y_min = std::min(bb.y_min, y);
                    bb.x_max = std::max(bb.x_max, x);
                    bb.y_max = std::max(bb.y_max, y);
                    ++bb.pixel_count;
                }
            }
        }

        // Scale subsampled coordinates back to full-resolution pixel space.
        // pixel_count is scaled by subsample_^2 so area tests remain in real-pixel units.
        const int                  sub = subsample_;
        std::vector<ComponentBBox> result;
        result.reserve(components_buf_.size());
        for (auto& [root, bbox] : components_buf_) {
            ComponentBBox fb;
            fb.x_min       = bbox.x_min * sub;
            fb.y_min       = bbox.y_min * sub;
            fb.x_max       = bbox.x_max * sub + (sub - 1);
            fb.y_max       = bbox.y_max * sub + (sub - 1);
            fb.pixel_count = bbox.pixel_count * sub * sub;
            result.push_back(fb);
        }
        return result;
    }
};

}  // namespace drone::perception
