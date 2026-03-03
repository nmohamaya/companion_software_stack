// process2_perception/include/perception/color_contour_detector.h
// Real detector: HSV color segmentation + connected-component labeling.
// Processes actual RGB camera frames — no external CV libraries.
#pragma once

#include "perception/detector_interface.h"
#include "util/config.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include <spdlog/spdlog.h>

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
    float h;  // [0, 360)
    float s;  // [0, 1]
    float v;  // [0, 1]
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
    int x_min, y_min, x_max, y_max;
    int pixel_count;

    int width() const { return x_max - x_min + 1; }
    int height() const { return y_max - y_min + 1; }
    int area() const { return pixel_count; }
};

// ═══════════════════════════════════════════════════════════
// ColorContourDetector
// ═══════════════════════════════════════════════════════════
class ColorContourDetector : public IDetector {
public:
    /// Construct with default color ranges (tuned for Gazebo obstacle colors).
    ColorContourDetector() { init_default_colors(); }

    /// Construct from config — reads HSV ranges and min_area from JSON.
    explicit ColorContourDetector(const drone::Config& cfg) {
        min_contour_area_ = cfg.get<int>("perception.detector.min_contour_area", 80);
        max_detections_   = cfg.get<int>("perception.detector.max_detections", 20);

        // Try to read custom color ranges from config
        if (cfg.has("perception.detector.colors")) {
            auto colors_json = cfg.section("perception.detector.colors");
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

        spdlog::info("[ColorContourDetector] Configured with {} color ranges, "
                     "min_area={}, max_dets={}",
                     color_ranges_.size(), min_contour_area_, max_detections_);
    }

    std::vector<Detection2D> detect(const uint8_t* frame_data, uint32_t width, uint32_t height,
                                    uint32_t channels) override {
        if (!frame_data || width == 0 || height == 0 || channels < 3) {
            return {};
        }

        std::vector<Detection2D> all_detections;
        auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                std::chrono::steady_clock::now().time_since_epoch())
                                                .count());

        // For each color range, create binary mask → find components → extract bboxes
        for (const auto& color : color_ranges_) {
            auto bboxes = detect_color(frame_data, width, height, channels, color);

            for (const auto& bbox : bboxes) {
                if (bbox.area() < min_contour_area_) continue;

                Detection2D det;
                det.x              = static_cast<float>(bbox.x_min);
                det.y              = static_cast<float>(bbox.y_min);
                det.w              = static_cast<float>(bbox.width());
                det.h              = static_cast<float>(bbox.height());
                det.class_id       = color.class_id;
                det.timestamp_ns   = now_ns;
                det.frame_sequence = 0;

                // Confidence based on area: larger objects → higher confidence
                float area_ratio = static_cast<float>(bbox.area()) /
                                   static_cast<float>(width * height);
                det.confidence = std::min(0.95f, 0.5f + area_ratio * 50.0f);

                all_detections.push_back(det);
            }
        }

        // Sort by confidence (descending) and cap at max_detections
        std::sort(
            all_detections.begin(), all_detections.end(),
            [](const Detection2D& a, const Detection2D& b) { return a.confidence > b.confidence; });
        if (static_cast<int>(all_detections.size()) > max_detections_) {
            all_detections.resize(max_detections_);
        }

        return all_detections;
    }

    std::string name() const override { return "ColorContourDetector"; }

    /// Expose color ranges for testing
    const std::vector<HsvRange>& color_ranges() const { return color_ranges_; }

private:
    std::vector<HsvRange> color_ranges_;
    int                   min_contour_area_ = 80;
    int                   max_detections_   = 20;

    // Reusable per-frame buffers (avoid heap churn on every detect() call)
    mutable std::vector<uint8_t> mask_buf_;
    mutable std::vector<int>     parent_buf_;
    mutable std::vector<int>     rank_buf_;

    void init_default_colors() {
        color_ranges_.clear();
        // Red (hue wraps around 0/360) — ends at 15 to avoid overlap with orange
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

    /// Detect all connected components of a given color in the frame.
    /// Uses pre-allocated buffers (mask_buf_, parent_buf_, rank_buf_) to
    /// avoid heap churn on each call.
    std::vector<ComponentBBox> detect_color(const uint8_t* frame_data, uint32_t width,
                                            uint32_t height, uint32_t channels,
                                            const HsvRange& color) const {
        const int    w      = static_cast<int>(width);
        const int    h      = static_cast<int>(height);
        const int    stride = static_cast<int>(channels);
        const size_t npix   = static_cast<size_t>(w) * static_cast<size_t>(h);

        // Resize reusable buffers (no-op if already correct size)
        mask_buf_.resize(npix);
        std::fill(mask_buf_.begin(), mask_buf_.end(), 0);

        parent_buf_.resize(npix);
        std::iota(parent_buf_.begin(), parent_buf_.end(), 0);

        rank_buf_.assign(npix, 0);

        // Step 1: Build binary mask (1 = pixel matches color)
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                int  idx = (y * w + x) * stride;
                auto hsv = rgb_to_hsv(frame_data[idx], frame_data[idx + 1], frame_data[idx + 2]);
                if (color.contains(hsv.h, hsv.s, hsv.v)) {
                    mask_buf_[y * w + x] = 1;
                }
            }
        }

        // Step 2: Connected-component labeling (4-connectivity, inline union-find
        //         on parent_buf_/rank_buf_ to avoid allocating UnionFind object)
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

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (!mask_buf_[y * w + x]) continue;
                if (x > 0 && mask_buf_[y * w + (x - 1)]) uf_unite(y * w + x, y * w + (x - 1));
                if (y > 0 && mask_buf_[(y - 1) * w + x]) uf_unite(y * w + x, (y - 1) * w + x);
            }
        }

        // Step 3: Collect bounding boxes per component
        std::unordered_map<int, ComponentBBox> components;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (!mask_buf_[y * w + x]) continue;
                int  root = uf_find(y * w + x);
                auto it   = components.find(root);
                if (it == components.end()) {
                    components[root] = {x, y, x, y, 1};
                } else {
                    auto& bb = it->second;
                    bb.x_min = std::min(bb.x_min, x);
                    bb.y_min = std::min(bb.y_min, y);
                    bb.x_max = std::max(bb.x_max, x);
                    bb.y_max = std::max(bb.y_max, y);
                    bb.pixel_count++;
                }
            }
        }

        // Step 4: Collect results
        std::vector<ComponentBBox> result;
        result.reserve(components.size());
        for (auto& [root, bbox] : components) {
            result.push_back(bbox);
        }
        return result;
    }
};

}  // namespace drone::perception
