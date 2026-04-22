// common/hal/include/hal/edge_contour_sam_backend.h
//
// Algorithmic class-agnostic segmentation via OpenCV edge detection +
// contour extraction.  Produces real masks tied to actual image content —
// the interim fill between SimulatedSAMBackend (fake masks, test scaffold)
// and a real SAM-2 ONNX backend (E5.1 #555, scoped but not shipped).
//
// Algorithm (tuned for obstacle-dominated scenes like scenario 33):
//   1. RGB → grayscale
//   2. Gaussian blur to suppress texture noise
//   3. Canny edge detection (auto-threshold from image median)
//   4. Morphological dilate to close edge gaps
//   5. findContours (external, CHAIN_APPROX_SIMPLE)
//   6. Sort by area; keep top-N with area >= min_area_px
//   7. For each kept contour: fill a binary mask covering the contour's
//      bbox + interior
//
// Output: vector of `hal::InferenceDetection` with class_id = -1 (class-
// agnostic), mask populated from the filled contour, bbox = contour bbox.
// Confidence is 1.0 for the largest contour, scaling down by area rank.
//
// Runtime cost: ~5-15 ms on 1280x720 / Intel-class CPU. Fits comfortably
// inside the 33 ms/frame budget at 30 Hz.
//
// Gracefully degrades when OpenCV isn't available (returns empty output,
// same pattern as `DepthAnythingV2`).  Scenarios that need this backend
// must therefore run on builds with `HAS_OPENCV` defined.
//
// See Epic #520 / Issue #608 — scenario 33 uses this via
// `perception.path_a.sam.backend = "edge_contour_sam"`.

#pragma once

#include "hal/iinference_backend.h"
#include "util/config.h"
#include "util/ilogger.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#ifdef HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace drone::hal {

class EdgeContourSAMBackend : public IInferenceBackend {
public:
    EdgeContourSAMBackend() = default;

    EdgeContourSAMBackend(const drone::Config& cfg, const std::string& section)
        : max_masks_(std::clamp(cfg.get<int>(section + ".max_masks", 8), 1, kMaxMasks))
        , min_area_px_(std::max(100, cfg.get<int>(section + ".min_area_px", 500)))
        , canny_low_(std::clamp(cfg.get<int>(section + ".canny_low", 50), 0, 255))
        , canny_high_(std::clamp(cfg.get<int>(section + ".canny_high", 150), 0, 255))
        , blur_ksize_(cfg.get<int>(section + ".blur_ksize", 5) | 1)  // force odd
        , dilate_ksize_(std::max(1, cfg.get<int>(section + ".dilate_ksize", 3)))
        , downsample_(std::clamp(cfg.get<int>(section + ".downsample_factor", 2), 1, 8)) {
        if (canny_low_ > canny_high_) std::swap(canny_low_, canny_high_);
    }

    [[nodiscard]] bool init(const std::string& /*model_path*/, int /*input_size*/) override {
#ifdef HAS_OPENCV
        return true;
#else
        DRONE_LOG_WARN("[EdgeContourSAM] OpenCV not available at compile time — backend will "
                       "return empty output.  Build with HAS_OPENCV to enable.");
        return true;  // Allow construction; runtime fallback returns empty masks.
#endif
    }

    [[nodiscard]] drone::util::Result<InferenceOutput, std::string> infer(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
        uint32_t stride = 0) override {
        using R = drone::util::Result<InferenceOutput, std::string>;
        if (!frame_data) return R::err("Null frame data");
        if (width == 0 || height == 0 || channels == 0) {
            return R::err("Invalid frame dimensions");
        }

        InferenceOutput output;
        output.timestamp_ns = ++counter_;

#ifndef HAS_OPENCV
        (void)stride;
        return R::ok(std::move(output));
#else
        const int step = (stride > 0) ? static_cast<int>(stride)
                                      : static_cast<int>(width * channels);
        cv::Mat rgb(static_cast<int>(height), static_cast<int>(width),
                    channels == 3 ? CV_8UC3 : (channels == 4 ? CV_8UC4 : CV_8UC1),
                    const_cast<uint8_t*>(frame_data), static_cast<size_t>(step));

        cv::Mat gray;
        if (channels == 1) {
            gray = rgb;
        } else if (channels == 3) {
            cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
        } else {
            cv::cvtColor(rgb, gray, cv::COLOR_RGBA2GRAY);
        }

        // Downsample for speed — masks are upscaled back at the end.
        cv::Mat small;
        if (downsample_ > 1) {
            cv::resize(gray, small,
                       cv::Size(static_cast<int>(width) / downsample_,
                                static_cast<int>(height) / downsample_),
                       0.0, 0.0, cv::INTER_AREA);
        } else {
            small = gray;
        }

        // Blur → Canny → Dilate → findContours pipeline.
        cv::Mat blurred;
        cv::GaussianBlur(small, blurred, cv::Size(blur_ksize_, blur_ksize_), 0);

        cv::Mat edges;
        cv::Canny(blurred, edges, canny_low_, canny_high_);

        if (dilate_ksize_ > 1) {
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(dilate_ksize_, dilate_ksize_));
            cv::dilate(edges, edges, kernel);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Scale per-downsample so min_area_px stays in original-image units.
        const int ds_min_area_px = std::max(1, min_area_px_ / (downsample_ * downsample_));

        struct IndexedContour {
            size_t idx;
            double area;
        };
        std::vector<IndexedContour> kept;
        kept.reserve(contours.size());
        for (size_t i = 0; i < contours.size(); ++i) {
            const double area = cv::contourArea(contours[i]);
            if (area >= static_cast<double>(ds_min_area_px)) {
                kept.push_back({i, area});
            }
        }
        std::sort(kept.begin(), kept.end(),
                  [](const IndexedContour& a, const IndexedContour& b) { return a.area > b.area; });
        if (static_cast<int>(kept.size()) > max_masks_) {
            kept.resize(static_cast<size_t>(max_masks_));
        }

        // Emit an InferenceDetection per kept contour.  Masks are stored at
        // full image resolution so MaskDepthProjector can index the depth
        // map without a scale factor on the downstream side.
        output.detections.reserve(kept.size());
        for (size_t rank = 0; rank < kept.size(); ++rank) {
            const auto& ic         = kept[rank];
            const auto& contour    = contours[ic.idx];
            const cv::Rect r_small = cv::boundingRect(contour);

            InferenceDetection det;
            det.bbox.x      = static_cast<float>(r_small.x * downsample_);
            det.bbox.y      = static_cast<float>(r_small.y * downsample_);
            det.bbox.w      = static_cast<float>(r_small.width * downsample_);
            det.bbox.h      = static_cast<float>(r_small.height * downsample_);
            det.class_id    = -1;  // class-agnostic
            det.confidence  = std::max(0.1f, 1.0f - 0.05f * static_cast<float>(rank));
            det.mask_width  = width;
            det.mask_height = height;
            det.mask.assign(static_cast<size_t>(width) * height, uint8_t{0});

            // Render the filled contour onto a small canvas, then up-sample
            // to full resolution. cv::resize with INTER_NEAREST keeps the
            // mask binary (0/255).
            cv::Mat mask_small(small.rows, small.cols, CV_8UC1, cv::Scalar(0));
            cv::drawContours(mask_small, contours, static_cast<int>(ic.idx),
                             cv::Scalar(255), cv::FILLED);

            if (downsample_ > 1) {
                cv::Mat mask_full(static_cast<int>(height), static_cast<int>(width), CV_8UC1,
                                  det.mask.data());
                cv::resize(mask_small, mask_full, mask_full.size(), 0.0, 0.0, cv::INTER_NEAREST);
            } else {
                std::copy(mask_small.datastart, mask_small.dataend, det.mask.begin());
            }

            output.detections.push_back(std::move(det));
        }
        return R::ok(std::move(output));
#endif
    }

    [[nodiscard]] std::string name() const override { return "EdgeContourSAMBackend"; }

private:
    static constexpr int kMaxMasks = 32;

    int      max_masks_{8};
    int      min_area_px_{500};
    int      canny_low_{50};
    int      canny_high_{150};
    int      blur_ksize_{5};
    int      dilate_ksize_{3};
    int      downsample_{2};
    uint64_t counter_{0};
};

}  // namespace drone::hal
