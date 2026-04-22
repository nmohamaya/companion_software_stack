// common/hal/include/hal/simulated_sam_backend.h
// Simulated SAM (Segment Anything Model) backend: returns deterministic
// synthetic rectangular masks with class_id = -1 (class-agnostic).
// Used by unit tests and dev cycles without real SAM ONNX weights.
#pragma once

#include "hal/iinference_backend.h"
#include "util/config.h"

#include <algorithm>
#include <cstring>

namespace drone::hal {

class SimulatedSAMBackend : public IInferenceBackend {
public:
    explicit SimulatedSAMBackend(int num_masks = 3, float confidence = 0.9f)
        : num_masks_(num_masks), confidence_(confidence) {}

    SimulatedSAMBackend(const drone::Config& cfg, const std::string& section)
        : num_masks_(cfg.get<int>(section + ".num_masks", 3))
        , confidence_(cfg.get<float>(section + ".confidence", 0.9f)) {}

    [[nodiscard]] bool init(const std::string& /*model_path*/, int input_size) override {
        input_size_  = input_size;
        initialized_ = true;
        return true;
    }

    [[nodiscard]] drone::util::Result<InferenceOutput, std::string> infer(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
        uint32_t /*stride*/) override {
        using R = drone::util::Result<InferenceOutput, std::string>;
        if (!frame_data) {
            return R::err("Null frame data");
        }
        if (width == 0 || height == 0 || channels == 0) {
            return R::err("Invalid frame dimensions");
        }

        InferenceOutput output;
        output.timestamp_ns = counter_++;

        const int n = std::max(0, num_masks_);
        output.detections.reserve(static_cast<size_t>(n));

        for (int i = 0; i < n; ++i) {
            InferenceDetection det;

            // Evenly spaced rectangular masks across the frame width
            const float region_w = static_cast<float>(width) / static_cast<float>(n);
            const float pad      = region_w * 0.1f;
            const float mask_x   = region_w * static_cast<float>(i) + pad;
            const float mask_y   = static_cast<float>(height) * 0.2f;
            const float mask_w   = region_w - 2.0f * pad;
            const float mask_h   = static_cast<float>(height) * 0.5f;

            det.bbox.x     = mask_x;
            det.bbox.y     = mask_y;
            det.bbox.w     = mask_w;
            det.bbox.h     = mask_h;
            det.class_id   = -1;  // SAM is class-agnostic
            det.confidence = confidence_;

            // Generate binary mask: 255 inside rectangle, 0 outside
            det.mask_width  = width;
            det.mask_height = height;
            det.mask.resize(static_cast<size_t>(width) * height, 0);

            const auto x0 = static_cast<uint32_t>(std::max(0.0f, mask_x));
            const auto y0 = static_cast<uint32_t>(std::max(0.0f, mask_y));
            const auto x1 = std::min(width, static_cast<uint32_t>(mask_x + mask_w));
            const auto y1 = std::min(height, static_cast<uint32_t>(mask_y + mask_h));

            for (uint32_t row = y0; row < y1; ++row) {
                std::memset(&det.mask[static_cast<size_t>(row) * width + x0], 255,
                            static_cast<size_t>(x1 - x0));
            }

            output.detections.push_back(std::move(det));
        }
        return R::ok(std::move(output));
    }

    [[nodiscard]] std::string name() const override { return "SimulatedSAMBackend"; }

private:
    int      num_masks_;
    float    confidence_;
    int      input_size_{1024};
    bool     initialized_{false};
    uint64_t counter_{0};
};

}  // namespace drone::hal
