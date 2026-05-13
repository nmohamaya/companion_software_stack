// common/hal/include/hal/simulated_sam_backend.h
// Simulated SAM (Segment Anything Model) backend: returns deterministic
// synthetic rectangular masks with class_id = -1 (class-agnostic).
// Used by unit tests and dev cycles without real SAM ONNX weights.
#pragma once

#include "hal/iinference_backend.h"
#include "util/config.h"

#include <algorithm>

namespace drone::hal {

class SimulatedSAMBackend : public IInferenceBackend {
public:
    SimulatedSAMBackend(const SimulatedSAMBackend&)            = delete;
    SimulatedSAMBackend& operator=(const SimulatedSAMBackend&) = delete;
    SimulatedSAMBackend(SimulatedSAMBackend&&)                 = default;
    SimulatedSAMBackend& operator=(SimulatedSAMBackend&&)      = default;

    explicit SimulatedSAMBackend(int num_masks = 3, float confidence = 0.9f)
        : num_masks_(std::clamp(num_masks, 0, kMaxMasks))
        , confidence_(std::clamp(confidence, 0.0f, 1.0f)) {}

    SimulatedSAMBackend(const drone::Config& cfg, const std::string& section)
        : num_masks_(std::clamp(cfg.get<int>(section + ".num_masks", 3), 0, kMaxMasks))
        , confidence_(std::clamp(cfg.get<float>(section + ".confidence", 0.9f), 0.0f, 1.0f)) {}

    [[nodiscard]] bool init(const std::string& /*model_path*/, int /*input_size*/) override {
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

        output.detections.reserve(static_cast<size_t>(num_masks_));

        for (int i = 0; i < num_masks_; ++i) {
            InferenceDetection det;

            const float region_w = static_cast<float>(width) / static_cast<float>(num_masks_);
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

            det.mask_width  = width;
            det.mask_height = height;
            det.mask.resize(static_cast<size_t>(width) * height, 0);

            const auto x0 = static_cast<uint32_t>(std::max(0.0f, mask_x));
            const auto y0 = static_cast<uint32_t>(std::max(0.0f, mask_y));
            const auto x1 = std::min(width, static_cast<uint32_t>(mask_x + mask_w));
            const auto y1 = std::min(height, static_cast<uint32_t>(mask_y + mask_h));

            for (uint32_t row = y0; row < y1; ++row) {
                std::fill(&det.mask[static_cast<size_t>(row) * width + x0],
                          &det.mask[static_cast<size_t>(row) * width + x1], uint8_t{255});
            }

            output.detections.push_back(std::move(det));
        }
        return R::ok(std::move(output));
    }

    [[nodiscard]] std::string name() const override { return "SimulatedSAMBackend"; }

private:
    static constexpr int kMaxMasks = 256;

    int      num_masks_{3};
    float    confidence_{0.9f};
    uint64_t counter_{0};
};

}  // namespace drone::hal
