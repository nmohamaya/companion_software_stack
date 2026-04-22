// common/hal/include/hal/simulated_inference_backend.h
// Simulated inference backend: returns deterministic synthetic detections.
// Used by unit tests and quick dev cycles without ML models.
#pragma once

#include "hal/iinference_backend.h"
#include "util/config.h"

#include <algorithm>

namespace drone::hal {

class SimulatedInferenceBackend : public IInferenceBackend {
public:
    explicit SimulatedInferenceBackend(int num_detections = 2, float confidence_min = 0.4f,
                                       float confidence_max = 0.9f)
        : num_detections_(num_detections)
        , confidence_min_(confidence_min)
        , confidence_max_(confidence_max) {}

    SimulatedInferenceBackend(const drone::Config& cfg, const std::string& section)
        : num_detections_(cfg.get<int>(section + ".num_detections", 2))
        , confidence_min_(cfg.get<float>(section + ".confidence_min", 0.4f))
        , confidence_max_(cfg.get<float>(section + ".confidence_max", 0.9f)) {}

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

        const int n = std::max(0, num_detections_);
        output.detections.reserve(static_cast<size_t>(n));

        for (int i = 0; i < n; ++i) {
            InferenceDetection det;
            const float frac = (n > 1) ? static_cast<float>(i) / static_cast<float>(n - 1) : 0.5f;
            det.bbox.x       = static_cast<float>(width) * (0.1f + 0.6f * frac);
            det.bbox.y       = static_cast<float>(height) * 0.2f;
            det.bbox.w       = static_cast<float>(width) * 0.15f;
            det.bbox.h       = static_cast<float>(height) * 0.3f;
            det.class_id     = i % 8;
            det.confidence   = confidence_min_ + (confidence_max_ - confidence_min_) * frac;
            output.detections.push_back(std::move(det));
        }
        return R::ok(std::move(output));
    }

    [[nodiscard]] std::string name() const override { return "SimulatedInferenceBackend"; }

private:
    int      num_detections_;
    float    confidence_min_;
    float    confidence_max_;
    int      input_size_{640};
    bool     initialized_{false};
    uint64_t counter_{0};
};

}  // namespace drone::hal
