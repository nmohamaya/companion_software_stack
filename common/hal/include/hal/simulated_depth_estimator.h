// common/hal/include/hal/simulated_depth_estimator.h
// Simulated depth estimator: returns configurable fixed depth + Gaussian noise.
// Used for testing and development without a real ML model.
// Implements Issue #430.
#pragma once

#include "hal/idepth_estimator.h"
#include "util/config.h"
#include "util/config_keys.h"

#include <algorithm>
#include <random>
#include <string>

namespace drone::hal {

/// Simulated depth estimator returning fixed depth with Gaussian noise.
/// NOT thread-safe: rng_ is mutated on each estimate() call.
/// Only call from a single thread (the depth thread in P2).
/// Config keys:
///   perception.depth_estimator.default_depth_m  (default 10.0)
///   perception.depth_estimator.noise_std_m      (default 0.5)
class SimulatedDepthEstimator : public IDepthEstimator {
public:
    /// Construct from config.
    explicit SimulatedDepthEstimator(const drone::Config& cfg, const std::string& section)
        : default_depth_m_(cfg.get<float>(section + ".default_depth_m", 10.0f))
        , noise_std_m_(cfg.get<float>(section + ".noise_std_m", 0.5f))
        , rng_(std::random_device{}())
        , noise_dist_(0.0f, std::max(0.0f, noise_std_m_)) {}

    /// Construct with explicit parameters (for testing).
    explicit SimulatedDepthEstimator(float default_depth_m = 10.0f, float noise_std_m = 0.5f)
        : default_depth_m_(default_depth_m)
        , noise_std_m_(noise_std_m)
        , rng_(std::random_device{}())
        , noise_dist_(0.0f, std::max(0.0f, noise_std_m)) {}

    [[nodiscard]] drone::util::Result<DepthMap, std::string> estimate(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
        uint32_t stride = 0) override {
        // Simulated backend doesn't read pixel data — stride is unused.
        (void)stride;
        if (frame_data == nullptr || width == 0 || height == 0) {
            return drone::util::Result<DepthMap, std::string>::err(
                "SimulatedDepthEstimator: invalid frame (null data or zero dimensions)");
        }
        if (channels == 0) {
            return drone::util::Result<DepthMap, std::string>::err(
                "SimulatedDepthEstimator: invalid channel count (0)");
        }

        DepthMap map;
        map.width        = width;
        map.height       = height;
        map.scale        = 1.0f;
        map.confidence   = 0.5f;  // simulated confidence
        map.timestamp_ns = 0;     // caller should set from frame timestamp

        const auto num_pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
        map.data.resize(num_pixels);

        for (size_t i = 0; i < num_pixels; ++i) {
            const float noisy_depth = default_depth_m_ + noise_dist_(rng_);
            // Clamp to positive depth — negative depth is physically impossible
            map.data[i] = std::max(0.1f, noisy_depth);
        }

        return drone::util::Result<DepthMap, std::string>::ok(std::move(map));
    }

    [[nodiscard]] std::string name() const override { return "SimulatedDepthEstimator"; }

private:
    float                           default_depth_m_;
    float                           noise_std_m_;
    std::mt19937                    rng_;
    std::normal_distribution<float> noise_dist_;
};

}  // namespace drone::hal
