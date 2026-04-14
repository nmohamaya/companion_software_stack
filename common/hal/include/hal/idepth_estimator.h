// common/hal/include/hal/idepth_estimator.h
// HAL interface: Monocular depth estimation abstraction.
// Implementations: SimulatedDepthEstimator (built-in), DepthAnythingV2 (future).
// Implements Issue #430 — ML depth estimation infrastructure.
#pragma once

#include "util/result.h"

#include <cstdint>
#include <string>
#include <vector>

namespace drone::hal {

/// Result of a single depth estimation pass.
/// Stores per-pixel metric depth (meters) in row-major order.
struct DepthMap {
    std::vector<float> data;       // depth in meters, row-major
    uint32_t           width{0};   // width in pixels
    uint32_t           height{0};  // height in pixels
    uint64_t           timestamp_ns{0};
    float              scale{1.0f};       // depth scale factor (1.0 = meters)
    float              confidence{0.0f};  // overall confidence [0.0, 1.0]
};

/// Abstract depth estimator interface.
/// One instance per estimation backend (simulated, Depth Anything V2, etc.).
class IDepthEstimator {
public:
    virtual ~IDepthEstimator() = default;

    /// Estimate depth from a single RGB frame.
    /// @param frame_data  Pointer to RGB pixel data (valid for this call only)
    /// @param width       Frame width in pixels
    /// @param height      Frame height in pixels
    /// @param channels    Number of channels (3=RGB, 4=RGBA)
    /// @return DepthMap on success, error string on failure
    [[nodiscard]] virtual drone::util::Result<DepthMap, std::string> estimate(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels) = 0;

    /// Human-readable name (e.g. "SimulatedDepthEstimator", "DepthAnythingV2").
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
