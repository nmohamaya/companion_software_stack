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
    uint32_t           width{0};   // depth map width in pixels
    uint32_t           height{0};  // depth map height in pixels
    uint64_t           timestamp_ns{0};
    float              scale{1.0f};       // depth scale factor (1.0 = meters)
    float              confidence{0.0f};  // overall confidence [0.0, 1.0]

    // Source frame dimensions — used to map bbox pixel coords to depth map coords.
    // Set by the estimator to the input frame size. May differ from width/height
    // if the model outputs at a different resolution than its input.
    uint32_t source_width{0};   // 0 = same as width (backward compat)
    uint32_t source_height{0};  // 0 = same as height (backward compat)
};

/// Abstract depth estimator interface.
/// One instance per estimation backend (simulated, Depth Anything V2, etc.).
class IDepthEstimator {
public:
    virtual ~IDepthEstimator() = default;

    /// Estimate depth from a single RGB frame.
    /// @param frame_data  Pointer to pixel data (valid for this call only).
    /// @param width       Frame width in pixels (must be > 0)
    /// @param height      Frame height in pixels (must be > 0)
    /// @param channels    Number of channels (3=RGB, 4=RGBA; must be > 0)
    /// @param stride      Row stride in bytes (0 = tightly packed, i.e. width * channels).
    ///                    Some cameras pad rows for alignment; real backends must
    ///                    respect stride when indexing pixels.
    /// @return DepthMap on success, error string on failure
    /// @pre frame_data points to a buffer of at least height * stride bytes
    ///      (or height * width * channels if stride == 0).
    [[nodiscard]] virtual drone::util::Result<DepthMap, std::string> estimate(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
        uint32_t stride = 0) = 0;

    /// Human-readable name (e.g. "SimulatedDepthEstimator", "DepthAnythingV2").
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
