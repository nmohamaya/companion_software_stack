// common/hal/include/hal/depth_anything_v2.h
// Depth Anything V2 depth estimator backend using OpenCV DNN.
// Loads a DA V2 ViT-S ONNX model and performs monocular metric depth estimation.
// Implements Issue #455.
#pragma once

#include "hal/idepth_estimator.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#ifdef HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <atomic>
#include <chrono>
#include <limits>
#include <string>

namespace drone::hal {

/// Depth Anything V2 monocular depth estimator using OpenCV DNN.
///
/// Loads a DA V2 ONNX model (ViT-S ~100MB) and produces per-pixel metric depth.
///
/// Two conversion paths (select via `dav2.calibration_enabled`):
///
/// - **Per-frame (default)**: compute min/max of raw output per frame, linearly
///   remap to `[min_depth, max_depth]`.  Simplest correct conversion for a
///   relative-depth model.  Scale shifts between frames based on scene content
///   (looking at a near cube vs a distant horizon remaps identical raw values
///   to different metres) — acceptable for most uses but produces drifting
///   voxel positions in PATH A.
///
/// - **Calibrated (Issue #616)**: use a fitted `(raw_min_ref, raw_max_ref)`
///   pair to anchor the normalisation across frames, then apply an optional
///   linear fit `metric = a × metric_anchored + b`.  Produces stable scale.
///   Fit with `tools/calibrate_depth_anything_v2.py` from a scenario run's
///   `perception.log`.
///
/// NOT thread-safe: net_ is mutated on each estimate() call.
/// Only call from a single thread (the depth thread in P2).
///
/// Config keys:
///   perception.depth_estimator.backend                     = "depth_anything_v2"
///   perception.depth_estimator.model_path                  (default "models/depth_anything_v2_vits.onnx")
///   perception.depth_estimator.input_size                  (default 518)
///   perception.depth_estimator.max_fps                     (default 15)
///   perception.depth_estimator.max_depth_m                 (default 20.0 — metric ceiling)
///
///   perception.depth_estimator.dav2.calibration_enabled    (default false — per-frame path)
///   perception.depth_estimator.dav2.raw_min_ref            (default NaN — per-frame min used)
///   perception.depth_estimator.dav2.raw_max_ref            (default NaN — per-frame max used)
///   perception.depth_estimator.dav2.calibration_coef_a     (default 1.0 — identity passthrough)
///   perception.depth_estimator.dav2.calibration_coef_b     (default 0.0 — identity passthrough)
class DepthAnythingV2Estimator : public IDepthEstimator {
public:
    /// Construct from config.
    explicit DepthAnythingV2Estimator(const drone::Config& cfg, const std::string& section);

    /// Construct with explicit parameters (for testing / direct use).
    /// Always uses the per-frame normalisation path.  To exercise the
    /// calibrated path in tests, use the Config-driven constructor.
    explicit DepthAnythingV2Estimator(const std::string& model_path, int input_size = 518,
                                      float max_depth_m = 20.0f);

    [[nodiscard]] drone::util::Result<DepthMap, std::string> estimate(const uint8_t* frame_data,
                                                                      uint32_t       width,
                                                                      uint32_t       height,
                                                                      uint32_t       channels,
                                                                      uint32_t stride = 0) override;

    [[nodiscard]] std::string name() const override { return "DepthAnythingV2Estimator"; }

    /// Check if ONNX model was loaded successfully.
    [[nodiscard]] bool is_loaded() const { return model_loaded_; }

    /// True if the calibrated conversion path is active.  Exposed for
    /// diagnostics + tests.
    [[nodiscard]] bool calibration_enabled() const { return calibration_enabled_; }

private:
    void load_model(const std::string& model_path);

#ifdef HAS_OPENCV
    cv::dnn::Net net_;
#endif
    bool  model_loaded_ = false;
    int   input_size_   = 518;
    float max_depth_m_  = 20.0f;  // metric conversion: depth = max_depth / (inv_depth + eps)

    // ── Calibration path (Issue #616) ──────────────────────────────
    // All zero-valued by default; enabled only when calibration_enabled_
    // is set via the Config-driven constructor.
    bool  calibration_enabled_ = false;
    float raw_min_ref_         = std::numeric_limits<float>::quiet_NaN();
    float raw_max_ref_         = std::numeric_limits<float>::quiet_NaN();
    float calibration_coef_a_  = 1.0f;
    float calibration_coef_b_  = 0.0f;
};

}  // namespace drone::hal
