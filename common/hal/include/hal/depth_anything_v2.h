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
#include <string>

namespace drone::hal {

/// Depth Anything V2 monocular depth estimator using OpenCV DNN.
///
/// Loads a DA V2 ONNX model (ViT-S ~100MB) and produces per-pixel metric depth.
/// The model outputs relative inverse depth which is converted to metric depth via:
///   metric_depth = max_depth / (normalized_output + epsilon)
///
/// NOT thread-safe: net_ is mutated on each estimate() call.
/// Only call from a single thread (the depth thread in P2).
///
/// Config keys:
///   perception.depth_estimator.backend      = "depth_anything_v2"
///   perception.depth_estimator.model_path   (default "models/depth_anything_v2_vits.onnx")
///   perception.depth_estimator.input_size   (default 518)
///   perception.depth_estimator.max_fps      (default 15)
///   perception.depth_estimator.max_depth_m  (default 20.0 — metric conversion ceiling)
class DepthAnythingV2Estimator : public IDepthEstimator {
public:
    /// Construct from config.
    explicit DepthAnythingV2Estimator(const drone::Config& cfg, const std::string& section);

    /// Construct with explicit parameters (for testing / direct use).
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

private:
    void load_model(const std::string& model_path);

#ifdef HAS_OPENCV
    cv::dnn::Net net_;
#endif
    bool  model_loaded_ = false;
    int   input_size_   = 518;
    float max_depth_m_  = 20.0f;  // metric conversion: depth = max_depth / (inv_depth + eps)
};

}  // namespace drone::hal
