// common/hal/src/depth_anything_v2.cpp
// Depth Anything V2 depth estimator implementation using OpenCV DNN module.
// Issue #455.

#include "hal/depth_anything_v2.h"

#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>

namespace drone::hal {

// ── Config constructor ──────────────────────────────────────
DepthAnythingV2Estimator::DepthAnythingV2Estimator(const drone::Config& cfg,
                                                   const std::string&   section) {
    std::string model_path = cfg.get<std::string>(section + ".model_path",
                                                  "models/depth_anything_v2_vits.onnx");
    input_size_            = cfg.get<int>(section + ".input_size", 518);
    max_depth_m_           = cfg.get<float>(section + ".max_depth_m", 20.0f);

    DRONE_LOG_INFO("[DepthAnythingV2] Config: input_size={}, max_depth_m={:.1f}", input_size_,
                   max_depth_m_);
    load_model(model_path);
}

// ── Explicit constructor ────────────────────────────────────
DepthAnythingV2Estimator::DepthAnythingV2Estimator(const std::string& model_path, int input_size,
                                                   float max_depth_m)
    : input_size_(input_size), max_depth_m_(max_depth_m) {
    load_model(model_path);
}

// ── Model loading ───────────────────────────────────────────
void DepthAnythingV2Estimator::load_model(const std::string& model_path) {
    // Path traversal guard: reject paths containing ".." components.
    // Prevents directory traversal from config-injected paths.
    if (model_path.find("..") != std::string::npos) {
        DRONE_LOG_ERROR("[DepthAnythingV2] Rejected model path with '..': {}", model_path);
        model_loaded_ = false;
        return;
    }

    // Canonicalize path — weakly_canonical can throw on invalid paths or IO errors
    std::filesystem::path canonical;
    try {
        canonical = std::filesystem::weakly_canonical(model_path);
    } catch (const std::filesystem::filesystem_error& e) {
        DRONE_LOG_ERROR("[DepthAnythingV2] Invalid model path '{}': {}", model_path, e.what());
        model_loaded_ = false;
        return;
    }

#ifdef HAS_OPENCV
    try {
        net_ = cv::dnn::readNetFromONNX(canonical.string());
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        model_loaded_ = true;

        DRONE_LOG_INFO("[DepthAnythingV2] Model loaded: {} (input={}x{})", model_path, input_size_,
                       input_size_);
    } catch (const cv::Exception& e) {
        DRONE_LOG_ERROR("[DepthAnythingV2] Failed to load model '{}': {}", model_path, e.what());
        model_loaded_ = false;
    }
#else
    (void)canonical;
    DRONE_LOG_WARN("[DepthAnythingV2] OpenCV not available — model not loaded");
    model_loaded_ = false;
#endif
}

// ── Depth estimation ────────────────────────────────────────
[[nodiscard]] drone::util::Result<DepthMap, std::string> DepthAnythingV2Estimator::estimate(
    const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
    uint32_t stride) {
    if (frame_data == nullptr || width == 0 || height == 0) {
        return drone::util::Result<DepthMap, std::string>::err(
            "DepthAnythingV2: invalid frame (null data or zero dimensions)");
    }
    if (channels < 3 || channels > 4) {
        return drone::util::Result<DepthMap, std::string>::err(
            "DepthAnythingV2: requires 3 (RGB) or 4 (RGBA) channels, got " +
            std::to_string(channels));
    }

#ifdef HAS_OPENCV
    if (!model_loaded_) {
        return drone::util::Result<DepthMap, std::string>::err("DepthAnythingV2: model not loaded");
    }

    auto t0 = std::chrono::steady_clock::now();

    // ── Step 1: Wrap raw pixel data as cv::Mat ──────────────
    int     cv_type = (channels == 4) ? CV_8UC4 : CV_8UC3;
    size_t  step    = (stride > 0) ? static_cast<size_t>(stride) : cv::Mat::AUTO_STEP;
    cv::Mat frame(static_cast<int>(height), static_cast<int>(width), cv_type,
                  const_cast<uint8_t*>(frame_data), step);

    // If RGBA, convert to RGB
    cv::Mat rgb;
    if (channels == 4) {
        cv::cvtColor(frame, rgb, cv::COLOR_RGBA2RGB);
    } else {
        rgb = frame;  // Already RGB, no copy
    }

    // ── Step 2: Create blob (resize + normalize) ────────────
    // DA V2 expects [1, 3, H, W] normalized to [0, 1]
    cv::Mat blob;
    cv::dnn::blobFromImage(rgb, blob,
                           1.0 / 255.0,                         // scale
                           cv::Size(input_size_, input_size_),  // target size
                           cv::Scalar(0, 0, 0),                 // mean subtraction
                           false,                               // swapRB (input is RGB)
                           false);                              // crop

    // ── Step 3: Forward pass ────────────────────────────────
    net_.setInput(blob);
    cv::Mat output;
    try {
        output = net_.forward();
    } catch (const cv::Exception& e) {
        return drone::util::Result<DepthMap, std::string>::err(
            std::string("DepthAnythingV2: forward() failed: ") + e.what());
    }

    // ── Step 4: Parse output ────────────────────────────────
    // DA V2 output: [1, 1, H, W] or [1, H, W] — relative inverse depth (float32)
    // Reshape to 2D if needed
    const int out_dims = output.dims;
    int       out_h    = 0;
    int       out_w    = 0;

    if (out_dims == 4) {
        // [1, 1, H, W]
        out_h = output.size[2];
        out_w = output.size[3];
    } else if (out_dims == 3) {
        // [1, H, W]
        out_h = output.size[1];
        out_w = output.size[2];
    } else if (out_dims == 2) {
        // [H, W]
        out_h = output.size[0];
        out_w = output.size[1];
    } else {
        return drone::util::Result<DepthMap, std::string>::err(
            "DepthAnythingV2: unexpected output dims=" + std::to_string(out_dims));
    }

    if (out_h <= 0 || out_w <= 0) {
        return drone::util::Result<DepthMap, std::string>::err(
            "DepthAnythingV2: invalid output dimensions " + std::to_string(out_h) + "x" +
            std::to_string(out_w));
    }

    CV_Assert(output.type() == CV_32F);
    const auto* raw_data   = reinterpret_cast<const float*>(output.data);
    const auto  num_pixels = static_cast<size_t>(out_h) * static_cast<size_t>(out_w);

    // ── Step 5: Convert relative inverse depth → metric depth ──
    // DA V2 outputs unnormalized inverse depth: higher values = closer objects.
    // We normalize to [0,1] then invert: depth = max_depth / (normalized + eps).
    // This maps: normalized≈1 (closest) → depth≈max_depth/(1+eps)≈max_depth,
    //            normalized≈0 (farthest) → depth≈max_depth/eps (clamped to max_depth).
    // The final clamp to [0.1, max_depth_m_] ensures valid metric range.
    // Note: inverse depth means the "closest" pixel gets the highest raw value,
    // so after inversion the depth spread depends on the scene's actual depth range.
    float min_val = raw_data[0];
    float max_val = raw_data[0];
    for (size_t i = 1; i < num_pixels; ++i) {
        min_val = std::min(min_val, raw_data[i]);
        max_val = std::max(max_val, raw_data[i]);
    }

    constexpr float eps   = 1e-6f;
    const float     range = max_val - min_val;

    DepthMap map;
    map.width         = static_cast<uint32_t>(out_w);
    map.height        = static_cast<uint32_t>(out_h);
    map.source_width  = width;  // Original frame dimensions — critical for fusion bbox mapping
    map.source_height = height;
    map.scale         = 1.0f;
    map.confidence    = 0.7f;  // ML model confidence — higher than simulated (0.5)
    map.data.resize(num_pixels);

    if (range < eps) {
        // Flat output (all pixels same depth) — return uniform max_depth
        std::fill(map.data.begin(), map.data.end(), max_depth_m_);
    } else {
        for (size_t i = 0; i < num_pixels; ++i) {
            // Normalize inverse depth to [0, 1]
            const float normalized = (raw_data[i] - min_val) / range;
            // Convert: high inverse depth → close, low inverse depth → far
            // Clamp metric depth to [0.1, max_depth_m_]
            const float metric = max_depth_m_ / (normalized + eps);
            map.data[i]        = std::clamp(metric, 0.1f, max_depth_m_);
        }
    }

    auto t1 = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    static std::atomic<uint64_t> call_count{0};
    if (++call_count % 30 == 0) {
        DRONE_LOG_INFO("[DepthAnythingV2] {}x{} depth map in {}ms (inv_depth range: [{:.3f}, "
                       "{:.3f}])",
                       out_w, out_h, ms, min_val, max_val);
    }

    return drone::util::Result<DepthMap, std::string>::ok(std::move(map));

#else
    return drone::util::Result<DepthMap, std::string>::err(
        "DepthAnythingV2: OpenCV not available (compiled without HAS_OPENCV)");
#endif
}

}  // namespace drone::hal
