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

    // Calibration (Issue #616) — all default-off; per-frame path is still used
    // unless all refs + coefs are explicitly provided.
    namespace dav2       = drone::cfg_key::perception::depth_estimator::dav2;
    calibration_enabled_ = cfg.get<bool>(dav2::CALIBRATION_ENABLED, false);
    raw_min_ref_         = cfg.get<float>(dav2::RAW_MIN_REF, raw_min_ref_);
    raw_max_ref_         = cfg.get<float>(dav2::RAW_MAX_REF, raw_max_ref_);
    calibration_coef_a_  = cfg.get<float>(dav2::CALIBRATION_COEF_A, calibration_coef_a_);
    calibration_coef_b_  = cfg.get<float>(dav2::CALIBRATION_COEF_B, calibration_coef_b_);

    // Safety-check the calibration inputs.  If enabled but refs are invalid,
    // fall back to per-frame normalisation rather than silently producing
    // garbage metres.
    if (calibration_enabled_) {
        const bool refs_valid = std::isfinite(raw_min_ref_) && std::isfinite(raw_max_ref_) &&
                                raw_max_ref_ > raw_min_ref_;
        if (!refs_valid) {
            DRONE_LOG_WARN("[DepthAnythingV2] calibration_enabled=true but raw_min_ref={:.3f} / "
                           "raw_max_ref={:.3f} invalid — falling back to per-frame normalisation.",
                           raw_min_ref_, raw_max_ref_);
            calibration_enabled_ = false;
        }
    }

    DRONE_LOG_INFO("[DepthAnythingV2] Config: input_size={}, max_depth_m={:.1f}, "
                   "calibration={} (a={:.3f}, b={:.3f}, raw_ref=[{:.3f},{:.3f}])",
                   input_size_, max_depth_m_, calibration_enabled_ ? "on" : "off",
                   calibration_coef_a_, calibration_coef_b_, raw_min_ref_, raw_max_ref_);
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
    [[maybe_unused]] const uint8_t* frame_data, [[maybe_unused]] uint32_t width,
    [[maybe_unused]] uint32_t height, [[maybe_unused]] uint32_t channels,
    [[maybe_unused]] uint32_t stride) {
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
    size_t  step    = (stride > 0) ? static_cast<size_t>(stride)
                                   : static_cast<size_t>(cv::Mat::AUTO_STEP);
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

    if (output.type() != CV_32F) {
        return drone::util::Result<DepthMap, std::string>::err(
            "DepthAnythingV2: unexpected output type=" + std::to_string(output.type()) +
            ", expected CV_32F (" + std::to_string(CV_32F) + ")");
    }
    const auto* raw_data   = reinterpret_cast<const float*>(output.data);
    const auto  num_pixels = static_cast<size_t>(out_h) * static_cast<size_t>(out_w);

    // ── Step 5: Convert relative inverse depth → metric depth ──
    // DA V2 outputs unnormalized inverse depth: higher raw value = closer object.
    // Two conversion paths — see class docstring for the design rationale:
    //
    //   (a) Per-frame normalisation (default, pre-#616 behaviour):
    //       min/max of raw_data this frame → linear map to [min_depth, max_depth].
    //
    //   (b) Calibrated (Issue #616, opt-in via `dav2.calibration_enabled`):
    //       use fitted `(raw_min_ref, raw_max_ref)` to anchor normalisation
    //       across frames, then apply linear fit `a * metric_anchored + b`.
    //
    // Either way we always compute per-frame min/max for the diagnostic log
    // below (operators parse these values to fit calibration coefficients
    // later with `tools/calibrate_depth_anything_v2.py`).
    float min_val = raw_data[0];
    float max_val = raw_data[0];
    for (size_t i = 1; i < num_pixels; ++i) {
        min_val = std::min(min_val, raw_data[i]);
        max_val = std::max(max_val, raw_data[i]);
    }

    constexpr float eps       = 1e-6f;
    constexpr float min_depth = 0.1f;  // Minimum metric depth (closest objects)

    // Choose which min/max pair drives the normalisation.  In calibrated mode
    // the refs are fixed; in default mode the per-frame values are used so
    // every frame self-scales (today's behaviour, byte-identical when off).
    const float norm_min = calibration_enabled_ ? raw_min_ref_ : min_val;
    const float norm_max = calibration_enabled_ ? raw_max_ref_ : max_val;
    const float range    = norm_max - norm_min;

    DepthMap map;
    map.width         = static_cast<uint32_t>(out_w);
    map.height        = static_cast<uint32_t>(out_h);
    map.source_width  = width;  // Original frame dimensions — critical for fusion bbox mapping
    map.source_height = height;
    map.scale         = 1.0f;
    map.confidence    = 0.7f;  // ML model confidence — higher than simulated (0.5)
    map.data.resize(num_pixels);

    if (range < eps) {
        // Flat scale range — return uniform max_depth.  Occurs when the
        // per-frame raw output is constant (flat scene) OR when a
        // mis-configured calibration provides near-equal refs.
        std::fill(map.data.begin(), map.data.end(), max_depth_m_);
    } else {
        for (size_t i = 0; i < num_pixels; ++i) {
            // Normalise inverse depth to roughly [0, 1]: 1 = closest, 0 = farthest.
            // In calibrated mode, a raw sample outside the reference window
            // clamps to [0, 1] — that's intentional (a frame that looks closer
            // than anything in the calibration set still reports its best
            // metric estimate instead of extrapolating wildly).
            float normalized = (raw_data[i] - norm_min) / range;
            if (calibration_enabled_) {
                normalized = std::clamp(normalized, 0.0f, 1.0f);
            }
            // Linear map: high inverse depth (close) → small depth, low → large depth
            float metric = min_depth + (max_depth_m_ - min_depth) * (1.0f - normalized);
            // Calibrated mode applies the final linear fit.  Identity (a=1,b=0)
            // is a no-op — operators only need to provide a/b when the
            // anchored-scale output still has a systematic bias vs ground truth.
            if (calibration_enabled_) {
                metric = calibration_coef_a_ * metric + calibration_coef_b_;
                metric = std::clamp(metric, min_depth, max_depth_m_);
            }
            map.data[i] = metric;
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
