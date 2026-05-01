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
    // Lower metric bound — closest reportable obstacle (PR #620 code-quality
    // review).  `max_depth_m_` was configurable but `min_depth_m_` was hardcoded
    // at 0.1 — asymmetric.  Operators tuning tight indoor flights may want a
    // narrower floor; symmetric tunability closes the gap.
    namespace dav2 = drone::cfg_key::perception::depth_estimator::dav2;
    min_depth_m_   = cfg.get<float>(dav2::MIN_DEPTH_M, 0.1f);

    // Calibration (Issue #616) — all default-off; per-frame path is still used
    // unless all refs + coefs are explicitly provided.
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

    // CUDA inference (Issue #626) — engage only when the config opts in.
    // Silently ignored in every scenario before this PR.  Set via
    // `perception.depth_estimator.use_cuda`.
    // PR #631 P2 review: use the canonical config-key constant rather
    // than `section + ".use_cuda"` string concatenation.  The
    // constant is fully qualified (matches the documented key), so
    // bypassing the section param keeps caller and constant in sync
    // — a future rename of either side surfaces as a build error
    // instead of a silently-different runtime key.
    use_cuda_ = cfg.get<bool>(drone::cfg_key::perception::depth_estimator::USE_CUDA, false);

    DRONE_LOG_INFO("[DepthAnythingV2] Config: input_size={}, max_depth_m={:.1f}, "
                   "use_cuda={}, calibration={} (a={:.3f}, b={:.3f}, raw_ref=[{:.3f},{:.3f}])",
                   input_size_, max_depth_m_, use_cuda_ ? "true" : "false",
                   calibration_enabled_ ? "on" : "off", calibration_coef_a_, calibration_coef_b_,
                   raw_min_ref_, raw_max_ref_);

    // PR #631 P2 review: relative-only path policy applies to
    // CONFIG-injected paths (the actual attack surface).  Direct
    // construction via the (model_path, ...) ctor is trusted —
    // tests + calibration tools own that path themselves.
    if (!model_path.empty() && model_path.front() == '/') {
        DRONE_LOG_ERROR("[DepthAnythingV2] Rejected absolute model path from config "
                        "(relative-only allowed): {}",
                        model_path);
        model_loaded_ = false;
        return;
    }
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
    // Prevents directory traversal from config-injected paths.  The
    // stricter "no absolute path" check lives in the config constructor
    // (`load_model_from_config`); direct construction via the explicit
    // (model_path, input_size, max_depth_m) ctor is trusted because the
    // caller (tests, calibration tooling) builds the path itself.
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

        // Issue #626 — CUDA backend engagement.  Falls back to CPU with a
        // WARN when (a) no CUDA device is visible or (b) a canary forward
        // pass fails (the common case on machines with a cuDNN/CUDA
        // version mismatch — e.g. cuDNN 8.5 built for CUDA 11 on a
        // CUDA 12 host, where the CUDA backend's dlopen of cuDNN dies
        // at the first inference call and takes the perception process
        // with it).  The canary runs one forward pass on a zero-input
        // blob so any load-time failure happens here in the constructor
        // (before the Depth thread starts) rather than in the hot path.
        bool cuda_engaged = false;
        if (use_cuda_) {
            const int cuda_devices = cv::cuda::getCudaEnabledDeviceCount();
            if (cuda_devices == 0) {
                DRONE_LOG_WARN("[DepthAnythingV2] use_cuda=true but "
                               "cv::cuda::getCudaEnabledDeviceCount()=0 — falling back to CPU. "
                               "Rebuild OpenCV with -DWITH_CUDA=ON -DWITH_CUDNN=ON, or set "
                               "use_cuda=false to silence this warning.");
            } else {
                try {
                    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
                    // Canary inference — zero blob at the configured input size.
                    // A cuDNN/CUDA version mismatch shows up here rather than
                    // in the real Depth thread's first frame.
                    cv::Mat canary_blob = cv::Mat::zeros(cv::Size(input_size_, input_size_),
                                                         CV_32FC3);
                    cv::Mat canary_input;
                    cv::dnn::blobFromImage(canary_blob, canary_input, 1.0 / 255.0,
                                           cv::Size(input_size_, input_size_), cv::Scalar(0, 0, 0),
                                           false, false);
                    net_.setInput(canary_input);
                    (void)net_.forward();  // throws or silently OK
                    cuda_engaged = true;
                } catch (const cv::Exception& e) {
                    DRONE_LOG_WARN("[DepthAnythingV2] CUDA canary inference failed ({}) — "
                                   "falling back to CPU.  Common causes: cuDNN/CUDA version "
                                   "mismatch, or OpenCV built without CUDA_DNN.",
                                   e.what());
                } catch (const std::exception& e) {
                    DRONE_LOG_WARN("[DepthAnythingV2] CUDA canary inference threw std::exception "
                                   "({}) — falling back to CPU",
                                   e.what());
                } catch (...) {
                    // PR #631 P2 review: previously a non-cv::Exception
                    // non-std::exception escaped the outer catch
                    // silently — model_loaded_ stayed true but the
                    // backend was in an indeterminate state.  Catch-all
                    // logs the unknown exception class so the fault
                    // surfaces in the Depth thread's startup logs
                    // instead of taking the perception process down on
                    // first inference.
                    DRONE_LOG_WARN("[DepthAnythingV2] CUDA canary inference threw unknown "
                                   "exception type — falling back to CPU");
                }
            }
        }
        if (!cuda_engaged) {
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
        model_loaded_ = true;

        DRONE_LOG_INFO("[DepthAnythingV2] Model loaded: {} (input={}x{}, backend={})", model_path,
                       input_size_, input_size_, cuda_engaged ? "CUDA" : "CPU");
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
        // PR #631 P2 review: track consecutive failures so persistent
        // runtime degradation (e.g. CUDA driver crash) is visible to
        // P7 / run-report via `consecutive_inference_failures()`.
        // Escalating WARN every 10 failures so the log surfaces
        // chronic failure even when callers swallow the Err result.
        const uint64_t fails =
            consecutive_inference_failures_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (fails == 1 || fails % 10 == 0) {
            DRONE_LOG_WARN("[DepthAnythingV2] forward() failed ({} consecutive): {}", fails,
                           e.what());
        }
        return drone::util::Result<DepthMap, std::string>::err(
            std::string("DepthAnythingV2: forward() failed: ") + e.what());
    }
    // Successful inference resets the streak counter.
    consecutive_inference_failures_.store(0, std::memory_order_relaxed);

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
    // DA V2 outputs unnormalised inverse depth: higher raw value = closer object.
    // Two conversion paths — see class docstring for the design rationale:
    //
    //   (a) Per-frame normalisation (default, pre-#616 behaviour):
    //       min/max of raw_data this frame → linear remap to
    //       [min_depth_m_, max_depth_m_].
    //
    //   (b) Calibrated (Issue #616, opt-in via `dav2.calibration_enabled`):
    //       use fitted `(raw_min_ref, raw_max_ref)` to anchor normalisation
    //       across frames, then apply linear fit `a * metric_anchored + b`.
    //
    // Delegates per-pixel math to `convert_raw_to_metric()` (free function at
    // file scope in depth_anything_v2.h) so the conversion is unit-testable
    // without a loaded ONNX model (PR #620 review refactor).
    constexpr float eps = 1e-6f;

    // Per-frame min/max is required for (a) the diagnostic log emitted every
    // 30th call and (b) the per-frame conversion path.  In calibrated mode
    // the output doesn't need it, but we still scan every 30th frame so the
    // log continues to fire and the calibration tool keeps working.  That
    // saves ~29/30 × 268k FP compares when calibration is on.
    static std::atomic<uint64_t> scan_counter{0};
    const bool                   need_scan = !calibration_enabled_ ||
                           (scan_counter.fetch_add(1, std::memory_order_relaxed) % 30 == 0);
    float min_val = 0.0f;
    float max_val = 0.0f;
    if (need_scan) {
        min_val = raw_data[0];
        max_val = raw_data[0];
        for (size_t i = 1; i < num_pixels; ++i) {
            min_val = std::min(min_val, raw_data[i]);
            max_val = std::max(max_val, raw_data[i]);
        }
    }

    // Build conversion parameters — anchor choice hoisted outside the pixel
    // loop (per PR #620 code-quality review: `calibration_enabled_` is
    // invariant across all pixels).
    DepthConversionParams params;
    params.min_depth_m         = min_depth_m_;
    params.max_depth_m         = max_depth_m_;
    params.calibration_enabled = calibration_enabled_;
    params.calibration_coef_a  = calibration_coef_a_;
    params.calibration_coef_b  = calibration_coef_b_;
    if (calibration_enabled_) {
        params.anchor_min = raw_min_ref_;
        params.anchor_max = raw_max_ref_;
    } else {
        params.anchor_min = min_val;
        params.anchor_max = max_val;
    }
    const float range = params.anchor_max - params.anchor_min;

    DepthMap map;
    map.width         = static_cast<uint32_t>(out_w);
    map.height        = static_cast<uint32_t>(out_h);
    map.source_width  = width;  // Original frame dimensions — critical for fusion bbox mapping
    map.source_height = height;
    map.scale         = 1.0f;
    map.confidence    = 0.7f;  // ML model confidence — higher than simulated (0.5)
    map.data.resize(num_pixels);

    if (range < eps) {
        // Flat scale range — uniform max_depth output.  Per-frame path: raw
        // scene is constant (flat wall / unlit frame).  Calibrated path: refs
        // are near-equal (config rounding — the constructor's `>` guard lets
        // refs that differ by a hair through).  Either way a fall-back to
        // "everything is far" is conservative: the grid gets no near
        // obstacles, the planner doesn't route around phantoms.
        std::fill(map.data.begin(), map.data.end(), max_depth_m_);
    } else {
        for (size_t i = 0; i < num_pixels; ++i) {
            map.data[i] = convert_raw_to_metric(raw_data[i], range, params);
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
