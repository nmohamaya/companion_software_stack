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
#include <opencv2/core/cuda.hpp>  // Issue #626 — cv::cuda::getCudaEnabledDeviceCount probe
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
///   perception.depth_estimator.use_cuda                    (default false — opt-in CUDA backend
///                                                                  with canary-inference fallback)
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

    /// PR #631 P2 review: tests need to verify config wiring for
    /// `use_cuda` without loading an actual ONNX model.  Accessor
    /// returns the configured intent (not whether CUDA actually
    /// engaged at runtime — that depends on canary inference).
    [[nodiscard]] bool use_cuda_configured() const noexcept { return use_cuda_; }

    /// PR #631 P2 review: number of CONSECUTIVE estimate() calls whose
    /// `cv::dnn::Net::forward()` threw.  Resets to 0 on a successful
    /// estimate.  P7 / run-report can poll this to detect a depth
    /// pipeline that's silently degraded (CUDA driver crash mid-run,
    /// OOM, etc.) — without it, persistent runtime failure was
    /// invisible to the system monitor since each estimate() returned
    /// `Err` but the process kept running.  Atomic + relaxed since
    /// it's a pure observability counter (depth thread writes,
    /// watchdog/run-report reads).
    [[nodiscard]] uint64_t consecutive_inference_failures() const noexcept {
        return consecutive_inference_failures_.load(std::memory_order_relaxed);
    }

private:
    void load_model(const std::string& model_path);

#ifdef HAS_OPENCV
    cv::dnn::Net net_;
#endif
    bool  model_loaded_ = false;
    int   input_size_   = 518;
    float max_depth_m_  = 20.0f;  // upper metric bound (output is clamped here)
    float min_depth_m_  = 0.1f;   // lower metric bound (closest reportable obstacle)

    // ── Calibration path (Issue #616) ──────────────────────────────
    // All zero-valued by default; enabled only when calibration_enabled_
    // is set via the Config-driven constructor.
    bool  calibration_enabled_ = false;
    float raw_min_ref_         = std::numeric_limits<float>::quiet_NaN();
    float raw_max_ref_         = std::numeric_limits<float>::quiet_NaN();
    float calibration_coef_a_  = 1.0f;
    float calibration_coef_b_  = 0.0f;

    // ── CUDA inference path (Issue #626) ───────────────────────────
    // Request OpenCV DNN's CUDA backend + target at model-load time.
    // Default false — keeps CPU-only builds working and is safe on
    // machines without an NVIDIA GPU.  At load time we probe
    // `cv::cuda::getCudaEnabledDeviceCount()`: zero devices → WARN +
    // silently fall back to CPU (so a mis-configured deploy doesn't
    // silently die).  When it engages, DA V2 ViT-S inference drops
    // from ~2-3 s/frame (CPU, scenario 33 logs) to ~20-100 ms/frame
    // depending on GPU — the headline fix for no-HD-map scenarios
    // that depend on PATH A batch rate.
    bool use_cuda_ = false;

    // PR #631 P2 review: persistent runtime depth failure surfaced
    // only as repeated `Err` returns from estimate() — invisible to
    // P7 / run-report.  Counter increments on each forward() throw
    // and resets to 0 on a successful estimate.  See accessor
    // `consecutive_inference_failures()` above.
    // PR #680 Copilot review: dropped `mutable` — only `estimate()`
    // (non-const) writes the counter, and `std::atomic::load()` is
    // already const, so const-correctness doesn't need the qualifier.
    std::atomic<uint64_t> consecutive_inference_failures_{0};
};

// ═══════════════════════════════════════════════════════════════════════
// Calibration math — free functions (Issue #616, PR #620 review refactor)
//
// Exposed at file scope so the per-pixel math is unit-testable without
// needing the ONNX model loaded.  `DepthAnythingV2Estimator::estimate()`
// calls these helpers once the model has produced its raw inverse-depth
// output.  See the class docstring for the two conversion paths.
// ═══════════════════════════════════════════════════════════════════════

struct DepthConversionParams {
    float min_depth_m         = 0.1f;
    float max_depth_m         = 20.0f;
    bool  calibration_enabled = false;
    float anchor_min          = 0.0f;
    float anchor_max          = 1.0f;
    float calibration_coef_a  = 1.0f;
    float calibration_coef_b  = 0.0f;
};

/// Convert a single raw DA V2 inverse-depth sample to metric depth.
///
/// Preserves pre-#616 per-frame semantics when `calibration_enabled=false`:
/// caller supplies `anchor_min`/`anchor_max` = this frame's min/max, the
/// `range = anchor_max - anchor_min` is > eps, and no clamping is applied
/// to the result.
///
/// When `calibration_enabled=true`:
///   1. Normalise with the fitted anchor window (clamp to [0,1] so a raw
///      sample outside the window doesn't extrapolate wildly).
///   2. Linear-remap to [min_depth_m, max_depth_m].
///   3. Apply the linear fit `metric = a*metric + b`.
///   4. Clamp output to [min_depth_m, max_depth_m].
///
/// NaN / Inf guard (memory-safety P3 / fault-recovery P3): a non-finite raw
/// sample fails closed — returns `max_depth_m` (far-depth), the most
/// conservative default for obstacle avoidance.  Prevents NaN propagation
/// into downstream voxel positions.
[[nodiscard]] inline float convert_raw_to_metric(float raw, float range,
                                                 const DepthConversionParams& p) {
    // Non-finite raw output (ONNX corruption / numerical glitch) → fail
    // closed to far-depth.  Applies to both per-frame and calibrated paths
    // so the behaviour is consistent regardless of `calibration_enabled`.
    if (!std::isfinite(raw)) {
        return p.max_depth_m;
    }
    // Both paths compute the anchored normalisation identically; what
    // differs is whether the anchor is per-frame or fitted, and whether
    // the output gets a post-linear-fit clamp.
    float normalized = (raw - p.anchor_min) / range;
    if (p.calibration_enabled) {
        // Raw sample outside the fitted window: clamp rather than extrapolate.
        normalized = std::clamp(normalized, 0.0f, 1.0f);
    }
    float metric = p.min_depth_m + (p.max_depth_m - p.min_depth_m) * (1.0f - normalized);
    if (p.calibration_enabled) {
        metric = p.calibration_coef_a * metric + p.calibration_coef_b;
        // Final safety clamp — covers degenerate `a*x+b` fits that push
        // output outside the physical depth range (e.g. operator-supplied
        // `coef_b` exceeds max_depth_m).
        metric = std::clamp(metric, p.min_depth_m, p.max_depth_m);
    }
    return metric;
}

}  // namespace drone::hal
