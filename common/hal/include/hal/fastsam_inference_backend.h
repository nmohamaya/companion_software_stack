// common/hal/include/hal/fastsam_inference_backend.h
//
// FastSAM class-agnostic segmentation via OpenCV DNN — real SAM for the
// PATH A pipeline.  This is what Epic #520 / Issue #555 scoped and never
// actually shipped: a drop-in class-agnostic segmenter that finds objects
// in the image regardless of category.
//
// FastSAM (github.com/CASIA-IVA-Lab/FastSAM) is YOLOv8-seg architecture
// trained on SA-1B (Meta's Segment Anything dataset).  It produces
// bounding boxes + instance masks for every distinct object/surface in a
// frame in a single forward pass, where standard SAM-2 needs prompts or
// an expensive grid-of-prompts AMG loop.  Output format matches YOLOv8-seg
// exactly — 2 tensors: detection head (4 bbox + 1 class + 32 mask-coeffs)
// and mask prototypes (32 × 160 × 160 at 1024×1024 input).
//
// Why a dedicated backend (vs. reusing YoloSegInferenceBackend):
//   * YoloSeg lives in process2_perception/ and pulls in detector_class_maps
//     (COCO/VisDrone classes); FastSAM is class-agnostic, would carry a
//     layer-violation include (HAL → perception).
//   * This backend outputs class_id = -1 on every detection so downstream
//     MaskClassAssigner correctly labels unmatched masks as
//     GEOMETRIC_OBSTACLE — the behaviour scenario 33 needs.
//
// Model: run `models/download_fastsam.sh` to fetch FastSAM-s.pt and export
// to `models/fastsam_s.onnx` (~50 MB).  The script installs `ultralytics`
// into the local pip environment — same pattern as
// `download_depth_anything_v2.sh`.
//
// Runtime: ~80 ms per 1024×1024 frame on CPU (OpenCV DNN backend), ~20 ms
// with OpenVINO/CUDA target.  Gracefully no-ops when OpenCV isn't available.

#pragma once

#include "hal/iinference_backend.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#ifdef HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>  // Issue #626 — getCudaEnabledDeviceCount probe
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace drone::hal {

class FastSamInferenceBackend : public IInferenceBackend {
public:
    FastSamInferenceBackend() = default;

    FastSamInferenceBackend(const drone::Config& cfg, const std::string& section) {
        model_path_ = cfg.get<std::string>(section + ".model_path", "models/fastsam_s.onnx");
        confidence_threshold_ = std::clamp(cfg.get<float>(section + ".confidence_threshold", 0.40f),
                                           0.0f, 1.0f);
        nms_threshold_ = std::clamp(cfg.get<float>(section + ".nms_threshold", 0.45f), 0.0f, 1.0f);
        input_size_    = std::clamp(cfg.get<int>(section + ".input_size", 1024), kMinInputSize,
                                    kMaxInputSize);
        mask_channels_ = std::clamp(cfg.get<int>(section + ".mask_channels", 32), 1, 128);
        // Issue #626 follow-up — opt into OpenCV DNN CUDA backend.  Default
        // false preserves CPU-only behaviour.  Load-time canary + fallback
        // mirrors DA V2's pattern so a cuDNN/CUDA mismatch degrades cleanly
        // to CPU instead of crashing the perception process at first frame.
        // PR #633 P2 review: use the canonical config-key constant
        // (`SAM_USE_CUDA`) instead of string-concatenating `section + ".use_cuda"`.
        // Other SAM keys all have constants — this one was the outlier.
        use_cuda_ = cfg.get<bool>(drone::cfg_key::perception::path_a::SAM_USE_CUDA, false);
    }

    FastSamInferenceBackend(const FastSamInferenceBackend&)            = delete;
    FastSamInferenceBackend& operator=(const FastSamInferenceBackend&) = delete;
    // PR #633 P2 review: defaulted moves on a class with std::string + cv::Mat
    // members are noexcept-eligible; mark explicitly so STL containers select
    // the move path (silent quadratic copies otherwise).
    FastSamInferenceBackend(FastSamInferenceBackend&&) noexcept            = default;
    FastSamInferenceBackend& operator=(FastSamInferenceBackend&&) noexcept = default;

    [[nodiscard]] bool init(const std::string& model_path, int input_size) override {
        if (!model_path.empty()) model_path_ = model_path;
        if (input_size > 0) input_size_ = std::clamp(input_size, kMinInputSize, kMaxInputSize);
        return load_model();
    }

    [[nodiscard]] drone::util::Result<InferenceOutput, std::string> infer(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
        uint32_t /*stride*/) override {
        using R = drone::util::Result<InferenceOutput, std::string>;
        if (!frame_data) return R::err("Null frame data");
        if (width == 0 || height == 0 || channels == 0) return R::err("Invalid frame dimensions");

        InferenceOutput output;
        output.timestamp_ns =
            static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());

#ifdef HAS_OPENCV
        if (!model_loaded_) return R::ok(std::move(output));

        const int cv_type = (channels == 4) ? CV_8UC4 : CV_8UC3;
        cv::Mat   frame(static_cast<int>(height), static_cast<int>(width), cv_type,
                        const_cast<uint8_t*>(frame_data));
        cv::Mat   rgb;
        if (channels == 4) {
            cv::cvtColor(frame, rgb, cv::COLOR_RGBA2RGB);
        } else if (channels == 1) {
            cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
        } else {
            rgb = frame;
        }

        // Blob → forward
        cv::Mat blob;
        cv::dnn::blobFromImage(rgb, blob, 1.0 / 255.0, cv::Size(input_size_, input_size_),
                               cv::Scalar(), true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> outputs;
        try {
            net_.forward(outputs, output_layer_names_);
        } catch (const cv::Exception& e) {
            return R::err(std::string("[FastSAM] forward() failed: ") + e.what());
        }
        if (outputs.size() < 2) {
            return R::err("[FastSAM] expected 2 outputs (det + mask proto), got " +
                          std::to_string(outputs.size()));
        }

        // outputs[0] shape: [1, 4 + 1 + mask_channels, num_proposals]
        // outputs[1] shape: [1, mask_channels, mask_h, mask_w]
        cv::Mat det_output  = outputs[0];
        cv::Mat mask_protos = outputs[1];

        const int rows          = det_output.size[1];
        const int cols          = det_output.size[2];
        const int expected_rows = 4 + kNumClasses + mask_channels_;
        if (rows != expected_rows) {
            return R::err("[FastSAM] detection output shape: expected " +
                          std::to_string(expected_rows) + " rows, got " + std::to_string(rows));
        }
        if (cols <= 0 || cols > kMaxProposals) {
            return R::err("[FastSAM] unexpected proposal count: " + std::to_string(cols));
        }

        // PR #633 P2 review: CV_Assert was outside try/catch and
        // would terminate the SAM thread on a malformed ONNX output
        // — violates the Result<>-return contract of `infer()`.
        // Return Err so the caller can degrade gracefully (the SAM
        // thread loops on the next frame instead of taking down P2).
        if (det_output.type() != CV_32F) {
            return R::err("[FastSAM] detection output type expected CV_32F (" +
                          std::to_string(CV_32F) + "), got " + std::to_string(det_output.type()));
        }
        float* data = reinterpret_cast<float*>(det_output.data);

        const float x_scale  = static_cast<float>(width) / static_cast<float>(input_size_);
        const float y_scale  = static_cast<float>(height) / static_cast<float>(input_size_);
        const int   mask_h   = mask_protos.size[2];
        const int   mask_w   = mask_protos.size[3];
        cv::Mat     proto_2d = mask_protos.reshape(1, {mask_channels_, mask_h * mask_w});

        std::vector<float>              confidences;
        std::vector<cv::Rect>           boxes;
        std::vector<std::vector<float>> mask_coeffs_all;
        const auto                      reserve_count = static_cast<size_t>(std::min(cols, 512));
        confidences.reserve(reserve_count);
        boxes.reserve(reserve_count);
        mask_coeffs_all.reserve(reserve_count);

        // FastSAM is class-agnostic: single class score at index 4.
        for (int i = 0; i < cols; ++i) {
            const float conf = data[4 * cols + i];
            if (conf < confidence_threshold_) continue;

            const float cx = data[0 * cols + i] * x_scale;
            const float cy = data[1 * cols + i] * y_scale;
            const float w  = data[2 * cols + i] * x_scale;
            const float h  = data[3 * cols + i] * y_scale;

            const int left = static_cast<int>(cx - w / 2.0f);
            const int top  = static_cast<int>(cy - h / 2.0f);
            const int bw   = static_cast<int>(w);
            const int bh   = static_cast<int>(h);
            boxes.emplace_back(left, top, bw, bh);
            confidences.push_back(conf);

            std::vector<float> mc(static_cast<size_t>(mask_channels_));
            for (int m = 0; m < mask_channels_; ++m) {
                mc[static_cast<size_t>(m)] = data[(4 + kNumClasses + m) * cols + i];
            }
            mask_coeffs_all.push_back(std::move(mc));
        }

        std::vector<int> nms_indices;
        cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, nms_indices);

        output.detections.reserve(nms_indices.size());
        for (int idx : nms_indices) {
            InferenceDetection det;
            det.bbox.x     = static_cast<float>(std::max(0, boxes[idx].x));
            det.bbox.y     = static_cast<float>(std::max(0, boxes[idx].y));
            det.bbox.w     = static_cast<float>(boxes[idx].width);
            det.bbox.h     = static_cast<float>(boxes[idx].height);
            det.confidence = confidences[idx];
            det.class_id   = -1;  // class-agnostic — MaskClassAssigner applies label downstream

            if (det.bbox.x + det.bbox.w > static_cast<float>(width))
                det.bbox.w = static_cast<float>(width) - det.bbox.x;
            if (det.bbox.y + det.bbox.h > static_cast<float>(height))
                det.bbox.h = static_cast<float>(height) - det.bbox.y;
            if (det.bbox.w <= 0.0f || det.bbox.h <= 0.0f) continue;

            // Decode mask: coeffs @ proto → sigmoid → crop → resize to bbox
            cv::Mat coeffs(1, mask_channels_, CV_32F,
                           mask_coeffs_all[static_cast<size_t>(idx)].data());
            cv::Mat mask_raw = coeffs * proto_2d;
            mask_raw         = mask_raw.reshape(1, {mask_h, mask_w});
            cv::exp(-mask_raw, mask_raw);
            mask_raw = 1.0f / (1.0f + mask_raw);

            const float proto_scale_x = static_cast<float>(mask_w) /
                                        static_cast<float>(input_size_);
            const float proto_scale_y = static_cast<float>(mask_h) /
                                        static_cast<float>(input_size_);
            const int rx = std::max(0, static_cast<int>(boxes[idx].x / x_scale * proto_scale_x));
            const int ry = std::max(0, static_cast<int>(boxes[idx].y / y_scale * proto_scale_y));
            const int rw =
                std::min(mask_w - rx,
                         std::max(1, static_cast<int>(boxes[idx].width / x_scale * proto_scale_x)));
            const int rh = std::min(
                mask_h - ry,
                std::max(1, static_cast<int>(boxes[idx].height / y_scale * proto_scale_y)));
            cv::Mat mask_crop = mask_raw(cv::Rect(rx, ry, rw, rh));

            const int bbox_px_w = std::max(1, static_cast<int>(det.bbox.w));
            const int bbox_px_h = std::max(1, static_cast<int>(det.bbox.h));
            cv::Mat   mask_resized;
            cv::resize(mask_crop, mask_resized, cv::Size(bbox_px_w, bbox_px_h), 0, 0,
                       cv::INTER_LINEAR);
            cv::Mat mask_binary;
            mask_resized.convertTo(mask_binary, CV_8U, 255.0);
            cv::threshold(mask_binary, mask_binary, 127, 255, cv::THRESH_BINARY);

            // Output mask is full-image resolution (matches EdgeContour /
            // SimulatedSAM convention — CpuSemanticProjector auto-detects).
            det.mask_width  = width;
            det.mask_height = height;
            det.mask.assign(static_cast<size_t>(width) * height, uint8_t{0});
            const int dst_x = static_cast<int>(det.bbox.x);
            const int dst_y = static_cast<int>(det.bbox.y);
            for (int row = 0; row < bbox_px_h && (dst_y + row) < static_cast<int>(height); ++row) {
                const auto dst_off = static_cast<size_t>(dst_y + row) * width +
                                     static_cast<size_t>(dst_x);
                const int copy_w = std::min(bbox_px_w, static_cast<int>(width) - dst_x);
                if (copy_w > 0) {
                    std::copy(mask_binary.ptr<uint8_t>(row), mask_binary.ptr<uint8_t>(row) + copy_w,
                              &det.mask[dst_off]);
                }
            }
            output.detections.push_back(std::move(det));
        }
        return R::ok(std::move(output));
#else
        (void)width;
        (void)height;
        (void)channels;
        return R::ok(std::move(output));
#endif
    }

    [[nodiscard]] std::string name() const override { return "FastSamInferenceBackend"; }

    // PR #633 P2 review: accessors for the configured-and-clamped values
    // so tests can assert the clamping actually happened (the previous
    // tests asserted name() instead, which proved nothing).  Also useful
    // for run-report observability.
    [[nodiscard]] float confidence_threshold() const noexcept { return confidence_threshold_; }
    [[nodiscard]] float nms_threshold() const noexcept { return nms_threshold_; }
    [[nodiscard]] int   input_size() const noexcept { return input_size_; }
    [[nodiscard]] int   mask_channels() const noexcept { return mask_channels_; }
    [[nodiscard]] bool  use_cuda_configured() const noexcept { return use_cuda_; }

private:
    static constexpr int kMinInputSize = 128;
    static constexpr int kMaxInputSize = 2048;
    // FastSAM at 1024×1024 input emits 21504 proposals from the anchor-free
    // head (strides 8/16/32: 128² + 64² + 32² = 21504).  Larger inputs scale
    // roughly ∝ h·w, so give generous headroom without becoming unbounded —
    // a malformed model should still be caught.  YOLOv8-seg at 640 emits
    // 8400, so this cap covers both pathways.
    static constexpr int kMaxProposals = 65536;
    static constexpr int kNumClasses   = 1;  // FastSAM is class-agnostic

    [[nodiscard]] bool load_model() {
#ifdef HAS_OPENCV
        if (model_path_.empty()) {
            DRONE_LOG_ERROR("[FastSAM] Empty model path — no inference available.  Run "
                            "models/download_fastsam.sh to fetch + export FastSAM-s.");
            model_loaded_ = false;
            return false;
        }
        // PR #633 P2 review: path traversal guard.  Same policy as
        // DepthAnythingV2Estimator's load_model — reject `..` here, with
        // an additional config-time absolute-path check to live in the
        // config constructor (where config injection is the actual
        // attack surface).  The earlier diff comment claimed DA V2
        // parity but the guard was omitted entirely.
        if (model_path_.find("..") != std::string::npos) {
            DRONE_LOG_ERROR("[FastSAM] Rejected model path with '..': {}", model_path_);
            model_loaded_ = false;
            return false;
        }
        if (!std::filesystem::exists(model_path_)) {
            DRONE_LOG_ERROR("[FastSAM] Model file does not exist: '{}'.  Run "
                            "models/download_fastsam.sh.",
                            model_path_);
            model_loaded_ = false;
            return false;
        }
        try {
            net_ = cv::dnn::readNetFromONNX(model_path_);

            // Issue #626 follow-up — CUDA backend engagement with canary
            // fallback (same pattern as DepthAnythingV2Estimator).  FastSAM
            // at 1024×1024 CPU is ~80-150 ms/frame; on CUDA ~10-20 ms,
            // which roughly 10× the MaskProj batch rate feeding PATH A.
            bool cuda_engaged = false;
            if (use_cuda_) {
                const int cuda_devices = cv::cuda::getCudaEnabledDeviceCount();
                if (cuda_devices == 0) {
                    DRONE_LOG_WARN("[FastSAM] use_cuda=true but "
                                   "cv::cuda::getCudaEnabledDeviceCount()=0 — "
                                   "falling back to CPU.");
                } else {
                    try {
                        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
                        // Canary inference — zero blob at the configured
                        // input size.  Flushes any cuDNN/CUDA mismatch up
                        // into the constructor instead of killing the
                        // SAM thread at first frame.
                        cv::Mat canary_blob = cv::Mat::zeros(cv::Size(input_size_, input_size_),
                                                             CV_32FC3);
                        cv::Mat canary_input;
                        cv::dnn::blobFromImage(canary_blob, canary_input, 1.0 / 255.0,
                                               cv::Size(input_size_, input_size_),
                                               cv::Scalar(0, 0, 0), false, false);
                        net_.setInput(canary_input);
                        (void)net_.forward();
                        cuda_engaged = true;
                    } catch (const cv::Exception& e) {
                        DRONE_LOG_WARN("[FastSAM] CUDA canary failed ({}) — "
                                       "falling back to CPU.  Common cause: cuDNN/CUDA version "
                                       "mismatch.",
                                       e.what());
                    } catch (const std::exception& e) {
                        DRONE_LOG_WARN("[FastSAM] CUDA canary threw std::exception ({}) — "
                                       "falling back to CPU",
                                       e.what());
                    } catch (...) {
                        // PR #633 P2 review: unknown exception types could
                        // previously escape the outer catch silently and
                        // leave net_ in an indeterminate state after a
                        // partial CUDA setup.  Catch-all rebuilds the
                        // network from the freshly-loaded ONNX so the
                        // CPU fallback path starts from a known state.
                        DRONE_LOG_WARN("[FastSAM] CUDA canary threw unknown exception type — "
                                       "falling back to CPU after rebuild");
                    }
                    // PR #633 P2 review: if the canary path partially
                    // configured the CUDA backend then threw, net_ is in
                    // an indeterminate state — rebuild from ONNX before
                    // the CPU-fallback target is set so we start clean.
                    if (!cuda_engaged) {
                        net_ = cv::dnn::readNetFromONNX(model_path_);
                    }
                }
            }
            if (!cuda_engaged) {
                net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            }
            output_layer_names_ = net_.getUnconnectedOutLayersNames();
            model_loaded_       = true;
            DRONE_LOG_INFO("[FastSAM] Model loaded: {} (conf={:.2f}, nms={:.2f}, input={}x{}, "
                           "mask_channels={}, backend={})",
                           model_path_, confidence_threshold_, nms_threshold_, input_size_,
                           input_size_, mask_channels_, cuda_engaged ? "CUDA" : "CPU");
            return true;
        } catch (const cv::Exception& e) {
            DRONE_LOG_ERROR("[FastSAM] Failed to load '{}': {}", model_path_, e.what());
            model_loaded_ = false;
            return false;
        }
#else
        DRONE_LOG_WARN("[FastSAM] OpenCV not available at compile time — backend inert.");
        return false;
#endif
    }

#ifdef HAS_OPENCV
    cv::dnn::Net             net_;
    std::vector<std::string> output_layer_names_;
#endif
    std::string model_path_;
    bool        model_loaded_         = false;
    float       confidence_threshold_ = 0.40f;
    float       nms_threshold_        = 0.45f;
    int         input_size_           = 1024;
    int         mask_channels_        = 32;
    // Issue #626 follow-up — opt into OpenCV DNN's CUDA backend.  Default
    // false keeps CPU-only builds working; load_model()'s canary catches
    // cuDNN/CUDA mismatches before the SAM thread's first inference.
    bool use_cuda_ = false;
};

}  // namespace drone::hal
