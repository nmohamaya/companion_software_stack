// process2_perception/src/yolo_seg_inference_backend.cpp
// YOLOv8-seg inference backend — produces bbox + class + mask.

#include "perception/yolo_seg_inference_backend.h"

#include "util/config_keys.h"

#include <algorithm>
#include <chrono>

namespace drone::perception {

static constexpr int kMinInputSize = 32;
static constexpr int kMaxInputSize = 1920;
static constexpr int kMinClasses   = 1;
static constexpr int kMaxClasses   = 1000;
static constexpr int kMinMaskCh    = 1;
static constexpr int kMaxMaskCh    = 256;
static constexpr int kMaxProposals = 16384;

// ── Config constructor ──────────────────────────────────────
YoloSegInferenceBackend::YoloSegInferenceBackend(const drone::Config& cfg,
                                                 const std::string&   section) {
    std::string model_path = cfg.get<std::string>(section + ".model_path",
                                                  "models/yolov8n-seg.onnx");
    confidence_threshold_  = cfg.get<float>(section + ".confidence_threshold", 0.25f);
    nms_threshold_         = cfg.get<float>(section + ".nms_threshold", 0.45f);
    input_size_            = std::clamp(cfg.get<int>(section + ".input_size", 640), kMinInputSize,
                                        kMaxInputSize);
    mask_channels_         = std::clamp(cfg.get<int>(section + ".mask_channels", 32), kMinMaskCh,
                                        kMaxMaskCh);

    auto dataset_str = cfg.get<std::string>(section + ".dataset", "coco");
    if (dataset_str == "visdrone") {
        dataset_     = DetectorDataset::VISDRONE;
        num_classes_ = std::clamp(cfg.get<int>(section + ".num_classes", 10), kMinClasses,
                                  kMaxClasses);
    } else {
        dataset_     = DetectorDataset::COCO;
        num_classes_ = std::clamp(cfg.get<int>(section + ".num_classes", 80), kMinClasses,
                                  kMaxClasses);
    }

    load_model(model_path);
}

// ── Explicit constructor ────────────────────────────────────
YoloSegInferenceBackend::YoloSegInferenceBackend(const std::string& model_path,
                                                 float confidence_threshold, float nms_threshold,
                                                 int input_size, DetectorDataset dataset)
    : confidence_threshold_(confidence_threshold)
    , nms_threshold_(nms_threshold)
    , input_size_(std::clamp(input_size, kMinInputSize, kMaxInputSize))
    , num_classes_((dataset == DetectorDataset::VISDRONE) ? 10 : 80)
    , dataset_(dataset) {
    load_model(model_path);
}

// ── Init ────────────────────────────────────────────────────
// Passing an empty model_path skips loading; returns false and leaves prior
// load state unchanged.
bool YoloSegInferenceBackend::init(const std::string& model_path, int input_size) {
    input_size_ = std::clamp(input_size, kMinInputSize, kMaxInputSize);
    if (!model_path.empty()) {
        load_model(model_path);
    }
    return model_loaded_;
}

// ── Path validation ─────────────────────────────────────────
// Uses lexical normalization (cwd-independent) to ensure model paths
// stay under a models/ directory.  Relative paths must start with
// "models/"; absolute paths (e.g. compile-time YOLO_MODEL_PATH) must
// contain "models" as a path component.
static bool validate_model_path(const std::string& raw_path, const std::string& tag) {
    std::filesystem::path p(raw_path);
    auto                  normalized = p.lexically_normal();

    if (p.is_relative()) {
        auto it = normalized.begin();
        if (it == normalized.end() || it->string() != "models") {
            DRONE_LOG_ERROR("[{}] Model path must be under models/ directory: {}", tag, raw_path);
            return false;
        }
    } else {
        bool found_models = false;
        for (const auto& component : normalized) {
            if (component.string() == "models") {
                found_models = true;
                break;
            }
        }
        if (!found_models) {
            DRONE_LOG_ERROR("[{}] Absolute model path must include models/ directory: {}", tag,
                            raw_path);
            return false;
        }
    }

    // Defense-in-depth: reject any remaining ".." after normalization
    for (const auto& component : normalized) {
        if (component.string() == "..") {
            DRONE_LOG_ERROR("[{}] Path traversal detected in model path: {}", tag, raw_path);
            return false;
        }
    }
    return true;
}

// ── Model loading ───────────────────────────────────────────
void YoloSegInferenceBackend::load_model(const std::string& model_path) {
    if (model_path.empty()) {
        DRONE_LOG_WARN("[YoloSegBackend] Empty model path — running without model");
        model_loaded_ = false;
        return;
    }

    if (!validate_model_path(model_path, "YoloSegBackend")) {
        model_loaded_ = false;
        return;
    }

#ifdef HAS_OPENCV
    try {
        net_ = cv::dnn::readNetFromONNX(model_path);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        output_layer_names_ = net_.getUnconnectedOutLayersNames();
        model_loaded_       = true;

        DRONE_LOG_INFO("[YoloSegBackend] Model loaded: {} (conf={:.2f}, input={})", model_path,
                       confidence_threshold_, input_size_);
    } catch (const cv::Exception& e) {
        DRONE_LOG_ERROR("[YoloSegBackend] Failed to load model '{}': {}", model_path, e.what());
        model_loaded_ = false;
    }
#else
    DRONE_LOG_WARN("[YoloSegBackend] OpenCV not available — model not loaded");
    model_loaded_ = false;
#endif
}

// ── Inference ───────────────────────────────────────────────
drone::util::Result<drone::hal::InferenceOutput, std::string> YoloSegInferenceBackend::infer(
    const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
    uint32_t /*stride*/) {
    using R = drone::util::Result<drone::hal::InferenceOutput, std::string>;
    if (!frame_data) return R::err("Null frame data");
    if (width == 0 || height == 0 || channels == 0) return R::err("Invalid frame dimensions");

    drone::hal::InferenceOutput output;
    output.timestamp_ns =
        static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());

#ifdef HAS_OPENCV
    if (!model_loaded_) {
        return R::ok(std::move(output));
    }

    // const_cast is safe here: cv::Mat wraps the pointer without copying, and
    // no in-place modification occurs on this Mat (3ch shares the buffer
    // read-only; 4ch converts into a separate `rgb` Mat).
    int     cv_type = (channels == 4) ? CV_8UC4 : CV_8UC3;
    cv::Mat frame(static_cast<int>(height), static_cast<int>(width), cv_type,
                  const_cast<uint8_t*>(frame_data));

    cv::Mat rgb;
    if (channels == 4) {
        cv::cvtColor(frame, rgb, cv::COLOR_RGBA2RGB);
    } else {
        rgb = frame;
    }

    cv::Mat blob;
    cv::dnn::blobFromImage(rgb, blob, 1.0 / 255.0, cv::Size(input_size_, input_size_),
                           cv::Scalar(0, 0, 0), false, false);

    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    try {
        net_.forward(outputs, output_layer_names_);
    } catch (const cv::Exception& e) {
        return R::err(std::string("[YoloSegBackend] forward() failed: ") + e.what());
    }

    // YOLOv8-seg produces 2 outputs:
    //   outputs[0]: detection head [1, 4+num_classes+mask_channels, num_proposals]
    //   outputs[1]: mask prototypes [1, mask_channels, mask_h, mask_w]
    if (outputs.size() < 2) {
        DRONE_LOG_WARN("[YoloSegBackend] Expected 2 outputs (det + mask proto), got {}",
                       outputs.size());
        return R::ok(std::move(output));
    }

    cv::Mat det_output  = outputs[0];
    cv::Mat mask_protos = outputs[1];

    const int rows          = det_output.size[1];
    const int cols          = det_output.size[2];
    const int expected_rows = 4 + num_classes_ + mask_channels_;

    if (rows != expected_rows) {
        return R::err("Detection output shape mismatch: expected " + std::to_string(expected_rows) +
                      " rows, got " + std::to_string(rows));
    }

    if (cols <= 0 || cols > kMaxProposals) {
        return R::err("Unexpected proposal count: " + std::to_string(cols));
    }

    CV_Assert(det_output.type() == CV_32F);
    // reinterpret_cast is the standard OpenCV pattern for accessing cv::Mat
    // raw data; CV_Assert above guarantees the type matches.
    float* data = reinterpret_cast<float*>(det_output.data);

    float x_scale = static_cast<float>(width) / static_cast<float>(input_size_);
    float y_scale = static_cast<float>(height) / static_cast<float>(input_size_);

    // Mask prototype dimensions
    const int mask_h   = mask_protos.size[2];
    const int mask_w   = mask_protos.size[3];
    cv::Mat   proto_2d = mask_protos.reshape(1, {mask_channels_, mask_h * mask_w});

    std::vector<int>                class_ids;
    std::vector<float>              confidences;
    std::vector<cv::Rect>           boxes;
    std::vector<std::vector<float>> mask_coeffs_all;

    // Reserve for typical detection counts to reduce heap churn
    const auto reserve_count = static_cast<size_t>(std::min(cols, 512));
    class_ids.reserve(reserve_count);
    confidences.reserve(reserve_count);
    boxes.reserve(reserve_count);
    mask_coeffs_all.reserve(reserve_count);

    for (int i = 0; i < cols; ++i) {
        float max_conf  = 0.0f;
        int   max_class = 0;

        for (int c = 4; c < 4 + num_classes_; ++c) {
            float conf = data[c * cols + i];
            if (conf > max_conf) {
                max_conf  = conf;
                max_class = c - 4;
            }
        }

        if (max_conf < confidence_threshold_) continue;

        float cx = data[0 * cols + i] * x_scale;
        float cy = data[1 * cols + i] * y_scale;
        float w  = data[2 * cols + i] * x_scale;
        float h  = data[3 * cols + i] * y_scale;

        int left = static_cast<int>(cx - w / 2.0f);
        int top  = static_cast<int>(cy - h / 2.0f);
        int bw   = static_cast<int>(w);
        int bh   = static_cast<int>(h);

        boxes.emplace_back(left, top, bw, bh);
        confidences.push_back(max_conf);
        class_ids.push_back(max_class);

        // Extract mask coefficients
        std::vector<float> mc(static_cast<size_t>(mask_channels_));
        for (int m = 0; m < mask_channels_; ++m) {
            mc[static_cast<size_t>(m)] = data[(4 + num_classes_ + m) * cols + i];
        }
        mask_coeffs_all.push_back(std::move(mc));
    }

    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, nms_indices);

    for (int idx : nms_indices) {
        drone::hal::InferenceDetection det;
        det.bbox.x     = static_cast<float>(std::max(0, boxes[idx].x));
        det.bbox.y     = static_cast<float>(std::max(0, boxes[idx].y));
        det.bbox.w     = static_cast<float>(boxes[idx].width);
        det.bbox.h     = static_cast<float>(boxes[idx].height);
        det.confidence = confidences[idx];
        det.class_id   = class_ids[idx];

        // Clamp to image bounds
        if (det.bbox.x + det.bbox.w > static_cast<float>(width))
            det.bbox.w = static_cast<float>(width) - det.bbox.x;
        if (det.bbox.y + det.bbox.h > static_cast<float>(height))
            det.bbox.h = static_cast<float>(height) - det.bbox.y;

        // Decode instance mask: coeffs @ proto → sigmoid → threshold → resize
        // Crop to bbox region first, then resize — avoids full-frame resize cost.
        cv::Mat coeffs(1, mask_channels_, CV_32F, mask_coeffs_all[static_cast<size_t>(idx)].data());
        cv::Mat mask_raw = coeffs * proto_2d;
        mask_raw         = mask_raw.reshape(1, {mask_h, mask_w});

        // Sigmoid activation
        cv::exp(-mask_raw, mask_raw);
        mask_raw = 1.0f / (1.0f + mask_raw);

        // Crop mask to bbox region in proto space, then resize only the crop
        const float proto_scale_x = static_cast<float>(mask_w) / static_cast<float>(input_size_);
        const float proto_scale_y = static_cast<float>(mask_h) / static_cast<float>(input_size_);

        int rx = std::max(0, static_cast<int>(boxes[idx].x / x_scale * proto_scale_x));
        int ry = std::max(0, static_cast<int>(boxes[idx].y / y_scale * proto_scale_y));
        int rw = std::min(
            mask_w - rx, std::max(1, static_cast<int>(boxes[idx].width / x_scale * proto_scale_x)));
        int rh = std::min(mask_h - ry, std::max(1, static_cast<int>(boxes[idx].height / y_scale *
                                                                    proto_scale_y)));

        cv::Mat mask_crop = mask_raw(cv::Rect(rx, ry, rw, rh));

        // Resize crop to bbox pixel dimensions
        int     bbox_px_w = std::max(1, static_cast<int>(det.bbox.w));
        int     bbox_px_h = std::max(1, static_cast<int>(det.bbox.h));
        cv::Mat mask_resized;
        cv::resize(mask_crop, mask_resized, cv::Size(bbox_px_w, bbox_px_h), 0, 0, cv::INTER_LINEAR);

        // Threshold and convert to uint8
        cv::Mat mask_binary;
        mask_resized.convertTo(mask_binary, CV_8U, 255.0);
        cv::threshold(mask_binary, mask_binary, 127, 255, cv::THRESH_BINARY);

        // Place into full-frame mask
        det.mask_width  = width;
        det.mask_height = height;
        det.mask.resize(static_cast<size_t>(width) * height, 0);

        int dst_x = static_cast<int>(det.bbox.x);
        int dst_y = static_cast<int>(det.bbox.y);
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
#else
    (void)width;
    (void)height;
    (void)channels;
    DRONE_LOG_WARN("[YoloSegBackend] OpenCV not available — returning empty output");
#endif

    return R::ok(std::move(output));
}

}  // namespace drone::perception
