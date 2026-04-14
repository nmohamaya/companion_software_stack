// process2_perception/src/opencv_yolo_detector.cpp
// YOLOv8-nano detector implementation using OpenCV DNN module.

#include "perception/opencv_yolo_detector.h"

#include "util/config_keys.h"
#include "util/ilogger.h"

#include <atomic>
#include <chrono>
#include <filesystem>

namespace drone::perception {

// ── Config constructor ──────────────────────────────────────
OpenCvYoloDetector::OpenCvYoloDetector(const drone::Config& cfg) {
    std::string model_path = cfg.get<std::string>(drone::cfg_key::perception::detector::MODEL_PATH,
                                                  "models/yolov8n.onnx");
    confidence_threshold_ =
        cfg.get<float>(drone::cfg_key::perception::detector::CONFIDENCE_THRESHOLD, 0.25f);
    nms_threshold_ = cfg.get<float>(drone::cfg_key::perception::detector::NMS_THRESHOLD, 0.45f);
    input_size_    = cfg.get<int>(drone::cfg_key::perception::detector::INPUT_SIZE, 640);

    // Dataset selection: "coco" (80 classes) or "visdrone" (10 aerial classes)
    auto dataset_str = cfg.get<std::string>(drone::cfg_key::perception::detector::DATASET, "coco");
    if (dataset_str == "visdrone") {
        dataset_     = DetectorDataset::VISDRONE;
        num_classes_ = cfg.get<int>(drone::cfg_key::perception::detector::NUM_CLASSES, 10);
    } else {
        dataset_     = DetectorDataset::COCO;
        num_classes_ = cfg.get<int>(drone::cfg_key::perception::detector::NUM_CLASSES, 80);
    }

    DRONE_LOG_INFO("[OpenCvYoloDetector] Dataset: {}, num_classes: {}", dataset_str, num_classes_);

    load_model(model_path);
}

// ── Explicit constructor ────────────────────────────────────
OpenCvYoloDetector::OpenCvYoloDetector(const std::string& model_path, float confidence_threshold,
                                       float nms_threshold, int input_size)
    : confidence_threshold_(confidence_threshold)
    , nms_threshold_(nms_threshold)
    , input_size_(input_size) {
    load_model(model_path);
}

// ── Model loading ───────────────────────────────────────────
void OpenCvYoloDetector::load_model(const std::string& model_path) {
    // Path traversal guard: reject paths containing ".." components.
    // Real backends will load model files from disk — prevent directory traversal
    // from config-injected paths (e.g. "../../etc/passwd").
    const auto canonical = std::filesystem::weakly_canonical(model_path);
    if (model_path.find("..") != std::string::npos) {
        DRONE_LOG_ERROR("[OpenCvYoloDetector] Rejected model path with '..': {}", model_path);
        model_loaded_ = false;
        return;
    }

#ifdef HAS_OPENCV
    try {
        net_ = cv::dnn::readNetFromONNX(canonical.string());
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        model_loaded_ = true;

        DRONE_LOG_INFO("[OpenCvYoloDetector] Model loaded: {} "
                       "(conf={:.2f}, nms={:.2f}, input={})",
                       model_path, confidence_threshold_, nms_threshold_, input_size_);
    } catch (const cv::Exception& e) {
        DRONE_LOG_ERROR("[OpenCvYoloDetector] Failed to load model '{}': {}", model_path, e.what());
        model_loaded_ = false;
    }
#else
    (void)canonical;
    DRONE_LOG_WARN("[OpenCvYoloDetector] OpenCV not available — model not loaded");
    model_loaded_ = false;
#endif
}

// ── Detection ───────────────────────────────────────────────
std::vector<Detection2D> OpenCvYoloDetector::detect(const uint8_t* frame_data, uint32_t width,
                                                    uint32_t height, uint32_t channels) {
    if (!frame_data || width == 0 || height == 0 || channels < 3) {
        return {};
    }

#ifdef HAS_OPENCV
    if (!model_loaded_) {
        return {};
    }

    auto t0     = std::chrono::steady_clock::now();
    auto now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count());

    // ── Step 1: Wrap raw pixel data as cv::Mat ──────────────
    // frame_data is RGB, OpenCV expects BGR for most operations,
    // but blobFromImage can handle RGB with swapRB=true.
    int     cv_type = (channels == 4) ? CV_8UC4 : CV_8UC3;
    cv::Mat frame(static_cast<int>(height), static_cast<int>(width), cv_type,
                  const_cast<uint8_t*>(frame_data));

    // If RGBA, convert to RGB
    cv::Mat rgb;
    if (channels == 4) {
        cv::cvtColor(frame, rgb, cv::COLOR_RGBA2RGB);
    } else {
        rgb = frame;  // Already RGB, no copy
    }

    // ── Step 2: Create blob (letterbox + normalize) ─────────
    // YOLOv8 expects [1, 3, 640, 640] normalized [0, 1]
    cv::Mat blob;
    cv::dnn::blobFromImage(rgb, blob,
                           1.0 / 255.0,                         // scale
                           cv::Size(input_size_, input_size_),  // target size
                           cv::Scalar(0, 0, 0),                 // mean subtraction
                           false,                               // swapRB (input already RGB)
                           false);                              // crop

    // ── Step 3: Forward pass ────────────────────────────────
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    try {
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
    } catch (const cv::Exception& e) {
        DRONE_LOG_ERROR("[OpenCvYoloDetector] forward() failed: {}", e.what());
        return {};
    }
    if (outputs.empty()) {
        DRONE_LOG_ERROR("[OpenCvYoloDetector] forward() produced no outputs");
        return {};
    }

    // ── Step 4: Parse YOLOv8 output ─────────────────────────
    // YOLOv8 output shape: [1, 84, 8400] for 80 classes
    // Rows 0-3: cx, cy, w, h
    // Rows 4-83: class confidences
    cv::Mat output = outputs[0];

    // output is [1, 84, 8400], reshape to [84, 8400]
    const int rows = output.size[1];  // 84 (4 bbox + 80 classes)
    const int cols = output.size[2];  // 8400 proposals

    // Pointer to raw data — assert float dtype before reinterpret_cast
    CV_Assert(output.type() == CV_32F);
    float* data = reinterpret_cast<float*>(output.data);

    // Scale factors from input_size back to original image
    float x_scale = static_cast<float>(width) / static_cast<float>(input_size_);
    float y_scale = static_cast<float>(height) / static_cast<float>(input_size_);

    std::vector<int>      class_ids;
    std::vector<float>    confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < cols; ++i) {
        // For each proposal, find the class with max confidence
        float max_conf  = 0.0f;
        int   max_class = 0;

        for (int c = 4; c < rows; ++c) {
            float conf = data[c * cols + i];
            if (conf > max_conf) {
                max_conf  = conf;
                max_class = c - 4;
            }
        }

        if (max_conf < confidence_threshold_) continue;

        // Extract bounding box (center format → corner format)
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
    }

    // ── Step 5: Non-Maximum Suppression ─────────────────────
    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, nms_indices);

    // ── Step 6: Build Detection2D results ───────────────────
    std::vector<Detection2D> detections;
    detections.reserve(nms_indices.size());

    for (int idx : nms_indices) {
        Detection2D det;
        det.x              = static_cast<float>(std::max(0, boxes[idx].x));
        det.y              = static_cast<float>(std::max(0, boxes[idx].y));
        det.w              = static_cast<float>(boxes[idx].width);
        det.h              = static_cast<float>(boxes[idx].height);
        det.confidence     = confidences[idx];
        det.class_id       = (dataset_ == DetectorDataset::VISDRONE)
                                 ? visdrone_to_object_class(class_ids[idx])
                                 : coco_to_object_class(class_ids[idx]);
        det.timestamp_ns   = now_ns;
        det.frame_sequence = 0;

        // Clamp to image bounds
        if (det.x + det.w > static_cast<float>(width)) det.w = static_cast<float>(width) - det.x;
        if (det.y + det.h > static_cast<float>(height)) det.h = static_cast<float>(height) - det.y;

        detections.push_back(det);
    }

    auto t1 = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    static std::atomic<uint64_t> call_count{0};
    if (++call_count % 30 == 0) {
        DRONE_LOG_INFO("[OpenCvYoloDetector] {} detections in {}ms "
                       "(frame {}x{}, {} proposals after NMS)",
                       detections.size(), ms, width, height, nms_indices.size());
    }

    return detections;

#else
    (void)frame_data;
    (void)width;
    (void)height;
    (void)channels;
    return {};
#endif
}

}  // namespace drone::perception
