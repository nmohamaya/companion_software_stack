// process2_perception/include/perception/opencv_yolo_detector.h
// YOLOv8-nano detector using OpenCV DNN module.
// Loads an ONNX model and performs real object detection + classification.
#pragma once

#include "perception/detector_interface.h"
#include "util/config.h"

#ifdef HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <string>
#include <vector>
#include <chrono>
#include <spdlog/spdlog.h>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// COCO class ID → ObjectClass mapping
// ═══════════════════════════════════════════════════════════

/// Map COCO 80-class index to our ObjectClass enum.
/// Only a subset of COCO classes map to our classes; rest → UNKNOWN.
inline ObjectClass coco_to_object_class(int coco_id) {
    switch (coco_id) {
        case 0:  return ObjectClass::PERSON;         // person
        case 2:  return ObjectClass::VEHICLE_CAR;    // car
        case 5:  return ObjectClass::VEHICLE_CAR;    // bus → car
        case 7:  return ObjectClass::VEHICLE_TRUCK;  // truck
        case 14: return ObjectClass::ANIMAL;         // bird
        case 15: return ObjectClass::ANIMAL;         // cat
        case 16: return ObjectClass::ANIMAL;         // dog
        case 17: return ObjectClass::ANIMAL;         // horse
        case 18: return ObjectClass::ANIMAL;         // sheep
        case 19: return ObjectClass::ANIMAL;         // cow
        default: return ObjectClass::UNKNOWN;
    }
}

/// COCO class names (80 classes) for logging.
inline const char* coco_class_name(int id) {
    static const char* names[] = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus",
        "train", "truck", "boat", "traffic light", "fire hydrant",
        "stop sign", "parking meter", "bench", "bird", "cat", "dog",
        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
        "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
        "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
        "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
        "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"
    };
    if (id >= 0 && id < 80) return names[id];
    return "unknown";
}

// ═══════════════════════════════════════════════════════════
// OpenCvYoloDetector
// ═══════════════════════════════════════════════════════════
class OpenCvYoloDetector : public IDetector {
public:
    /// Construct from config (reads model path, thresholds, input size).
    explicit OpenCvYoloDetector(const drone::Config& cfg);

    /// Construct with explicit parameters (for testing / direct use).
    OpenCvYoloDetector(const std::string& model_path,
                        float confidence_threshold = 0.25f,
                        float nms_threshold = 0.45f,
                        int input_size = 640);

    std::vector<Detection2D> detect(const uint8_t* frame_data,
                                     uint32_t width, uint32_t height,
                                     uint32_t channels) override;

    std::string name() const override { return "OpenCvYoloDetector"; }

    /// Check if model was loaded successfully.
    bool is_loaded() const { return model_loaded_; }

private:
    void load_model(const std::string& model_path);

#ifdef HAS_OPENCV
    cv::dnn::Net net_;
#endif
    bool model_loaded_ = false;
    float confidence_threshold_ = 0.25f;
    float nms_threshold_ = 0.45f;
    int input_size_ = 640;
    int num_classes_ = 80;
};

} // namespace drone::perception
