// process2_perception/include/perception/opencv_yolo_detector.h
// YOLOv8-nano detector using OpenCV DNN module.
// Loads an ONNX model and performs real object detection + classification.
#pragma once

#include "perception/detector_class_maps.h"
#include "perception/detector_interface.h"
#include "util/config.h"

#ifdef HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include "util/ilogger.h"

#include <chrono>
#include <string>
#include <vector>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// OpenCvYoloDetector
// ═══════════════════════════════════════════════════════════
class OpenCvYoloDetector : public IDetector {
public:
    /// Construct from config (reads model path, thresholds, input size).
    explicit OpenCvYoloDetector(const drone::Config& cfg);

    /// Construct with explicit parameters (for testing / direct use).
    OpenCvYoloDetector(const std::string& model_path, float confidence_threshold = 0.25f,
                       float nms_threshold = 0.45f, int input_size = 640);

    std::vector<Detection2D> detect(const uint8_t* frame_data, uint32_t width, uint32_t height,
                                    uint32_t channels) override;

    std::string name() const override { return "OpenCvYoloDetector"; }

    /// Check if model was loaded successfully.
    bool is_loaded() const { return model_loaded_; }

private:
    void load_model(const std::string& model_path);

#ifdef HAS_OPENCV
    cv::dnn::Net net_;
#endif
    bool            model_loaded_         = false;
    float           confidence_threshold_ = 0.25f;
    float           nms_threshold_        = 0.45f;
    int             input_size_           = 640;
    int             num_classes_          = 80;
    DetectorDataset dataset_              = DetectorDataset::COCO;
};

}  // namespace drone::perception
