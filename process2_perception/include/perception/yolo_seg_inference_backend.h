// process2_perception/include/perception/yolo_seg_inference_backend.h
// YOLOv8-seg inference backend: produces bbox + class + mask via OpenCV DNN.
// Implements IInferenceBackend (not IDetector) to include mask data.
// When HAS_OPENCV is not defined, returns empty detections with a warning.
#pragma once

#include "hal/iinference_backend.h"
#include "perception/detector_class_maps.h"
#include "util/config.h"
#include "util/ilogger.h"

#ifdef HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <filesystem>
#include <string>
#include <vector>

namespace drone::perception {

class YoloSegInferenceBackend : public drone::hal::IInferenceBackend {
public:
    explicit YoloSegInferenceBackend(const drone::Config& cfg, const std::string& section);

    YoloSegInferenceBackend(const std::string& model_path, float confidence_threshold = 0.25f,
                            float nms_threshold = 0.45f, int input_size = 640,
                            DetectorDataset dataset = DetectorDataset::COCO);

    [[nodiscard]] bool init(const std::string& model_path, int input_size) override;

    [[nodiscard]] drone::util::Result<drone::hal::InferenceOutput, std::string> infer(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
        uint32_t stride = 0) override;

    [[nodiscard]] std::string name() const override { return "YoloSegInferenceBackend"; }

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
    int             mask_channels_        = 32;
    DetectorDataset dataset_              = DetectorDataset::COCO;
    uint64_t        counter_{0};
};

}  // namespace drone::perception
