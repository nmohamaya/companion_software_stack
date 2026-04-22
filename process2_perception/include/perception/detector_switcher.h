// process2_perception/include/perception/detector_switcher.h
// Selects the active detector dataset (COCO vs VisDrone) and model path based
// on altitude AGL.  The caller is responsible for reloading the model if the
// dataset changes.
#pragma once

#include "perception/detector_class_maps.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/ilogger.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace drone::perception {

class DetectorSwitcher {
public:
    static constexpr float kMinAltitudeThreshold = 0.0f;
    static constexpr float kMaxAltitudeThreshold = 500.0f;

    explicit DetectorSwitcher(const drone::Config& cfg)
        : altitude_threshold_m_(clamp_threshold(cfg.get<float>(
              drone::cfg_key::perception::detector_switcher::ALTITUDE_THRESHOLD_M, 30.0f)))
        , coco_model_path_(
              cfg.get<std::string>(drone::cfg_key::perception::detector_switcher::COCO_MODEL_PATH,
                                   "models/yolov8n-seg.onnx"))
        , visdrone_model_path_(cfg.get<std::string>(
              drone::cfg_key::perception::detector_switcher::VISDRONE_MODEL_PATH,
              "models/yolov8n-visdrone-seg.onnx")) {}

    DetectorSwitcher(float altitude_threshold_m, std::string coco_model_path,
                     std::string visdrone_model_path)
        : altitude_threshold_m_(clamp_threshold(altitude_threshold_m))
        , coco_model_path_(std::move(coco_model_path))
        , visdrone_model_path_(std::move(visdrone_model_path)) {}

    [[nodiscard]] DetectorDataset select_dataset(float altitude_agl_m) const {
        // NaN/inf defaults to COCO (safe fallback for sensor failure)
        if (!std::isfinite(altitude_agl_m)) return DetectorDataset::COCO;
        return (altitude_agl_m > altitude_threshold_m_) ? DetectorDataset::VISDRONE
                                                        : DetectorDataset::COCO;
    }

    [[nodiscard]] const std::string& model_path(DetectorDataset ds) const {
        return (ds == DetectorDataset::VISDRONE) ? visdrone_model_path_ : coco_model_path_;
    }

    [[nodiscard]] float altitude_threshold() const { return altitude_threshold_m_; }

private:
    static float clamp_threshold(float v) {
        if (!std::isfinite(v)) return 30.0f;
        return std::clamp(v, kMinAltitudeThreshold, kMaxAltitudeThreshold);
    }

    float       altitude_threshold_m_{30.0f};
    std::string coco_model_path_{"models/yolov8n-seg.onnx"};
    std::string visdrone_model_path_{"models/yolov8n-visdrone-seg.onnx"};
};

}  // namespace drone::perception
