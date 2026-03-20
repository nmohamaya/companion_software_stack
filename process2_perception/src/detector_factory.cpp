// process2_perception/src/detector_factory.cpp
// Factory function: creates the correct IDetector backend from config string.

#include "perception/color_contour_detector.h"
#include "perception/detector_interface.h"

#ifdef HAS_OPENCV
#include "perception/opencv_yolo_detector.h"
#endif

#include "util/config.h"

#include <algorithm>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace drone::perception {

std::unique_ptr<IDetector> create_detector(const std::string& backend, const drone::Config* cfg) {
    if (backend == "yolov8") {
#ifdef HAS_OPENCV
        if (cfg) {
            return std::make_unique<OpenCvYoloDetector>(*cfg);
        }
        return std::make_unique<OpenCvYoloDetector>("models/yolov8n.onnx");
#else
        spdlog::warn("[detector_factory] 'yolov8' backend requested but OpenCV "
                     "not available — falling back to 'color_contour'");
        if (cfg) {
            return std::make_unique<ColorContourDetector>(*cfg);
        }
        return std::make_unique<ColorContourDetector>();
#endif
    }
    if (backend == "color_contour") {
        if (cfg) {
            return std::make_unique<ColorContourDetector>(*cfg);
        }
        return std::make_unique<ColorContourDetector>();
    }
    if (backend == "simulated" || backend.empty()) {
        SimulatedDetectorConfig sim_cfg;
        if (cfg) {
            sim_cfg.min_detections = cfg->get<int>("perception.detector.sim.min_detections",
                                                   sim_cfg.min_detections);
            sim_cfg.max_detections = cfg->get<int>("perception.detector.sim.max_detections",
                                                   sim_cfg.max_detections);
            sim_cfg.margin_px      = cfg->get<float>("perception.detector.sim.margin_px",
                                                     sim_cfg.margin_px);
            sim_cfg.size_min_px    = cfg->get<float>("perception.detector.sim.size_min_px",
                                                     sim_cfg.size_min_px);
            sim_cfg.size_max_px    = cfg->get<float>("perception.detector.sim.size_max_px",
                                                     sim_cfg.size_max_px);
            sim_cfg.confidence_min = cfg->get<float>("perception.detector.sim.confidence_min",
                                                     sim_cfg.confidence_min);
            sim_cfg.confidence_max = cfg->get<float>("perception.detector.sim.confidence_max",
                                                     sim_cfg.confidence_max);
            sim_cfg.num_classes    = cfg->get<int>("perception.detector.sim.num_classes",
                                                   sim_cfg.num_classes);
        }

        // Sanitize ranges to prevent UB in std::uniform_*_distribution
        sim_cfg.min_detections = std::max(0, sim_cfg.min_detections);
        if (sim_cfg.max_detections < sim_cfg.min_detections)
            sim_cfg.max_detections = sim_cfg.min_detections;
        sim_cfg.margin_px   = std::max(0.0f, sim_cfg.margin_px);
        sim_cfg.size_min_px = std::max(0.0f, sim_cfg.size_min_px);
        if (sim_cfg.size_max_px < sim_cfg.size_min_px) sim_cfg.size_max_px = sim_cfg.size_min_px;
        sim_cfg.confidence_min = std::clamp(sim_cfg.confidence_min, 0.0f, 1.0f);
        sim_cfg.confidence_max = std::clamp(sim_cfg.confidence_max, 0.0f, 1.0f);
        if (sim_cfg.confidence_max < sim_cfg.confidence_min)
            sim_cfg.confidence_max = sim_cfg.confidence_min;
        sim_cfg.num_classes = std::max(1, sim_cfg.num_classes);

        return std::make_unique<SimulatedDetector>(sim_cfg);
    }
    throw std::runtime_error("Unknown detector backend: " + backend);
}

}  // namespace drone::perception
