// process2_perception/src/detector_factory.cpp
// Factory function: creates the correct IDetector backend from config string.

#include "perception/detector_interface.h"
#include "perception/color_contour_detector.h"

#ifdef HAS_OPENCV
#include "perception/opencv_yolo_detector.h"
#endif

#include "util/config.h"
#include <stdexcept>
#include <spdlog/spdlog.h>

namespace drone::perception {

std::unique_ptr<IDetector> create_detector(const std::string& backend,
                                            const drone::Config* cfg) {
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
        return std::make_unique<SimulatedDetector>();
    }
    throw std::runtime_error("Unknown detector backend: " + backend);
}

} // namespace drone::perception
