// process2_perception/include/perception/detector_interface.h
// Strategy pattern: IDetector interface for swappable detection backends.
#pragma once
#include "perception/types.h"

#include <memory>
#include <string>
#include <vector>

// Forward-declare Config to avoid circular include
namespace drone {
class Config;
}

namespace drone::perception {

class IDetector {
public:
    virtual ~IDetector()                                                        = default;
    virtual std::vector<Detection2D> detect(const uint8_t* frame_data, uint32_t width,
                                            uint32_t height, uint32_t channels) = 0;
    virtual std::string              name() const                               = 0;
};

/// Factory: create detector from config backend string.
/// Supported backends: "simulated", "color_contour", "yolov8"
/// Defined in detector_factory.cpp to avoid circular includes.
std::unique_ptr<IDetector> create_detector(const std::string&   backend,
                                           const drone::Config* cfg = nullptr);

}  // namespace drone::perception
