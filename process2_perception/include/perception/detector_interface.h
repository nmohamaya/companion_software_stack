// process2_perception/include/perception/detector_interface.h
// Strategy pattern: IDetector interface for swappable detection backends.
#pragma once
#include "perception/types.h"
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

// Forward-declare Config to avoid circular include
namespace drone { class Config; }

namespace drone::perception {

class IDetector {
public:
    virtual ~IDetector() = default;
    virtual std::vector<Detection2D> detect(const uint8_t* frame_data,
                                             uint32_t width, uint32_t height,
                                             uint32_t channels) = 0;
    virtual std::string name() const = 0;
};

/// Simulated detector — generates fake detections for testing.
class SimulatedDetector : public IDetector {
public:
    std::vector<Detection2D> detect(const uint8_t* frame_data,
                                     uint32_t width, uint32_t height,
                                     uint32_t channels) override;
    std::string name() const override { return "SimulatedDetector"; }
};

/// Factory: create detector from config backend string.
/// Supported backends: "simulated", "color_contour"
inline std::unique_ptr<IDetector> create_detector(const std::string& backend,
                                                   const drone::Config* cfg = nullptr);

} // namespace drone::perception

// Include concrete detectors after interface is fully declared
#include "perception/color_contour_detector.h"

namespace drone::perception {

inline std::unique_ptr<IDetector> create_detector(const std::string& backend,
                                                   const drone::Config* cfg) {
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
