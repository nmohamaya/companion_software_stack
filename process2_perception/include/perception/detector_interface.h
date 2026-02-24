// process2_perception/include/perception/detector_interface.h
// Strategy pattern: IDetector interface for swappable detection backends.
#pragma once
#include "perception/types.h"
#include <vector>
#include <string>
#include <memory>

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

} // namespace drone::perception
