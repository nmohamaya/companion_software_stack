// process2_perception/include/perception/simulated_detector.h
// Simulated detector — generates fake detections for testing.
// Extracted from detector_interface.h for single-responsibility.
#pragma once

#include "perception/detector_interface.h"

#include <string>
#include <vector>

namespace drone::perception {

/// Configuration for SimulatedDetector random distribution ranges.
struct SimulatedDetectorConfig {
    int   min_detections = 1;       // min random detections per frame
    int   max_detections = 5;       // max random detections per frame
    float margin_px      = 50.0f;   // pixel margin from frame edges
    float size_min_px    = 40.0f;   // min bbox dimension (px)
    float size_max_px    = 200.0f;  // max bbox dimension (px)
    float confidence_min = 0.4f;    // min confidence
    float confidence_max = 0.99f;   // max confidence
    int   num_classes    = 5;       // number of ObjectClass values to sample
};

/// Simulated detector — generates fake detections for testing.
class SimulatedDetector : public IDetector {
public:
    SimulatedDetector() = default;
    explicit SimulatedDetector(const SimulatedDetectorConfig& cfg) : cfg_(cfg) {}

    std::vector<Detection2D> detect(const uint8_t* frame_data, uint32_t width, uint32_t height,
                                    uint32_t channels) override;
    std::string              name() const override { return "SimulatedDetector"; }

    const SimulatedDetectorConfig& config() const { return cfg_; }

private:
    SimulatedDetectorConfig cfg_;
};

}  // namespace drone::perception
