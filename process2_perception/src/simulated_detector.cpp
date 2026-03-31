// process2_perception/src/simulated_detector.cpp
// Simulated object detector — generates fake detections for testing.
#include "perception/simulated_detector.h"

#include <chrono>
#include <random>

namespace drone::perception {

std::vector<Detection2D> SimulatedDetector::detect(const uint8_t* /*frame_data*/, uint32_t width,
                                                   uint32_t height, uint32_t /*channels*/) {
    thread_local std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int>    num_dist(cfg_.min_detections, cfg_.max_detections);
    std::uniform_real_distribution<float> x_dist(
        cfg_.margin_px, static_cast<float>(width) - cfg_.margin_px - cfg_.size_max_px);
    std::uniform_real_distribution<float> y_dist(
        cfg_.margin_px, static_cast<float>(height) - cfg_.margin_px - cfg_.size_max_px);
    std::uniform_real_distribution<float> size_dist(cfg_.size_min_px, cfg_.size_max_px);
    std::uniform_real_distribution<float> conf_dist(cfg_.confidence_min, cfg_.confidence_max);
    std::uniform_int_distribution<int>    class_dist(0, std::max(0, cfg_.num_classes - 1));

    int                      n = num_dist(rng);
    std::vector<Detection2D> dets;
    dets.reserve(n);

    auto now = std::chrono::steady_clock::now().time_since_epoch();

    for (int i = 0; i < n; ++i) {
        Detection2D d;
        d.x              = x_dist(rng);
        d.y              = y_dist(rng);
        d.w              = size_dist(rng);
        d.h              = size_dist(rng);
        d.confidence     = conf_dist(rng);
        d.class_id       = static_cast<ObjectClass>(class_dist(rng));
        d.timestamp_ns   = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        d.frame_sequence = 0;
        dets.push_back(d);
    }
    return dets;
}

}  // namespace drone::perception
