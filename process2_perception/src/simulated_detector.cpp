// process2_perception/src/simulated_detector.cpp
// Simulated object detector — generates fake detections for testing.
#include "perception/detector_interface.h"

#include <chrono>
#include <random>

namespace drone::perception {

std::vector<Detection2D> SimulatedDetector::detect(const uint8_t* /*frame_data*/, uint32_t width,
                                                   uint32_t height, uint32_t /*channels*/) {
    thread_local std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int>    num_dist(1, 5);
    std::uniform_real_distribution<float> x_dist(50.0f, static_cast<float>(width - 200));
    std::uniform_real_distribution<float> y_dist(50.0f, static_cast<float>(height - 200));
    std::uniform_real_distribution<float> size_dist(40.0f, 200.0f);
    std::uniform_real_distribution<float> conf_dist(0.4f, 0.99f);
    std::uniform_int_distribution<int>    class_dist(0, 4);

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
