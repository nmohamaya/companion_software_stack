// common/hal/include/hal/simulated_thermal_camera.h
// Simulated LWIR thermal camera backend (640×512, GRAY8).
// Generates synthetic heat-blob frames for simulation testing.
// Phase 1C (Issue #114).
#pragma once
#include "hal/icamera.h"

#include <chrono>
#include <cmath>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::hal {

/// Simulated LWIR thermal camera.
/// Default resolution 640×512 (typical uncooled micro-bolometer).
/// Produces 8-bit grayscale frames with synthetic warm-object blobs.
class SimulatedThermalCamera : public ICamera {
public:
    bool open(uint32_t width, uint32_t height, int fps) override {
        width_    = width;
        height_   = height;
        fps_      = fps;
        channels_ = 1;  // thermal is always single-channel
        stride_   = width_;
        buffer_.resize(static_cast<size_t>(height_) * stride_, 0);
        open_ = true;
        seq_  = 0;
        spdlog::info("[SimulatedThermalCamera] Opened {}x{}@{}Hz", width_, height_, fps_);
        return true;
    }

    void close() override {
        open_ = false;
        buffer_.clear();
        spdlog::info("[SimulatedThermalCamera] Closed");
    }

    CapturedFrame capture() override {
        CapturedFrame f;
        if (!open_) return f;

        // Background: ~25°C → pixel ~128
        constexpr uint8_t bg_temp = 128;
        std::fill(buffer_.begin(), buffer_.end(), bg_temp);

        // Simulate 1-2 warm blobs (persons ~37°C → pixel ~220)
        generate_heat_blob(static_cast<int>(width_) / 3, static_cast<int>(height_) / 2, 30, 50,
                           220);
        if (seq_ % 3 == 0) {
            // Second blob appears every 3rd frame
            generate_heat_blob(static_cast<int>(width_) * 2 / 3, static_cast<int>(height_) / 3, 25,
                               40, 200);
        }

        f.timestamp_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());
        f.sequence = seq_++;
        f.width    = width_;
        f.height   = height_;
        f.channels = channels_;
        f.stride   = stride_;
        f.data     = buffer_.data();
        f.valid    = true;

        return f;
    }

    bool is_open() const override { return open_; }

    std::string name() const override { return "SimulatedThermalCamera"; }

private:
    void generate_heat_blob(int cx, int cy, int rx, int ry, uint8_t peak) {
        for (int dy = -ry; dy <= ry; ++dy) {
            for (int dx = -rx; dx <= rx; ++dx) {
                int px = cx + dx;
                int py = cy + dy;
                if (px < 0 || px >= static_cast<int>(width_) || py < 0 ||
                    py >= static_cast<int>(height_))
                    continue;
                float dist = std::sqrt(static_cast<float>(dx * dx) / (rx * rx) +
                                       static_cast<float>(dy * dy) / (ry * ry));
                if (dist <= 1.0f) {
                    auto val = static_cast<uint8_t>(peak * (1.0f - dist * 0.4f));  // radial falloff
                    auto idx = static_cast<size_t>(py) * stride_ + px;
                    if (val > buffer_[idx]) buffer_[idx] = val;
                }
            }
        }
    }

    bool                 open_{false};
    uint32_t             width_{0}, height_{0}, channels_{1};
    uint32_t             stride_{0};
    int                  fps_{9};  // typical thermal FPS
    uint64_t             seq_{0};
    std::vector<uint8_t> buffer_;
};

}  // namespace drone::hal
