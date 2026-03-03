// common/hal/include/hal/simulated_camera.h
// Simulated camera backend: generates synthetic frames for testing.
#pragma once
#include "hal/icamera.h"

#include <chrono>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::hal {

class SimulatedCamera : public ICamera {
public:
    bool open(uint32_t width, uint32_t height, int fps) override {
        width_    = width;
        height_   = height;
        fps_      = fps;
        channels_ = (height <= 480 && width <= 640) ? 1u : 3u;  // stereo=GRAY, mission=RGB
        stride_   = width_ * channels_;
        buffer_.resize(static_cast<size_t>(height_) * stride_, 0);
        open_ = true;
        seq_  = 0;
        spdlog::info("[SimulatedCamera] Opened {}x{}@{}Hz ch={}", width_, height_, fps_, channels_);
        return true;
    }

    void close() override {
        open_ = false;
        buffer_.clear();
        spdlog::info("[SimulatedCamera] Closed");
    }

    CapturedFrame capture() override {
        CapturedFrame f;
        if (!open_) return f;

        // Generate synthetic gradient pattern
        for (uint32_t y = 0; y < height_; y += 16) {
            for (uint32_t x = 0; x < width_; x += 16) {
                size_t idx = (static_cast<size_t>(y) * width_ + x) * channels_;
                if (idx + channels_ - 1 < buffer_.size()) {
                    buffer_[idx] = static_cast<uint8_t>(x * 255 / width_);
                    if (channels_ >= 3) {
                        buffer_[idx + 1] = static_cast<uint8_t>(y * 255 / height_);
                        buffer_[idx + 2] = static_cast<uint8_t>((seq_ * 10) % 256);
                    }
                }
            }
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

    std::string name() const override { return "SimulatedCamera"; }

    /// Allow explicit channel override (e.g., 1 for stereo, 3 for mission).
    void set_channels(uint32_t ch) {
        channels_ = ch;
        stride_   = width_ * channels_;
        buffer_.resize(static_cast<size_t>(height_) * stride_, 0);
    }

private:
    bool                 open_{false};
    uint32_t             width_{0}, height_{0}, channels_{3};
    uint32_t             stride_{0};
    int                  fps_{30};
    uint64_t             seq_{0};
    std::vector<uint8_t> buffer_;
};

}  // namespace drone::hal
