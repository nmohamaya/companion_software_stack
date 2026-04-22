// common/hal/include/hal/simulated_event_camera.h
// Simulated event camera: generates deterministic synthetic events.
// Used by unit tests. No external dependencies.
#pragma once

#include "hal/ievent_camera.h"

namespace drone::hal {

class SimulatedEventCamera : public IEventCamera {
public:
    explicit SimulatedEventCamera(int events_per_batch = 100)
        : events_per_batch_(events_per_batch) {}

    [[nodiscard]] bool open(uint32_t width, uint32_t height) override {
        if (width == 0 || height == 0) return false;
        width_  = width;
        height_ = height;
        open_   = true;
        return true;
    }

    void close() override { open_ = false; }

    [[nodiscard]] EventBatch read_events() override {
        EventBatch batch;
        if (!open_) return batch;

        batch.valid          = true;
        batch.batch_start_ns = counter_ * 1000000;
        batch.events.reserve(static_cast<size_t>(events_per_batch_));

        for (int i = 0; i < events_per_batch_; ++i) {
            EventCD ev;
            ev.x        = static_cast<uint16_t>((counter_ + static_cast<uint64_t>(i)) % width_);
            ev.y        = static_cast<uint16_t>((counter_ + static_cast<uint64_t>(i)) % height_);
            ev.polarity = (i % 2 == 0) ? int8_t{1} : int8_t{-1};
            ev.timestamp_ns = batch.batch_start_ns + static_cast<uint64_t>(i) * 10;
            batch.events.push_back(ev);
        }

        batch.batch_end_ns = batch.batch_start_ns + static_cast<uint64_t>(events_per_batch_) * 10;
        ++counter_;
        return batch;
    }

    [[nodiscard]] bool is_open() const override { return open_; }

    [[nodiscard]] PixelFormat pixel_format() const override { return PixelFormat::EVENT_CD; }

    [[nodiscard]] std::string name() const override { return "SimulatedEventCamera"; }

private:
    int      events_per_batch_;
    uint32_t width_{0};
    uint32_t height_{0};
    bool     open_{false};
    uint64_t counter_{0};
};

}  // namespace drone::hal
