// common/hal/include/hal/ievent_camera.h
// HAL interface: Event camera (DVS) abstraction.
// Reserved slot for future event camera integration.
// Implementations: SimulatedEventCamera (built-in).
// Future: Prophesee EVK4, iniVation DAVIS backends.
#pragma once

#include "hal/pixel_format.h"

#include <cstdint>
#include <string>
#include <vector>

namespace drone::hal {

/// Single contrast detection event from a DVS sensor.
struct EventCD {
    uint16_t x{0};
    uint16_t y{0};
    int8_t   polarity{0};
    uint64_t timestamp_ns{0};
};

/// Batch of events read from the sensor in one call.
struct EventBatch {
    std::vector<EventCD> events;
    uint64_t             batch_start_ns{0};
    uint64_t             batch_end_ns{0};
    bool                 valid{false};
};

/// Abstract event camera interface.
class IEventCamera {
public:
    virtual ~IEventCamera() = default;

    /// Open the event sensor with the given resolution.
    [[nodiscard]] virtual bool open(uint32_t width, uint32_t height) = 0;

    /// Close the sensor and release resources.
    virtual void close() = 0;

    /// Read the next batch of events. May block or return immediately
    /// depending on backend. Check EventBatch::valid.
    [[nodiscard]] virtual EventBatch read_events() = 0;

    /// Check whether the sensor is currently open.
    [[nodiscard]] virtual bool is_open() const = 0;

    /// The pixel format this sensor produces.
    [[nodiscard]] virtual PixelFormat pixel_format() const = 0;

    /// Human-readable name.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
