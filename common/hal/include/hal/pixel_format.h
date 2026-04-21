// common/hal/include/hal/pixel_format.h
// Pixel format enumeration for camera and sensor data.
// Includes reserved slots for thermal imaging and event cameras.
#pragma once

#include <cstdint>

namespace drone::hal {

enum class PixelFormat : uint8_t {
    GRAY8       = 0,
    RGB8        = 1,
    BGR8        = 2,
    RGBA8       = 3,
    NV12        = 4,
    YUYV        = 5,
    BAYER_RGGB8 = 6,

    // Reserved: thermal imaging (see docs/design/perception_architecture.md)
    THERMAL_8  = 20,
    THERMAL_16 = 21,

    // Reserved: event cameras / DVS (see docs/design/perception_architecture.md)
    EVENT_CD = 30,
};

inline constexpr uint8_t pixel_format_channels(PixelFormat fmt) {
    switch (fmt) {
        case PixelFormat::GRAY8: return 1;
        case PixelFormat::RGB8: return 3;
        case PixelFormat::BGR8: return 3;
        case PixelFormat::RGBA8: return 4;
        case PixelFormat::THERMAL_8: return 1;
        case PixelFormat::THERMAL_16: return 1;
        default: return 0;
    }
}

}  // namespace drone::hal
