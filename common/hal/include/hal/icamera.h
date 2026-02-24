// common/hal/include/hal/icamera.h
// HAL interface: Camera abstraction for mission and stereo cameras.
// Implementations: SimulatedCamera (built-in), V4L2Camera (future).
#pragma once
#include <cstdint>
#include <string>

namespace drone::hal {

/// Result of a single frame capture.
struct CapturedFrame {
    uint64_t    timestamp_ns{0};
    uint64_t    sequence{0};
    uint32_t    width{0};
    uint32_t    height{0};
    uint32_t    channels{0};      // 1=GRAY, 3=RGB
    uint32_t    stride{0};
    const uint8_t* data{nullptr}; // pointer to internal buffer — valid until next capture()
    bool        valid{false};
};

/// Abstract camera interface.
/// One instance per physical camera (mission, stereo-left, stereo-right).
class ICamera {
public:
    virtual ~ICamera() = default;

    /// Open the camera with the given resolution and frame rate.
    /// Returns true on success.
    virtual bool open(uint32_t width, uint32_t height, int fps) = 0;

    /// Close the camera and release resources.
    virtual void close() = 0;

    /// Capture one frame. Blocks until a frame is ready (or timeout).
    /// The returned CapturedFrame::data pointer is valid until the next call to capture().
    virtual CapturedFrame capture() = 0;

    /// Check whether the camera is currently open.
    virtual bool is_open() const = 0;

    /// Human-readable name (e.g. "SimulatedCamera", "V4L2Camera(/dev/video0)").
    virtual std::string name() const = 0;
};

}  // namespace drone::hal
