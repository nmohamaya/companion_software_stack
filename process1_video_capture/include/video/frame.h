// process1_video_capture/include/video/frame.h
// Frame metadata and pixel format definitions for all camera sources.
#pragma once
#include <cstdint>

namespace drone::video {

enum class PixelFormat : uint32_t {
    BAYER_RGGB8 = 0, BAYER_GRBG8 = 1, YUV422 = 2,
    RGB24 = 3, NV12 = 4, GRAY8 = 5,
};

enum class CameraId : uint8_t {
    MISSION  = 0, STEREO_L = 1, STEREO_R = 2,
};

struct FrameMetadata {
    uint64_t    timestamp_ns;
    uint64_t    sequence_number;
    CameraId    camera_id;
    PixelFormat format;
    uint32_t    width;
    uint32_t    height;
    uint32_t    stride;
    uint32_t    exposure_us;
    float       gain_db;
    float       temperature_c;
};

} // namespace drone::video
