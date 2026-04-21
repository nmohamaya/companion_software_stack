// common/hal/include/hal/iinference_backend.h
// HAL interface: Detector-agnostic inference abstraction.
// Wraps any model (YOLO, RT-DETR, SAM, etc.) behind a uniform API.
// Implementations: SimulatedInferenceBackend (built-in).
// Future: OpenCV DNN, ONNX Runtime, TensorRT backends.
#pragma once

#include "util/result.h"

#include <cstdint>
#include <string>
#include <vector>

namespace drone::hal {

struct BoundingBox2D {
    float x{0.0f};
    float y{0.0f};
    float w{0.0f};
    float h{0.0f};
};

struct InferenceDetection {
    BoundingBox2D        bbox;
    int                  class_id{-1};
    float                confidence{0.0f};
    std::vector<uint8_t> mask;
    uint32_t             mask_width{0};
    uint32_t             mask_height{0};
};

struct InferenceOutput {
    std::vector<InferenceDetection> detections;
    uint64_t                        timestamp_ns{0};
};

/// Abstract inference backend interface.
/// One instance per model (detector, segmenter, depth estimator, etc.).
class IInferenceBackend {
public:
    virtual ~IInferenceBackend() = default;

    /// Load the model from the given path. input_size is the model's expected
    /// square input dimension (e.g. 640 for YOLOv8).
    [[nodiscard]] virtual bool init(const std::string& model_path, int input_size) = 0;

    /// Run inference on a single frame.
    /// @param frame_data  Pointer to pixel data (RGB, valid for this call only).
    /// @param width       Frame width in pixels
    /// @param height      Frame height in pixels
    /// @param channels    Number of channels (3=RGB, 4=RGBA)
    /// @param stride      Row stride in bytes (0 = tightly packed)
    [[nodiscard]] virtual drone::util::Result<InferenceOutput, std::string> infer(
        const uint8_t* frame_data, uint32_t width, uint32_t height, uint32_t channels,
        uint32_t stride = 0) = 0;

    /// Human-readable backend name.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
