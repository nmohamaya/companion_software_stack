# cmake/ModelPresets.cmake — ML model size presets for different deployment targets.
#
# Presets map deployment targets to appropriate model sizes:
#   edge  — YOLOv8n + DA V2 ViT-S  (4 GB VRAM budget)
#   orin  — YOLOv8s + DA V2 ViT-S  (8-16 GB VRAM budget)
#   cloud — YOLOv8m + DA V2 ViT-B  (24 GB VRAM budget)
#
# These are non-cache derived variables — they always track MODEL_PRESET.
# Runtime config (perception.detector.model_path) overrides these defaults.
set(MODEL_PRESET "edge" CACHE STRING "ML model size preset: edge|orin|cloud|dev")
set_property(CACHE MODEL_PRESET PROPERTY STRINGS edge orin cloud dev)

if(MODEL_PRESET STREQUAL "cloud")
    set(DEFAULT_YOLO_MODEL  "yolov8m.onnx")
    set(DEFAULT_DEPTH_MODEL "depth_anything_v2_vitb.onnx")
elseif(MODEL_PRESET STREQUAL "orin")
    set(DEFAULT_YOLO_MODEL  "yolov8s.onnx")
    set(DEFAULT_DEPTH_MODEL "depth_anything_v2_vits.onnx")
elseif(MODEL_PRESET STREQUAL "dev")
    # dev — GTX 1080 Ti desktop (11 GB VRAM minus ~8 GB UE5 = ~3 GB for ML)
    set(DEFAULT_YOLO_MODEL  "yolov8s.onnx")
    set(DEFAULT_DEPTH_MODEL "depth_anything_v2_vits.onnx")
else()
    # edge (default) — smallest models for constrained devices
    set(DEFAULT_YOLO_MODEL  "yolov8n.onnx")
    set(DEFAULT_DEPTH_MODEL "depth_anything_v2_vits.onnx")
endif()

message(STATUS "  Model Preset : ${MODEL_PRESET}")
message(STATUS "  YOLO Model   : ${DEFAULT_YOLO_MODEL}")
message(STATUS "  Depth Model  : ${DEFAULT_DEPTH_MODEL}")
