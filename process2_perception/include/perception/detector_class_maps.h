// process2_perception/include/perception/detector_class_maps.h
// Dataset class-ID → ObjectClass mapping functions.
// Shared between detector backends and tests.
// Extracted from opencv_yolo_detector.h (Issue #430 review fix W4).
#pragma once

#include "perception/detector_interface.h"

#include <cstdint>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// Dataset selector
// ═══════════════════════════════════════════════════════════

/// Dataset selector for YOLO model output interpretation.
enum class DetectorDataset : uint8_t { COCO = 0, VISDRONE = 1 };

// ═══════════════════════════════════════════════════════════
// COCO class ID → ObjectClass mapping
// ═══════════════════════════════════════════════════════════

/// Map COCO 80-class index to our ObjectClass enum.
/// Only a subset of COCO classes map to our classes; rest → UNKNOWN.
inline ObjectClass coco_to_object_class(int coco_id) {
    switch (coco_id) {
        case 0: return ObjectClass::PERSON;         // person
        case 2: return ObjectClass::VEHICLE_CAR;    // car
        case 5: return ObjectClass::VEHICLE_CAR;    // bus → car
        case 7: return ObjectClass::VEHICLE_TRUCK;  // truck
        case 14: return ObjectClass::ANIMAL;        // bird
        case 15: return ObjectClass::ANIMAL;        // cat
        case 16: return ObjectClass::ANIMAL;        // dog
        case 17: return ObjectClass::ANIMAL;        // horse
        case 18: return ObjectClass::ANIMAL;        // sheep
        case 19: return ObjectClass::ANIMAL;        // cow
        default: return ObjectClass::UNKNOWN;
    }
}

/// COCO class names (80 classes) for logging.
inline const char* coco_class_name(int id) {
    static const char* names[] = {"person",        "bicycle",      "car",
                                  "motorcycle",    "airplane",     "bus",
                                  "train",         "truck",        "boat",
                                  "traffic light", "fire hydrant", "stop sign",
                                  "parking meter", "bench",        "bird",
                                  "cat",           "dog",          "horse",
                                  "sheep",         "cow",          "elephant",
                                  "bear",          "zebra",        "giraffe",
                                  "backpack",      "umbrella",     "handbag",
                                  "tie",           "suitcase",     "frisbee",
                                  "skis",          "snowboard",    "sports ball",
                                  "kite",          "baseball bat", "baseball glove",
                                  "skateboard",    "surfboard",    "tennis racket",
                                  "bottle",        "wine glass",   "cup",
                                  "fork",          "knife",        "spoon",
                                  "bowl",          "banana",       "apple",
                                  "sandwich",      "orange",       "broccoli",
                                  "carrot",        "hot dog",      "pizza",
                                  "donut",         "cake",         "chair",
                                  "couch",         "potted plant", "bed",
                                  "dining table",  "toilet",       "tv",
                                  "laptop",        "mouse",        "remote",
                                  "keyboard",      "cell phone",   "microwave",
                                  "oven",          "toaster",      "sink",
                                  "refrigerator",  "book",         "clock",
                                  "vase",          "scissors",     "teddy bear",
                                  "hair drier",    "toothbrush"};
    if (id >= 0 && id < 80) return names[id];
    return "unknown";
}

// ═══════════════════════════════════════════════════════════
// VisDrone dataset class mapping (10 aerial-view classes)
// ═══════════════════════════════════════════════════════════

/// Map VisDrone 10-class index to our ObjectClass enum.
/// VisDrone is an aerial-perspective dataset optimised for drone use cases.
inline ObjectClass visdrone_to_object_class(int visdrone_id) {
    switch (visdrone_id) {
        case 0: return ObjectClass::PERSON;         // pedestrian
        case 1: return ObjectClass::PERSON;         // people (group)
        case 2: return ObjectClass::UNKNOWN;        // bicycle
        case 3: return ObjectClass::VEHICLE_CAR;    // car
        case 4: return ObjectClass::VEHICLE_CAR;    // van
        case 5: return ObjectClass::VEHICLE_TRUCK;  // truck
        case 6: return ObjectClass::UNKNOWN;        // tricycle
        case 7: return ObjectClass::UNKNOWN;        // awning-tricycle
        case 8: return ObjectClass::VEHICLE_TRUCK;  // bus
        case 9: return ObjectClass::UNKNOWN;        // motor
        default: return ObjectClass::UNKNOWN;
    }
}

/// VisDrone class names (10 classes) for logging.
inline const char* visdrone_class_name(int id) {
    static const char* names[] = {"pedestrian", "people",   "bicycle",         "car", "van",
                                  "truck",      "tricycle", "awning-tricycle", "bus", "motor"};
    if (id >= 0 && id < 10) return names[id];
    return "unknown";
}

}  // namespace drone::perception
