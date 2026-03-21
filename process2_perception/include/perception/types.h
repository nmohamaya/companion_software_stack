// process2_perception/include/perception/types.h
// All data types for perception: detections, tracked/fused objects.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// Object Classification
// ═══════════════════════════════════════════════════════════
enum class ObjectClass : uint8_t {
    UNKNOWN       = 0,
    PERSON        = 1,
    VEHICLE_CAR   = 2,
    VEHICLE_TRUCK = 3,
    DRONE         = 4,
    ANIMAL        = 5,
    BUILDING      = 6,
    TREE          = 7,
};

inline const char* object_class_name(ObjectClass c) {
    switch (c) {
        case ObjectClass::PERSON: return "Person";
        case ObjectClass::VEHICLE_CAR: return "Car";
        case ObjectClass::VEHICLE_TRUCK: return "Truck";
        case ObjectClass::DRONE: return "Drone";
        case ObjectClass::ANIMAL: return "Animal";
        case ObjectClass::BUILDING: return "Building";
        case ObjectClass::TREE: return "Tree";
        default: return "Unknown";
    }
}

// ═══════════════════════════════════════════════════════════
// 2D Detections (Camera)
// ═══════════════════════════════════════════════════════════
struct Detection2D {
    float       x = 0.0f, y = 0.0f, w = 0.0f, h = 0.0f;  // bounding box in pixel coords
    float       confidence     = 0.0f;
    ObjectClass class_id       = ObjectClass::UNKNOWN;
    uint64_t    timestamp_ns   = 0;
    uint64_t    frame_sequence = 0;

    Eigen::Vector2f center() const { return {x + w / 2.0f, y + h / 2.0f}; }
};

struct Detection2DList {
    std::vector<Detection2D> detections;
    uint64_t                 timestamp_ns   = 0;
    uint64_t                 frame_sequence = 0;
};

// ═══════════════════════════════════════════════════════════
// Tracked Objects (after Kalman filtering)
// ═══════════════════════════════════════════════════════════
struct TrackedObject {
    uint32_t        track_id     = 0;
    ObjectClass     class_id     = ObjectClass::UNKNOWN;
    float           confidence   = 0.0f;
    Eigen::Vector2f position_2d  = Eigen::Vector2f::Zero();
    Eigen::Vector2f velocity_2d  = Eigen::Vector2f::Zero();
    float           bbox_w       = 0.0f;  // bounding box width in pixels
    float           bbox_h       = 0.0f;  // bounding box height in pixels
    uint32_t        age          = 0;
    uint32_t        hits         = 0;
    uint32_t        misses       = 0;
    uint64_t        timestamp_ns = 0;
    enum class State : uint8_t { TENTATIVE, CONFIRMED, LOST };
    State state = State::TENTATIVE;
};

struct TrackedObjectList {
    std::vector<TrackedObject> objects;
    uint64_t                   timestamp_ns   = 0;
    uint64_t                   frame_sequence = 0;
};

// ═══════════════════════════════════════════════════════════
// Fused Objects (multi-sensor fusion output)
// ═══════════════════════════════════════════════════════════
struct FusedObject {
    uint32_t        track_id            = 0;
    ObjectClass     class_id            = ObjectClass::UNKNOWN;
    float           confidence          = 0.0f;
    Eigen::Vector3f position_3d         = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_3d         = Eigen::Vector3f::Zero();
    float           heading             = 0.0f;
    bool            has_camera          = false;
    bool            has_radar           = false;
    Eigen::Matrix3f position_covariance = Eigen::Matrix3f::Identity();
    uint64_t        timestamp_ns        = 0;
};

struct FusedObjectList {
    std::vector<FusedObject> objects;
    uint64_t                 timestamp_ns   = 0;
    uint32_t                 frame_sequence = 0;
};

}  // namespace drone::perception

// ═══════════════════════════════════════════════════════════
// Calibration Data (shared between fusion engines)
// ═══════════════════════════════════════════════════════════
namespace drone::perception {

/// Calibration data for camera-based depth estimation.
struct CalibrationData {
    Eigen::Matrix3f camera_intrinsics = Eigen::Matrix3f::Identity();
    float           camera_height_m   = 1.5f;  // camera height above ground
    float assumed_obstacle_height_m   = 3.0f;  // assumed obstacle height for apparent-size depth
};

}  // namespace drone::perception
