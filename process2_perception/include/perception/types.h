// process2_perception/include/perception/types.h
// All data types for perception: detections, tracked/fused objects.
#pragma once
#include <array>
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
    bool            in_world_frame     = false;  // true = position already in world frame (re-ID'd)
    float           estimated_radius_m = 0.0f;   // back-projected obstacle radius from bbox+range
    float           estimated_height_m = 0.0f;   // back-projected obstacle height from bbox+range
    uint32_t        radar_update_count = 0;      // number of radar updates (for trust weighting)
    float           depth_confidence   = 0.0f;   // depth estimation quality [0.0=guess, 1.0=radar]
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

inline constexpr uint8_t kNumObjectClasses = 8;
// height_priors[] is indexed by static_cast<uint8_t>(ObjectClass).
// Update kNumObjectClasses when adding new ObjectClass values.
static_assert(static_cast<uint8_t>(ObjectClass::TREE) < kNumObjectClasses,
              "ObjectClass enum grew beyond kNumObjectClasses — update height_priors array");

/// Calibration data for camera-based depth estimation.
struct CalibrationData {
    Eigen::Matrix3f camera_intrinsics = Eigen::Matrix3f::Identity();
    float           camera_height_m   = 1.5f;  // camera height above ground
    float assumed_obstacle_height_m   = 3.0f;  // assumed obstacle height for apparent-size depth
    float depth_scale = 0.7f;           // conservative depth scaling (<1.0 places obstacles closer)
    float bbox_height_noise_px = 2.5f;  // bbox height measurement noise for covariance model (§4.2)
    std::array<float, kNumObjectClasses> height_priors = {
        3.0f,   // UNKNOWN
        1.7f,   // PERSON
        1.5f,   // VEHICLE_CAR
        3.5f,   // VEHICLE_TRUCK
        0.3f,   // DRONE
        0.8f,   // ANIMAL
        10.0f,  // BUILDING
        6.0f,   // TREE
    };
};

}  // namespace drone::perception
