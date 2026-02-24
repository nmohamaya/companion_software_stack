// process2_perception/include/perception/types.h
// All data types for perception: detections, point clouds, radar, tracked/fused objects.
#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <Eigen/Core>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// Object Classification
// ═══════════════════════════════════════════════════════════
enum class ObjectClass : uint8_t {
    UNKNOWN = 0, PERSON = 1, VEHICLE_CAR = 2, VEHICLE_TRUCK = 3,
    DRONE = 4, ANIMAL = 5, BUILDING = 6, TREE = 7,
};

inline const char* object_class_name(ObjectClass c) {
    switch (c) {
        case ObjectClass::PERSON:        return "Person";
        case ObjectClass::VEHICLE_CAR:   return "Car";
        case ObjectClass::VEHICLE_TRUCK: return "Truck";
        case ObjectClass::DRONE:         return "Drone";
        case ObjectClass::ANIMAL:        return "Animal";
        case ObjectClass::BUILDING:      return "Building";
        case ObjectClass::TREE:          return "Tree";
        default:                         return "Unknown";
    }
}

// ═══════════════════════════════════════════════════════════
// 2D Detections (Camera)
// ═══════════════════════════════════════════════════════════
struct Detection2D {
    float x, y, w, h;         // bounding box in pixel coords
    float confidence;
    ObjectClass class_id;
    uint64_t timestamp_ns;
    uint64_t frame_sequence;

    Eigen::Vector2f center() const { return {x + w / 2.0f, y + h / 2.0f}; }
};

struct Detection2DList {
    std::vector<Detection2D> detections;
    uint64_t timestamp_ns;
    uint64_t frame_sequence;
};

// ═══════════════════════════════════════════════════════════
// 3D Point Cloud (LiDAR)
// ═══════════════════════════════════════════════════════════
struct Point3D {
    float x, y, z;
    float intensity;
    uint8_t ring;
};

struct PointCloud {
    std::vector<Point3D> points;
    uint64_t timestamp_ns;
    uint32_t scan_id;
    void reserve(size_t n)  { points.reserve(n); }
    void clear()            { points.clear(); }
    size_t size() const     { return points.size(); }
};

struct LiDARCluster {
    std::vector<Point3D> points;
    Eigen::Vector3f centroid  = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min  = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max  = Eigen::Vector3f::Zero();
    float distance = 0.0f;
};

// ═══════════════════════════════════════════════════════════
// Radar Detections
// ═══════════════════════════════════════════════════════════
struct RadarDetection {
    float range;
    float azimuth;
    float elevation;
    float velocity_radial;
    float rcs;
    uint64_t timestamp_ns;
};

struct RadarDetectionList {
    std::vector<RadarDetection> detections;
    uint64_t timestamp_ns;
};

// ═══════════════════════════════════════════════════════════
// Tracked Objects (after Kalman filtering)
// ═══════════════════════════════════════════════════════════
struct TrackedObject {
    uint32_t track_id;
    ObjectClass class_id;
    float confidence;
    Eigen::Vector2f position_2d;
    Eigen::Vector2f velocity_2d;
    uint32_t age;
    uint32_t hits;
    uint32_t misses;
    uint64_t timestamp_ns;
    enum class State { TENTATIVE, CONFIRMED, LOST };
    State state;
};

struct TrackedObjectList {
    std::vector<TrackedObject> objects;
    uint64_t timestamp_ns;
    uint64_t frame_sequence;
};

// ═══════════════════════════════════════════════════════════
// Fused Objects (multi-sensor fusion output)
// ═══════════════════════════════════════════════════════════
struct FusedObject {
    uint32_t track_id;
    ObjectClass class_id;
    float confidence;
    Eigen::Vector3f position_3d;
    Eigen::Vector3f velocity_3d;
    float heading;
    bool has_camera;
    bool has_lidar;
    bool has_radar;
    Eigen::Matrix3f position_covariance;
    uint64_t timestamp_ns;
};

struct FusedObjectList {
    std::vector<FusedObject> objects;
    uint64_t timestamp_ns;
    uint32_t frame_sequence;
};

} // namespace drone::perception
