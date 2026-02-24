// process2_perception/include/perception/fusion_engine.h
// Multi-sensor fusion: camera + LiDAR + radar → fused objects.
#pragma once
#include "perception/types.h"
#include <vector>

namespace drone::perception {

/// Calibration data for projecting between sensor coordinate frames.
struct CalibrationData {
    Eigen::Matrix3f camera_intrinsics;
    Eigen::Matrix4f T_cam_lidar;   // LiDAR → camera transform
    Eigen::Matrix4f T_cam_radar;   // radar → camera transform
    float camera_height_m = 1.5f;  // camera height above ground
};

/// Fuses detections from camera tracker, LiDAR clusters, and radar scans.
class FusionEngine {
public:
    explicit FusionEngine(const CalibrationData& calib) : calib_(calib) {}

    FusedObjectList fuse(const TrackedObjectList& tracked,
                         const std::vector<LiDARCluster>& lidar_clusters,
                         const RadarDetectionList& radar_detections);

private:
    CalibrationData calib_;

    /// Project LiDAR point into camera image coordinates.
    Eigen::Vector2f project_lidar_to_camera(const Eigen::Vector3f& pt_lidar) const;

    /// Check if a projected point falls inside a 2D bounding box.
    bool point_in_bbox(const Eigen::Vector2f& pt, const Detection2D& det) const;

    /// Convert radar polar to Cartesian.
    Eigen::Vector3f radar_to_cartesian(const RadarDetection& det) const;
};

} // namespace drone::perception
