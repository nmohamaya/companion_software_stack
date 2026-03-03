// process2_perception/src/fusion_engine.cpp
// Multi-sensor fusion implementation.
#include "perception/fusion_engine.h"

#include <cmath>

#include <spdlog/spdlog.h>

namespace drone::perception {

Eigen::Vector2f FusionEngine::project_lidar_to_camera(const Eigen::Vector3f& pt_lidar) const {
    Eigen::Vector4f pt_h;
    pt_h << pt_lidar, 1.0f;
    Eigen::Vector4f pt_cam = calib_.T_cam_lidar * pt_h;

    if (pt_cam(2) <= 0.01f) return {-1.0f, -1.0f};  // behind camera

    Eigen::Vector3f projected = calib_.camera_intrinsics * pt_cam.head<3>();
    return {projected(0) / projected(2), projected(1) / projected(2)};
}

bool FusionEngine::point_in_bbox(const Eigen::Vector2f& pt, const Detection2D& det) const {
    return pt.x() >= det.x && pt.x() <= det.x + det.w && pt.y() >= det.y && pt.y() <= det.y + det.h;
}

Eigen::Vector3f FusionEngine::radar_to_cartesian(const RadarDetection& det) const {
    float x = det.range * std::cos(det.elevation) * std::cos(det.azimuth);
    float y = det.range * std::cos(det.elevation) * std::sin(det.azimuth);
    float z = det.range * std::sin(det.elevation);
    return {x, y, z};
}

FusedObjectList FusionEngine::fuse(const TrackedObjectList&         tracked,
                                   const std::vector<LiDARCluster>& lidar_clusters,
                                   const RadarDetectionList&        radar_detections) {
    FusedObjectList output;
    output.timestamp_ns   = tracked.timestamp_ns;
    output.frame_sequence = tracked.frame_sequence;

    // For each tracked camera detection, try to associate LiDAR + radar
    for (const auto& trk : tracked.objects) {
        FusedObject fused;
        fused.track_id            = trk.track_id;
        fused.class_id            = trk.class_id;
        fused.confidence          = trk.confidence;
        fused.has_camera          = true;
        fused.has_lidar           = false;
        fused.has_radar           = false;
        fused.heading             = 0.0f;
        fused.timestamp_ns        = trk.timestamp_ns;
        fused.position_covariance = Eigen::Matrix3f::Identity() * 5.0f;

        // Estimate 3D from camera (rough depth from bbox height)
        float estimated_depth = calib_.camera_height_m * 500.0f /
                                std::max(10.0f, trk.position_2d.y());
        fused.position_3d = {estimated_depth, 0.0f, 0.0f};
        fused.velocity_3d = {trk.velocity_2d.x() * 0.1f, trk.velocity_2d.y() * 0.1f, 0.0f};

        // Try to associate with LiDAR cluster (nearest centroid)
        float               best_lidar_dist = 5.0f;  // max association distance
        const LiDARCluster* best_lidar      = nullptr;
        for (const auto& cluster : lidar_clusters) {
            float d = (cluster.centroid - fused.position_3d).norm();
            if (d < best_lidar_dist) {
                best_lidar_dist = d;
                best_lidar      = &cluster;
            }
        }

        if (best_lidar) {
            fused.has_lidar = true;
            // Weighted merge: trust LiDAR position more (weight 0.8)
            fused.position_3d = 0.8f * best_lidar->centroid + 0.2f * fused.position_3d;
            fused.confidence  = std::min(0.99f, fused.confidence + 0.15f);
            fused.position_covariance *= 0.3f;  // tighter uncertainty
        }

        // Try to associate with radar detection
        for (const auto& rdet : radar_detections.detections) {
            Eigen::Vector3f radar_pos = radar_to_cartesian(rdet);
            float           d         = (radar_pos - fused.position_3d).norm();
            if (d < 3.0f) {
                fused.has_radar       = true;
                fused.velocity_3d.x() = rdet.velocity_radial * std::cos(rdet.azimuth);
                fused.velocity_3d.y() = rdet.velocity_radial * std::sin(rdet.azimuth);
                fused.confidence      = std::min(0.99f, fused.confidence + 0.1f);
                break;
            }
        }

        output.objects.push_back(fused);
    }

    // Add any unmatched LiDAR-only detections
    for (const auto& cluster : lidar_clusters) {
        bool matched = false;
        for (const auto& f : output.objects) {
            if (f.has_lidar && (f.position_3d - cluster.centroid).norm() < 1.0f) {
                matched = true;
                break;
            }
        }
        if (!matched && cluster.distance < 50.0f) {
            FusedObject fused;
            fused.track_id            = 0;
            fused.class_id            = ObjectClass::UNKNOWN;
            fused.confidence          = 0.4f;
            fused.position_3d         = cluster.centroid;
            fused.velocity_3d         = Eigen::Vector3f::Zero();
            fused.heading             = 0.0f;
            fused.has_camera          = false;
            fused.has_lidar           = true;
            fused.has_radar           = false;
            fused.position_covariance = Eigen::Matrix3f::Identity() * 2.0f;
            fused.timestamp_ns        = tracked.timestamp_ns;
            output.objects.push_back(fused);
        }
    }

    return output;
}

}  // namespace drone::perception
