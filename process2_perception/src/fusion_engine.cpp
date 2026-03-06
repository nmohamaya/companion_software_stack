// process2_perception/src/fusion_engine.cpp
// Camera-only fusion implementation.
// LiDAR/radar paths removed (Phase 1A, Issue #112).
#include "perception/fusion_engine.h"

#include <algorithm>
#include <cmath>

#include <spdlog/spdlog.h>

namespace drone::perception {

FusedObjectList FusionEngine::fuse(const TrackedObjectList& tracked) {
    FusedObjectList output;
    output.timestamp_ns   = tracked.timestamp_ns;
    output.frame_sequence = tracked.frame_sequence;

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

        output.objects.push_back(fused);
    }

    return output;
}

}  // namespace drone::perception
