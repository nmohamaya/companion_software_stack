// process2_perception/src/fusion_engine.cpp
// Monocular depth estimation from tracked 2D bounding boxes.
// Phase 1C: renamed FusionEngine → CameraOnlyFusionEngine (IFusionEngine).
//
// NOTE: This is NOT multi-sensor fusion. It estimates 3D positions from a
// single monocular RGB camera using pinhole geometry (apparent-size depth).
// The stereo camera feeds P3 (VIO/SLAM), not P2 perception.
//
// Output positions are in camera body frame (forward=X, right=Y, down=Z).
// The camera→world rotation (yaw-only) is applied by fusion_thread in
// main.cpp using the latest drone pose from /slam_pose.
//
// Bug fix (issue #129): position_3d correctly uses pinhole unproject.
#include "perception/fusion_engine.h"

#include "util/ilogger.h"

#include <algorithm>
#include <cmath>

namespace drone::perception {

FusedObjectList CameraOnlyFusionEngine::fuse(const TrackedObjectList& tracked) {
    FusedObjectList output;
    output.timestamp_ns   = tracked.timestamp_ns;
    output.frame_sequence = tracked.frame_sequence;

    const float fx = calib_.camera_intrinsics(0, 0);
    const float fy = calib_.camera_intrinsics(1, 1);
    const float cx = calib_.camera_intrinsics(0, 2);
    const float cy = calib_.camera_intrinsics(1, 2);

    for (const auto& trk : tracked.objects) {
        FusedObject fused;
        fused.track_id            = trk.track_id;
        fused.class_id            = trk.class_id;
        fused.confidence          = trk.confidence;
        fused.has_camera          = true;
        fused.heading             = 0.0f;
        fused.timestamp_ns        = trk.timestamp_ns;
        fused.position_covariance = Eigen::Matrix3f::Identity() * depth_cfg_.covariance_init;

        // position_2d is the Kalman-filtered bbox center in pixels.
        // Unproject through the pinhole model to get a unit ray in camera frame:
        //   camera frame: X = forward (boresight), Y = right, Z = down
        // Normalised image coords:
        const float u         = trk.position_2d.x();  // pixel col (center)
        const float v         = trk.position_2d.y();  // pixel row (center)
        const float ray_fwd   = 1.0f;
        const float ray_right = (u - cx) / std::max(1.0f, fx);
        const float ray_down  = (v - cy) / std::max(1.0f, fy);

        // Depth estimation — three-tier model:
        //
        // 1. Apparent-size (primary): if the tracker supplies a valid bbox height,
        //    use the pinhole size formula:
        //      depth = assumed_obstacle_height_m * fy / bbox_pixel_height
        //    This gives accurate monocular range for known-height obstacles viewed
        //    from any angle, including direct horizontal approach.
        //
        // 2. Ground-plane (fallback): for objects below the horizon where bbox is
        //    unreliable, use the ground-contact formula:
        //      depth ≈ camera_height_m / ray_down
        //
        // 3. Near-horizon fallback: object is near the image centre (horizontal
        //    approach); use a conservative 8 m estimate so the avoider inside its
        //    5 m influence radius reacts well before contact.
        float depth;
        if (trk.bbox_h > depth_cfg_.bbox_h_threshold) {
            // Primary: apparent-size monocular depth
            depth = std::clamp(calib_.assumed_obstacle_height_m * fy / trk.bbox_h,
                               depth_cfg_.depth_min_m, depth_cfg_.depth_max_m);
        } else if (ray_down > depth_cfg_.ray_down_threshold) {
            // Ground-plane fallback
            depth = std::clamp(calib_.camera_height_m / ray_down, depth_cfg_.depth_min_m,
                               depth_cfg_.depth_max_m);
        } else {
            // Near-horizon conservative estimate (was 20 m — too far for 5 m influence radius)
            depth = depth_cfg_.fallback_depth_m;
        }

        // Camera-frame position (forward, right, down)
        fused.position_3d = {depth * ray_fwd, depth * ray_right, depth * ray_down};

        // Velocity: scale 2-D pixel velocity by depth/focal to get approx m/s
        fused.velocity_3d = {0.0f, trk.velocity_2d.x() * depth / std::max(1.0f, fx),
                             trk.velocity_2d.y() * depth / std::max(1.0f, fy)};

        output.objects.push_back(fused);
    }

    return output;
}

}  // namespace drone::perception
