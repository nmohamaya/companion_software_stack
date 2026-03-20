// process2_perception/include/perception/fusion_engine.h
// Camera-only fusion: tracked 2D objects → 3D fused objects.
// Implements IFusionEngine for factory-based construction (Phase 1C, Issue #114).
#pragma once
#include "perception/ifusion_engine.h"
#include "perception/types.h"

#include <vector>

namespace drone::perception {

/// Configuration for monocular depth estimation in CameraOnlyFusionEngine.
struct DepthEstimationConfig {
    float covariance_init    = 5.0f;   // initial position_covariance diagonal value
    float bbox_h_threshold   = 10.0f;  // min bbox height (px) for apparent-size depth
    float depth_min_m        = 1.0f;   // minimum clamped depth (metres)
    float depth_max_m        = 40.0f;  // maximum clamped depth (metres)
    float ray_down_threshold = 0.01f;  // min ray_down for ground-plane fallback
    float fallback_depth_m   = 8.0f;   // near-horizon conservative depth estimate
};

/// Converts tracked 2D detections into 3D positions using monocular depth estimation.
/// This is NOT multi-sensor fusion — it uses a single RGB camera's bounding boxes
/// with pinhole geometry to estimate depth. See ifusion_engine.h for full context.
/// The fusion_thread in main.cpp rotates these camera-frame positions to world ENU
/// frame using the latest drone yaw from /slam_pose.
class CameraOnlyFusionEngine : public IFusionEngine {
public:
    explicit CameraOnlyFusionEngine(const CalibrationData&       calib,
                                    const DepthEstimationConfig& depth_cfg = {})
        : calib_(calib), depth_cfg_(depth_cfg) {}

    FusedObjectList fuse(const TrackedObjectList& tracked) override;
    std::string     name() const override { return "camera_only"; }
    void            reset() override {}  // stateless — nothing to reset

private:
    CalibrationData       calib_;
    DepthEstimationConfig depth_cfg_;
};

/// Backward compatibility alias.
using FusionEngine = CameraOnlyFusionEngine;

}  // namespace drone::perception
