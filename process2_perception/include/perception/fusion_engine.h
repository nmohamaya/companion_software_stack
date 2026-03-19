// process2_perception/include/perception/fusion_engine.h
// Camera-only fusion: tracked 2D objects → 3D fused objects.
// Implements IFusionEngine for factory-based construction (Phase 1C, Issue #114).
#pragma once
#include "perception/ifusion_engine.h"
#include "perception/types.h"

#include <vector>

namespace drone::perception {

/// Converts tracked 2D detections into 3D positions using monocular depth estimation.
/// This is NOT multi-sensor fusion — it uses a single RGB camera's bounding boxes
/// with pinhole geometry to estimate depth. See ifusion_engine.h for full context.
/// The fusion_thread in main.cpp rotates these camera-frame positions to world ENU
/// frame using the latest drone yaw from /slam_pose.
class CameraOnlyFusionEngine : public IFusionEngine {
public:
    explicit CameraOnlyFusionEngine(const CalibrationData& calib) : calib_(calib) {}

    FusedObjectList fuse(const TrackedObjectList& tracked) override;
    std::string     name() const override { return "camera_only"; }
    void            reset() override {}  // stateless — nothing to reset

private:
    CalibrationData calib_;
};

/// Backward compatibility alias.
using FusionEngine = CameraOnlyFusionEngine;

}  // namespace drone::perception
