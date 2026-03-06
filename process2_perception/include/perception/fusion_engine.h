// process2_perception/include/perception/fusion_engine.h
// Camera-only fusion: tracked 2D objects → 3D fused objects.
// LiDAR/radar paths removed (Phase 1A, Issue #112).
// Thermal camera fusion will be added in Phase 1C (Issue #114).
#pragma once
#include "perception/types.h"

#include <vector>

namespace drone::perception {

/// Calibration data for camera-based depth estimation.
struct CalibrationData {
    Eigen::Matrix3f camera_intrinsics;
    float           camera_height_m = 1.5f;  // camera height above ground
};

/// Converts tracked 2D detections into 3D fused objects using camera geometry.
class FusionEngine {
public:
    explicit FusionEngine(const CalibrationData& calib) : calib_(calib) {}

    FusedObjectList fuse(const TrackedObjectList& tracked);

private:
    CalibrationData calib_;
};

}  // namespace drone::perception
