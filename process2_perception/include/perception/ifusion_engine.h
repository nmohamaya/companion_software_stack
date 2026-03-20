// process2_perception/include/perception/ifusion_engine.h
// Abstract fusion engine interface — strategy pattern for multi-sensor fusion.
// Concrete implementations: CameraOnlyFusionEngine, UKFFusionEngine.
// Phase 1C (Issue #114).
#pragma once
#include "perception/types.h"

#include <memory>
#include <string>

namespace drone {
class Config;
}

namespace drone::perception {

/// Abstract fusion engine interface.
/// Implementations consume tracked 2D objects and produce 3D fused objects
/// with position + velocity estimates.
///
/// IMPORTANT TERMINOLOGY NOTE:
/// Despite the name "fusion", the current backends do NOT perform multi-sensor
/// fusion (e.g., fusing camera + LiDAR + radar). What they actually do:
///
///   1. camera_only: Monocular depth estimation via pinhole geometry
///      (apparent-size formula), then yaw rotation to world frame.
///      Input: single RGB camera tracked 2D bboxes.
///      Output: 3D camera-frame positions.
///
///   2. ukf: Per-object Unscented Kalman Filter for temporal smoothing
///      of the monocular 3D estimates. Still single-camera input.
///
/// The stereo camera feeds P3 (VIO/SLAM) for pose estimation, NOT P2.
/// True multi-sensor fusion would require integrating stereo depth, LiDAR
/// point clouds, or radar returns — none of which are implemented yet.
class IFusionEngine {
public:
    virtual ~IFusionEngine() = default;

    /// Fuse one frame of tracked objects into 3D world-space estimates.
    [[nodiscard]] virtual FusedObjectList fuse(const TrackedObjectList& tracked) = 0;

    /// Human-readable name for logging.
    [[nodiscard]] virtual std::string name() const = 0;

    /// Reset all internal state.
    virtual void reset() = 0;
};

/// Factory: create a fusion engine from a backend name.
/// Supported backends: "camera_only" (default), "ukf".
std::unique_ptr<IFusionEngine> create_fusion_engine(const std::string&     backend,
                                                    const CalibrationData& calib,
                                                    const drone::Config*   cfg = nullptr);

}  // namespace drone::perception
