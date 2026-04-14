// process2_perception/include/perception/ifusion_engine.h
// Abstract fusion engine interface — strategy pattern for multi-sensor fusion.
// Concrete implementations: CameraOnlyFusionEngine, UKFFusionEngine.
// Phase 1C (Issue #114), radar fusion (Issue #210).
#pragma once
#include "hal/idepth_estimator.h"
#include "ipc/ipc_types.h"
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
/// Sensor modalities:
///   1. camera_only: Monocular depth estimation via pinhole geometry
///      (apparent-size formula), then yaw rotation to world frame.
///
///   2. ukf: Per-object Unscented Kalman Filter with camera + radar fusion.
///      Camera provides bearing + monocular depth; radar provides direct range,
///      azimuth, elevation, and Doppler radial velocity. The UKF fuses both
///      sensor modalities for tighter state estimates.
///
/// The stereo camera feeds P3 (VIO/SLAM) for pose estimation, NOT P2.
class IFusionEngine {
public:
    virtual ~IFusionEngine() = default;

    /// Fuse one frame of tracked objects into 3D world-space estimates.
    [[nodiscard]] virtual FusedObjectList fuse(const TrackedObjectList& tracked) = 0;

    /// Human-readable name for logging.
    [[nodiscard]] virtual std::string name() const = 0;

    /// Reset all internal state.
    virtual void reset() = 0;

    /// Provide radar detections for the next fuse() call.
    /// Default no-op — only UKFFusionEngine uses radar data.
    virtual void set_radar_detections(const drone::ipc::RadarDetectionList& /*detections*/) {}

    /// Provide current drone altitude (AGL) for radar ground-plane filtering.
    /// Default no-op — only UKFFusionEngine uses altitude data.
    virtual void set_drone_altitude(float /*altitude_m*/) {}

    /// Provide full drone pose for world-frame dormant obstacle re-identification.
    /// Default no-op — only UKFFusionEngine uses this.
    virtual void set_drone_pose(float /*north*/, float /*east*/, float /*up*/, float /*yaw*/) {}

    /// Provide ML depth map for depth-enhanced fusion.
    /// Default no-op — only UKFFusionEngine uses ML depth data.
    virtual void set_depth_map(const drone::hal::DepthMap& /*depth_map*/) {}
};

/// Factory: create a fusion engine from a backend name.
/// Supported backends: "camera_only" (default), "ukf".
std::unique_ptr<IFusionEngine> create_fusion_engine(const std::string&     backend,
                                                    const CalibrationData& calib,
                                                    const drone::Config*   cfg = nullptr);

}  // namespace drone::perception
