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
/// Implementations consume tracked 2D objects (and optionally thermal data)
/// and produce 3D fused objects with position + velocity estimates.
class IFusionEngine {
public:
    virtual ~IFusionEngine() = default;

    /// Fuse one frame of tracked objects into 3D world-space estimates.
    [[nodiscard]] virtual FusedObjectList fuse(const TrackedObjectList& tracked) = 0;

    /// Provide thermal detections for the current frame (optional).
    /// Engines that don't use thermal data may ignore this.
    virtual void set_thermal_detections([[maybe_unused]] const Detection2DList& thermal) {}

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
