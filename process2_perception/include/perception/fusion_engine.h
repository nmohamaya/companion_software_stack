// process2_perception/include/perception/fusion_engine.h
// Camera-only fusion: tracked 2D objects → 3D fused objects.
// Implements IFusionEngine for factory-based construction (Phase 1C, Issue #114).
#pragma once
#include "perception/ifusion_engine.h"
#include "perception/types.h"

#include <vector>

namespace drone::perception {

/// Converts tracked 2D detections into 3D fused objects using camera geometry.
/// This is the simple depth-from-bbox approach — no filtering, no thermal.
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
