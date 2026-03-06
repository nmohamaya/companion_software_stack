// process2_perception/include/perception/ukf_fusion_engine.h
// Unscented Kalman Filter fusion engine.
// Per-object UKF state: [x, y, z, vx, vy, vz].
// Measurement models: camera (2D bbox → bearing + depth estimate),
//                     thermal (bearing confirmation + heat confidence).
// Phase 1C (Issue #114).
#pragma once
#include "perception/ifusion_engine.h"
#include "perception/types.h"

#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace drone::perception {

/// Per-object UKF state for 3D tracking.
class ObjectUKF {
public:
    static constexpr int STATE_DIM = 6;  // [x, y, z, vx, vy, vz]
    static constexpr int MEAS_DIM  = 3;  // camera: [bearing_x, bearing_y, depth]

    using StateVec = Eigen::Matrix<float, STATE_DIM, 1>;
    using StateMat = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;
    using MeasVec  = Eigen::Matrix<float, MEAS_DIM, 1>;
    using MeasMat  = Eigen::Matrix<float, MEAS_DIM, MEAS_DIM>;

    /// Initialize from a 2D tracked object + estimated depth.
    explicit ObjectUKF(const TrackedObject& trk, float initial_depth);

    /// Predict step (constant-velocity model).
    void predict(float dt = 1.0f / 30.0f);

    /// Update with camera measurement (bearing + depth estimate).
    void update_camera(const TrackedObject& trk, float estimated_depth);

    /// Update with thermal confirmation (reduces covariance).
    void update_thermal(float thermal_confidence);

    /// Get current 3D position estimate.
    [[nodiscard]] Eigen::Vector3f position() const;

    /// Get current 3D velocity estimate.
    [[nodiscard]] Eigen::Vector3f velocity() const;

    /// Get position covariance (upper-left 3×3 of P).
    [[nodiscard]] Eigen::Matrix3f position_covariance() const;

    uint32_t track_id{0};
    uint32_t age{0};
    bool     has_thermal{false};

private:
    /// Generate sigma points using unscented transform.
    std::vector<StateVec> generate_sigma_points() const;

    /// UKF tuning parameters
    static constexpr float kAlpha = 1e-3f;
    static constexpr float kBeta  = 2.0f;
    static constexpr float kKappa = 0.0f;

    StateVec x_;  // state estimate
    StateMat P_;  // state covariance
    StateMat Q_;  // process noise
    MeasMat  R_;  // measurement noise
};

/// UKF-based fusion engine.
/// Maintains per-object UKF instances, matched by track_id.
/// Supports camera + thermal measurement updates.
class UKFFusionEngine : public IFusionEngine {
public:
    explicit UKFFusionEngine(const CalibrationData& calib);

    FusedObjectList fuse(const TrackedObjectList& tracked) override;
    void            set_thermal_detections(const Detection2DList& thermal) override;
    std::string     name() const override { return "ukf"; }
    void            reset() override;

private:
    CalibrationData                         calib_;
    std::unordered_map<uint32_t, ObjectUKF> filters_;  // track_id → UKF
    Detection2DList                         thermal_dets_;
    bool                                    has_thermal_frame_{false};

    float estimate_depth(const TrackedObject& trk) const;
};

}  // namespace drone::perception
