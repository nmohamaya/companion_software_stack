// process2_perception/include/perception/ukf_fusion_engine.h
// Unscented Kalman Filter fusion engine.
// Per-object UKF state: [x, y, z, vx, vy, vz].
// Measurement models: camera (bearing + depth), radar (range, azimuth, elevation, radial_velocity).
// Phase 1C (Issue #114), radar fusion (Issue #210).
#pragma once
#include "ipc/ipc_types.h"
#include "perception/ifusion_engine.h"
#include "perception/types.h"

#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace drone::perception {

/// Wrap angle to [-π, π].
inline float wrap_angle(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

/// Configurable radar measurement noise parameters.
struct RadarNoiseConfig {
    float range_std_m       = 0.3f;    // ±0.3 m
    float azimuth_std_rad   = 0.026f;  // ±1.5°
    float elevation_std_rad = 0.026f;  // ±1.5°
    float velocity_std_mps  = 0.1f;    // ±0.1 m/s
    float gate_threshold    = 9.21f;   // χ²(4) at 95% confidence
};

/// Per-object UKF state for 3D tracking.
class ObjectUKF {
public:
    static constexpr int STATE_DIM      = 6;  // [x, y, z, vx, vy, vz]
    static constexpr int MEAS_DIM       = 3;  // camera: [bearing_x, bearing_y, depth]
    static constexpr int RADAR_MEAS_DIM = 4;  // radar: [range, azimuth, elevation, radial_velocity]

    using StateVec     = Eigen::Matrix<float, STATE_DIM, 1>;
    using StateMat     = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;
    using MeasVec      = Eigen::Matrix<float, MEAS_DIM, 1>;
    using MeasMat      = Eigen::Matrix<float, MEAS_DIM, MEAS_DIM>;
    using RadarMeasVec = Eigen::Matrix<float, RADAR_MEAS_DIM, 1>;
    using RadarMeasMat = Eigen::Matrix<float, RADAR_MEAS_DIM, RADAR_MEAS_DIM>;

    /// Initialize from a 2D tracked object + estimated depth.
    explicit ObjectUKF(const TrackedObject& trk, float initial_depth,
                       const RadarNoiseConfig& radar_cfg = RadarNoiseConfig{});

    /// Predict step (constant-velocity model).
    void predict(float dt = 1.0f / 30.0f);

    /// Update with camera measurement (bearing + depth estimate).
    void update_camera(const TrackedObject& trk, float estimated_depth);

    /// Update with radar measurement (range, azimuth, elevation, radial velocity).
    void update_radar(const drone::ipc::RadarDetection& det);

    /// Convert current state to radar measurement space for association.
    [[nodiscard]] RadarMeasVec predicted_radar_measurement() const;

    /// Get current 3D position estimate.
    [[nodiscard]] Eigen::Vector3f position() const;

    /// Get current 3D velocity estimate.
    [[nodiscard]] Eigen::Vector3f velocity() const;

    /// Get position covariance (upper-left 3×3 of P).
    [[nodiscard]] Eigen::Matrix3f position_covariance() const;

    /// Access radar noise matrix for gating computations.
    [[nodiscard]] const RadarMeasMat& radar_noise() const { return R_radar_; }

    uint32_t track_id{0};
    uint32_t age{0};

private:
    /// Generate sigma points using unscented transform.
    std::vector<StateVec> generate_sigma_points() const;

    /// Radar measurement model: state → [range, azimuth, elevation, radial_velocity].
    static RadarMeasVec radar_measurement_model(const StateVec& s);

    /// UKF tuning parameters
    static constexpr float kAlpha = 1e-3f;
    static constexpr float kBeta  = 2.0f;
    static constexpr float kKappa = 0.0f;

    StateVec     x_;        // state estimate
    StateMat     P_;        // state covariance
    StateMat     Q_;        // process noise
    MeasMat      R_;        // camera measurement noise
    RadarMeasMat R_radar_;  // radar measurement noise
};

/// UKF-based fusion engine with camera + radar fusion.
/// Maintains per-object UKF instances, matched by track_id.
class UKFFusionEngine : public IFusionEngine {
public:
    explicit UKFFusionEngine(const CalibrationData&  calib,
                             const RadarNoiseConfig& radar_cfg     = RadarNoiseConfig{},
                             bool                    radar_enabled = false);

    FusedObjectList fuse(const TrackedObjectList& tracked) override;
    std::string     name() const override { return "ukf"; }
    void            reset() override;

    /// Provide radar detections for the next fuse() call.
    void set_radar_detections(const drone::ipc::RadarDetectionList& detections) override;

private:
    CalibrationData                         calib_;
    RadarNoiseConfig                        radar_cfg_;
    std::unordered_map<uint32_t, ObjectUKF> filters_;  // track_id → UKF

    drone::ipc::RadarDetectionList radar_dets_;
    bool                           has_radar_data_{false};
    bool                           radar_enabled_{false};

    float estimate_depth(const TrackedObject& trk) const;
};

}  // namespace drone::perception
