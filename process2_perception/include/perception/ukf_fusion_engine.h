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

/// Local pi constant — avoids dependence on non-standard M_PI macro.
inline constexpr float kPi = 3.14159265358979323846f;

/// Wrap angle to [-π, π].
inline float wrap_angle(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

/// Camera intrinsic parameters for pinhole unprojection.
struct CameraIntrinsics {
    float fx = 277.0f;  // focal length x
    float fy = 277.0f;  // focal length y
    float cx = 320.0f;  // principal point x
    float cy = 240.0f;  // principal point y
};

/// Configurable radar measurement noise parameters.
struct RadarNoiseConfig {
    float range_std_m           = 0.3f;    // ±0.3 m
    float azimuth_std_rad       = 0.026f;  // ±1.5°
    float elevation_std_rad     = 0.026f;  // ±1.5°
    float velocity_std_mps      = 0.1f;    // ±0.1 m/s
    float gate_threshold        = 9.21f;   // χ²(4) at 95% confidence
    float min_object_altitude_m = 0.3f;    // reject radar returns below this AGL
    bool  ground_filter_enabled = true;    // enable/disable ground-plane filter
    float altitude_gate_m       = 2.0f;    // reject radar-track pairs with |body_z diff| > this

    // Radar-primary architecture (Phase D) — independent radar track creation
    float    radar_orphan_proximity_m    = 3.0f;   // min dist to existing track for orphan creation
    float    radar_adopt_gate_m          = 5.0f;   // max range error for camera adoption
    float    radar_only_default_radius_m = 1.5f;   // conservative default inflation (no bbox)
    float    radar_max_orphan_range_m    = 40.0f;  // max range for orphan creation (77GHz ≈ 50m)
    int      radar_orphan_min_hits       = 1;      // min radar observations before output
    uint32_t radar_only_promotion_hits   = 3;      // radar hits for static promotion (tunable)
    bool     radar_only_enabled          = true;  // enable radar-only track initiation (Issue #231)
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
                       const RadarNoiseConfig& radar_cfg      = RadarNoiseConfig{},
                       const CameraIntrinsics& cam_intrinsics = CameraIntrinsics{});

    /// Initialize from a radar detection only (no camera).
    /// Converts spherical (range, azimuth, elevation) → body-frame FRD Cartesian.
    explicit ObjectUKF(const drone::ipc::RadarDetection& det, uint32_t id,
                       const RadarNoiseConfig& radar_cfg = RadarNoiseConfig{});

    /// Predict step (constant-velocity model).
    void predict(float dt = 1.0f / 30.0f);

    /// Update with camera measurement (bearing + depth estimate).
    void update_camera(const TrackedObject& trk, float estimated_depth,
                       const CameraIntrinsics& cam_intrinsics = CameraIntrinsics{});

    /// Update with radar measurement (range, azimuth, elevation, radial velocity).
    void update_radar(const drone::ipc::RadarDetection& det);

    /// Convert current state to radar measurement space for association.
    [[nodiscard]] RadarMeasVec predicted_radar_measurement() const;

    /// Compute radar innovation covariance S = Pzz + R_radar via sigma points.
    /// Used for proper Mahalanobis gating that accounts for state uncertainty.
    [[nodiscard]] RadarMeasMat predicted_radar_innovation_cov() const;

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
    uint32_t radar_update_count{0};  ///< Number of radar updates received (for B1 trust)
    bool     radar_only{false};      ///< True if created from radar without camera (Phase D)

    /// Tighten depth covariance after radar provides accurate range.
    void set_radar_confirmed_depth(float radar_range);

    /// Set depth covariance P(0,0) — used to inflate uncertainty for camera-only tracks.
    void set_depth_covariance(float p00) { P_(0, 0) = p00; }

    /// Adopt a camera observation into a radar-only track.
    /// Refines lateral/vertical state from camera bearing and clears radar_only flag.
    void adopt_camera(const TrackedObject& trk, const CameraIntrinsics& cam_intr);

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

/// A dormant obstacle remembered in world frame after its ByteTrack track was lost.
/// When a new track appears near a dormant obstacle, it is re-identified as the
/// same physical object — preventing duplicate grid entries from parallax.
struct DormantObstacle {
    Eigen::Vector3f world_pos    = Eigen::Vector3f::Zero();  // averaged world-frame centroid
    int             observations = 0;                        // number of merged track observations
    uint64_t        last_seen_ns = 0;                        // timestamp of last observation
};

/// UKF-based fusion engine with camera + radar fusion.
/// Maintains per-object UKF instances, matched by track_id.
/// Dormant obstacle pool enables cross-view re-identification (Issue #237).
class UKFFusionEngine : public IFusionEngine {
public:
    explicit UKFFusionEngine(const CalibrationData&  calib,
                             const RadarNoiseConfig& radar_cfg = RadarNoiseConfig{},
                             bool radar_enabled = false, float dormant_merge_radius_m = 5.0f,
                             int max_dormant = 32);

    FusedObjectList fuse(const TrackedObjectList& tracked) override;
    std::string     name() const override { return "ukf"; }
    void            reset() override;

    /// Provide radar detections for the next fuse() call.
    void set_radar_detections(const drone::ipc::RadarDetectionList& detections) override;

    /// Provide current drone altitude for radar ground-plane filtering.
    /// @param altitude_m  World-frame z with ground at z=0 (i.e. AGL when
    ///                    the world origin sits on the ground plane).
    void set_drone_altitude(float altitude_m) override;

    /// Provide full drone pose for world-frame dormant re-identification.
    void set_drone_pose(float north, float east, float up, float yaw) override;

    /// Access dormant obstacles (for testing).
    [[nodiscard]] const std::vector<DormantObstacle>& dormant_obstacles() const {
        return dormant_obstacles_;
    }

private:
    CalibrationData                         calib_;
    RadarNoiseConfig                        radar_cfg_;
    std::unordered_map<uint32_t, ObjectUKF> filters_;  // track_id → UKF

    drone::ipc::RadarDetectionList radar_dets_;
    bool                           has_radar_data_{false};
    bool                           radar_enabled_{false};
    float                          drone_altitude_m_{0.0f};
    bool                           has_altitude_{false};

    // Drone pose for body→world transform (dormant re-ID)
    float drone_north_{0.0f};
    float drone_east_{0.0f};
    float drone_up_{0.0f};
    float drone_yaw_{0.0f};
    bool  has_pose_{false};

    // Dormant obstacle pool — world-frame memory of lost tracks
    std::vector<DormantObstacle> dormant_obstacles_;
    float                        dormant_merge_radius_m_;
    int                          max_dormant_;

    // Map active track_id → index into dormant_obstacles_ (-1 = no match)
    std::unordered_map<uint32_t, int> track_to_dormant_;

    // Radar-primary: monotonic ID counter for radar-only tracks (high bit set)
    uint32_t next_radar_track_id_{0x80000000u};

    float           estimate_depth(const TrackedObject& trk) const;
    Eigen::Vector3f body_to_world(const Eigen::Vector3f& body) const;
    int             find_nearest_dormant(const Eigen::Vector3f& world_pos,
                                         const Eigen::Matrix3f& pos_cov = Eigen::Matrix3f::Identity() *
                                                                          10.0f) const;

    /// Try to associate a UKF filter with unmatched radar detections.
    /// Returns true and updates the filter if a match is found.
    bool try_associate_radar(ObjectUKF& ukf, std::vector<bool>& radar_matched,
                             const Eigen::LLT<ObjectUKF::RadarMeasMat>& radar_llt,
                             bool radar_llt_ok, const ObjectUKF::RadarMeasVec& R_diag_inv,
                             float range_3sigma);
};

}  // namespace drone::perception
