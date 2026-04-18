// process3_slam_vio_nav/include/slam/swvio_types.h
// Types for the Sliding-Window Visual-Inertial Odometry (SWVIO) backend.
//
// Defines the error-state parameterization:
//   IMU state: 15-DOF (orientation[3], position[3], velocity[3],
//              gyro_bias[3], accel_bias[3])
//   Camera clone: 6-DOF (orientation[3], position[3])
//   Full state: 15 + 6*N_clones
//
// Covariance is maintained over the full error state and grows/shrinks
// as camera clones are augmented/marginalized.
#pragma once

#include <cstdint>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone::slam {

/// Parameters for the SWVIO backend.
/// All tunables should be loaded from drone::Config at construction time.
struct SWVIOParams {
    int    max_clones            = 15;     // sliding window size
    int    min_track_length      = 3;      // minimum feature track length for triangulation
    int    max_gn_iterations     = 5;      // Gauss-Newton iterations per update
    double convergence_threshold = 1e-6;   // GN convergence criterion
    double chi2_threshold        = 5.991;  // chi-squared gate for outlier rejection (95%, 2 DOF)
    double noise_image_pixel     = 1.0;    // image measurement noise (pixels)
    int    init_frames           = 10;     // frames before transitioning out of INITIALIZING
    bool   enable_marginalization_prior = true;
    double good_trace_max               = 0.1;  // position trace threshold for NOMINAL health
    double degraded_trace_max           = 1.0;  // position trace threshold for DEGRADED health

    // Gravity vector in world frame (NED convention: [0,0,9.81] for "up"
    // in accelerometer reading, but gravity acceleration is [0,0,-9.81])
    Eigen::Vector3d gravity{0, 0, -9.81};

    // IMU-camera extrinsic transformation (identity = co-located sensors)
    Eigen::Quaterniond T_imu_cam_rotation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d    T_imu_cam_translation{Eigen::Vector3d::Zero()};

    [[nodiscard]] constexpr bool validate() const {
        return max_clones > 0 && max_clones <= 30 && init_frames > 0 && good_trace_max > 0.0 &&
               degraded_trace_max > good_trace_max && noise_image_pixel > 0.0 &&
               chi2_threshold > 0.0;
    }
};

/// IMU state: full nominal state (position, velocity, orientation, biases).
struct IMUState {
    double             timestamp = 0;
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d    position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d    velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d    gyro_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d    accel_bias{Eigen::Vector3d::Zero()};
};

/// A camera clone stores the pose at the time a frame was captured.
/// Used for multi-view geometry constraints in the sliding window.
struct CameraClone {
    uint64_t           clone_id  = 0;
    double             timestamp = 0;
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d    position{Eigen::Vector3d::Zero()};
};

/// Full SWVIO state: IMU state + sliding window of camera clones + covariance.
struct SWVIOState {
    IMUState                imu;
    std::deque<CameraClone> clones;
    // Covariance is pre-allocated to max capacity; active_cov_dim_ tracks the used region.
    Eigen::MatrixXd covariance;  // (15 + 6*N_clones) x (15 + 6*N_clones)

    // ── Error state indices ─────────────────────────────────
    static constexpr int kIdxTheta      = 0;   // orientation error (3)
    static constexpr int kIdxPos        = 3;   // position error (3)
    static constexpr int kIdxVel        = 6;   // velocity error (3)
    static constexpr int kIdxBg         = 9;   // gyro bias error (3)
    static constexpr int kIdxBa         = 12;  // accel bias error (3)
    static constexpr int kImuStateDim   = 15;
    static constexpr int kCloneStateDim = 6;  // orientation(3) + position(3)

    /// Total dimension of the error state vector.
    [[nodiscard]] int state_dim() const {
        return kImuStateDim + kCloneStateDim * static_cast<int>(clones.size());
    }

    /// Start index of clone i in the error state vector.
    [[nodiscard]] int clone_index(int i) const { return kImuStateDim + kCloneStateDim * i; }
};

}  // namespace drone::slam
