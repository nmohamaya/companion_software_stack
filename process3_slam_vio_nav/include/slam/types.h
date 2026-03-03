// process3_slam_vio_nav/include/slam/types.h
// SLAM/VIO data types: poses, IMU, features, map.
#pragma once
#include <array>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone::slam {

/// 6-DOF pose (position + orientation).
struct Pose {
    double                      timestamp;
    Eigen::Vector3d             position;
    Eigen::Quaterniond          orientation;
    Eigen::Matrix<double, 6, 6> covariance;
    uint32_t                    quality;  // 0=lost, 1=degraded, 2=good

    Pose()
        : timestamp(0)
        , position(Eigen::Vector3d::Zero())
        , orientation(Eigen::Quaterniond::Identity())
        , covariance(Eigen::Matrix<double, 6, 6>::Identity())
        , quality(0) {}
};

/// IMU sample (accelerometer + gyroscope).
struct ImuSample {
    double          timestamp;
    Eigen::Vector3d accel;  // m/s²
    Eigen::Vector3d gyro;   // rad/s
};

/// Visual feature (2D keypoint with 3D triangulated position).
struct Feature {
    uint32_t        id;
    Eigen::Vector2f pixel;        // image coordinates
    Eigen::Vector3d position_3d;  // triangulated world position
    float           response;     // corner response / quality
    bool            is_triangulated;
};

struct Keyframe {
    uint64_t             id;
    double               timestamp;
    Pose                 pose;
    std::vector<Feature> features;
};

/// IMU noise parameters (from datasheet / calibration).
struct ImuNoiseParams {
    double gyro_noise_density  = 0.004;   // rad/s/√Hz
    double gyro_random_walk    = 2.2e-5;  // rad/s²/√Hz
    double accel_noise_density = 0.012;   // m/s²/√Hz
    double accel_random_walk   = 8.0e-5;  // m/s³/√Hz
};

/// Keyframe creation policy.
struct KeyframePolicy {
    double min_parallax_px   = 15.0;
    double min_tracked_ratio = 0.6;
    double max_time_sec      = 0.5;

    bool should_create_keyframe(double median_parallax, int tracked_count, int initial_count,
                                double dt_since_last_kf) const {
        if (median_parallax > min_parallax_px) return true;
        double ratio = static_cast<double>(tracked_count) / initial_count;
        if (ratio < min_tracked_ratio) return true;
        if (dt_since_last_kf > max_time_sec) return true;
        return false;
    }
};

}  // namespace drone::slam
