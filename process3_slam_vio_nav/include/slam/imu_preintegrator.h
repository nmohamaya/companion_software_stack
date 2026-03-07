// process3_slam_vio_nav/include/slam/imu_preintegrator.h
// IMU pre-integration: accumulates gyroscope and accelerometer readings
// between visual keyframes into a compact PreintegratedMeasurement.
//
// Follows the Forster et al. (2017) on-manifold pre-integration:
//   Δα (position), Δβ (velocity), Δγ (rotation)
// with first-order noise propagation and bias Jacobians.
//
// Error handling:
//   - Returns VIOResult<PreintegratedMeasurement> on integrate()
//   - Detects and reports IMU data gaps (>2× expected dt)
//   - Detects IMU saturation (readings exceeding sensor limits)
//   - Records per-sample diagnostics into FrameDiagnostics
#pragma once

#include "slam/types.h"
#include "slam/vio_types.h"
#include "util/diagnostic.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone::slam {

/// ImuPreintegrator — accumulates IMU measurements between keyframes.
///
/// Lifecycle:
///   1. Construct with noise parameters
///   2. Call add_sample() for each IMU reading (400 Hz typical)
///   3. Call integrate() to get the PreintegratedMeasurement
///   4. Call reset() to begin a new interval
///
/// Thread safety: NOT thread-safe. Call from the VIO pipeline thread only.
/// The IMU reader thread pushes samples into a lock-free queue; the VIO
/// thread drains that queue and calls add_sample() sequentially.
class ImuPreintegrator {
public:
    /// @param params  IMU noise parameters (from datasheet / calibration).
    explicit ImuPreintegrator(ImuNoiseParams params = {});

    /// Set the current gyro/accel bias estimates.
    /// Call whenever the backend updates bias estimates.
    void set_bias(const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias);

    /// Add a single IMU sample.
    /// Samples MUST be added in chronological order.
    /// @param sample  The IMU measurement.
    void add_sample(const ImuSample& sample);

    /// Integrate all accumulated samples and produce a
    /// PreintegratedMeasurement.
    ///
    /// @param diag  Diagnostics collector for warnings/errors.
    /// @return Ok(measurement) or Err(VIOError) if integration fails.
    [[nodiscard]] VIOResult<PreintegratedMeasurement> integrate(
        drone::util::FrameDiagnostics& diag) const;

    /// Reset internal state for a new inter-keyframe interval.
    void reset();

    /// Number of samples buffered since last reset.
    [[nodiscard]] int sample_count() const { return static_cast<int>(samples_.size()); }

    /// Whether any samples have been added.
    [[nodiscard]] bool has_samples() const { return !samples_.empty(); }

private:
    ImuNoiseParams         params_;
    std::vector<ImuSample> samples_;
    Eigen::Vector3d        gyro_bias_  = Eigen::Vector3d::Zero();
    Eigen::Vector3d        accel_bias_ = Eigen::Vector3d::Zero();

    // ── Sensor limits for saturation detection ──────────────
    static constexpr double kMaxGyro_rad_s  = 34.9;    // ~2000 deg/s
    static constexpr double kMaxAccel_m_s2  = 156.96;  // 16g
    static constexpr double kMaxDtGap_ratio = 2.5;     // >2.5× expected = gap
};

}  // namespace drone::slam
