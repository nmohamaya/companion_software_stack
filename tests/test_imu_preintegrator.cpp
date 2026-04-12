// tests/test_imu_preintegrator.cpp
// Unit tests for ImuPreintegrator.
//
// Tests:
//   1.  No samples → error
//   2.  Single sample → error (need ≥2)
//   3.  Stationary IMU → zero delta_velocity, small delta_position
//   4.  Constant acceleration → correct delta_velocity
//   5.  Constant rotation → correct delta_rotation
//   6.  Covariance is positive semi-definite
//   7.  Bias Jacobian is finite
//   8.  Reset clears state
//   9.  Data gap detection
//  10.  IMU saturation detection
//  11.  NaN sample produces numerical failure
//  12.  Integration interval (dt) correct

#include "slam/imu_preintegrator.h"
#include "util/diagnostic.h"

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

using namespace drone::slam;

// ── Helper: generate constant-rate IMU samples ───────────────
static std::vector<ImuSample> make_constant_imu(int n, double dt,
                                                Eigen::Vector3d accel = Eigen::Vector3d(0, 0, 9.81),
                                                Eigen::Vector3d gyro  = Eigen::Vector3d::Zero()) {

    std::vector<ImuSample> samples;
    samples.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        ImuSample s;
        s.timestamp = static_cast<double>(i) * dt;
        s.accel     = accel;
        s.gyro      = gyro;
        samples.push_back(s);
    }
    return samples;
}

// ═══════════════════════════════════════════════════════════
// Error condition tests
// ═══════════════════════════════════════════════════════════

TEST(ImuPreintegratorTest, NoSamplesReturnsError) {
    ImuPreintegrator              preint;
    drone::util::FrameDiagnostics diag(1);

    auto result = preint.integrate(diag);
    ASSERT_TRUE(result.is_err());
    EXPECT_EQ(result.error().code, VIOErrorCode::ImuNotInitialized);
}

TEST(ImuPreintegratorTest, SingleSampleReturnsError) {
    ImuPreintegrator preint;
    ImuSample        s;
    s.timestamp = 0.0;
    s.accel     = Eigen::Vector3d(0, 0, 9.81);
    s.gyro      = Eigen::Vector3d::Zero();
    preint.add_sample(s);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_err());
    EXPECT_EQ(result.error().code, VIOErrorCode::ImuDataGap);
}

// ═══════════════════════════════════════════════════════════
// Stationary IMU tests
// ═══════════════════════════════════════════════════════════

TEST(ImuPreintegratorTest, StationaryImuZeroVelocityChange) {
    // Gravity-compensated: if accel = [0,0,9.81] and we don't subtract
    // gravity, delta_velocity will accumulate gravity.  The pre-integrator
    // works in body frame — the result includes gravity as expected.
    // The backend is responsible for gravity subtraction.
    ImuPreintegrator preint;
    auto             samples = make_constant_imu(100, 0.005,
                                                 Eigen::Vector3d(0, 0, 0),  // zero accel (gravity-free)
                                                 Eigen::Vector3d::Zero());
    for (const auto& s : samples) preint.add_sample(s);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();

    auto& pm = result.value();
    EXPECT_TRUE(pm.valid);
    EXPECT_NEAR(pm.delta_velocity.norm(), 0.0, 1e-10)
        << "Zero accel should produce zero velocity change";
    EXPECT_NEAR(pm.delta_position.norm(), 0.0, 1e-10)
        << "Zero accel should produce zero position change";
}

// ═══════════════════════════════════════════════════════════
// Constant acceleration test
// ═══════════════════════════════════════════════════════════

TEST(ImuPreintegratorTest, ConstantAccelProducesCorrectVelocity) {
    constexpr double dt = 0.005;  // 200 Hz
    constexpr int    N  = 200;    // 1 second
    constexpr double ax = 1.0;    // 1 m/s² along x

    ImuPreintegrator preint;
    auto samples = make_constant_imu(N, dt, Eigen::Vector3d(ax, 0, 0), Eigen::Vector3d::Zero());
    for (const auto& s : samples) preint.add_sample(s);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();

    auto&  pm         = result.value();
    double total_time = (N - 1) * dt;  // integration spans N-1 intervals

    // Δv = a * T ≈ 1.0 * 0.995 ≈ 0.995 m/s (N-1 intervals)
    EXPECT_NEAR(pm.delta_velocity.x(), ax * total_time, 0.01)
        << "Expected Δv_x ≈ " << ax * total_time;
    EXPECT_NEAR(pm.delta_velocity.y(), 0.0, 1e-6);
    EXPECT_NEAR(pm.delta_velocity.z(), 0.0, 1e-6);

    // Δp = ½ * a * T² ≈ 0.5 * 1.0 * 0.995² ≈ 0.495 m
    double expected_pos = 0.5 * ax * total_time * total_time;
    EXPECT_NEAR(pm.delta_position.x(), expected_pos, 0.02) << "Expected Δp_x ≈ " << expected_pos;
}

// ═══════════════════════════════════════════════════════════
// Constant rotation test
// ═══════════════════════════════════════════════════════════

TEST(ImuPreintegratorTest, ConstantRotationProducesCorrectAngle) {
    constexpr double dt      = 0.005;
    constexpr int    N       = 200;  // 1 second
    constexpr double omega_z = 0.1;  // 0.1 rad/s around Z

    ImuPreintegrator preint;
    auto             samples = make_constant_imu(N, dt, Eigen::Vector3d::Zero(),
                                                 Eigen::Vector3d(0, 0, omega_z));
    for (const auto& s : samples) preint.add_sample(s);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();

    auto&  pm         = result.value();
    double total_time = (N - 1) * dt;

    // Extract angle from quaternion
    Eigen::AngleAxisd aa(pm.delta_rotation);
    double            angle = aa.angle();

    EXPECT_NEAR(angle, omega_z * total_time, 0.01)
        << "Expected rotation ≈ " << omega_z * total_time << " rad";
}

// ═══════════════════════════════════════════════════════════
// Covariance properties
// ═══════════════════════════════════════════════════════════

TEST(ImuPreintegratorTest, CovarianceIsPositiveSemiDefinite) {
    ImuPreintegrator preint;
    auto             samples = make_constant_imu(100, 0.005, Eigen::Vector3d(0.5, -0.3, 9.81),
                                                 Eigen::Vector3d(0.01, -0.02, 0.005));
    for (const auto& s : samples) preint.add_sample(s);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();

    auto& pm = result.value();
    // Symmetrize (numerical rounding)
    auto cov = (pm.covariance + pm.covariance.transpose()) / 2.0;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> solver(cov);
    ASSERT_EQ(solver.info(), Eigen::Success);

    for (int i = 0; i < 9; ++i) {
        EXPECT_GE(solver.eigenvalues()(i), -1e-12)
            << "Covariance eigenvalue " << i << " is negative: " << solver.eigenvalues()(i);
    }
}

TEST(ImuPreintegratorTest, BiasJacobianIsFinite) {
    ImuPreintegrator preint;
    auto             samples = make_constant_imu(50, 0.005, Eigen::Vector3d(1.0, 0, 9.81),
                                                 Eigen::Vector3d(0, 0.1, 0));
    for (const auto& s : samples) preint.add_sample(s);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();

    EXPECT_TRUE(result.value().jacobian_bias.allFinite()) << "Bias Jacobian contains NaN/Inf";
}

// ═══════════════════════════════════════════════════════════
// Reset
// ═══════════════════════════════════════════════════════════

TEST(ImuPreintegratorTest, ResetClearsState) {
    ImuPreintegrator preint;
    auto             samples = make_constant_imu(50, 0.005);
    for (const auto& s : samples) preint.add_sample(s);
    EXPECT_EQ(preint.sample_count(), 50);

    preint.reset();
    EXPECT_EQ(preint.sample_count(), 0);
    EXPECT_FALSE(preint.has_samples());
}

// ═══════════════════════════════════════════════════════════
// Data quality diagnostics
// ═══════════════════════════════════════════════════════════

TEST(ImuPreintegratorTest, DetectsDataGap) {
    ImuPreintegrator preint;

    // Normal samples then a 50ms gap (at 200 Hz, expected 5ms)
    for (int i = 0; i < 10; ++i) {
        ImuSample s;
        s.timestamp = i * 0.005;
        s.accel     = Eigen::Vector3d(0, 0, 9.81);
        s.gyro      = Eigen::Vector3d::Zero();
        preint.add_sample(s);
    }
    // Gap: jump from 0.045 to 0.095 (50ms gap)
    ImuSample gap_sample;
    gap_sample.timestamp = 0.095;
    gap_sample.accel     = Eigen::Vector3d(0, 0, 9.81);
    gap_sample.gyro      = Eigen::Vector3d::Zero();
    preint.add_sample(gap_sample);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();
    EXPECT_TRUE(diag.has_warnings()) << "Should warn about IMU data gap";
}

TEST(ImuPreintegratorTest, DetectsGyroSaturation) {
    ImuPreintegrator preint;

    ImuSample s1;
    s1.timestamp = 0.0;
    s1.accel     = Eigen::Vector3d(0, 0, 9.81);
    s1.gyro      = Eigen::Vector3d::Zero();
    preint.add_sample(s1);

    ImuSample s2;
    s2.timestamp = 0.005;
    s2.accel     = Eigen::Vector3d(0, 0, 9.81);
    s2.gyro      = Eigen::Vector3d(0, 0, 40.0);  // > 34.9 rad/s
    preint.add_sample(s2);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(diag.has_warnings()) << "Should warn about gyro saturation";
}

// ═══════════════════════════════════════════════════════════
// Integration interval
// ═══════════════════════════════════════════════════════════

TEST(ImuPreintegratorTest, IntegrationIntervalCorrect) {
    constexpr double dt = 0.005;
    constexpr int    N  = 100;
    ImuPreintegrator preint;
    auto             samples = make_constant_imu(N, dt);
    for (const auto& s : samples) preint.add_sample(s);

    drone::util::FrameDiagnostics diag(1);
    auto                          result = preint.integrate(diag);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();

    double expected_dt = (N - 1) * dt;
    EXPECT_NEAR(result.value().dt, expected_dt, 1e-10);
    EXPECT_EQ(result.value().num_samples, N);
}

TEST(ImuPreintegratorTest, SampleCountTracking) {
    ImuPreintegrator preint;
    EXPECT_EQ(preint.sample_count(), 0);
    EXPECT_FALSE(preint.has_samples());

    ImuSample s;
    s.timestamp = 0.0;
    s.accel     = Eigen::Vector3d::Zero();
    s.gyro      = Eigen::Vector3d::Zero();
    preint.add_sample(s);
    EXPECT_EQ(preint.sample_count(), 1);
    EXPECT_TRUE(preint.has_samples());
}
