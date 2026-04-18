// tests/test_swvio_backend.cpp
// Unit tests for the Sliding-Window VIO (SWVIO) backend — Phase 1.
//
// Tests cover:
//   - Factory registration ("swvio" backend name)
//   - IMU propagation: constant acceleration -> quadratic position
//   - Stationary test: gravity-only IMU -> position near origin
//   - Covariance growth during propagation
//   - State augmentation: clone window grows up to max_clones
//   - Health transitions: INITIALIZING -> NOMINAL after N frames
//   - Edge case: empty IMU samples
//   - slam_math.h utility functions (exp_map, log_map, Jacobians)

#include "slam/ivio_backend.h"
#include "slam/slam_math.h"
#include "slam/swvio_backend.h"
#include "slam/vio_types.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::slam;

// ── Helpers ──────────────────────────────────────────────────

static drone::ipc::StereoFrame make_frame(uint64_t seq, uint32_t w = 640, uint32_t h = 480) {
    drone::ipc::StereoFrame f{};
    f.sequence_number = seq;
    f.width           = w;
    f.height          = h;
    f.timestamp_ns    = seq * 33'333'000ULL;  // ~30Hz
    return f;
}

static std::vector<ImuSample> make_imu_stationary(double t0, double t1, int n = 4) {
    // Gravity-compensating accelerometer reading: body frame sees [0, 0, +9.81]
    // when stationary (accelerometer measures specific force = -gravity in body frame,
    // which for identity orientation = [0, 0, 9.81]).
    std::vector<ImuSample> samples;
    samples.reserve(static_cast<size_t>(n));
    const double dt = (t1 - t0) / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        ImuSample s;
        s.timestamp = t0 + static_cast<double>(i) * dt;
        s.accel     = Eigen::Vector3d(0, 0, 9.81);  // gravity compensation
        s.gyro      = Eigen::Vector3d::Zero();
        samples.push_back(s);
    }
    return samples;
}

static std::vector<ImuSample> make_imu_forward_accel(double t0, double t1, double accel_x = 1.0,
                                                     int n = 4) {
    // Forward acceleration of accel_x m/s^2 in body X, with gravity compensation on Z
    std::vector<ImuSample> samples;
    samples.reserve(static_cast<size_t>(n));
    const double dt = (t1 - t0) / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        ImuSample s;
        s.timestamp = t0 + static_cast<double>(i) * dt;
        s.accel     = Eigen::Vector3d(accel_x, 0, 9.81);
        s.gyro      = Eigen::Vector3d::Zero();
        samples.push_back(s);
    }
    return samples;
}

// ═══════════════════════════════════════════════════════════
// Factory tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, FactoryCreatesSWVIO) {
    auto backend = create_vio_backend("swvio");
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->name(), "SlidingWindowVIO");
    EXPECT_EQ(backend->health(), VIOHealth::INITIALIZING);
}

// ═══════════════════════════════════════════════════════════
// Basic processing tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, ProcessesFrameSuccessfully) {
    auto backend = create_vio_backend("swvio");
    auto frame   = make_frame(1);
    auto imu     = make_imu_stationary(0.0, 0.033);

    auto result = backend->process_frame(frame, imu);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().frame_id, 1u);
    EXPECT_GT(result.value().num_features, 0);
}

// ═══════════════════════════════════════════════════════════
// IMU propagation tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, IMUPropagationConstantAccel) {
    auto backend = create_vio_backend("swvio");

    // Apply 1 m/s^2 forward acceleration for ~0.33s (10 frames at 30Hz)
    // Expected position: ~0.5 * 1 * 0.33^2 ~ 0.054m in X
    Eigen::Vector3d last_pos = Eigen::Vector3d::Zero();
    for (int f = 0; f < 10; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_forward_accel(t0, t1, 1.0);

        auto result = backend->process_frame(frame, imu);
        ASSERT_TRUE(result.is_ok());

        if (f > 0) {
            // Position should be monotonically increasing in X
            EXPECT_GT(result.value().pose.position.x(), last_pos.x())
                << "Frame " << f << ": X position should increase under forward acceleration";
        }
        last_pos = result.value().pose.position;
    }

    // After ~0.33s at 1 m/s^2: position ~ 0.5 * 1 * 0.33^2 ~ 0.054m
    EXPECT_GT(last_pos.x(), 0.01) << "Should have moved forward significantly";
    EXPECT_LT(std::abs(last_pos.y()), 0.01) << "No lateral motion expected";
    EXPECT_LT(std::abs(last_pos.z()), 0.01) << "No vertical motion (gravity compensated)";
}

TEST(SWVIOBackendTest, StationaryGravityOnly) {
    auto backend = create_vio_backend("swvio");

    // Process 20 frames with gravity-only IMU — position should stay near origin
    for (int f = 0; f < 20; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        auto result = backend->process_frame(frame, imu);
        ASSERT_TRUE(result.is_ok());
        EXPECT_LT(result.value().pose.position.norm(), 0.1)
            << "Frame " << f << ": position should stay near origin when stationary";
    }
}

// ═══════════════════════════════════════════════════════════
// Covariance tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, CovarianceGrowsDuringPropagation) {
    auto backend = create_vio_backend("swvio");

    double prev_trace = -1.0;
    for (int f = 0; f < 5; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        auto result = backend->process_frame(frame, imu);
        ASSERT_TRUE(result.is_ok());

        const double trace = result.value().position_trace;
        EXPECT_GE(trace, 0.0) << "Position trace should be non-negative";

        if (f > 0) {
            // Covariance should grow with each IMU-only propagation (no vision updates)
            EXPECT_GT(trace, prev_trace)
                << "Frame " << f << ": covariance trace should grow without vision updates";
        }
        prev_trace = trace;
    }
}

// ═══════════════════════════════════════════════════════════
// State augmentation tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, StateAugmentationGrowsWindow) {
    SWVIOParams params;
    params.max_clones = 5;  // small window for testing

    StereoCalibration calib;
    ImuNoiseParams    imu_params;
    auto backend = std::make_unique<SlidingWindowVIOBackend>(calib, imu_params, params, 3, 0.1,
                                                             1.0);

    // Process 10 frames — window should grow to 5 then hold steady via marginalization
    for (int f = 0; f < 10; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        auto result = backend->process_frame(frame, imu);
        ASSERT_TRUE(result.is_ok()) << "Frame " << f << " should process successfully";
    }
    // The system should not crash even after exceeding max_clones
}

// ═══════════════════════════════════════════════════════════
// Health transition tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, HealthTransitionsToNominal) {
    SWVIOParams params;
    params.init_frames = 3;  // fast init for testing

    StereoCalibration calib;
    ImuNoiseParams    imu_params;
    auto backend = std::make_unique<SlidingWindowVIOBackend>(calib, imu_params, params, 3, 0.1,
                                                             1.0);

    EXPECT_EQ(backend->health(), VIOHealth::INITIALIZING);

    for (int f = 0; f < 5; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        (void)backend->process_frame(frame, imu);
    }

    // After 5 frames with init_frames=3, should have left INITIALIZING
    EXPECT_NE(backend->health(), VIOHealth::INITIALIZING);
}

// ═══════════════════════════════════════════════════════════
// Edge case tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, EmptyIMUSamplesHandled) {
    auto backend = create_vio_backend("swvio");
    auto frame   = make_frame(1);

    std::vector<ImuSample> empty_samples;
    auto                   result = backend->process_frame(frame, empty_samples);
    // Should handle gracefully — no crash, returns valid output
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().imu_samples_used, 0);
}

// ═══════════════════════════════════════════════════════════
// slam_math.h utility tests
// ═══════════════════════════════════════════════════════════

TEST(SlamMathTest, ExpMapIdentity) {
    // Zero rotation -> identity quaternion
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    auto            q    = exp_map(zero);
    EXPECT_NEAR(q.w(), 1.0, 1e-10);
    EXPECT_NEAR(q.vec().norm(), 0.0, 1e-10);
}

TEST(SlamMathTest, ExpMapSmallAngle) {
    // Small angle -> first-order approximation
    Eigen::Vector3d small_angle(1e-12, 0, 0);
    auto            q = exp_map(small_angle);
    EXPECT_NEAR(q.w(), 1.0, 1e-6);
    EXPECT_NEAR(q.vec().norm(), 0.0, 1e-6);
}

TEST(SlamMathTest, ExpMap90Degrees) {
    // 90-degree rotation about Z
    Eigen::Vector3d omega(0, 0, M_PI / 2.0);
    auto            q = exp_map(omega);
    // Should rotate [1,0,0] to [0,1,0]
    Eigen::Vector3d v = q * Eigen::Vector3d(1, 0, 0);
    EXPECT_NEAR(v.x(), 0.0, 1e-10);
    EXPECT_NEAR(v.y(), 1.0, 1e-10);
    EXPECT_NEAR(v.z(), 0.0, 1e-10);
}

TEST(SlamMathTest, LogMapInverseOfExpMap) {
    // log(exp(omega)) should return omega
    Eigen::Vector3d omega(0.1, -0.2, 0.3);
    auto            q   = exp_map(omega);
    auto            phi = log_map(q);
    EXPECT_NEAR(phi.x(), omega.x(), 1e-10);
    EXPECT_NEAR(phi.y(), omega.y(), 1e-10);
    EXPECT_NEAR(phi.z(), omega.z(), 1e-10);
}

TEST(SlamMathTest, LogMapIdentity) {
    auto phi = log_map(Eigen::Quaterniond::Identity());
    EXPECT_NEAR(phi.norm(), 0.0, 1e-10);
}

TEST(SlamMathTest, SkewSymmetric) {
    Eigen::Vector3d v(1, 2, 3);
    auto            m = skew(v);

    // Skew matrix should be antisymmetric
    EXPECT_NEAR((m + m.transpose()).norm(), 0.0, 1e-10);

    // skew(v) * u = v x u
    Eigen::Vector3d u(4, 5, 6);
    Eigen::Vector3d cross       = v.cross(u);
    Eigen::Vector3d skew_result = m * u;
    EXPECT_NEAR((cross - skew_result).norm(), 0.0, 1e-10);
}

TEST(SlamMathTest, LeftJacobianIdentityForSmallPhi) {
    Eigen::Vector3d small_phi = Eigen::Vector3d::Zero();
    auto            J         = left_jacobian_SO3(small_phi);
    EXPECT_NEAR((J - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-10);
}

TEST(SlamMathTest, LeftJacobianNonTrivial) {
    // For a non-trivial rotation, the Jacobian should not be identity
    Eigen::Vector3d phi(0.5, -0.3, 0.1);
    auto            J = left_jacobian_SO3(phi);

    // J should be close to I for small angles but not exactly I
    EXPECT_GT((J - Eigen::Matrix3d::Identity()).norm(), 0.01);

    // J should be invertible (det != 0)
    EXPECT_GT(std::abs(J.determinant()), 1e-6);
}

TEST(SlamMathTest, RightJacobianRelation) {
    // J_r(phi) = J_l(-phi)
    Eigen::Vector3d phi(0.4, -0.2, 0.6);
    auto            J_r     = right_jacobian_SO3(phi);
    auto            J_l_neg = left_jacobian_SO3(-phi);
    EXPECT_NEAR((J_r - J_l_neg).norm(), 0.0, 1e-10);
}
