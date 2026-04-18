// tests/test_swvio_backend.cpp
// Unit tests for the Sliding-Window VIO (SWVIO) backend — Phase 1.
//
// Tests cover:
//   - Factory registration ("swvio" backend name)
//   - IMU propagation: constant acceleration -> quadratic position
//   - Stationary test: gravity-only IMU -> position near origin
//   - Covariance growth during propagation
//   - State augmentation: window grows, then marginalization caps it
//   - Health transitions: INITIALIZING -> NOMINAL / DEGRADED / LOST
//   - Edge cases: empty IMU, NaN input, negative dt, timestamp overflow
//   - slam_math.h utility functions (exp_map, log_map, Jacobians)

#include "slam/ivio_backend.h"
#include "slam/slam_math.h"
#include "slam/swvio_backend.h"
#include "slam/vio_types.h"

#include <cmath>
#include <limits>
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

static std::unique_ptr<SlidingWindowVIOBackend> make_swvio(SWVIOParams params = {}) {
    StereoCalibration calib;
    ImuNoiseParams    imu_params;
    return std::make_unique<SlidingWindowVIOBackend>(calib, imu_params, params);
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
            EXPECT_GT(result.value().pose.position.x(), last_pos.x())
                << "Frame " << f << ": X position should increase under forward acceleration";
        }
        last_pos = result.value().pose.position;
    }

    // After ~0.33s at 1 m/s^2: position ~ 0.5 * 1 * 0.33^2 ~ 0.054m
    const double expected_x = 0.5 * 1.0 * 0.33 * 0.33;
    EXPECT_NEAR(last_pos.x(), expected_x, 0.005) << "Should approximate s = 0.5*a*t^2";
    EXPECT_NEAR(last_pos.y(), 0.0, 0.005) << "No lateral motion expected";
    EXPECT_NEAR(last_pos.z(), 0.0, 0.005) << "No vertical motion (gravity compensated)";
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
        EXPECT_LT(result.value().pose.position.norm(), 0.05)
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
            EXPECT_GT(trace, prev_trace)
                << "Frame " << f << ": covariance trace should grow without vision updates";
        }
        prev_trace = trace;
    }
}

// ═══════════════════════════════════════════════════════════
// State augmentation and marginalization tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, StateAugmentationGrowsWindow) {
    SWVIOParams params;
    params.max_clones = 5;

    auto backend = make_swvio(params);

    for (int f = 0; f < 5; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        auto result = backend->process_frame(frame, imu);
        ASSERT_TRUE(result.is_ok());

        EXPECT_EQ(backend->clone_count(), f + 1)
            << "Window should grow by 1 clone per frame before reaching max";
        EXPECT_EQ(backend->state_dim(), 15 + 6 * (f + 1));
    }
}

TEST(SWVIOBackendTest, MarginalizationCapsWindow) {
    SWVIOParams params;
    params.max_clones = 3;

    auto backend = make_swvio(params);

    // Process 6 frames with max_clones=3
    for (int f = 0; f < 6; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        auto result = backend->process_frame(frame, imu);
        ASSERT_TRUE(result.is_ok());
    }

    // After 6 frames with max_clones=3, window should be capped
    EXPECT_LE(backend->clone_count(), 3) << "Clone count should not exceed max_clones";
    EXPECT_EQ(backend->state_dim(), 15 + 6 * backend->clone_count());
}

TEST(SWVIOBackendTest, CovarianceDimensionsMatchState) {
    SWVIOParams params;
    params.max_clones = 4;

    auto backend = make_swvio(params);

    for (int f = 0; f < 8; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        auto result = backend->process_frame(frame, imu);
        ASSERT_TRUE(result.is_ok());

        // state_dim() should always match 15 + 6*clone_count
        const int expected_dim = 15 + 6 * backend->clone_count();
        EXPECT_EQ(backend->state_dim(), expected_dim) << "Frame " << f << ": state_dim mismatch";
    }
}

// ═══════════════════════════════════════════════════════════
// Health transition tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, HealthTransitionsToNominal) {
    SWVIOParams params;
    params.init_frames        = 3;
    params.good_trace_max     = 100.0;  // very high — should be NOMINAL after init
    params.degraded_trace_max = 1000.0;

    auto backend = make_swvio(params);
    EXPECT_EQ(backend->health(), VIOHealth::INITIALIZING);

    for (int f = 0; f < 5; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        (void)backend->process_frame(frame, imu);
    }

    EXPECT_EQ(backend->health(), VIOHealth::NOMINAL)
        << "Should be NOMINAL with high trace thresholds";
}

TEST(SWVIOBackendTest, HealthDegradedWhenTraceExceedsGoodMax) {
    SWVIOParams params;
    params.init_frames        = 1;
    params.good_trace_max     = 1e-10;  // impossibly tight — position trace will exceed this
    params.degraded_trace_max = 100.0;  // but won't exceed this

    auto backend = make_swvio(params);

    for (int f = 0; f < 3; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        (void)backend->process_frame(frame, imu);
    }

    EXPECT_EQ(backend->health(), VIOHealth::DEGRADED)
        << "Should be DEGRADED when trace > good_trace_max but < degraded_trace_max";
}

TEST(SWVIOBackendTest, HealthLostWhenTraceExceedsDegradedMax) {
    SWVIOParams params;
    params.init_frames    = 1;
    params.good_trace_max = 1e-10;
    params.degraded_trace_max =
        2e-10;  // both impossibly tight, but degraded > good (validate() contract)

    auto backend = make_swvio(params);

    for (int f = 0; f < 3; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        (void)backend->process_frame(frame, imu);
    }

    EXPECT_EQ(backend->health(), VIOHealth::LOST)
        << "Should be LOST when trace exceeds degraded_trace_max";
}

TEST(SWVIOBackendTest, HealthStaysInitializingDuringWarmup) {
    SWVIOParams params;
    params.init_frames = 10;

    auto backend = make_swvio(params);
    EXPECT_EQ(backend->health(), VIOHealth::INITIALIZING);

    // Process 5 frames (less than init_frames=10)
    for (int f = 0; f < 5; ++f) {
        auto         frame = make_frame(static_cast<uint64_t>(f + 1));
        const double t0    = f * 0.033;
        const double t1    = (f + 1) * 0.033;
        auto         imu   = make_imu_stationary(t0, t1);

        (void)backend->process_frame(frame, imu);
    }

    EXPECT_EQ(backend->health(), VIOHealth::INITIALIZING)
        << "Should stay INITIALIZING before init_frames reached";
}

// ═══════════════════════════════════════════════════════════
// Edge case tests
// ═══════════════════════════════════════════════════════════

TEST(SWVIOBackendTest, EmptyIMUSamplesHandled) {
    auto backend = create_vio_backend("swvio");
    auto frame   = make_frame(1);

    std::vector<ImuSample> empty_samples;
    auto                   result = backend->process_frame(frame, empty_samples);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().imu_samples_used, 0);
}

TEST(SWVIOBackendTest, NaNIMUSampleRejected) {
    auto backend = make_swvio();
    auto frame   = make_frame(1);

    std::vector<ImuSample> samples;
    ImuSample              s0;
    s0.timestamp = 0.0;
    s0.accel     = Eigen::Vector3d(0, 0, 9.81);
    s0.gyro      = Eigen::Vector3d::Zero();
    samples.push_back(s0);

    ImuSample s1;
    s1.timestamp = 0.01;
    s1.accel     = Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 9.81);
    s1.gyro      = Eigen::Vector3d::Zero();
    samples.push_back(s1);

    // Should not crash; NaN sample is skipped, state stays near origin
    auto result = backend->process_frame(frame, samples);
    ASSERT_TRUE(result.is_ok());
    EXPECT_LT(result.value().pose.position.norm(), 0.01)
        << "NaN sample should be skipped — position must stay near origin";
}

TEST(SWVIOBackendTest, NegativeDtIMUSamplesSkipped) {
    auto backend = make_swvio();
    auto frame   = make_frame(1);

    // Non-monotonic timestamps — backward sample should be skipped
    std::vector<ImuSample> samples;
    ImuSample              s0;
    s0.timestamp = 0.01;
    s0.accel     = Eigen::Vector3d(0, 0, 9.81);
    s0.gyro      = Eigen::Vector3d::Zero();
    samples.push_back(s0);

    ImuSample s1;
    s1.timestamp = 0.005;  // backward in time
    s1.accel     = Eigen::Vector3d(0, 0, 9.81);
    s1.gyro      = Eigen::Vector3d::Zero();
    samples.push_back(s1);

    ImuSample s2;
    s2.timestamp = 0.02;
    s2.accel     = Eigen::Vector3d(0, 0, 9.81);
    s2.gyro      = Eigen::Vector3d::Zero();
    samples.push_back(s2);

    auto result = backend->process_frame(frame, samples);
    ASSERT_TRUE(result.is_ok());
    // Position should stay near origin (gravity-compensated, only valid samples used)
    EXPECT_LT(result.value().pose.position.norm(), 0.05);
}

TEST(SWVIOBackendTest, ZeroTimestampRejected) {
    auto backend       = create_vio_backend("swvio");
    auto frame         = make_frame(1);
    frame.timestamp_ns = 0;

    auto imu    = make_imu_stationary(0.0, 0.033);
    auto result = backend->process_frame(frame, imu);
    EXPECT_TRUE(result.is_err()) << "Should reject zero timestamp";
}

TEST(SWVIOBackendTest, TimestampOverflowRejected) {
    auto backend = create_vio_backend("swvio");
    auto frame   = make_frame(1);

    // Set timestamp_ns to a value that would overflow when converted to seconds
    frame.timestamp_ns = std::numeric_limits<uint64_t>::max();

    auto imu    = make_imu_stationary(0.0, 0.033);
    auto result = backend->process_frame(frame, imu);
    EXPECT_TRUE(result.is_err()) << "Should reject implausible timestamp";
}

TEST(SWVIOBackendTest, TimestampAtBoundaryAccepted) {
    auto backend = create_vio_backend("swvio");
    auto frame   = make_frame(1);

    // 10 years in nanoseconds — the maximum accepted value
    constexpr uint64_t kMaxTimestampNs = 10ULL * 365ULL * 24ULL * 3600ULL * 1'000'000'000ULL;
    frame.timestamp_ns                 = kMaxTimestampNs;

    auto imu    = make_imu_stationary(0.0, 0.033);
    auto result = backend->process_frame(frame, imu);
    EXPECT_TRUE(result.is_ok()) << "Timestamp at 10-year boundary should be accepted";
}

TEST(SWVIOBackendTest, TimestampBeyondBoundaryRejected) {
    auto backend = create_vio_backend("swvio");
    auto frame   = make_frame(1);

    constexpr uint64_t kMaxTimestampNs = 10ULL * 365ULL * 24ULL * 3600ULL * 1'000'000'000ULL;
    frame.timestamp_ns                 = kMaxTimestampNs + 1;

    auto imu    = make_imu_stationary(0.0, 0.033);
    auto result = backend->process_frame(frame, imu);
    EXPECT_TRUE(result.is_err()) << "Timestamp beyond 10-year boundary should be rejected";
}

// ═══════════════════════════════════════════════════════════
// slam_math.h utility tests
// ═══════════════════════════════════════════════════════════

TEST(SlamMathTest, ExpMapIdentity) {
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    auto            q    = exp_map(zero);
    EXPECT_NEAR(q.w(), 1.0, 1e-10);
    EXPECT_NEAR(q.vec().norm(), 0.0, 1e-10);
}

TEST(SlamMathTest, ExpMapSmallAngle) {
    Eigen::Vector3d small_angle(1e-12, 0, 0);
    auto            q = exp_map(small_angle);
    EXPECT_NEAR(q.w(), 1.0, 1e-6);
    EXPECT_NEAR(q.vec().norm(), 0.0, 1e-6);
}

TEST(SlamMathTest, ExpMap90Degrees) {
    Eigen::Vector3d omega(0, 0, M_PI / 2.0);
    auto            q = exp_map(omega);
    Eigen::Vector3d v = q * Eigen::Vector3d(1, 0, 0);
    EXPECT_NEAR(v.x(), 0.0, 1e-10);
    EXPECT_NEAR(v.y(), 1.0, 1e-10);
    EXPECT_NEAR(v.z(), 0.0, 1e-10);
}

TEST(SlamMathTest, ExpMap180Degrees) {
    // Pi rotation about Z axis — boundary case for Rodrigues
    Eigen::Vector3d omega(0, 0, M_PI);
    auto            q = exp_map(omega);
    // Should rotate [1,0,0] to [-1,0,0]
    Eigen::Vector3d v = q * Eigen::Vector3d(1, 0, 0);
    EXPECT_NEAR(v.x(), -1.0, 1e-10);
    EXPECT_NEAR(v.y(), 0.0, 1e-10);
    EXPECT_NEAR(v.z(), 0.0, 1e-10);
}

TEST(SlamMathTest, LogMapInverseOfExpMap) {
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

    EXPECT_NEAR((m + m.transpose()).norm(), 0.0, 1e-10);

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
    Eigen::Vector3d phi(0.5, -0.3, 0.1);
    auto            J = left_jacobian_SO3(phi);

    EXPECT_GT((J - Eigen::Matrix3d::Identity()).norm(), 0.01);
    EXPECT_GT(std::abs(J.determinant()), 1e-6);
}

TEST(SlamMathTest, RightJacobianRelation) {
    // J_r(phi) = J_l(-phi)
    Eigen::Vector3d phi(0.4, -0.2, 0.6);
    auto            J_r     = right_jacobian_SO3(phi);
    auto            J_l_neg = left_jacobian_SO3(-phi);
    EXPECT_NEAR((J_r - J_l_neg).norm(), 0.0, 1e-10);
}
