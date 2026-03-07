// tests/test_vio_backend.cpp
// Unit tests for SimulatedVIOBackend: frame processing, health state
// transitions, pose output, IMU consumption, diagnostics, factory,
// and multi-frame trajectory behaviour.

#include "slam/ivio_backend.h"
#include "slam/vio_types.h"
#include "util/diagnostic.h"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::slam;

// ── Helpers ──────────────────────────────────────────────────
static drone::ipc::ShmStereoFrame make_frame(uint64_t seq, uint32_t w = 640, uint32_t h = 480) {
    drone::ipc::ShmStereoFrame f{};
    f.sequence_number = seq;
    f.width           = w;
    f.height          = h;
    f.timestamp_ns    = seq * 33'000'000ULL;
    return f;
}

static std::vector<ImuSample> make_imu_samples(int n, double start_time = 0.0, double dt = 0.0025) {
    std::vector<ImuSample> samples;
    samples.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        ImuSample s;
        s.timestamp = start_time + static_cast<double>(i) * dt;
        s.accel     = Eigen::Vector3d(0, 0, 9.81);
        s.gyro      = Eigen::Vector3d(0.001, -0.002, 0.0005);
        samples.push_back(s);
    }
    return samples;
}

// ═══════════════════════════════════════════════════════════
// SimulatedVIOBackend tests
// ═══════════════════════════════════════════════════════════

TEST(VIOBackendTest, ProcessesFrameSuccessfully) {
    SimulatedVIOBackend backend;
    auto                frame = make_frame(1);
    auto                imu   = make_imu_samples(13);

    auto result = backend.process_frame(frame, imu);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();
}

TEST(VIOBackendTest, HealthTransitionsToNominal) {
    SimulatedVIOBackend backend({}, {}, 3);  // 3 init frames

    EXPECT_EQ(backend.health(), VIOHealth::INITIALIZING);

    // Process enough frames to exit initialization
    for (int i = 0; i < 5; ++i) {
        auto frame = make_frame(static_cast<uint64_t>(i));
        auto imu   = make_imu_samples(13, static_cast<double>(i) * 0.033);
        auto r     = backend.process_frame(frame, imu);
        ASSERT_TRUE(r.is_ok()) << r.error().to_string();
    }

    EXPECT_EQ(backend.health(), VIOHealth::NOMINAL);
}

TEST(VIOBackendTest, PoseHasValidPositionAndOrientation) {
    SimulatedVIOBackend backend;
    auto                frame = make_frame(10);
    auto                imu   = make_imu_samples(13);

    auto result = backend.process_frame(frame, imu);
    ASSERT_TRUE(result.is_ok());

    auto& pose = result.value().pose;
    EXPECT_TRUE(pose.position.allFinite()) << "Position contains NaN/Inf";
    EXPECT_NEAR(pose.orientation.norm(), 1.0, 1e-6) << "Quaternion should be unit-norm";
}

TEST(VIOBackendTest, ImuSamplesConsumed) {
    SimulatedVIOBackend backend;
    auto                frame = make_frame(1);
    auto                imu   = make_imu_samples(26);

    auto result = backend.process_frame(frame, imu);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().imu_samples_used, 26);
}

TEST(VIOBackendTest, ZeroImuStillProducesPose) {
    SimulatedVIOBackend    backend;
    auto                   frame = make_frame(1);
    std::vector<ImuSample> empty_imu;

    auto result = backend.process_frame(frame, empty_imu);
    ASSERT_TRUE(result.is_ok()) << result.error().to_string();
    EXPECT_EQ(result.value().imu_samples_used, 0);
    EXPECT_TRUE(result.value().pose.position.allFinite());
}

TEST(VIOBackendTest, FrameIdPropagated) {
    SimulatedVIOBackend backend;
    auto                frame = make_frame(42);
    auto                imu   = make_imu_samples(13);

    auto result = backend.process_frame(frame, imu);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().frame_id, 42u);
}

TEST(VIOBackendTest, FeatureCountOnSuccess) {
    SimulatedVIOBackend backend;
    auto                frame = make_frame(1);
    auto                imu   = make_imu_samples(13);

    auto result = backend.process_frame(frame, imu);
    ASSERT_TRUE(result.is_ok());
    EXPECT_GT(result.value().num_features, 0);
}

TEST(VIOBackendTest, StereoMatchCountOnSuccess) {
    SimulatedVIOBackend backend;
    auto                frame = make_frame(1);
    auto                imu   = make_imu_samples(13);

    auto result = backend.process_frame(frame, imu);
    ASSERT_TRUE(result.is_ok());
    EXPECT_GT(result.value().num_stereo_matches, 0);
}

TEST(VIOBackendTest, MultipleFramesProduceAdvancingTrajectory) {
    SimulatedVIOBackend backend;

    Eigen::Vector3d prev_pos = Eigen::Vector3d::Zero();
    bool            moved    = false;

    for (int i = 0; i < 10; ++i) {
        auto frame = make_frame(static_cast<uint64_t>(i));
        auto imu   = make_imu_samples(13, static_cast<double>(i) * 0.033);
        auto r     = backend.process_frame(frame, imu);
        ASSERT_TRUE(r.is_ok()) << r.error().to_string();

        if (i > 0 && (r.value().pose.position - prev_pos).norm() > 0.001) {
            moved = true;
        }
        prev_pos = r.value().pose.position;
    }
    EXPECT_TRUE(moved) << "Trajectory should advance over multiple frames";
}

TEST(VIOBackendTest, TimingIsRecorded) {
    SimulatedVIOBackend backend;
    auto                frame = make_frame(1);
    auto                imu   = make_imu_samples(13);

    auto result = backend.process_frame(frame, imu);
    ASSERT_TRUE(result.is_ok());
    EXPECT_GE(result.value().total_ms, 0.0);
}

TEST(VIOBackendTest, FactoryCreatesSimulated) {
    auto backend = create_vio_backend("simulated");
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->name(), "SimulatedVIOBackend");
}

TEST(VIOBackendTest, FactoryThrowsOnUnknown) {
    EXPECT_THROW(create_vio_backend("nonexistent"), std::runtime_error);
}

TEST(VIOBackendTest, InvalidFrameDimensionsCauseError) {
    SimulatedVIOBackend backend;
    auto                frame = make_frame(1, 0, 0);  // invalid dimensions
    auto                imu   = make_imu_samples(13);

    auto result = backend.process_frame(frame, imu);
    // Feature extraction should fail → backend returns error
    ASSERT_TRUE(result.is_err());
    EXPECT_EQ(result.error().code, VIOErrorCode::FeatureExtractionFailed);
}

// ═══════════════════════════════════════════════════════════
// VIOHealth helpers
// ═══════════════════════════════════════════════════════════

TEST(VIOHealthTest, NameStrings) {
    EXPECT_STREQ(vio_health_name(VIOHealth::INITIALIZING), "INITIALIZING");
    EXPECT_STREQ(vio_health_name(VIOHealth::NOMINAL), "NOMINAL");
    EXPECT_STREQ(vio_health_name(VIOHealth::DEGRADED), "DEGRADED");
    EXPECT_STREQ(vio_health_name(VIOHealth::LOST), "LOST");
}

TEST(VIOErrorTest, ToStringFormat) {
    VIOError err(VIOErrorCode::InsufficientFeatures, "FeatureExtractor", "Only 5 features detected",
                 42);
    std::string s = err.to_string();
    EXPECT_NE(s.find("FeatureExtractor"), std::string::npos);
    EXPECT_NE(s.find("InsufficientFeatures"), std::string::npos);
    EXPECT_NE(s.find("42"), std::string::npos);
}
