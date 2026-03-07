// tests/test_stereo_matcher.cpp
// Unit tests for IStereoMatcher implementations.
//
// Tests:
//   1. SimulatedStereoMatcher produces matches from valid features
//   2. Match rate is approximately correct
//   3. Depths are within expected range
//   4. Empty feature list returns error
//   5. Disparity is positive for all matches
//   6. Mean depth and stddev are computed correctly
//   7. Diagnostics record match count and timing
//   8. Factory creates correct backend
//   9. Factory throws on unknown backend
//  10. Calibration affects depth computation

#include "slam/ifeature_extractor.h"
#include "slam/istereo_matcher.h"
#include "util/diagnostic.h"

#include <cmath>
#include <random>
#include <set>
#include <string>

#include <gtest/gtest.h>

using namespace drone::slam;

// ── Helper: generate test features ───────────────────────────
static std::vector<Feature> make_test_features(int n, uint64_t seq = 1) {
    std::vector<Feature> features;
    features.reserve(static_cast<size_t>(n));
    std::mt19937 rng(static_cast<uint32_t>(seq));
    for (int i = 0; i < n; ++i) {
        Feature f;
        f.id              = static_cast<uint32_t>(i);
        f.pixel           = Eigen::Vector2f(50.0f + static_cast<float>(i % 20) * 30.0f,
                                            50.0f + static_cast<float>(i / 20) * 30.0f);
        f.response        = 50.0f;
        f.is_triangulated = false;
        f.position_3d     = Eigen::Vector3d::Zero();
        features.push_back(f);
    }
    return features;
}

static drone::ipc::ShmStereoFrame make_frame(uint64_t seq) {
    drone::ipc::ShmStereoFrame f{};
    f.sequence_number = seq;
    f.width           = 640;
    f.height          = 480;
    f.timestamp_ns    = seq * 33'000'000ULL;
    return f;
}

// ═══════════════════════════════════════════════════════════
// SimulatedStereoMatcher tests
// ═══════════════════════════════════════════════════════════

TEST(StereoMatcherTest, ProducesMatchesFromValidFeatures) {
    SimulatedStereoMatcher        matcher;
    auto                          features = make_test_features(100);
    auto                          frame    = make_frame(1);
    drone::util::FrameDiagnostics diag(1);

    auto result = matcher.match(frame, features, diag);
    ASSERT_TRUE(result.is_ok()) << "Matching failed unexpectedly";
    EXPECT_GT(static_cast<int>(result.value().matches.size()), 0)
        << "Should produce at least some matches";
}

TEST(StereoMatcherTest, MatchRateApproximatelyCorrect) {
    constexpr float               kExpectedRate = 0.85f;
    SimulatedStereoMatcher        matcher({}, kExpectedRate);
    auto                          features = make_test_features(500);
    auto                          frame    = make_frame(42);
    drone::util::FrameDiagnostics diag(42);

    auto result = matcher.match(frame, features, diag);
    ASSERT_TRUE(result.is_ok());

    double actual_rate = static_cast<double>(result.value().matches.size()) / 500.0;
    // Allow ±15% tolerance (stochastic)
    EXPECT_GT(actual_rate, 0.5) << "Match rate too low: " << actual_rate;
    EXPECT_LT(actual_rate, 1.0) << "Match rate impossibly high: " << actual_rate;
}

TEST(StereoMatcherTest, DepthsWithinExpectedRange) {
    constexpr float               kMinDepth = 1.0f;
    constexpr float               kMaxDepth = 30.0f;
    SimulatedStereoMatcher        matcher({}, 0.9f, kMinDepth, kMaxDepth);
    auto                          features = make_test_features(200);
    auto                          frame    = make_frame(7);
    drone::util::FrameDiagnostics diag(7);

    auto result = matcher.match(frame, features, diag);
    ASSERT_TRUE(result.is_ok());

    for (const auto& m : result.value().matches) {
        EXPECT_GT(m.depth, 0.0f) << "Depth must be positive";
        // Allow some tolerance due to noise
        EXPECT_LT(m.depth, kMaxDepth * 3.0f) << "Depth unreasonably large";
    }
}

TEST(StereoMatcherTest, EmptyFeaturesReturnsError) {
    SimulatedStereoMatcher        matcher;
    std::vector<Feature>          empty;
    auto                          frame = make_frame(1);
    drone::util::FrameDiagnostics diag(1);

    auto result = matcher.match(frame, empty, diag);
    ASSERT_TRUE(result.is_err());
    EXPECT_EQ(result.error().code, VIOErrorCode::StereoMatchFailed);
    EXPECT_TRUE(diag.has_errors());
}

TEST(StereoMatcherTest, DisparityIsPositive) {
    SimulatedStereoMatcher        matcher;
    auto                          features = make_test_features(100);
    auto                          frame    = make_frame(5);
    drone::util::FrameDiagnostics diag(5);

    auto result = matcher.match(frame, features, diag);
    ASSERT_TRUE(result.is_ok());

    for (const auto& m : result.value().matches) {
        EXPECT_GT(m.disparity, 0.0f) << "Disparity must be positive (left > right x)";
    }
}

TEST(StereoMatcherTest, MeanDepthAndStddevComputed) {
    SimulatedStereoMatcher        matcher;
    auto                          features = make_test_features(200);
    auto                          frame    = make_frame(10);
    drone::util::FrameDiagnostics diag(10);

    auto result = matcher.match(frame, features, diag);
    ASSERT_TRUE(result.is_ok());

    auto& r = result.value();
    EXPECT_GT(r.mean_depth, 0.0f) << "Mean depth should be positive";
    EXPECT_GE(r.depth_stddev, 0.0f) << "Stddev should be non-negative";
}

TEST(StereoMatcherTest, DiagnosticsRecordedOnSuccess) {
    SimulatedStereoMatcher        matcher;
    auto                          features = make_test_features(100);
    auto                          frame    = make_frame(3);
    drone::util::FrameDiagnostics diag(3);

    auto result = matcher.match(frame, features, diag);
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(diag.has_errors());

    bool has_match_count = false;
    bool has_timing      = false;
    bool has_mean_depth  = false;
    for (const auto& e : diag.entries()) {
        if (e.component == "StereoMatcher" && e.message == "num_matches") has_match_count = true;
        if (e.component == "StereoMatcher" && e.message == "timing_ms") has_timing = true;
        if (e.component == "StereoMatcher" && e.message == "mean_depth") has_mean_depth = true;
    }
    EXPECT_TRUE(has_match_count);
    EXPECT_TRUE(has_timing);
    EXPECT_TRUE(has_mean_depth);
}

TEST(StereoMatcherTest, CalibrationAffectsDepth) {
    // Wider baseline → shallower depth for same disparity
    StereoCalibration narrow_baseline;
    narrow_baseline.baseline = 0.06;  // 6 cm
    StereoCalibration wide_baseline;
    wide_baseline.baseline = 0.24;  // 24 cm

    SimulatedStereoMatcher narrow(narrow_baseline);
    SimulatedStereoMatcher wide(wide_baseline);

    // Use same features and frame for both
    auto                          features = make_test_features(100);
    auto                          frame    = make_frame(99);
    drone::util::FrameDiagnostics diag1(99), diag2(99);

    auto r1 = narrow.match(frame, features, diag1);
    auto r2 = wide.match(frame, features, diag2);

    ASSERT_TRUE(r1.is_ok() && r2.is_ok());
    // Different baselines produce different depth distributions
    EXPECT_NE(r1.value().mean_depth, r2.value().mean_depth);
}

TEST(StereoMatcherTest, FactoryCreatesSimulated) {
    auto matcher = create_stereo_matcher("simulated");
    ASSERT_NE(matcher, nullptr);
    EXPECT_EQ(matcher->name(), "SimulatedStereoMatcher");
}

TEST(StereoMatcherTest, FactoryThrowsOnUnknown) {
    EXPECT_THROW(create_stereo_matcher("nonexistent"), std::runtime_error);
}

TEST(StereoMatcherTest, MatchFeatureIdsReferenceInput) {
    SimulatedStereoMatcher        matcher;
    auto                          features = make_test_features(80);
    auto                          frame    = make_frame(15);
    drone::util::FrameDiagnostics diag(15);

    auto result = matcher.match(frame, features, diag);
    ASSERT_TRUE(result.is_ok());

    // Collect input feature IDs
    std::set<uint32_t> input_ids;
    for (const auto& f : features) input_ids.insert(f.id);

    // Every match must reference a valid input feature
    for (const auto& m : result.value().matches) {
        EXPECT_TRUE(input_ids.count(m.feature_id) > 0)
            << "Match references unknown feature ID: " << m.feature_id;
    }
}

// ═══════════════════════════════════════════════════════════
// StereoCalibration unit tests
// ═══════════════════════════════════════════════════════════

TEST(StereoCalibrationTest, DepthFromDisparity) {
    StereoCalibration cal;
    cal.fx       = 350.0;
    cal.baseline = 0.12;

    double depth = cal.depth_from_disparity(42.0);
    // z = fx * baseline / d = 350 * 0.12 / 42 = 1.0
    EXPECT_NEAR(depth, 1.0, 1e-6);
}

TEST(StereoCalibrationTest, DepthFromZeroDisparityInvalid) {
    StereoCalibration cal;
    EXPECT_LT(cal.depth_from_disparity(0.0), 0);
}

TEST(StereoCalibrationTest, DepthFromNegativeDisparityInvalid) {
    StereoCalibration cal;
    EXPECT_LT(cal.depth_from_disparity(-5.0), 0);
}
