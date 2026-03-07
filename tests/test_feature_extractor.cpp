// tests/test_feature_extractor.cpp
// Unit tests for SimulatedFeatureExtractor: feature count, pixel
// coordinate bounds, uniqueness, determinism, error handling,
// factory correctness, and diagnostics recording.

#include "slam/ifeature_extractor.h"
#include "util/diagnostic.h"

#include <algorithm>
#include <set>
#include <string>

#include <gtest/gtest.h>

using namespace drone::slam;

// ── Helper: create a valid ShmStereoFrame ────────────────────
static drone::ipc::ShmStereoFrame make_frame(uint64_t seq, uint32_t w = 640, uint32_t h = 480) {
    drone::ipc::ShmStereoFrame f{};
    f.sequence_number = seq;
    f.width           = w;
    f.height          = h;
    f.timestamp_ns    = seq * 33'000'000ULL;  // ~30Hz
    return f;
}

// ═══════════════════════════════════════════════════════════
// SimulatedFeatureExtractor tests
// ═══════════════════════════════════════════════════════════

TEST(FeatureExtractorTest, ProducesExpectedFeatureCount) {
    constexpr int                 kExpected = 100;
    SimulatedFeatureExtractor     extractor(kExpected);
    auto                          frame = make_frame(1);
    drone::util::FrameDiagnostics diag(1);

    auto result = extractor.extract(frame, diag);
    ASSERT_TRUE(result.is_ok()) << "Extraction failed unexpectedly";
    EXPECT_EQ(static_cast<int>(result.value().features.size()), kExpected);
}

TEST(FeatureExtractorTest, FeaturesWithinImageBounds) {
    constexpr int                 kW = 640, kH = 480;
    SimulatedFeatureExtractor     extractor(200, kW, kH);
    auto                          frame = make_frame(42);
    drone::util::FrameDiagnostics diag(42);

    auto result = extractor.extract(frame, diag);
    ASSERT_TRUE(result.is_ok());

    for (const auto& f : result.value().features) {
        EXPECT_GE(f.pixel.x(), 0.0f);
        EXPECT_LT(f.pixel.x(), static_cast<float>(kW));
        EXPECT_GE(f.pixel.y(), 0.0f);
        EXPECT_LT(f.pixel.y(), static_cast<float>(kH));
    }
}

TEST(FeatureExtractorTest, UniqueFeatureIDs) {
    SimulatedFeatureExtractor     extractor(150);
    auto                          frame = make_frame(7);
    drone::util::FrameDiagnostics diag(7);

    auto result = extractor.extract(frame, diag);
    ASSERT_TRUE(result.is_ok());

    std::set<uint32_t> ids;
    for (const auto& f : result.value().features) {
        ids.insert(f.id);
    }
    EXPECT_EQ(ids.size(), result.value().features.size()) << "Duplicate feature IDs detected";
}

TEST(FeatureExtractorTest, DeterministicForSameSequence) {
    SimulatedFeatureExtractor extractor(80);
    auto                      frame = make_frame(99);

    drone::util::FrameDiagnostics diag1(99);
    auto                          r1 = extractor.extract(frame, diag1);

    drone::util::FrameDiagnostics diag2(99);
    auto                          r2 = extractor.extract(frame, diag2);

    ASSERT_TRUE(r1.is_ok() && r2.is_ok());
    ASSERT_EQ(r1.value().features.size(), r2.value().features.size());

    for (size_t i = 0; i < r1.value().features.size(); ++i) {
        EXPECT_FLOAT_EQ(r1.value().features[i].pixel.x(), r2.value().features[i].pixel.x());
        EXPECT_FLOAT_EQ(r1.value().features[i].pixel.y(), r2.value().features[i].pixel.y());
    }
}

TEST(FeatureExtractorTest, DifferentFeaturesForDifferentFrames) {
    SimulatedFeatureExtractor extractor(50);

    auto                          frame1 = make_frame(1);
    auto                          frame2 = make_frame(2);
    drone::util::FrameDiagnostics diag1(1), diag2(2);

    auto r1 = extractor.extract(frame1, diag1);
    auto r2 = extractor.extract(frame2, diag2);

    ASSERT_TRUE(r1.is_ok() && r2.is_ok());
    // At least some features should differ
    bool any_different = false;
    for (size_t i = 0; i < r1.value().features.size(); ++i) {
        if (r1.value().features[i].pixel.x() != r2.value().features[i].pixel.x()) {
            any_different = true;
            break;
        }
    }
    EXPECT_TRUE(any_different) << "Features should differ between frames";
}

TEST(FeatureExtractorTest, ZeroDimensionFrameReturnsError) {
    SimulatedFeatureExtractor     extractor(100);
    auto                          frame = make_frame(5, 0, 0);  // invalid dimensions
    drone::util::FrameDiagnostics diag(5);

    auto result = extractor.extract(frame, diag);
    ASSERT_TRUE(result.is_err());
    EXPECT_EQ(result.error().code, VIOErrorCode::FeatureExtractionFailed);
    EXPECT_TRUE(diag.has_errors());
}

TEST(FeatureExtractorTest, FeaturesNotTriangulatedInitially) {
    SimulatedFeatureExtractor     extractor(50);
    auto                          frame = make_frame(10);
    drone::util::FrameDiagnostics diag(10);

    auto result = extractor.extract(frame, diag);
    ASSERT_TRUE(result.is_ok());

    for (const auto& f : result.value().features) {
        EXPECT_FALSE(f.is_triangulated);
    }
}

TEST(FeatureExtractorTest, DiagnosticsRecordedOnSuccess) {
    SimulatedFeatureExtractor     extractor(75);
    auto                          frame = make_frame(3);
    drone::util::FrameDiagnostics diag(3);

    auto result = extractor.extract(frame, diag);
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(diag.has_errors());

    // Should have at least the "num_features" metric + timing
    bool has_feature_count = false;
    bool has_timing        = false;
    for (const auto& e : diag.entries()) {
        if (e.component == "FeatureExtractor" && e.message == "num_features")
            has_feature_count = true;
        if (e.component == "FeatureExtractor" && e.message == "timing_ms") has_timing = true;
    }
    EXPECT_TRUE(has_feature_count) << "Diagnostics should record feature count";
    EXPECT_TRUE(has_timing) << "Diagnostics should record timing";
}

TEST(FeatureExtractorTest, FactoryCreatesSimulated) {
    auto extractor = create_feature_extractor("simulated");
    ASSERT_NE(extractor, nullptr);
    EXPECT_EQ(extractor->name(), "SimulatedFeatureExtractor");
}

TEST(FeatureExtractorTest, FactoryThrowsOnUnknown) {
    EXPECT_THROW(create_feature_extractor("nonexistent"), std::runtime_error);
}

TEST(FeatureExtractorTest, FrameIdPropagated) {
    SimulatedFeatureExtractor     extractor(50);
    auto                          frame = make_frame(42);
    drone::util::FrameDiagnostics diag(42);

    auto result = extractor.extract(frame, diag);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().frame_id, 42u);
}
