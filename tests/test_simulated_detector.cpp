// tests/test_simulated_detector.cpp
// Unit tests for SimulatedDetector after extraction to its own header.
// Validates that SimulatedDetector and SimulatedDetectorConfig work correctly
// after being extracted from detector_interface.h into simulated_detector.h.
#include "perception/detector_interface.h"
#include "perception/simulated_detector.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::perception;

// ═══════════════════════════════════════════════════════════
// SimulatedDetectorConfig defaults
// ═══════════════════════════════════════════════════════════

TEST(SimulatedDetectorConfigTest, DefaultValues) {
    SimulatedDetectorConfig cfg;
    EXPECT_EQ(cfg.min_detections, 1);
    EXPECT_EQ(cfg.max_detections, 5);
    EXPECT_FLOAT_EQ(cfg.margin_px, 50.0f);
    EXPECT_FLOAT_EQ(cfg.size_min_px, 40.0f);
    EXPECT_FLOAT_EQ(cfg.size_max_px, 200.0f);
    EXPECT_FLOAT_EQ(cfg.confidence_min, 0.4f);
    EXPECT_FLOAT_EQ(cfg.confidence_max, 0.99f);
    EXPECT_EQ(cfg.num_classes, 5);
}

TEST(SimulatedDetectorConfigTest, CustomValues) {
    SimulatedDetectorConfig cfg;
    cfg.min_detections = 3;
    cfg.max_detections = 10;
    cfg.margin_px      = 20.0f;
    cfg.size_min_px    = 10.0f;
    cfg.size_max_px    = 100.0f;
    cfg.confidence_min = 0.6f;
    cfg.confidence_max = 0.95f;
    cfg.num_classes    = 3;

    EXPECT_EQ(cfg.min_detections, 3);
    EXPECT_EQ(cfg.max_detections, 10);
    EXPECT_FLOAT_EQ(cfg.margin_px, 20.0f);
    EXPECT_FLOAT_EQ(cfg.size_min_px, 10.0f);
    EXPECT_FLOAT_EQ(cfg.size_max_px, 100.0f);
    EXPECT_FLOAT_EQ(cfg.confidence_min, 0.6f);
    EXPECT_FLOAT_EQ(cfg.confidence_max, 0.95f);
    EXPECT_EQ(cfg.num_classes, 3);
}

// ═══════════════════════════════════════════════════════════
// SimulatedDetector construction
// ═══════════════════════════════════════════════════════════

TEST(SimulatedDetectorTest, DefaultConstruction) {
    SimulatedDetector det;
    EXPECT_EQ(det.name(), "SimulatedDetector");

    const auto& cfg = det.config();
    EXPECT_EQ(cfg.min_detections, 1);
    EXPECT_EQ(cfg.max_detections, 5);
}

TEST(SimulatedDetectorTest, ConfigConstruction) {
    SimulatedDetectorConfig cfg;
    cfg.min_detections = 2;
    cfg.max_detections = 8;
    cfg.confidence_min = 0.5f;
    cfg.confidence_max = 0.9f;

    SimulatedDetector det(cfg);
    EXPECT_EQ(det.name(), "SimulatedDetector");
    EXPECT_EQ(det.config().min_detections, 2);
    EXPECT_EQ(det.config().max_detections, 8);
    EXPECT_FLOAT_EQ(det.config().confidence_min, 0.5f);
    EXPECT_FLOAT_EQ(det.config().confidence_max, 0.9f);
}

// ═══════════════════════════════════════════════════════════
// SimulatedDetector::detect()
// ═══════════════════════════════════════════════════════════

TEST(SimulatedDetectorTest, DetectReturnsDetections) {
    SimulatedDetectorConfig cfg;
    cfg.min_detections = 2;
    cfg.max_detections = 2;  // force exactly 2
    SimulatedDetector det(cfg);

    constexpr uint32_t   W = 640;
    constexpr uint32_t   H = 480;
    constexpr uint32_t   C = 3;
    std::vector<uint8_t> frame(W * H * C, 128);

    auto dets = det.detect(frame.data(), W, H, C);
    EXPECT_EQ(static_cast<int>(dets.size()), 2);
}

TEST(SimulatedDetectorTest, DetectionsHaveValidFields) {
    SimulatedDetectorConfig cfg;
    cfg.min_detections = 3;
    cfg.max_detections = 3;
    cfg.confidence_min = 0.4f;
    cfg.confidence_max = 0.99f;
    cfg.margin_px      = 50.0f;
    cfg.size_min_px    = 40.0f;
    cfg.size_max_px    = 200.0f;
    SimulatedDetector det(cfg);

    constexpr uint32_t   W = 1920;
    constexpr uint32_t   H = 1080;
    constexpr uint32_t   C = 3;
    std::vector<uint8_t> frame(W * H * C, 0);

    auto dets = det.detect(frame.data(), W, H, C);
    ASSERT_EQ(static_cast<int>(dets.size()), 3);

    for (const auto& d : dets) {
        // Bounding box within frame (accounting for margin)
        EXPECT_GE(d.x, 0.0f);
        EXPECT_GE(d.y, 0.0f);
        EXPECT_GT(d.w, 0.0f);
        EXPECT_GT(d.h, 0.0f);

        // Confidence within range
        EXPECT_GE(d.confidence, cfg.confidence_min);
        EXPECT_LE(d.confidence, cfg.confidence_max);

        // Timestamp non-zero
        EXPECT_GT(d.timestamp_ns, static_cast<uint64_t>(0));

        // Class ID within range
        EXPECT_LE(static_cast<int>(d.class_id), cfg.num_classes - 1);
    }
}

TEST(SimulatedDetectorTest, DetectCountWithinRange) {
    SimulatedDetectorConfig cfg;
    cfg.min_detections = 1;
    cfg.max_detections = 10;
    SimulatedDetector det(cfg);

    constexpr uint32_t   W = 640;
    constexpr uint32_t   H = 480;
    constexpr uint32_t   C = 3;
    std::vector<uint8_t> frame(W * H * C, 0);

    // Run multiple times to exercise the random distribution
    for (int i = 0; i < 20; ++i) {
        auto dets = det.detect(frame.data(), W, H, C);
        EXPECT_GE(static_cast<int>(dets.size()), cfg.min_detections);
        EXPECT_LE(static_cast<int>(dets.size()), cfg.max_detections);
    }
}

// ═══════════════════════════════════════════════════════════
// SimulatedDetector as IDetector (polymorphism)
// ═══════════════════════════════════════════════════════════

TEST(SimulatedDetectorTest, PolymorphicUsage) {
    auto       det   = std::make_unique<SimulatedDetector>();
    IDetector* iface = det.get();

    EXPECT_EQ(iface->name(), "SimulatedDetector");

    constexpr uint32_t   W = 640;
    constexpr uint32_t   H = 480;
    constexpr uint32_t   C = 3;
    std::vector<uint8_t> frame(W * H * C, 0);

    auto dets = iface->detect(frame.data(), W, H, C);
    EXPECT_GE(static_cast<int>(dets.size()), 1);
}

// ═══════════════════════════════════════════════════════════
// Factory creates SimulatedDetector
// ═══════════════════════════════════════════════════════════

TEST(SimulatedDetectorTest, FactoryCreatesSimulated) {
    auto det = create_detector("simulated");
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "SimulatedDetector");
}

TEST(SimulatedDetectorTest, FactoryEmptyStringFallsBackToSimulated) {
    auto det = create_detector("");
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "SimulatedDetector");
}

TEST(SimulatedDetectorTest, ZeroDetections) {
    SimulatedDetectorConfig cfg;
    cfg.min_detections = 0;
    cfg.max_detections = 0;
    SimulatedDetector det(cfg);

    constexpr uint32_t   W = 640;
    constexpr uint32_t   H = 480;
    constexpr uint32_t   C = 3;
    std::vector<uint8_t> frame(W * H * C, 0);

    auto dets = det.detect(frame.data(), W, H, C);
    EXPECT_EQ(static_cast<int>(dets.size()), 0);
}
