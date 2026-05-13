// tests/test_sam_backend.cpp
// Unit tests for SimulatedSAMBackend + factory registration.
#include "hal/hal_factory.h"
#include "hal/iinference_backend.h"
#include "hal/simulated_sam_backend.h"
#include "test_helpers.h"
#include "util/config.h"

#include <gtest/gtest.h>

using namespace drone::hal;

// Shared temp-config helper centralised in tests/test_helpers.h.
static drone::test::TempFileCleanup g_cleanup;

// ── Tests ──

TEST(SAMBackend, SimulatedName) {
    SimulatedSAMBackend backend;
    EXPECT_EQ(backend.name(), "SimulatedSAMBackend");
}

TEST(SAMBackend, InitReturnsTrue) {
    SimulatedSAMBackend backend;
    EXPECT_TRUE(backend.init("sam_model.onnx", 1024));
}

TEST(SAMBackend, InferReturnsMasks) {
    SimulatedSAMBackend backend(3);
    ASSERT_TRUE(backend.init("model.onnx", 1024));

    std::vector<uint8_t> frame(640 * 480 * 3, 128);
    auto                 result = backend.infer(frame.data(), 640, 480, 3, 0);

    ASSERT_TRUE(result.is_ok());
    const auto& output = result.value();
    EXPECT_EQ(output.detections.size(), 3u);
}

TEST(SAMBackend, MaskDimensionsValid) {
    SimulatedSAMBackend backend(2);
    ASSERT_TRUE(backend.init("model.onnx", 1024));

    constexpr uint32_t   kW = 640;
    constexpr uint32_t   kH = 480;
    std::vector<uint8_t> frame(kW * kH * 3, 128);
    auto                 result = backend.infer(frame.data(), kW, kH, 3, 0);

    ASSERT_TRUE(result.is_ok());
    for (const auto& det : result.value().detections) {
        EXPECT_EQ(det.mask_width, kW);
        EXPECT_EQ(det.mask_height, kH);
        EXPECT_EQ(det.mask.size(), static_cast<size_t>(kW) * kH);
        EXPECT_GT(det.bbox.w, 0.0f);
        EXPECT_GT(det.bbox.h, 0.0f);

        // Verify mask has non-zero pixels inside bbox region
        uint32_t nonzero = 0;
        for (auto px : det.mask) {
            if (px > 0) ++nonzero;
        }
        EXPECT_GT(nonzero, 0u);
    }
}

TEST(SAMBackend, ClassIdIsNegativeOne) {
    SimulatedSAMBackend backend(3);
    ASSERT_TRUE(backend.init("model.onnx", 1024));

    std::vector<uint8_t> frame(320 * 240 * 3, 64);
    auto                 result = backend.infer(frame.data(), 320, 240, 3, 0);

    ASSERT_TRUE(result.is_ok());
    for (const auto& det : result.value().detections) {
        EXPECT_EQ(det.class_id, -1) << "SAM masks must be class-agnostic (class_id = -1)";
    }
}

TEST(SAMBackend, NullFrameReturnsError) {
    SimulatedSAMBackend backend;
    auto                result = backend.infer(nullptr, 640, 480, 3, 0);
    EXPECT_TRUE(result.is_err());
}

TEST(SAMBackend, ZeroDimensionsReturnsError) {
    SimulatedSAMBackend  backend;
    std::vector<uint8_t> frame(100, 0);
    auto                 result = backend.infer(frame.data(), 0, 0, 3, 0);
    EXPECT_TRUE(result.is_err());
}

TEST(SAMBackend, FactoryCreation) {
    auto          path = drone::test::create_temp_config(R"({
        "perception": { "inference_backend": { "backend": "sam_simulated" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto backend = drone::hal::create_inference_backend(cfg);
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->name(), "SimulatedSAMBackend");
}

TEST(SAMBackend, ConfigDrivenMaskCount) {
    auto          path = drone::test::create_temp_config(R"({
        "perception": { "inference_backend": { "backend": "sam_simulated", "num_masks": 5, "confidence": 0.85 } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto backend = drone::hal::create_inference_backend(cfg);
    ASSERT_NE(backend, nullptr);
    ASSERT_TRUE(backend->init("", 1024));

    std::vector<uint8_t> frame(640 * 480 * 3, 100);
    auto                 result = backend->infer(frame.data(), 640, 480, 3, 0);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().detections.size(), 5u);
    for (const auto& det : result.value().detections) {
        EXPECT_FLOAT_EQ(det.confidence, 0.85f);
    }
}
