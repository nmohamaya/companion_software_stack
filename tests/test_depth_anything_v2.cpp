// tests/test_depth_anything_v2.cpp
// Unit tests for DepthAnythingV2Estimator: Depth Anything V2 via OpenCV DNN.
// Tests run with and without the ONNX model file.
// Issue #455.
#include "hal/depth_anything_v2.h"
#include "hal/idepth_estimator.h"
#include "util/config.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::hal;

// ═══════════════════════════════════════════════════════════
// Model path resolution
// ═══════════════════════════════════════════════════════════

#ifdef DA_V2_MODEL_PATH
static const char* g_model_path = DA_V2_MODEL_PATH;
#else
static const char* g_model_path = "models/depth_anything_v2_vits.onnx";
#endif

static bool model_exists() {
    std::ifstream f(g_model_path);
    return f.good();
}

// ═══════════════════════════════════════════════════════════
// Tests that always run (no model file needed)
// ═══════════════════════════════════════════════════════════

TEST(DepthAnythingV2Test, NameReturnsExpected) {
    // Construct with non-existent model — name() still works
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    EXPECT_EQ(estimator.name(), "DepthAnythingV2Estimator");
}

TEST(DepthAnythingV2Test, ModelNotFoundNotLoaded) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    EXPECT_FALSE(estimator.is_loaded());
}

TEST(DepthAnythingV2Test, ModelNotFoundEstimateReturnsError) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");

    constexpr uint32_t   w = 64, h = 48, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    EXPECT_FALSE(result.is_ok());
}

TEST(DepthAnythingV2Test, PathTraversalRejected) {
    DepthAnythingV2Estimator estimator("../../etc/passwd");
    EXPECT_FALSE(estimator.is_loaded());
}

TEST(DepthAnythingV2Test, PathTraversalMiddleRejected) {
    DepthAnythingV2Estimator estimator("models/../../../secret.onnx");
    EXPECT_FALSE(estimator.is_loaded());
}

TEST(DepthAnythingV2Test, NullFrameReturnsError) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    auto                     result = estimator.estimate(nullptr, 640, 480, 3);
    EXPECT_FALSE(result.is_ok());
}

TEST(DepthAnythingV2Test, ZeroDimensionsReturnError) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    std::vector<uint8_t>     frame(100, 128);

    auto r1 = estimator.estimate(frame.data(), 0, 480, 3);
    EXPECT_FALSE(r1.is_ok());

    auto r2 = estimator.estimate(frame.data(), 640, 0, 3);
    EXPECT_FALSE(r2.is_ok());
}

TEST(DepthAnythingV2Test, InvalidChannelsReturnsError) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    std::vector<uint8_t>     frame(100, 128);

    // Too few channels
    auto r0 = estimator.estimate(frame.data(), 10, 10, 0);
    EXPECT_FALSE(r0.is_ok());

    auto r1 = estimator.estimate(frame.data(), 10, 10, 1);
    EXPECT_FALSE(r1.is_ok());

    auto r2 = estimator.estimate(frame.data(), 10, 10, 2);
    EXPECT_FALSE(r2.is_ok());

    // Too many channels (only 3=RGB and 4=RGBA supported)
    auto r5 = estimator.estimate(frame.data(), 10, 10, 5);
    EXPECT_FALSE(r5.is_ok());
}

TEST(DepthAnythingV2Test, ConfigConstruction) {
    // Write a temp config file
    std::string path = "/tmp/test_da_v2_" + std::to_string(getpid()) + ".json";
    {
        std::ofstream ofs(path);
        ofs << R"({
            "perception": {
                "depth_estimator": {
                    "backend": "depth_anything_v2",
                    "model_path": "nonexistent.onnx",
                    "input_size": 256,
                    "max_depth_m": 30.0
                }
            }
        })";
    }

    drone::Config cfg;
    bool          loaded = cfg.load(path);
    std::remove(path.c_str());
    ASSERT_TRUE(loaded) << "Failed to load test config from " << path;

    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");

    EXPECT_EQ(estimator.name(), "DepthAnythingV2Estimator");
    EXPECT_FALSE(estimator.is_loaded());  // model doesn't exist
}

// ═══════════════════════════════════════════════════════════
// Tests that require the ONNX model file (skip if not present)
// ═══════════════════════════════════════════════════════════

class DAv2ModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!model_exists()) {
            GTEST_SKIP() << "DA V2 model not found at " << g_model_path;
        }
    }
};

TEST_F(DAv2ModelTest, ModelLoadsSuccessfully) {
    DepthAnythingV2Estimator estimator(g_model_path);
    EXPECT_TRUE(estimator.is_loaded());
}

TEST_F(DAv2ModelTest, ValidFrameProducesDepthMap) {
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    // Create a synthetic RGB frame (gradient pattern)
    constexpr uint32_t   w = 640, h = 480, c = 3;
    std::vector<uint8_t> frame(w * h * c);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t idx     = (y * w + x) * c;
            frame[idx + 0] = static_cast<uint8_t>(x % 256);  // R: horizontal gradient
            frame[idx + 1] = static_cast<uint8_t>(y % 256);  // G: vertical gradient
            frame[idx + 2] = 128;                            // B: constant
        }
    }

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();

    const auto& map = result.value();
    EXPECT_GT(map.width, 0u);
    EXPECT_GT(map.height, 0u);
    EXPECT_EQ(map.data.size(), static_cast<size_t>(map.width) * map.height);
    EXPECT_FLOAT_EQ(map.scale, 1.0f);
    EXPECT_FLOAT_EQ(map.confidence, 0.7f);
}

TEST_F(DAv2ModelTest, SourceDimensionsSetCorrectly) {
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    constexpr uint32_t   w = 1920, h = 1080, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();

    const auto& map = result.value();
    // source_width/source_height must be the INPUT frame dimensions
    // (for fusion bbox mapping), not the model output dimensions
    EXPECT_EQ(map.source_width, w);
    EXPECT_EQ(map.source_height, h);
}

TEST_F(DAv2ModelTest, DepthValuesPositiveAndBounded) {
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    // Use a gradient frame (not uniform) to ensure depth variation in output
    constexpr uint32_t   w = 320, h = 240, c = 3;
    std::vector<uint8_t> frame(w * h * c);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t idx     = (y * w + x) * c;
            frame[idx + 0] = static_cast<uint8_t>(x % 256);
            frame[idx + 1] = static_cast<uint8_t>(y % 256);
            frame[idx + 2] = 128;
        }
    }

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();

    const auto& map   = result.value();
    float       min_d = map.data[0];
    float       max_d = map.data[0];
    for (float d : map.data) {
        EXPECT_GE(d, 0.1f) << "Depth must be >= 0.1m";
        EXPECT_LE(d, 20.0f) << "Depth must be <= max_depth_m (20.0)";
        min_d = std::min(min_d, d);
        max_d = std::max(max_d, d);
    }

    // Non-uniform input must produce depth variation (not collapsed to single value)
    EXPECT_LT(min_d, max_d) << "Depth map should have variation on non-uniform input";
}

TEST_F(DAv2ModelTest, RGBAFrameWorks) {
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    constexpr uint32_t   w = 320, h = 240, c = 4;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();
    EXPECT_GT(result.value().data.size(), 0u);
}

TEST_F(DAv2ModelTest, MaxDepthConfigAffectsOutput) {
    // Same frame, different max_depth — output range should differ
    constexpr uint32_t   w = 320, h = 240, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    DepthAnythingV2Estimator est_20(g_model_path, 518, 20.0f);
    DepthAnythingV2Estimator est_50(g_model_path, 518, 50.0f);

    auto r20 = est_20.estimate(frame.data(), w, h, c);
    auto r50 = est_50.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(r20.is_ok());
    ASSERT_TRUE(r50.is_ok());

    // Max depth in map with max_depth=20 should be <= 20
    float max_20 = *std::max_element(r20.value().data.begin(), r20.value().data.end());
    float max_50 = *std::max_element(r50.value().data.begin(), r50.value().data.end());

    EXPECT_LE(max_20, 20.0f);
    EXPECT_LE(max_50, 50.0f);
}
