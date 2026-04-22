// tests/test_yolo_seg_backend.cpp
// Unit tests for YoloSegInferenceBackend — mostly compile-time + no-model behaviour.
// Real inference requires a YOLOv8-seg .onnx model and OpenCV; these tests exercise
// the control flow without a model loaded.
#include "perception/yolo_seg_inference_backend.h"
#include "test_helpers.h"
#include "util/config.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::perception;
static drone::test::TempFileCleanup g_cleanup;

// ── Tests ──

TEST(YoloSegBackend, Name) {
    YoloSegInferenceBackend backend("models/nonexistent.onnx");
    EXPECT_EQ(backend.name(), "YoloSegInferenceBackend");
}

TEST(YoloSegBackend, InitWithoutModel) {
    YoloSegInferenceBackend backend("");
    EXPECT_FALSE(backend.is_loaded());
    EXPECT_FALSE(backend.init("", 640));
}

TEST(YoloSegBackend, InitWithInvalidPathReturnsFalse) {
    YoloSegInferenceBackend backend("");
    EXPECT_FALSE(backend.init("/tmp/no_such_model_xyz.onnx", 640));
    EXPECT_FALSE(backend.is_loaded());
}

TEST(YoloSegBackend, InferWithoutModelReturnsEmpty) {
    YoloSegInferenceBackend backend("");
    std::vector<uint8_t>    frame(640 * 480 * 3, 128);
    auto                    result = backend.infer(frame.data(), 640, 480, 3, 0);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().detections.empty());
}

TEST(YoloSegBackend, NullFrameReturnsError) {
    YoloSegInferenceBackend backend("");
    auto                    result = backend.infer(nullptr, 640, 480, 3, 0);
    EXPECT_TRUE(result.is_err());
}

TEST(YoloSegBackend, ZeroDimensionsReturnsError) {
    YoloSegInferenceBackend backend("");
    std::vector<uint8_t>    frame(100, 0);
    auto                    result = backend.infer(frame.data(), 0, 0, 3, 0);
    EXPECT_TRUE(result.is_err());
}

TEST(YoloSegBackend, ZeroChannelsReturnsError) {
    YoloSegInferenceBackend backend("");
    std::vector<uint8_t>    frame(640 * 480, 0);
    auto                    result = backend.infer(frame.data(), 640, 480, 0, 0);
    EXPECT_TRUE(result.is_err());
}

TEST(YoloSegBackend, RejectsPathTraversal) {
    YoloSegInferenceBackend backend("../../etc/passwd");
    EXPECT_FALSE(backend.is_loaded());
}

TEST(YoloSegBackend, RejectsAbsolutePath) {
    YoloSegInferenceBackend backend("/etc/passwd");
    EXPECT_FALSE(backend.is_loaded());
}

TEST(YoloSegBackend, ConfigConstruction) {
    auto          path = drone::test::create_temp_config(R"({
        "perception": {
            "detector": {
                "model_path": "models/nonexistent.onnx",
                "confidence_threshold": 0.3,
                "nms_threshold": 0.5,
                "input_size": 416,
                "dataset": "coco",
                "num_classes": 80
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    YoloSegInferenceBackend backend(cfg, "perception.detector");
    EXPECT_EQ(backend.name(), "YoloSegInferenceBackend");
    EXPECT_FLOAT_EQ(backend.confidence_threshold(), 0.3f);
    EXPECT_FLOAT_EQ(backend.nms_threshold(), 0.5f);
    EXPECT_EQ(backend.input_size(), 416);
    EXPECT_EQ(backend.num_classes(), 80);
    EXPECT_EQ(backend.dataset(), DetectorDataset::COCO);
}

TEST(YoloSegBackend, VisDroneDatasetConfig) {
    auto          path = drone::test::create_temp_config(R"({
        "test": {
            "model_path": "",
            "dataset": "visdrone",
            "num_classes": 10,
            "input_size": 640
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    YoloSegInferenceBackend backend(cfg, "test");
    EXPECT_EQ(backend.name(), "YoloSegInferenceBackend");
    EXPECT_EQ(backend.dataset(), DetectorDataset::VISDRONE);
    EXPECT_EQ(backend.num_classes(), 10);
}

TEST(YoloSegBackend, DefaultParameters) {
    YoloSegInferenceBackend backend("", 0.25f, 0.45f, 640, DetectorDataset::COCO);
    EXPECT_FALSE(backend.is_loaded());
    EXPECT_EQ(backend.name(), "YoloSegInferenceBackend");
    EXPECT_EQ(backend.num_classes(), 80);
}

TEST(YoloSegBackend, VisDroneExplicitConstructorSetsNumClasses) {
    YoloSegInferenceBackend backend("", 0.25f, 0.45f, 640, DetectorDataset::VISDRONE);
    EXPECT_EQ(backend.num_classes(), 10);
    EXPECT_EQ(backend.dataset(), DetectorDataset::VISDRONE);
}

TEST(YoloSegBackend, TimestampIsRealClock) {
    YoloSegInferenceBackend backend("");
    std::vector<uint8_t>    frame(320 * 240 * 3, 64);

    auto r1 = backend.infer(frame.data(), 320, 240, 3, 0);
    auto r2 = backend.infer(frame.data(), 320, 240, 3, 0);

    ASSERT_TRUE(r1.is_ok());
    ASSERT_TRUE(r2.is_ok());
    EXPECT_LT(r1.value().timestamp_ns, r2.value().timestamp_ns);
    // Verify it's a real timestamp (> year 2020 in nanoseconds)
    EXPECT_GT(r1.value().timestamp_ns, uint64_t{1'000'000'000});
}

TEST(YoloSegBackend, InputSizeClampedToSaneRange) {
    YoloSegInferenceBackend backend("", 0.25f, 0.45f, 0, DetectorDataset::COCO);
    EXPECT_GE(backend.input_size(), 32);

    YoloSegInferenceBackend backend2("", 0.25f, 0.45f, 99999, DetectorDataset::COCO);
    EXPECT_LE(backend2.input_size(), 1920);
}
