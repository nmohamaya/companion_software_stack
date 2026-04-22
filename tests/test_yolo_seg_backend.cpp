// tests/test_yolo_seg_backend.cpp
// Unit tests for YoloSegInferenceBackend — mostly compile-time + no-model behaviour.
// Real inference requires a YOLOv8-seg .onnx model and OpenCV; these tests exercise
// the control flow without a model loaded.
#include "perception/yolo_seg_inference_backend.h"
#include "util/config.h"

#include <cstdio>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::perception;

// ── Temp config helper ──

static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_yolo_seg_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_yolo_seg_" + std::to_string(getpid()) + ".json";
        std::ofstream ofs(path);
        ofs << json_content;
        g_temp_files.push_back(path);
        return path;
    }
    ::close(fd);
    std::string   path(tmpl);
    std::ofstream ofs(path);
    ofs << json_content;
    g_temp_files.push_back(path);
    return path;
}

struct TempFileCleanup {
    ~TempFileCleanup() {
        for (auto& f : g_temp_files) std::remove(f.c_str());
    }
};
static TempFileCleanup g_cleanup;

// ── Tests ──

TEST(YoloSegBackend, Name) {
    YoloSegInferenceBackend backend("nonexistent.onnx");
    EXPECT_EQ(backend.name(), "YoloSegInferenceBackend");
}

TEST(YoloSegBackend, InitWithoutModel) {
    YoloSegInferenceBackend backend("");
    EXPECT_FALSE(backend.is_loaded());
    // init("") keeps model_loaded_ = false — returns false (no model to run)
    EXPECT_FALSE(backend.init("", 640));
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

TEST(YoloSegBackend, RejectsPathTraversal) {
    YoloSegInferenceBackend backend("../../etc/passwd");
    EXPECT_FALSE(backend.is_loaded());
}

TEST(YoloSegBackend, ConfigConstruction) {
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "model_path": "nonexistent.onnx",
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
}

TEST(YoloSegBackend, VisDroneDatasetConfig) {
    auto          path = create_temp_config(R"({
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
}

TEST(YoloSegBackend, DefaultParameters) {
    YoloSegInferenceBackend backend("", 0.25f, 0.45f, 640, DetectorDataset::COCO);
    EXPECT_FALSE(backend.is_loaded());
    EXPECT_EQ(backend.name(), "YoloSegInferenceBackend");
}

TEST(YoloSegBackend, TimestampIncrementsAcrossCalls) {
    YoloSegInferenceBackend backend("");
    std::vector<uint8_t>    frame(320 * 240 * 3, 64);

    auto r1 = backend.infer(frame.data(), 320, 240, 3, 0);
    auto r2 = backend.infer(frame.data(), 320, 240, 3, 0);

    ASSERT_TRUE(r1.is_ok());
    ASSERT_TRUE(r2.is_ok());
    EXPECT_LT(r1.value().timestamp_ns, r2.value().timestamp_ns);
}
