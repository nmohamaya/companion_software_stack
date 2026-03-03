// tests/test_opencv_yolo_detector.cpp
// Unit tests for OpenCvYoloDetector: YOLOv8-nano via OpenCV DNN.
// Tests run with and without the ONNX model file.
#include "perception/detector_interface.h"
#include "perception/opencv_yolo_detector.h"
#include "util/config.h"

#include <gtest/gtest.h>

#ifdef HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

using namespace drone::perception;

// ═══════════════════════════════════════════════════════════
// COCO mapping tests (always run, no OpenCV needed)
// ═══════════════════════════════════════════════════════════

TEST(CocoMappingTest, PersonMaps) {
    EXPECT_EQ(coco_to_object_class(0), ObjectClass::PERSON);
}

TEST(CocoMappingTest, CarMaps) {
    EXPECT_EQ(coco_to_object_class(2), ObjectClass::VEHICLE_CAR);
}

TEST(CocoMappingTest, TruckMaps) {
    EXPECT_EQ(coco_to_object_class(7), ObjectClass::VEHICLE_TRUCK);
}

TEST(CocoMappingTest, BusMapsToVehicle) {
    EXPECT_EQ(coco_to_object_class(5), ObjectClass::VEHICLE_CAR);
}

TEST(CocoMappingTest, AnimalsMaps) {
    EXPECT_EQ(coco_to_object_class(15), ObjectClass::ANIMAL);  // cat
    EXPECT_EQ(coco_to_object_class(16), ObjectClass::ANIMAL);  // dog
    EXPECT_EQ(coco_to_object_class(17), ObjectClass::ANIMAL);  // horse
}

TEST(CocoMappingTest, UnknownClassMaps) {
    EXPECT_EQ(coco_to_object_class(99), ObjectClass::UNKNOWN);
    EXPECT_EQ(coco_to_object_class(-1), ObjectClass::UNKNOWN);
    EXPECT_EQ(coco_to_object_class(56), ObjectClass::UNKNOWN);  // chair
}

TEST(CocoMappingTest, ClassNames) {
    EXPECT_STREQ(coco_class_name(0), "person");
    EXPECT_STREQ(coco_class_name(2), "car");
    EXPECT_STREQ(coco_class_name(79), "toothbrush");
    EXPECT_STREQ(coco_class_name(80), "unknown");
    EXPECT_STREQ(coco_class_name(-1), "unknown");
}

// ═══════════════════════════════════════════════════════════
// OpenCV-dependent tests
// ═══════════════════════════════════════════════════════════
#ifdef HAS_OPENCV

// ── Helpers (OpenCV-only) ──────────────────────────────────
static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    std::string path = "/tmp/test_yolo_" + std::to_string(getpid()) + "_" +
                       std::to_string(g_temp_files.size()) + ".json";
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    g_temp_files.push_back(path);
    return path;
}

struct TempCleanup {
    ~TempCleanup() {
        for (const auto& f : g_temp_files) std::remove(f.c_str());
    }
};
static TempCleanup g_cleanup;

/// Create a solid-color RGB image.
static std::vector<uint8_t> make_solid_image(uint32_t w, uint32_t h, uint8_t r, uint8_t g,
                                             uint8_t b) {
    std::vector<uint8_t> img(w * h * 3);
    for (uint32_t i = 0; i < w * h; ++i) {
        img[i * 3 + 0] = r;
        img[i * 3 + 1] = g;
        img[i * 3 + 2] = b;
    }
    return img;
}

/// Check if the YOLOv8n model file exists.
#ifdef YOLO_MODEL_PATH
static const char* g_model_path = YOLO_MODEL_PATH;
#else
static const char* g_model_path = "models/yolov8n.onnx";
#endif

static bool model_exists() {
    std::ifstream f(g_model_path);
    return f.good();
}

// ── Tests ──────────────────────────────────────────────────

TEST(OpenCvYoloDetectorTest, ConstructWithMissingModelGraceful) {
    // Should not crash — just sets model_loaded_ to false
    OpenCvYoloDetector det("nonexistent_model.onnx");
    EXPECT_FALSE(det.is_loaded());
    EXPECT_EQ(det.name(), "OpenCvYoloDetector");
}

TEST(OpenCvYoloDetectorTest, DetectWithUnloadedModelReturnsEmpty) {
    OpenCvYoloDetector det("nonexistent_model.onnx");
    auto               img    = make_solid_image(640, 480, 128, 128, 128);
    auto               result = det.detect(img.data(), 640, 480, 3);
    EXPECT_TRUE(result.empty());
}

TEST(OpenCvYoloDetectorTest, NullFrameReturnsEmpty) {
    OpenCvYoloDetector det("nonexistent_model.onnx");
    EXPECT_TRUE(det.detect(nullptr, 640, 480, 3).empty());
}

TEST(OpenCvYoloDetectorTest, ZeroDimensionsReturnsEmpty) {
    OpenCvYoloDetector   det("nonexistent_model.onnx");
    std::vector<uint8_t> data(100, 0);
    EXPECT_TRUE(det.detect(data.data(), 0, 480, 3).empty());
    EXPECT_TRUE(det.detect(data.data(), 640, 0, 3).empty());
}

TEST(OpenCvYoloDetectorTest, LessThanThreeChannelsReturnsEmpty) {
    OpenCvYoloDetector   det("nonexistent_model.onnx");
    std::vector<uint8_t> data(100, 0);
    EXPECT_TRUE(det.detect(data.data(), 10, 10, 2).empty());
    EXPECT_TRUE(det.detect(data.data(), 10, 10, 1).empty());
    EXPECT_TRUE(det.detect(data.data(), 10, 10, 0).empty());
}

TEST(OpenCvYoloDetectorTest, ConfigConstructionWithMissingModel) {
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "backend": "yolov8",
                "model_path": "nonexistent.onnx",
                "confidence_threshold": 0.3,
                "nms_threshold": 0.5,
                "input_size": 640
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    OpenCvYoloDetector det(cfg);
    EXPECT_FALSE(det.is_loaded());
}

// ── Tests that require the actual model file ────────────────

class YoloModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!model_exists()) {
            GTEST_SKIP() << "YOLOv8n model not found at " << g_model_path;
        }
    }
};

TEST_F(YoloModelTest, LoadsSuccessfully) {
    OpenCvYoloDetector det(g_model_path);
    EXPECT_TRUE(det.is_loaded());
}

TEST_F(YoloModelTest, BlackImageFewOrNoDetections) {
    OpenCvYoloDetector det(g_model_path, 0.5f, 0.45f, 640);
    auto               img    = make_solid_image(640, 480, 0, 0, 0);
    auto               result = det.detect(img.data(), 640, 480, 3);
    // Black image should produce very few or no detections
    EXPECT_LE(result.size(), 5u);
}

TEST_F(YoloModelTest, DetectionsHaveValidFields) {
    OpenCvYoloDetector det(g_model_path, 0.1f, 0.45f, 640);
    // Create a more interesting image (gradient pattern)
    std::vector<uint8_t> img(640 * 480 * 3);
    for (int y = 0; y < 480; ++y) {
        for (int x = 0; x < 640; ++x) {
            int idx      = (y * 640 + x) * 3;
            img[idx + 0] = static_cast<uint8_t>(x % 256);
            img[idx + 1] = static_cast<uint8_t>(y % 256);
            img[idx + 2] = static_cast<uint8_t>((x + y) % 256);
        }
    }
    auto result = det.detect(img.data(), 640, 480, 3);
    for (const auto& d : result) {
        EXPECT_GE(d.confidence, 0.0f);
        EXPECT_LE(d.confidence, 1.0f);
        EXPECT_GE(d.x, 0.0f);
        EXPECT_GE(d.y, 0.0f);
        EXPECT_GT(d.w, 0.0f);
        EXPECT_GT(d.h, 0.0f);
        EXPECT_GT(d.timestamp_ns, 0u);
    }
}

TEST_F(YoloModelTest, FourChannelImageWorks) {
    OpenCvYoloDetector det(g_model_path, 0.25f, 0.45f, 640);
    // RGBA image
    std::vector<uint8_t> img(640 * 480 * 4, 128);
    auto                 result = det.detect(img.data(), 640, 480, 4);
    // Just verify it runs without crashing
    SUCCEED();
}

TEST_F(YoloModelTest, ConfigConstruction) {
    std::string cfg_json = std::string(R"({
        "perception": {
            "detector": {
                "backend": "yolov8",
                "model_path": ")") +
                           g_model_path + R"(",
                "confidence_threshold": 0.5,
                "nms_threshold": 0.4,
                "input_size": 640
            }
        }
    })";
    auto          path = create_temp_config(cfg_json);
    drone::Config cfg;
    cfg.load(path);
    OpenCvYoloDetector det(cfg);
    EXPECT_TRUE(det.is_loaded());
}

TEST_F(YoloModelTest, HighConfidenceThresholdReducesDetections) {
    OpenCvYoloDetector low_thresh(g_model_path, 0.1f, 0.45f, 640);
    OpenCvYoloDetector high_thresh(g_model_path, 0.9f, 0.45f, 640);

    auto img       = make_solid_image(640, 480, 100, 150, 200);
    auto dets_low  = low_thresh.detect(img.data(), 640, 480, 3);
    auto dets_high = high_thresh.detect(img.data(), 640, 480, 3);

    // Higher threshold should produce <= detections
    EXPECT_LE(dets_high.size(), dets_low.size());
}

#endif  // HAS_OPENCV

// ═══════════════════════════════════════════════════════════
// Factory tests (always run, factory handles #ifdef internally)
// ═══════════════════════════════════════════════════════════

TEST(YoloFactoryTest, SimulatedBackendWorks) {
    auto det = create_detector("simulated");
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "SimulatedDetector");
}

TEST(YoloFactoryTest, ColorContourBackendWorks) {
    auto det = create_detector("color_contour");
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "ColorContourDetector");
}

#ifdef HAS_OPENCV
TEST(YoloFactoryTest, Yolov8BackendCreatesDetector) {
    // Factory should create the detector (model may or may not load)
    auto det = create_detector("yolov8");
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "OpenCvYoloDetector");
}

TEST(YoloFactoryTest, Yolov8BackendWithConfig) {
    auto          path = create_temp_config(R"({
        "perception": {
            "detector": {
                "model_path": "nonexistent.onnx",
                "confidence_threshold": 0.3
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    auto det = create_detector("yolov8", &cfg);
    ASSERT_NE(det, nullptr);
    EXPECT_EQ(det->name(), "OpenCvYoloDetector");
}
#endif

TEST(YoloFactoryTest, UnknownBackendThrows) {
    EXPECT_THROW(create_detector("tensorflow_lite"), std::runtime_error);
}
