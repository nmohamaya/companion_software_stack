// tests/test_model_presets.cpp
// Unit tests for CMake model presets and config overlay files (Issue #433).
#include "model_defaults.h"
#include "util/config.h"

#include <cstring>
#include <string>

#include <gtest/gtest.h>

// ── Generated header tests ──────────────────────────────────

TEST(ModelDefaultsHeader, ContainsValidPreset) {
    const std::string preset = drone::model_defaults::PRESET;
    EXPECT_TRUE(preset == "edge" || preset == "orin" || preset == "cloud")
        << "PRESET was '" << preset << "', expected one of: edge, orin, cloud";
}

TEST(ModelDefaultsHeader, YoloModelHasOnnxExtension) {
    const std::string model = drone::model_defaults::YOLO_MODEL;
    ASSERT_GE(model.size(), 5U);
    EXPECT_EQ(model.substr(model.size() - 5), ".onnx")
        << "YOLO_MODEL '" << model << "' does not end with .onnx";
}

TEST(ModelDefaultsHeader, DepthModelHasOnnxExtension) {
    const std::string model = drone::model_defaults::DEPTH_MODEL;
    ASSERT_GE(model.size(), 5U);
    EXPECT_EQ(model.substr(model.size() - 5), ".onnx")
        << "DEPTH_MODEL '" << model << "' does not end with .onnx";
}

TEST(ModelDefaultsHeader, YoloModelMatchesPreset) {
    const std::string preset = drone::model_defaults::PRESET;
    const std::string model  = drone::model_defaults::YOLO_MODEL;
    if (preset == "edge") {
        EXPECT_EQ(model, "yolov8n.onnx");
    } else if (preset == "orin") {
        EXPECT_EQ(model, "yolov8s.onnx");
    } else if (preset == "cloud") {
        EXPECT_EQ(model, "yolov8m.onnx");
    }
}

TEST(ModelDefaultsHeader, DepthModelMatchesPreset) {
    const std::string preset = drone::model_defaults::PRESET;
    const std::string model  = drone::model_defaults::DEPTH_MODEL;
    if (preset == "cloud") {
        EXPECT_EQ(model, "depth_anything_v2_vitb.onnx");
    } else {
        // edge and orin both use ViT-S
        EXPECT_EQ(model, "depth_anything_v2_vits.onnx");
    }
}

// ── Config overlay loading tests ────────────────────────────

#ifndef PROJECT_CONFIG_DIR
#error "PROJECT_CONFIG_DIR must be defined — set it in tests/CMakeLists.txt"
#endif

class ConfigOverlayTest : public ::testing::Test {
protected:
    drone::Config cfg_;

    bool load_overlay(const std::string& filename) {
        const std::string path = std::string(PROJECT_CONFIG_DIR) + "/" + filename;
        return cfg_.load(path);
    }
};

TEST_F(ConfigOverlayTest, CosysAirsimLoads) {
    ASSERT_TRUE(load_overlay("cosys_airsim.json"));
    EXPECT_EQ(cfg_.get<std::string>("perception.detector.model_path", ""), "models/yolov8m.onnx");
    EXPECT_EQ(cfg_.get<std::string>("perception.detector.backend", ""), "yolov8");
    EXPECT_EQ(cfg_.get<int>("perception.detector.num_classes", 0), 80);
    EXPECT_EQ(cfg_.get<std::string>("perception.depth_estimator.model_path", ""),
              "models/depth_anything_v2_vitb.onnx");
    EXPECT_TRUE(cfg_.get<bool>("perception.depth_estimator.enabled", false));
}

TEST_F(ConfigOverlayTest, HardwareOrinLoads) {
    ASSERT_TRUE(load_overlay("hardware_orin.json"));
    EXPECT_EQ(cfg_.get<std::string>("perception.detector.model_path", ""), "models/yolov8s.onnx");
    EXPECT_EQ(cfg_.get<std::string>("perception.detector.backend", ""), "yolov8");
    EXPECT_DOUBLE_EQ(cfg_.get<double>("perception.detector.confidence_threshold", 0.0), 0.35);
    EXPECT_EQ(cfg_.get<std::string>("perception.depth_estimator.model_path", ""),
              "models/depth_anything_v2_vits.onnx");
}

TEST_F(ConfigOverlayTest, HardwareEdgeLoads) {
    ASSERT_TRUE(load_overlay("hardware_edge.json"));
    EXPECT_EQ(cfg_.get<std::string>("perception.detector.model_path", ""), "models/yolov8n.onnx");
    EXPECT_EQ(cfg_.get<std::string>("perception.detector.backend", ""), "yolov8");
    EXPECT_DOUBLE_EQ(cfg_.get<double>("perception.detector.confidence_threshold", 0.0), 0.25);
    EXPECT_EQ(cfg_.get<std::string>("perception.depth_estimator.model_path", ""),
              "models/depth_anything_v2_vits.onnx");
}
