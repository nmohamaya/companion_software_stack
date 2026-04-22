// tests/test_detector_switcher.cpp
// Unit tests for DetectorSwitcher — altitude-based COCO/VisDrone switching.
#include "perception/detector_switcher.h"
#include "test_helpers.h"
#include "util/config.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using namespace drone::perception;
static drone::test::TempFileCleanup g_cleanup;

// ── Tests ──

TEST(DetectorSwitcher, BelowThresholdIsCoco) {
    DetectorSwitcher sw(30.0f, "coco.onnx", "visdrone.onnx");
    EXPECT_EQ(sw.select_dataset(10.0f), DetectorDataset::COCO);
    EXPECT_EQ(sw.select_dataset(29.9f), DetectorDataset::COCO);
}

TEST(DetectorSwitcher, AboveThresholdIsVisDrone) {
    DetectorSwitcher sw(30.0f, "coco.onnx", "visdrone.onnx");
    EXPECT_EQ(sw.select_dataset(30.1f), DetectorDataset::VISDRONE);
    EXPECT_EQ(sw.select_dataset(100.0f), DetectorDataset::VISDRONE);
}

TEST(DetectorSwitcher, ExactThresholdIsCoco) {
    DetectorSwitcher sw(30.0f, "coco.onnx", "visdrone.onnx");
    EXPECT_EQ(sw.select_dataset(30.0f), DetectorDataset::COCO);
}

TEST(DetectorSwitcher, ModelPathReturnsCorrectPath) {
    DetectorSwitcher sw(30.0f, "coco.onnx", "visdrone.onnx");
    EXPECT_EQ(sw.model_path(DetectorDataset::COCO), "coco.onnx");
    EXPECT_EQ(sw.model_path(DetectorDataset::VISDRONE), "visdrone.onnx");
}

TEST(DetectorSwitcher, ZeroAltitudeIsCoco) {
    DetectorSwitcher sw(30.0f, "coco.onnx", "visdrone.onnx");
    EXPECT_EQ(sw.select_dataset(0.0f), DetectorDataset::COCO);
}

TEST(DetectorSwitcher, NegativeAltitudeIsCoco) {
    DetectorSwitcher sw(30.0f, "coco.onnx", "visdrone.onnx");
    EXPECT_EQ(sw.select_dataset(-5.0f), DetectorDataset::COCO);
}

TEST(DetectorSwitcher, NaNAltitudeDefaultsToCoco) {
    DetectorSwitcher sw(30.0f, "coco.onnx", "visdrone.onnx");
    EXPECT_EQ(sw.select_dataset(std::numeric_limits<float>::quiet_NaN()), DetectorDataset::COCO);
}

TEST(DetectorSwitcher, InfinityAltitudeDefaultsToCoco) {
    DetectorSwitcher sw(30.0f, "coco.onnx", "visdrone.onnx");
    EXPECT_EQ(sw.select_dataset(std::numeric_limits<float>::infinity()), DetectorDataset::COCO);
    EXPECT_EQ(sw.select_dataset(-std::numeric_limits<float>::infinity()), DetectorDataset::COCO);
}

TEST(DetectorSwitcher, ConfigConstruction) {
    auto          path = drone::test::create_temp_config(R"({
        "perception": {
            "detector_switcher": {
                "altitude_threshold_m": 50.0,
                "coco_model_path": "custom_coco.onnx",
                "visdrone_model_path": "custom_visdrone.onnx"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    DetectorSwitcher sw(cfg);

    EXPECT_FLOAT_EQ(sw.altitude_threshold(), 50.0f);
    EXPECT_EQ(sw.select_dataset(40.0f), DetectorDataset::COCO);
    EXPECT_EQ(sw.select_dataset(60.0f), DetectorDataset::VISDRONE);
    EXPECT_EQ(sw.model_path(DetectorDataset::COCO), "custom_coco.onnx");
    EXPECT_EQ(sw.model_path(DetectorDataset::VISDRONE), "custom_visdrone.onnx");
}

TEST(DetectorSwitcher, DefaultConfigValues) {
    auto          path = drone::test::create_temp_config(R"({})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    DetectorSwitcher sw(cfg);

    EXPECT_FLOAT_EQ(sw.altitude_threshold(), 30.0f);
    EXPECT_EQ(sw.model_path(DetectorDataset::COCO), "models/yolov8n-seg.onnx");
    EXPECT_EQ(sw.model_path(DetectorDataset::VISDRONE), "models/yolov8n-visdrone-seg.onnx");
}

TEST(DetectorSwitcher, ThresholdClampedToSaneRange) {
    DetectorSwitcher sw(999.0f, "a.onnx", "b.onnx");
    EXPECT_LE(sw.altitude_threshold(), 500.0f);

    DetectorSwitcher sw2(-10.0f, "a.onnx", "b.onnx");
    EXPECT_GE(sw2.altitude_threshold(), 0.0f);
}

TEST(DetectorSwitcher, NaNThresholdDefaultsToSafe) {
    DetectorSwitcher sw(std::numeric_limits<float>::quiet_NaN(), "a.onnx", "b.onnx");
    EXPECT_FLOAT_EQ(sw.altitude_threshold(), 30.0f);
}
