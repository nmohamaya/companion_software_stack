// tests/test_detector_switcher.cpp
// Unit tests for DetectorSwitcher — altitude-based COCO/VisDrone switching.
#include "perception/detector_switcher.h"
#include "util/config.h"

#include <cstdio>
#include <fstream>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::perception;

// ── Temp config helper ──

static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_det_sw_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_det_sw_" + std::to_string(getpid()) + ".json";
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

TEST(DetectorSwitcher, ConfigConstruction) {
    auto          path = create_temp_config(R"({
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
    auto          path = create_temp_config(R"({})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    DetectorSwitcher sw(cfg);

    EXPECT_FLOAT_EQ(sw.altitude_threshold(), 30.0f);
    EXPECT_EQ(sw.model_path(DetectorDataset::COCO), "models/yolov8n-seg.onnx");
    EXPECT_EQ(sw.model_path(DetectorDataset::VISDRONE), "models/yolov8n-visdrone-seg.onnx");
}
