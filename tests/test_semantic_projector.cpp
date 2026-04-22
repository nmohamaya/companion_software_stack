// tests/test_semantic_projector.cpp
// Unit tests for ISemanticProjector HAL interface + CpuSemanticProjector.
#include "hal/cpu_semantic_projector.h"
#include "hal/hal_factory.h"
#include "hal/isemantic_projector.h"
#include "util/config.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::hal;

// ── Temp config helper ──

static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_sproj_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_sproj_" + std::to_string(getpid()) + ".json";
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

// ── Helper: create a uniform depth map ──

static DepthMap make_depth_map(uint32_t w, uint32_t h, float depth_val) {
    DepthMap dm;
    dm.width         = w;
    dm.height        = h;
    dm.source_width  = w;
    dm.source_height = h;
    dm.scale         = 1.0f;
    dm.confidence    = 0.9f;
    dm.data.assign(w * h, depth_val);
    return dm;
}

// ── Tests ──

TEST(SemanticProjector, CpuName) {
    CpuSemanticProjector proj;
    EXPECT_EQ(proj.name(), "CpuSemanticProjector");
}

TEST(SemanticProjector, InitValid) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    EXPECT_TRUE(proj.init(intr));
}

TEST(SemanticProjector, InitInvalidZeroDimensions) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 0, 0};
    EXPECT_FALSE(proj.init(intr));
}

TEST(SemanticProjector, InitInvalidFocalLength) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{0.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    EXPECT_FALSE(proj.init(intr));
}

TEST(SemanticProjector, ProjectBeforeInitFails) {
    CpuSemanticProjector proj;
    auto                 depth  = make_depth_map(640, 480, 5.0f);
    auto                 result = proj.project({}, depth, Eigen::Affine3f::Identity());
    EXPECT_TRUE(result.is_err());
}

TEST(SemanticProjector, ProjectEmptyDetections) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));

    auto depth  = make_depth_map(640, 480, 5.0f);
    auto result = proj.project({}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().empty());
}

TEST(SemanticProjector, ProjectSingleDetection) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));

    InferenceDetection det;
    det.bbox       = {300.0f, 220.0f, 40.0f, 40.0f};
    det.class_id   = 3;
    det.confidence = 0.85f;

    auto depth  = make_depth_map(640, 480, 10.0f);
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());

    const auto& updates = result.value();
    ASSERT_EQ(updates.size(), 1u);
    EXPECT_EQ(updates[0].semantic_label, 3);
    EXPECT_FLOAT_EQ(updates[0].confidence, 0.85f);
    EXPECT_FLOAT_EQ(updates[0].occupancy, 1.0f);

    // At identity pose, bbox centre (320, 240) with depth 10 should map near origin
    // u=320, cx=320 → x_cam = 0; v=240, cy=240 → y_cam = 0; z=10
    EXPECT_NEAR(updates[0].position_m.x(), 0.0f, 0.5f);
    EXPECT_NEAR(updates[0].position_m.y(), 0.0f, 0.5f);
    EXPECT_FLOAT_EQ(updates[0].position_m.z(), 10.0f);
}

TEST(SemanticProjector, DepthNaNSkipped) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));

    InferenceDetection det;
    det.bbox       = {300.0f, 220.0f, 40.0f, 40.0f};
    det.class_id   = 1;
    det.confidence = 0.9f;

    auto depth  = make_depth_map(640, 480, std::numeric_limits<float>::quiet_NaN());
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().empty());
}

TEST(SemanticProjector, DepthZeroSkipped) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));

    InferenceDetection det;
    det.bbox       = {300.0f, 220.0f, 40.0f, 40.0f};
    det.class_id   = 1;
    det.confidence = 0.9f;

    auto depth  = make_depth_map(640, 480, 0.0f);
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().empty());
}

TEST(SemanticProjector, WorldFrameTransform) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));

    InferenceDetection det;
    det.bbox       = {300.0f, 220.0f, 40.0f, 40.0f};
    det.class_id   = 2;
    det.confidence = 0.7f;

    auto depth = make_depth_map(640, 480, 10.0f);

    // Translate camera 5m along X
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.translation()   = Eigen::Vector3f(5.0f, 0.0f, 0.0f);

    auto result = proj.project({det}, depth, pose);
    ASSERT_TRUE(result.is_ok());
    ASSERT_EQ(result.value().size(), 1u);

    // Point at image centre with depth 10 → camera frame (0,0,10)
    // Translated by (5,0,0) → world frame (5, 0, 10)
    EXPECT_NEAR(result.value()[0].position_m.x(), 5.0f, 0.5f);
    EXPECT_NEAR(result.value()[0].position_m.z(), 10.0f, 0.5f);
}

TEST(SemanticProjector, InvalidDepthMapReturnsError) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));

    DepthMap empty_depth;
    auto     result = proj.project({}, empty_depth, Eigen::Affine3f::Identity());
    EXPECT_TRUE(result.is_err());
}

TEST(SemanticProjector, MaskedDetectionProducesMultipleUpdates) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));

    InferenceDetection det;
    det.bbox        = {100.0f, 100.0f, 80.0f, 80.0f};
    det.class_id    = 5;
    det.confidence  = 0.8f;
    det.mask_width  = 8;
    det.mask_height = 8;
    det.mask.assign(64, 255);  // all foreground

    auto depth  = make_depth_map(640, 480, 8.0f);
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());

    // 4x4 grid with all mask pixels active → 16 updates
    EXPECT_EQ(result.value().size(), 16u);
    for (const auto& vu : result.value()) {
        EXPECT_EQ(vu.semantic_label, 5);
    }
}

TEST(SemanticProjector, FactoryCpu) {
    auto          path = create_temp_config(R"({
        "perception": { "semantic_projector": { "backend": "cpu" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto proj = drone::hal::create_semantic_projector(cfg);
    ASSERT_NE(proj, nullptr);
    EXPECT_EQ(proj->name(), "CpuSemanticProjector");
}

TEST(SemanticProjector, FactoryUnknownThrows) {
    auto          path = create_temp_config(R"({
        "perception": { "semantic_projector": { "backend": "nonexistent" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_semantic_projector(cfg), std::runtime_error);
}
