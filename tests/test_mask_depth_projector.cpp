// tests/test_mask_depth_projector.cpp
// Unit tests for MaskDepthProjector — PATH A orchestrator.
// Uses CpuSemanticProjector directly (header-only, deterministic).
#include "hal/cpu_semantic_projector.h"
#include "perception/mask_depth_projector.h"
#include "perception/types.h"

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

using namespace drone::perception;
using namespace drone::hal;

// ── Helpers ──

static DepthMap make_depth_map(uint32_t w, uint32_t h, float depth_val) {
    DepthMap dm;
    dm.width         = w;
    dm.height        = h;
    dm.source_width  = w;
    dm.source_height = h;
    dm.scale         = 1.0f;
    dm.confidence    = 0.9f;
    dm.data.assign(static_cast<size_t>(w) * h, depth_val);
    return dm;
}

static InferenceDetection make_sam_mask(float x, float y, float w, float h,
                                        uint32_t mask_grid = 8) {
    InferenceDetection det;
    det.bbox        = {x, y, w, h};
    det.class_id    = -1;
    det.confidence  = 0.9f;
    det.mask_width  = mask_grid;
    det.mask_height = mask_grid;
    det.mask.assign(static_cast<size_t>(mask_grid) * mask_grid, 255);
    return det;
}

static InferenceDetection make_detector_output(float x, float y, float w, float h, int class_id,
                                               float conf) {
    InferenceDetection det;
    det.bbox       = {x, y, w, h};
    det.class_id   = class_id;
    det.confidence = conf;
    return det;
}

// Shared fixture: initialised projector with 640x480 pinhole
class MaskDepthProjectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        CameraIntrinsics intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
        ASSERT_TRUE(projector_.init(intr));
    }

    CpuSemanticProjector projector_;
};

// ── Tests ──

TEST_F(MaskDepthProjectorTest, ProjectSingleGeometricMask) {
    MaskDepthProjector mdp(projector_);
    auto               mask  = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto               depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());

    const auto& updates = result.value();
    EXPECT_FALSE(updates.empty());
    for (const auto& vu : updates) {
        EXPECT_EQ(vu.semantic_label, static_cast<uint8_t>(ObjectClass::GEOMETRIC_OBSTACLE));
        EXPECT_GT(vu.confidence, 0.0f);
        EXPECT_FLOAT_EQ(vu.occupancy, 1.0f);
    }
}

TEST_F(MaskDepthProjectorTest, ProjectMaskWithDetectorClass) {
    MaskDepthProjector mdp(projector_);
    auto               mask = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    // Overlapping detector: COCO class 0 = person
    auto det   = make_detector_output(280.0f, 200.0f, 80.0f, 80.0f, 0, 0.85f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());

    const auto& updates = result.value();
    EXPECT_FALSE(updates.empty());
    for (const auto& vu : updates) {
        EXPECT_EQ(vu.semantic_label, static_cast<uint8_t>(ObjectClass::PERSON));
    }
}

TEST_F(MaskDepthProjectorTest, ProjectMaskBelowIoUThreshold) {
    MaskDepthProjector mdp(projector_, 0.9f);
    // SAM mask at left, detector at right — minimal overlap
    auto mask  = make_sam_mask(100.0f, 200.0f, 80.0f, 80.0f);
    auto det   = make_detector_output(300.0f, 200.0f, 80.0f, 80.0f, 0, 0.85f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());

    const auto& updates = result.value();
    EXPECT_FALSE(updates.empty());
    for (const auto& vu : updates) {
        EXPECT_EQ(vu.semantic_label, static_cast<uint8_t>(ObjectClass::GEOMETRIC_OBSTACLE));
    }
}

TEST_F(MaskDepthProjectorTest, EmptyMasksReturnsEmpty) {
    MaskDepthProjector mdp(projector_);
    auto               depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().empty());
}

TEST_F(MaskDepthProjectorTest, EmptyDepthReturnsError) {
    MaskDepthProjector mdp(projector_);
    auto               mask = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    DepthMap           empty_depth;

    auto result = mdp.project({mask}, {}, empty_depth, Eigen::Affine3f::Identity());
    EXPECT_TRUE(result.is_err());
}

TEST_F(MaskDepthProjectorTest, UninitializedProjectorReturnsError) {
    CpuSemanticProjector uninit_proj;
    MaskDepthProjector   mdp(uninit_proj);
    auto                 mask  = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto                 depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    EXPECT_TRUE(result.is_err());
}

TEST_F(MaskDepthProjectorTest, MultipleMasksMixedAssignment) {
    MaskDepthProjector mdp(projector_);
    // 3 SAM masks, 1 detector overlapping the first
    auto mask1 = make_sam_mask(100.0f, 100.0f, 80.0f, 80.0f);
    auto mask2 = make_sam_mask(300.0f, 100.0f, 80.0f, 80.0f);
    auto mask3 = make_sam_mask(100.0f, 300.0f, 80.0f, 80.0f);
    auto det   = make_detector_output(100.0f, 100.0f, 80.0f, 80.0f, 2, 0.9f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask1, mask2, mask3}, {det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());

    const auto& updates = result.value();
    EXPECT_GE(updates.size(), 3u);

    // Should have a mix: some VEHICLE_CAR (from det class 2), some GEOMETRIC_OBSTACLE
    bool has_car       = false;
    bool has_geometric = false;
    for (const auto& vu : updates) {
        if (vu.semantic_label == static_cast<uint8_t>(ObjectClass::VEHICLE_CAR)) has_car = true;
        if (vu.semantic_label == static_cast<uint8_t>(ObjectClass::GEOMETRIC_OBSTACLE))
            has_geometric = true;
    }
    EXPECT_TRUE(has_car);
    EXPECT_TRUE(has_geometric);
}

TEST_F(MaskDepthProjectorTest, DepthNaNProducesFewerUpdates) {
    MaskDepthProjector mdp(projector_);
    auto               mask = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);

    // Valid depth → some updates
    auto depth_valid = make_depth_map(640, 480, 5.0f);
    auto r1          = mdp.project({mask}, {}, depth_valid, Eigen::Affine3f::Identity());
    ASSERT_TRUE(r1.is_ok());
    auto valid_count = r1.value().size();

    // NaN depth → zero or fewer updates (CpuSemanticProjector skips NaN samples)
    auto depth_nan = make_depth_map(640, 480, std::numeric_limits<float>::quiet_NaN());
    auto r2        = mdp.project({mask}, {}, depth_nan, Eigen::Affine3f::Identity());
    ASSERT_TRUE(r2.is_ok());
    EXPECT_LT(r2.value().size(), valid_count);
}

TEST_F(MaskDepthProjectorTest, MaskSamplingProducesMultipleVoxels) {
    MaskDepthProjector mdp(projector_);
    // Large mask → CpuSemanticProjector samples 4×4 grid = up to 16 voxels
    auto mask  = make_sam_mask(100.0f, 100.0f, 400.0f, 300.0f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    // 4×4 grid = 16 samples, all within mask → expect 16 voxels
    EXPECT_EQ(result.value().size(), 16u);
}

TEST(MaskDepthProjectorBasic, IouThresholdAccessor) {
    CpuSemanticProjector proj;
    MaskDepthProjector   mdp(proj, 0.75f);
    EXPECT_FLOAT_EQ(mdp.iou_threshold(), 0.75f);
}

TEST(MaskDepthProjectorBasic, DefaultIouThreshold) {
    CpuSemanticProjector proj;
    MaskDepthProjector   mdp(proj);
    EXPECT_FLOAT_EQ(mdp.iou_threshold(), 0.5f);
}
