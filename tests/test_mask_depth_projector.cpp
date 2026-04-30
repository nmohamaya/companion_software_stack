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

TEST_F(MaskDepthProjectorTest, UnmappedCocoClassProducesUnknown) {
    MaskDepthProjector mdp(projector_);
    auto               mask = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    // COCO class 1 = bicycle → coco_to_object_class returns UNKNOWN
    auto det   = make_detector_output(280.0f, 200.0f, 80.0f, 80.0f, 1, 0.85f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());

    const auto& updates = result.value();
    EXPECT_FALSE(updates.empty());
    for (const auto& vu : updates) {
        EXPECT_EQ(vu.semantic_label, static_cast<uint8_t>(ObjectClass::UNKNOWN));
    }
}

TEST_F(MaskDepthProjectorTest, IoUBoundaryJustBelowThreshold) {
    MaskDepthProjector mdp(projector_, 0.5f);
    auto               mask = make_sam_mask(100.0f, 100.0f, 80.0f, 80.0f);
    // Overlap 30px in x: intersection=30*80=2400, union=2*6400-2400=10400, IoU≈0.231 < 0.5
    auto det   = make_detector_output(150.0f, 100.0f, 80.0f, 80.0f, 0, 0.9f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    for (const auto& vu : result.value()) {
        EXPECT_EQ(vu.semantic_label, static_cast<uint8_t>(ObjectClass::GEOMETRIC_OBSTACLE));
    }
}

TEST_F(MaskDepthProjectorTest, IoUBoundaryAtThreshold) {
    MaskDepthProjector mdp(projector_, 0.5f);
    auto               mask = make_sam_mask(100.0f, 100.0f, 80.0f, 80.0f);
    // Identical bbox: IoU=1.0 >= 0.5 → should assign detector class
    auto det   = make_detector_output(100.0f, 100.0f, 80.0f, 80.0f, 0, 0.9f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    for (const auto& vu : result.value()) {
        EXPECT_EQ(vu.semantic_label, static_cast<uint8_t>(ObjectClass::PERSON));
    }
}

TEST_F(MaskDepthProjectorTest, VoxelPositionMatchesPinholeBackprojection) {
    MaskDepthProjector mdp(projector_);
    // Mask centred at image centre (320,240) with known depth=5.0 and identity pose.
    // Pinhole: x_cam=(320-320)*5/500=0, y_cam=(240-240)*5/500=0, z_cam=5
    // Expected world position: (0, 0, 5)
    auto mask  = make_sam_mask(310.0f, 230.0f, 20.0f, 20.0f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(result.value().empty());

    // All 4×4 samples within the 20×20 bbox centred at (320,240) should be near (0,0,5)
    for (const auto& vu : result.value()) {
        EXPECT_NEAR(vu.position_m.x(), 0.0f, 0.2f);
        EXPECT_NEAR(vu.position_m.y(), 0.0f, 0.2f);
        EXPECT_NEAR(vu.position_m.z(), 5.0f, 0.01f);
    }
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

TEST(MaskDepthProjectorBasic, IouThresholdClampedToRange) {
    CpuSemanticProjector proj;
    MaskDepthProjector   low(proj, -1.0f);
    EXPECT_FLOAT_EQ(low.iou_threshold(), 0.01f);

    MaskDepthProjector high(proj, 5.0f);
    EXPECT_FLOAT_EQ(high.iou_threshold(), 1.0f);
}

// ── Issue #645 — Altitude filter (#658) ─────────────────────────────────
// Drop voxels above max_z_m (sky misprojections) and below min_z_m
// (ground patches with depth bias) at the projector boundary.
// Diagnostic on run 2026-04-30_174147 found 22 % of voxels above 6 m
// and 10 % below 0.3 m — a guaranteed 32 % of every frame was ghost.

TEST_F(MaskDepthProjectorTest, AltitudeFilter_DropsVoxelsAboveMaxZ) {
    // Camera pose: 5 m altitude, looking straight down (+Z axis tilted to look
    // at +X direction).  Use identity pose + depth that produces world-Z
    // voxels comfortably above 6 m to hit the upper bound.
    //
    // Easiest fixture: identity pose, depth=15m.  CpuSemanticProjector with
    // pinhole intrinsics back-projects each mask pixel along the camera
    // forward axis (+Z in camera frame) → world-Z = 15 m for an identity
    // pose.  Above max_z_m=6.0, all dropped.
    MaskDepthProjector::AltitudeFilter alt{0.0f, 6.0f};
    MaskDepthProjector                 mdp(projector_, 0.5f, alt);

    auto mask  = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto depth = make_depth_map(640, 480, 15.0f);

    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    const auto& updates = result.value();
    EXPECT_TRUE(updates.empty()) << "Voxels at z=15 m should be dropped by max_z_m=6";
    EXPECT_GT(mdp.altitude_dropped_count(), 0u);
}

TEST_F(MaskDepthProjectorTest, AltitudeFilter_DropsVoxelsBelowMinZ) {
    // Identity pose + depth=0.1 m → world-Z = 0.1 m, below min_z_m=0.3 m.
    MaskDepthProjector::AltitudeFilter alt{0.3f, 0.0f};
    MaskDepthProjector                 mdp(projector_, 0.5f, alt);

    auto mask  = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto depth = make_depth_map(640, 480, 0.1f);

    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    const auto& updates = result.value();
    EXPECT_TRUE(updates.empty()) << "Voxels at z=0.1 m should be dropped by min_z_m=0.3";
    EXPECT_GT(mdp.altitude_dropped_count(), 0u);
}

TEST_F(MaskDepthProjectorTest, AltitudeFilter_PassesVoxelsInBand) {
    // Identity pose + depth=3 m → world-Z = 3 m, inside [0.3, 6.0] band.
    MaskDepthProjector::AltitudeFilter alt{0.3f, 6.0f};
    MaskDepthProjector                 mdp(projector_, 0.5f, alt);

    auto mask  = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto depth = make_depth_map(640, 480, 3.0f);

    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    const auto& updates = result.value();
    EXPECT_FALSE(updates.empty()) << "In-band voxels must survive the filter";
    EXPECT_EQ(mdp.altitude_dropped_count(), 0u);
}

// (#658 P2: the !isfinite(z) guard in the altitude filter is defensive
// against any future projector that might emit non-finite z.  The
// current CpuSemanticProjector rejects NaN/inf depth upstream, so the
// guard cannot be exercised through the public projection API today.
// Keeping the guard prevents a silent NaN-bypass regression if the
// projector contract ever changes.)

TEST_F(MaskDepthProjectorTest, AltitudeFilter_DefaultDisabled) {
    // No filter passed → no voxels dropped, even at extreme depths.
    MaskDepthProjector mdp(projector_);

    auto mask  = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto depth = make_depth_map(640, 480, 15.0f);  // would be dropped if filter active

    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(result.value().empty());
    EXPECT_EQ(mdp.altitude_dropped_count(), 0u);
}

// ── Issue #645 — SAM mask size filter (#659) ─────────────────────────
// Drop SAM masks whose bbox area falls outside [min_area_px, max_area_px]
// before back-projection.  Tiny masks → texture-feature ghosts; huge
// masks → sky/ground walls.
TEST_F(MaskDepthProjectorTest, MaskSizeFilter_DropsTinyMasks) {
    MaskDepthProjector::MaskSizeFilter ms{900.0f, 0.0f};  // min 900 px
    MaskDepthProjector mdp(projector_, 0.5f, MaskDepthProjector::AltitudeFilter{0.0f, 0.0f}, ms);

    // 20×20 = 400 px (below 900 px floor)
    auto tiny  = make_sam_mask(100.0f, 100.0f, 20.0f, 20.0f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({tiny}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().empty()) << "Tiny mask must be dropped before back-projection";
    EXPECT_EQ(mdp.mask_size_dropped_min_count(), 1u);
}

TEST_F(MaskDepthProjectorTest, MaskSizeFilter_DropsHugeMasks) {
    MaskDepthProjector::MaskSizeFilter ms{0.0f, 100000.0f};  // max 100k px
    MaskDepthProjector mdp(projector_, 0.5f, MaskDepthProjector::AltitudeFilter{0.0f, 0.0f}, ms);

    // 600×400 = 240,000 px (above 100k cap)
    auto huge  = make_sam_mask(20.0f, 40.0f, 600.0f, 400.0f);
    auto depth = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({huge}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().empty()) << "Huge mask must be dropped before back-projection";
    EXPECT_EQ(mdp.mask_size_dropped_max_count(), 1u);
}

TEST_F(MaskDepthProjectorTest, MaskSizeFilter_PassesInBandMasks) {
    MaskDepthProjector::MaskSizeFilter ms{900.0f, 100000.0f};
    MaskDepthProjector mdp(projector_, 0.5f, MaskDepthProjector::AltitudeFilter{0.0f, 0.0f}, ms);

    // 80×80 = 6400 px (in band)
    auto in_band = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto depth   = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({in_band}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(result.value().empty()) << "In-band mask must back-project";
    EXPECT_EQ(mdp.mask_size_dropped_min_count(), 0u);
    EXPECT_EQ(mdp.mask_size_dropped_max_count(), 0u);
}

TEST_F(MaskDepthProjectorTest, MaskSizeFilter_MixedBatch) {
    // Three masks: tiny / in-band / huge.  Only middle one survives.
    MaskDepthProjector::MaskSizeFilter ms{900.0f, 100000.0f};
    MaskDepthProjector mdp(projector_, 0.5f, MaskDepthProjector::AltitudeFilter{0.0f, 0.0f}, ms);

    auto tiny    = make_sam_mask(0.0f, 0.0f, 20.0f, 20.0f);
    auto in_band = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto huge    = make_sam_mask(0.0f, 0.0f, 600.0f, 400.0f);
    auto depth   = make_depth_map(640, 480, 5.0f);

    auto result = mdp.project({tiny, in_band, huge}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(result.value().empty());
    EXPECT_EQ(mdp.mask_size_dropped_min_count(), 1u);
    EXPECT_EQ(mdp.mask_size_dropped_max_count(), 1u);
}

TEST_F(MaskDepthProjectorTest, MaskSizeFilter_DefaultDisabled) {
    MaskDepthProjector mdp(projector_);
    auto               tiny   = make_sam_mask(100.0f, 100.0f, 20.0f, 20.0f);
    auto               depth  = make_depth_map(640, 480, 5.0f);
    auto               result = mdp.project({tiny}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(result.value().empty()) << "Default constructor disables mask-size filter";
    EXPECT_EQ(mdp.mask_size_dropped_min_count(), 0u);
}

TEST_F(MaskDepthProjectorTest, MaskSizeFilter_InvertedBandIsDisabled) {
    MaskDepthProjector::MaskSizeFilter ms{100000.0f, 1000.0f};  // min > max
    MaskDepthProjector mdp(projector_, 0.5f, MaskDepthProjector::AltitudeFilter{0.0f, 0.0f}, ms);
    auto               tiny   = make_sam_mask(100.0f, 100.0f, 20.0f, 20.0f);
    auto               depth  = make_depth_map(640, 480, 5.0f);
    auto               result = mdp.project({tiny}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(result.value().empty()) << "Inverted band must be treated as disabled";
}

TEST_F(MaskDepthProjectorTest, AltitudeFilter_InvertedBandIsDisabled) {
    // min_z_m >= max_z_m is nonsensical — constructor should disable filter
    // and emit a warning.
    MaskDepthProjector::AltitudeFilter alt{6.0f, 3.0f};
    MaskDepthProjector                 mdp(projector_, 0.5f, alt);

    auto mask   = make_sam_mask(280.0f, 200.0f, 80.0f, 80.0f);
    auto depth  = make_depth_map(640, 480, 15.0f);
    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_FALSE(result.value().empty()) << "Inverted band must be treated as disabled";
}

// #659 P2 review fix: malformed bbox with negative w or h must not silently
// pass the size band.  std::abs() in the projector forces magnitude
// comparison, so a -50×-50 bbox (positive area = 2500) lands in [900, 100k].
TEST_F(MaskDepthProjectorTest, MaskSizeFilter_NegativeBboxDimensionsUseAbsoluteArea) {
    MaskDepthProjector::MaskSizeFilter ms{900.0f, 100000.0f};
    MaskDepthProjector mdp(projector_, 0.5f, MaskDepthProjector::AltitudeFilter{0.0f, 0.0f}, ms);

    // Bbox with negative width and height — area would be +2500 with abs.
    auto mask   = make_sam_mask(280.0f, 200.0f, -50.0f, -50.0f);
    auto depth  = make_depth_map(640, 480, 5.0f);
    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(mdp.mask_size_dropped_min_count(), 0u)
        << "Negative-dim bbox with |area|=2500 must NOT drop on min<2500 gate";
    EXPECT_EQ(mdp.mask_size_dropped_max_count(), 0u)
        << "Negative-dim bbox with |area|=2500 must NOT drop on max>100000 gate";
}

// #659 P2 review fix: NaN bbox area must be dropped rather than silently
// passing both min and max checks (NaN comparisons are false in IEEE 754).
TEST_F(MaskDepthProjectorTest, MaskSizeFilter_NaNBboxAreaIsDropped) {
    MaskDepthProjector::MaskSizeFilter ms{900.0f, 100000.0f};
    MaskDepthProjector mdp(projector_, 0.5f, MaskDepthProjector::AltitudeFilter{0.0f, 0.0f}, ms);

    auto mask   = make_sam_mask(280.0f, 200.0f, std::numeric_limits<float>::quiet_NaN(), 50.0f);
    auto depth  = make_depth_map(640, 480, 5.0f);
    auto result = mdp.project({mask}, {}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(mdp.mask_size_dropped_min_count(), 1u)
        << "NaN bbox area must be dropped (counted as min) so it cannot reach the projector";
    EXPECT_TRUE(result.value().empty()) << "NaN-bbox mask was the only input; output must be empty";
}
