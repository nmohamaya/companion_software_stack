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

    // Default 4x4 grid with all mask pixels active → 16 updates
    EXPECT_EQ(result.value().size(), 16u);
    for (const auto& vu : result.value()) {
        EXPECT_EQ(vu.semantic_label, 5);
    }
}

// ─── Issue #629 — configurable sample density ─────────────────────────

TEST(SemanticProjector, SampleGridSize_DefaultIs4) {
    CpuSemanticProjector proj;
    EXPECT_EQ(proj.sample_grid_size(), 4);
}

TEST(SemanticProjector, SampleGridSize_ClampToMinimum2) {
    // Values below 2 would produce ≤1 probe per row — a zero-coverage
    // projector.  The setter must clamp.
    CpuSemanticProjector proj;
    proj.set_sample_grid_size(0);
    EXPECT_EQ(proj.sample_grid_size(), 2);
    proj.set_sample_grid_size(-5);
    EXPECT_EQ(proj.sample_grid_size(), 2);
}

TEST(SemanticProjector, SampleGridSize_ClampToMaximum64) {
    // 64×64 = 4096 probes/mask × 3 masks ≈ 12k back-projections per frame.
    // Higher values risk starving the detector thread on allocation alone.
    CpuSemanticProjector proj;
    proj.set_sample_grid_size(1000);
    EXPECT_EQ(proj.sample_grid_size(), 64);
}

// Helper for the parameterised probe-count tests below.
namespace {
size_t fully_masked_probe_count(int grid_size) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    EXPECT_TRUE(proj.init(intr));
    proj.set_sample_grid_size(grid_size);
    EXPECT_EQ(proj.sample_grid_size(), grid_size);

    InferenceDetection det;
    det.bbox        = {100.0f, 100.0f, 80.0f, 80.0f};
    det.class_id    = 5;
    det.confidence  = 0.8f;
    det.mask_width  = 640;
    det.mask_height = 480;
    det.mask.assign(static_cast<size_t>(det.mask_width) * det.mask_height, 255);

    auto depth  = make_depth_map(640, 480, 8.0f);
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    EXPECT_TRUE(result.is_ok());
    return result.value().size();
}
}  // namespace

TEST(SemanticProjector, SampleGridSize_8EmitsExpectedProbeCount) {
    // The smoking gun from Issue #629: with legacy N=4, a fully-masked
    // detection emits 16 voxels.  Bumping N=8 quadruples coverage.
    EXPECT_EQ(fully_masked_probe_count(8), 64u);
}

TEST(SemanticProjector, SampleGridSize_16EmitsExpectedProbeCount_Production) {
    // Production value used by scenario 33 (the no-HD-map perception case).
    // Without this test, the production density was unvalidated — the
    // earlier test mis-named "16" actually exercised N=8.
    // (Review-test-quality P1 on PR #630.)
    EXPECT_EQ(fully_masked_probe_count(16), 256u);
}

TEST(SemanticProjector, SampleGridSize_2EmitsExpectedProbeCount) {
    // Boundary: minimum clamped value.  An off-by-one in the loop bound
    // (e.g. `gy <= GRID`) would emit 9 probes instead of 4.
    EXPECT_EQ(fully_masked_probe_count(2), 4u);
}

TEST(SemanticProjector, SampleGridSize_64EmitsExpectedProbeCount) {
    // Boundary: maximum clamped value.  Verifies allocation succeeds and
    // no array-bounds fault under 4096 back-projections per mask
    // (review-fault-recovery P3 on PR #630).
    EXPECT_EQ(fully_masked_probe_count(64), 4096u);
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

// ═══════════════════════════════════════════════════════════
// Texture gate (Issue #616)
// ═══════════════════════════════════════════════════════════

namespace {
// Helper: depth map with a gradient ramp on one half and flat depth on the
// other, so we can sample a known "textured" pixel and a "flat" pixel.
DepthMap make_half_gradient_depth(uint32_t w, uint32_t h, float flat_depth = 5.0f,
                                  float gradient_step = 0.1f) {
    DepthMap dm;
    dm.width         = w;
    dm.height        = h;
    dm.source_width  = w;
    dm.source_height = h;
    dm.scale         = 1.0f;
    dm.confidence    = 0.9f;
    dm.data.resize(static_cast<size_t>(w) * h);
    // Left half: constant (flat / untextured).  Right half: linear ramp in x.
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const size_t idx = static_cast<size_t>(y) * w + x;
            if (x < w / 2) {
                dm.data[idx] = flat_depth;
            } else {
                dm.data[idx] = flat_depth + static_cast<float>(x - w / 2) * gradient_step;
            }
        }
    }
    return dm;
}

CameraIntrinsics make_intrinsics(uint32_t w, uint32_t h) {
    CameraIntrinsics i;
    i.width  = w;
    i.height = h;
    i.fx     = 300.0f;
    i.fy     = 300.0f;
    i.cx     = static_cast<float>(w) * 0.5f;
    i.cy     = static_cast<float>(h) * 0.5f;
    return i;
}

InferenceDetection make_det_at(uint32_t cx, uint32_t cy, uint32_t half = 4) {
    InferenceDetection d;
    d.bbox.x     = static_cast<int32_t>(cx) - static_cast<int32_t>(half);
    d.bbox.y     = static_cast<int32_t>(cy) - static_cast<int32_t>(half);
    d.bbox.w     = static_cast<int32_t>(half * 2);
    d.bbox.h     = static_cast<int32_t>(half * 2);
    d.class_id   = 1;
    d.confidence = 0.9f;
    return d;
}
}  // namespace

TEST(SemanticProjector, TextureGateDefaultDisabled) {
    // With threshold 0 (default), ALL sampled depths pass through — both the
    // flat half and the gradient half produce voxels.  Preserves pre-#616
    // behaviour for Gazebo scenarios that never opt in.
    CpuSemanticProjector proj;
    ASSERT_TRUE(proj.init(make_intrinsics(64, 64)));
    EXPECT_FLOAT_EQ(proj.texture_gate_threshold(), 0.0f);

    auto depth = make_half_gradient_depth(64, 64);

    // One detection in the flat half, one in the gradient half.
    std::vector<InferenceDetection> dets{make_det_at(16, 32), make_det_at(48, 32)};

    auto result = proj.project(dets, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().size(), 2u)
        << "With texture gate off, both detections produce voxels.";
}

TEST(SemanticProjector, TextureGateRejectsFlatRegion) {
    // Set a threshold high enough to reject the flat half but still accept
    // the gradient half. With step=0.1 and the un-normalised 3x3 Sobel
    // (kernel weights ±1/±2), the magnitude on the interior of the ramp is
    //   sqrt((-1*(d-Δ) + 1*(d+Δ) + -2*(d-Δ) + 2*(d+Δ) + -1*(d-Δ) + 1*(d+Δ))^2)
    //   = |8*Δ| = 0.8 (with Δ=0.1).  Flat half has gradient 0.  Threshold
    //   0.2 is comfortably between the two.
    CpuSemanticProjector proj;
    ASSERT_TRUE(proj.init(make_intrinsics(64, 64)));
    proj.set_texture_gate_threshold(0.2f);
    EXPECT_FLOAT_EQ(proj.texture_gate_threshold(), 0.2f);

    auto depth = make_half_gradient_depth(64, 64);

    // Flat-half detection (x=16) — expected to be rejected by the gate.
    std::vector<InferenceDetection> dets_flat{make_det_at(16, 32)};
    auto r_flat = proj.project(dets_flat, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(r_flat.is_ok());
    EXPECT_EQ(r_flat.value().size(), 0u)
        << "Texture gate should reject the flat-depth bbox centre; got " << r_flat.value().size()
        << " voxels.";

    // Positive control: the same flat-half detection MUST produce a voxel
    // when the gate is disabled. Proves the rejection above is caused by
    // the texture gate, not by an unrelated rejection (e.g. depth=0,
    // out-of-bounds projection, or another preflight check).
    CpuSemanticProjector proj_open;
    ASSERT_TRUE(proj_open.init(make_intrinsics(64, 64)));
    EXPECT_FLOAT_EQ(proj_open.texture_gate_threshold(), 0.0f);
    auto r_flat_open = proj_open.project(dets_flat, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(r_flat_open.is_ok());
    EXPECT_EQ(r_flat_open.value().size(), 1u)
        << "Without the gate, the same flat detection should produce a voxel — "
           "otherwise the rejection above is not attributable to the gate.";

    // Gradient-half detection (x=48) — expected to pass.
    std::vector<InferenceDetection> dets_grad{make_det_at(48, 32)};
    auto r_grad = proj.project(dets_grad, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(r_grad.is_ok());
    EXPECT_EQ(r_grad.value().size(), 1u)
        << "Texture gate should accept the high-gradient bbox centre; got " << r_grad.value().size()
        << " voxels.";
}

TEST(SemanticProjector, TextureGateNegativeClampedToZero) {
    // set_texture_gate_threshold clamps negative input to zero (a negative
    // threshold is nonsensical; silently clamping is safer than accepting it).
    CpuSemanticProjector proj;
    proj.set_texture_gate_threshold(-1.0f);
    EXPECT_FLOAT_EQ(proj.texture_gate_threshold(), 0.0f);
}

TEST(SemanticProjector, TextureGateDegenerateDepthMap) {
    // 3x3 Sobel needs at least 3 rows / cols of usable data.  When the
    // depth map is smaller than that (e.g. a 2x2 patch from a heavily
    // letter-boxed source), depth_gradient_magnitude() must fail closed
    // — return 0 so the gate rejects, rather than read out-of-bounds.
    // This is the safety-critical default when there's not enough data
    // to assess texture.
    CpuSemanticProjector proj;
    // Intrinsics still 64x64 (source frame) but the DepthMap is 2x2.
    ASSERT_TRUE(proj.init(make_intrinsics(64, 64)));
    proj.set_texture_gate_threshold(0.05f);  // tiny threshold

    DepthMap dm;
    dm.width         = 2;
    dm.height        = 2;
    dm.source_width  = 64;
    dm.source_height = 64;
    dm.scale         = 32.0f;  // 64 / 2
    dm.confidence    = 0.9f;
    dm.data          = {1.0f, 9.0f, 9.0f, 1.0f};  // huge swing — would pass any gate if computed

    std::vector<InferenceDetection> dets{make_det_at(32, 32)};
    auto                            r = proj.project(dets, dm, Eigen::Affine3f::Identity());
    ASSERT_TRUE(r.is_ok());
    EXPECT_EQ(r.value().size(), 0u)
        << "Degenerate (<3-pixel) DepthMap must fail closed via the texture gate, "
           "not produce voxels from extrapolated gradient values.";
}

// ── Issue #698 — DA V2 max-clamp saturation band ───────────────

// Default (band == 0) preserves legacy strict-greater-than rejection: a
// depth sample exactly at max_obstacle_depth_m_ still passes through.
TEST(SemanticProjector, SaturationBand_DefaultDisabledLegacyBehaviour) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));
    proj.set_max_obstacle_depth_m(20.0f);
    ASSERT_FLOAT_EQ(proj.max_depth_saturation_band_m(), 0.0f);

    InferenceDetection det;
    det.bbox       = {300.0f, 220.0f, 40.0f, 40.0f};
    det.class_id   = 3;
    det.confidence = 0.85f;

    auto depth  = make_depth_map(640, 480, 20.0f);  // exactly at max
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().size(), 1u)
        << "With band=0, a depth sample exactly at max_obstacle_depth_m must pass "
           "(legacy behaviour preserved).";
}

// With band > 0 a depth sample at exactly max gets rejected — this is the
// DA V2 "I don't know, defaulting to far" failure mode.
TEST(SemanticProjector, SaturationBand_RejectsSampleAtMax) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));
    proj.set_max_obstacle_depth_m(20.0f);
    proj.set_max_depth_saturation_band_m(0.5f);

    InferenceDetection det;
    det.bbox       = {300.0f, 220.0f, 40.0f, 40.0f};
    det.class_id   = 3;
    det.confidence = 0.85f;

    auto depth  = make_depth_map(640, 480, 20.0f);
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().size(), 0u)
        << "Depth at max_obstacle_depth_m must be rejected when saturation band is active.";
}

// Inside the band but below max — also rejected.
TEST(SemanticProjector, SaturationBand_RejectsSampleInsideBand) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));
    proj.set_max_obstacle_depth_m(20.0f);
    proj.set_max_depth_saturation_band_m(0.5f);

    InferenceDetection det;
    det.bbox       = {300.0f, 220.0f, 40.0f, 40.0f};
    det.class_id   = 3;
    det.confidence = 0.85f;

    auto depth  = make_depth_map(640, 480, 19.7f);  // inside the 0.5 m band → rejected
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().size(), 0u) << "Depth inside the saturation band must be rejected.";
}

// Just outside the band — accepted. Real obstacles below max - band still pass.
TEST(SemanticProjector, SaturationBand_AcceptsSampleBelowBand) {
    CpuSemanticProjector proj;
    CameraIntrinsics     intr{500.0f, 500.0f, 320.0f, 240.0f, 640, 480};
    ASSERT_TRUE(proj.init(intr));
    proj.set_max_obstacle_depth_m(20.0f);
    proj.set_max_depth_saturation_band_m(0.5f);

    InferenceDetection det;
    det.bbox       = {300.0f, 220.0f, 40.0f, 40.0f};
    det.class_id   = 3;
    det.confidence = 0.85f;

    auto depth  = make_depth_map(640, 480, 19.0f);  // 1 m below max → outside band → kept
    auto result = proj.project({det}, depth, Eigen::Affine3f::Identity());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().size(), 1u) << "Depth comfortably below max - band must pass through.";
}

// Negative band values clamp to 0 (disabled) — defensive against bad config.
TEST(SemanticProjector, SaturationBand_NegativeClampedToZero) {
    CpuSemanticProjector proj;
    proj.set_max_depth_saturation_band_m(-1.0f);
    EXPECT_FLOAT_EQ(proj.max_depth_saturation_band_m(), 0.0f);
}
