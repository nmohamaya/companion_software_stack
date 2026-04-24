// tests/test_depth_anything_v2.cpp
// Unit tests for DepthAnythingV2Estimator: Depth Anything V2 via OpenCV DNN.
// Tests run with and without the ONNX model file.
// Issue #455.
#include "hal/depth_anything_v2.h"
#include "hal/idepth_estimator.h"
#include "util/config.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::hal;

// ═══════════════════════════════════════════════════════════
// Model path resolution
// ═══════════════════════════════════════════════════════════

#ifdef DA_V2_MODEL_PATH
static const char* g_model_path = DA_V2_MODEL_PATH;
#else
static const char* g_model_path = "models/depth_anything_v2_vits.onnx";
#endif

static bool model_exists() {
    std::ifstream f(g_model_path);
    return f.good();
}

// ═══════════════════════════════════════════════════════════
// Tests that always run (no model file needed)
// ═══════════════════════════════════════════════════════════

TEST(DepthAnythingV2Test, NameReturnsExpected) {
    // Construct with non-existent model — name() still works
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    EXPECT_EQ(estimator.name(), "DepthAnythingV2Estimator");
}

TEST(DepthAnythingV2Test, ModelNotFoundNotLoaded) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    EXPECT_FALSE(estimator.is_loaded());
}

TEST(DepthAnythingV2Test, ModelNotFoundEstimateReturnsError) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");

    constexpr uint32_t   w = 64, h = 48, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    EXPECT_FALSE(result.is_ok());
}

TEST(DepthAnythingV2Test, PathTraversalRejected) {
    DepthAnythingV2Estimator estimator("../../etc/passwd");
    EXPECT_FALSE(estimator.is_loaded());
}

TEST(DepthAnythingV2Test, PathTraversalMiddleRejected) {
    DepthAnythingV2Estimator estimator("models/../../../secret.onnx");
    EXPECT_FALSE(estimator.is_loaded());
}

TEST(DepthAnythingV2Test, NullFrameReturnsError) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    auto                     result = estimator.estimate(nullptr, 640, 480, 3);
    EXPECT_FALSE(result.is_ok());
}

TEST(DepthAnythingV2Test, ZeroDimensionsReturnError) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    std::vector<uint8_t>     frame(100, 128);

    auto r1 = estimator.estimate(frame.data(), 0, 480, 3);
    EXPECT_FALSE(r1.is_ok());

    auto r2 = estimator.estimate(frame.data(), 640, 0, 3);
    EXPECT_FALSE(r2.is_ok());
}

TEST(DepthAnythingV2Test, InvalidChannelsReturnsError) {
    DepthAnythingV2Estimator estimator("nonexistent_model.onnx");
    std::vector<uint8_t>     frame(100, 128);

    // Too few channels
    auto r0 = estimator.estimate(frame.data(), 10, 10, 0);
    EXPECT_FALSE(r0.is_ok());

    auto r1 = estimator.estimate(frame.data(), 10, 10, 1);
    EXPECT_FALSE(r1.is_ok());

    auto r2 = estimator.estimate(frame.data(), 10, 10, 2);
    EXPECT_FALSE(r2.is_ok());

    // Too many channels (only 3=RGB and 4=RGBA supported)
    auto r5 = estimator.estimate(frame.data(), 10, 10, 5);
    EXPECT_FALSE(r5.is_ok());
}

TEST(DepthAnythingV2Test, ConfigConstruction) {
    // Write a temp config file
    std::string path = "/tmp/test_da_v2_" + std::to_string(getpid()) + ".json";
    {
        std::ofstream ofs(path);
        ofs << R"({
            "perception": {
                "depth_estimator": {
                    "backend": "depth_anything_v2",
                    "model_path": "nonexistent.onnx",
                    "input_size": 256,
                    "max_depth_m": 30.0
                }
            }
        })";
    }

    drone::Config cfg;
    bool          loaded = cfg.load(path);
    std::remove(path.c_str());
    ASSERT_TRUE(loaded) << "Failed to load test config from " << path;

    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");

    EXPECT_EQ(estimator.name(), "DepthAnythingV2Estimator");
    EXPECT_FALSE(estimator.is_loaded());  // model doesn't exist
    // Calibration defaults to off — by-design for backward compat.
    EXPECT_FALSE(estimator.calibration_enabled());
}

// ═══════════════════════════════════════════════════════════
// Calibration (Issue #616) — configuration & fallback logic
// ═══════════════════════════════════════════════════════════

namespace {
// Helper: build a throwaway Config with calibration fields set.
//
// PR #620 test-quality review P1: previously used `pid + rand()` for the
// tmpfile name.  Under `ctest -j` (parallel test execution), an unseeded
// `rand()` plus shared pid produces identical filenames across iterations
// of the same test binary, leading to file-write races + cross-test
// contamination.  `mkstemp` is the POSIX answer — it atomically opens a
// uniquely-named file and returns the fd; we close it and only need the
// path for `cfg.load`.
drone::Config make_calibration_cfg(bool enabled, float raw_min, float raw_max, float a = 1.0f,
                                   float b = 0.0f) {
    char tmpl[] = "/tmp/test_da_v2_calib_XXXXXX.json";
    int  fd     = ::mkstemps(tmpl, 5);  // 5 = length of ".json" suffix
    EXPECT_GE(fd, 0) << "mkstemps failed for tmpfile";
    if (fd >= 0) ::close(fd);
    std::string path(tmpl);
    {
        std::ofstream ofs(path);
        ofs << "{\n"
            << "  \"perception\": {\n"
            << "    \"depth_estimator\": {\n"
            << "      \"backend\": \"depth_anything_v2\",\n"
            << "      \"model_path\": \"nonexistent.onnx\",\n"
            << "      \"dav2\": {\n"
            << "        \"calibration_enabled\": " << (enabled ? "true" : "false") << ",\n"
            << "        \"raw_min_ref\": " << raw_min << ",\n"
            << "        \"raw_max_ref\": " << raw_max << ",\n"
            << "        \"calibration_coef_a\": " << a << ",\n"
            << "        \"calibration_coef_b\": " << b << "\n"
            << "      }\n"
            << "    }\n"
            << "  }\n"
            << "}\n";
    }
    drone::Config cfg;
    bool          ok = cfg.load(path);
    std::remove(path.c_str());
    EXPECT_TRUE(ok) << "Failed to load test config from " << path;
    return cfg;
}
}  // namespace

TEST(DepthAnythingV2Test, CalibrationEnabled_WhenRefsValid) {
    // Real refs (min < max, both finite) → calibration stays enabled.
    auto cfg = make_calibration_cfg(/*enabled=*/true, /*raw_min=*/0.1f, /*raw_max=*/0.9f);
    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");
    EXPECT_TRUE(estimator.calibration_enabled())
        << "Valid refs should keep calibration on; got disabled.";
}

TEST(DepthAnythingV2Test, CalibrationFallback_OnEqualRefs) {
    // raw_max == raw_min → degenerate window → fall back to per-frame.
    auto cfg = make_calibration_cfg(/*enabled=*/true, /*raw_min=*/0.5f, /*raw_max=*/0.5f);
    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");
    EXPECT_FALSE(estimator.calibration_enabled())
        << "Equal refs should trigger fallback to per-frame.";
}

TEST(DepthAnythingV2Test, CalibrationFallback_OnInvertedRefs) {
    // raw_min > raw_max → inverted window (operator typo) → fall back.
    // Distinct input shape from EqualRefs case; covers the strict `>` guard
    // path (PR #620 test-quality review P2).
    auto cfg = make_calibration_cfg(/*enabled=*/true, /*raw_min=*/0.9f, /*raw_max=*/0.1f);
    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");
    EXPECT_FALSE(estimator.calibration_enabled())
        << "Inverted refs (raw_min > raw_max) should trigger fallback to per-frame.";
}

TEST(DepthAnythingV2Test, CalibrationFallback_OnNaNRef) {
    // One ref NaN, other finite → fails the `isfinite` guard → fall back.
    // Covers the `isfinite` branch independently from the ordering branch
    // (PR #620 test-quality review P2).  We can't write NaN in JSON, so
    // omit raw_max_ref entirely and let the struct default (NaN) take over.
    char tmpl[] = "/tmp/test_da_v2_one_nan_XXXXXX.json";
    int  fd     = ::mkstemps(tmpl, 5);
    ASSERT_GE(fd, 0);
    ::close(fd);
    std::string path(tmpl);
    {
        std::ofstream ofs(path);
        ofs << R"({
            "perception": {
                "depth_estimator": {
                    "backend": "depth_anything_v2",
                    "model_path": "nonexistent.onnx",
                    "dav2": {
                        "calibration_enabled": true,
                        "raw_min_ref": 0.1
                    }
                }
            }
        })";
    }
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    std::remove(path.c_str());

    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");
    EXPECT_FALSE(estimator.calibration_enabled())
        << "One NaN ref (raw_max_ref omitted from JSON) should trigger fallback.";
}

TEST(DepthAnythingV2Test, CalibrationEnabled_AcceptsNarrowButValidWindow) {
    // raw_max = raw_min + 1e-6f → just above the strict-`>` guard threshold.
    // The constructor accepts it; the per-pixel `range < eps (=1e-6f)`
    // branch in estimate() may then apply (handled separately in the math
    // tests below).  This test pins that the *guard* lets it through —
    // callers can decide what to do with the narrow window.
    auto                     cfg = make_calibration_cfg(/*enabled=*/true,
                                    /*raw_min=*/0.5f, /*raw_max=*/0.5f + 1e-5f);
    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");
    EXPECT_TRUE(estimator.calibration_enabled())
        << "Narrow-but-valid window should pass the strict-`>` guard; got disabled.";
}

TEST(DepthAnythingV2Test, CalibrationFallbackOnMissingRefs) {
    // `calibration_enabled=true` but no raw_min_ref / raw_max_ref keys in
    // config → struct defaults (NaN) remain → fall back to per-frame path.
    // JSON doesn't have a NaN literal; this is the realistic shape of an
    // operator config mistake: the flag is flipped on but refs were forgotten.
    std::string path = "/tmp/test_da_v2_nanrefs_" + std::to_string(::getpid()) + ".json";
    {
        std::ofstream ofs(path);
        ofs << R"({
            "perception": {
                "depth_estimator": {
                    "backend": "depth_anything_v2",
                    "model_path": "nonexistent.onnx",
                    "dav2": { "calibration_enabled": true }
                }
            }
        })";
    }
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    std::remove(path.c_str());

    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");
    EXPECT_FALSE(estimator.calibration_enabled())
        << "Missing refs (struct-default NaN) should trigger fallback to per-frame.";
}

TEST(DepthAnythingV2Test, CalibrationDisabledByDefault) {
    // No dav2.* keys in config → calibration is off, behaviour is byte-identical
    // to pre-#616 per-frame normalisation.
    std::string path = "/tmp/test_da_v2_nocal_" + std::to_string(::getpid()) + ".json";
    {
        std::ofstream ofs(path);
        ofs << R"({
            "perception": { "depth_estimator": {
                "backend": "depth_anything_v2",
                "model_path": "nonexistent.onnx"
            }}
        })";
    }
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    std::remove(path.c_str());

    DepthAnythingV2Estimator estimator(cfg, "perception.depth_estimator");
    EXPECT_FALSE(estimator.calibration_enabled());
}

// ═══════════════════════════════════════════════════════════
// Per-pixel calibration math (PR #620 review refactor)
//
// `convert_raw_to_metric()` is a file-scope free function so the
// arithmetic that used to live inside `estimate()` is testable without
// loading the ONNX model.  These cover the branches reviewers couldn't
// reach: per-frame happy path, calibrated happy path, NaN guard,
// out-of-window clamping, output clamping, and the degenerate `coef_a=0`
// fit.
// ═══════════════════════════════════════════════════════════

namespace {
DepthConversionParams make_per_frame_params(float min_depth = 0.1f, float max_depth = 20.0f,
                                            float anchor_min = 0.0f, float anchor_max = 1.0f) {
    DepthConversionParams p;
    p.min_depth_m         = min_depth;
    p.max_depth_m         = max_depth;
    p.calibration_enabled = false;
    p.anchor_min          = anchor_min;
    p.anchor_max          = anchor_max;
    return p;
}

DepthConversionParams make_calibrated_params(float anchor_min, float anchor_max, float a = 1.0f,
                                             float b = 0.0f, float min_depth = 0.1f,
                                             float max_depth = 20.0f) {
    DepthConversionParams p;
    p.min_depth_m         = min_depth;
    p.max_depth_m         = max_depth;
    p.calibration_enabled = true;
    p.anchor_min          = anchor_min;
    p.anchor_max          = anchor_max;
    p.calibration_coef_a  = a;
    p.calibration_coef_b  = b;
    return p;
}
}  // namespace

TEST(DepthAnythingV2MathTest, PerFrame_HighRawMapsToMinDepth) {
    // raw at the per-frame max → normalized = 1 → metric = min_depth (closest).
    auto        p     = make_per_frame_params(/*min*/ 0.1f, /*max*/ 20.0f, /*amin*/ 0.0f,
                                   /*amax*/ 1.0f);
    const float range = p.anchor_max - p.anchor_min;
    EXPECT_NEAR(convert_raw_to_metric(1.0f, range, p), 0.1f, 1e-4f);
}

TEST(DepthAnythingV2MathTest, PerFrame_LowRawMapsToMaxDepth) {
    // raw at the per-frame min → normalized = 0 → metric = max_depth (farthest).
    auto        p     = make_per_frame_params();
    const float range = p.anchor_max - p.anchor_min;
    EXPECT_NEAR(convert_raw_to_metric(0.0f, range, p), 20.0f, 1e-4f);
}

TEST(DepthAnythingV2MathTest, PerFrame_NaN_FailsClosed_ToMaxDepth) {
    // Memory-safety / fault-recovery P3: a NaN raw sample must not
    // propagate to map.data.  Free function returns max_depth (far).
    auto        p     = make_per_frame_params();
    const float range = p.anchor_max - p.anchor_min;
    const float nan   = std::numeric_limits<float>::quiet_NaN();
    const float out   = convert_raw_to_metric(nan, range, p);
    EXPECT_TRUE(std::isfinite(out)) << "NaN raw must not produce NaN metric";
    EXPECT_FLOAT_EQ(out, 20.0f) << "NaN raw should fail closed to max_depth";
}

TEST(DepthAnythingV2MathTest, Calibrated_OutOfWindowRawClamps) {
    // raw above the calibrated window → normalized would exceed 1 → clamp to 1
    // → metric = min_depth.  Documents that out-of-window samples report
    // "closer than anything in the calibration set" rather than extrapolating.
    auto        p     = make_calibrated_params(/*amin*/ 0.2f, /*amax*/ 0.8f);
    const float range = p.anchor_max - p.anchor_min;
    EXPECT_NEAR(convert_raw_to_metric(1.5f, range, p), p.min_depth_m, 1e-4f);
}

TEST(DepthAnythingV2MathTest, Calibrated_BelowWindowRawClamps) {
    // raw below the calibrated window → normalized < 0 → clamp to 0 →
    // metric = max_depth.
    auto        p     = make_calibrated_params(/*amin*/ 0.2f, /*amax*/ 0.8f);
    const float range = p.anchor_max - p.anchor_min;
    EXPECT_NEAR(convert_raw_to_metric(-0.5f, range, p), p.max_depth_m, 1e-4f);
}

TEST(DepthAnythingV2MathTest, Calibrated_LinearFitApplied) {
    // a=2, b=1 → metric_anchored=10 → final = 21, then clamped to max_depth=20.
    auto p        = make_calibrated_params(/*amin*/ 0.0f, /*amax*/ 1.0f, /*a*/ 2.0f, /*b*/ 1.0f);
    p.max_depth_m = 20.0f;
    const float range = p.anchor_max - p.anchor_min;
    // raw=0.5 → normalized=0.5 → metric_anchored = 0.1 + (20-0.1)*(1-0.5) ≈ 10.05
    // a*x+b = 2*10.05 + 1 = 21.1 → clamp to 20.
    EXPECT_NEAR(convert_raw_to_metric(0.5f, range, p), 20.0f, 1e-4f);
}

TEST(DepthAnythingV2MathTest, Calibrated_ZeroCoefAProducesConstantOutput) {
    // PR #620 test-quality P2: a=0 reduces the linear fit to `metric = b`
    // (then clamped).  Defensive: confirms no divide-by-zero or NaN even
    // though the multiplication is the dangerous step, not a divide.
    auto        p = make_calibrated_params(/*amin*/ 0.0f, /*amax*/ 1.0f, /*a*/ 0.0f, /*b*/ 5.0f);
    const float range = p.anchor_max - p.anchor_min;
    // a=0 → metric = 0*x + 5 = 5 (within [0.1, 20] so no clamp).
    EXPECT_NEAR(convert_raw_to_metric(0.3f, range, p), 5.0f, 1e-4f);
    EXPECT_NEAR(convert_raw_to_metric(0.7f, range, p), 5.0f, 1e-4f);
    EXPECT_NEAR(convert_raw_to_metric(0.0f, range, p), 5.0f, 1e-4f);
}

TEST(DepthAnythingV2MathTest, Calibrated_OutputClampToMinDepth) {
    // Linear fit pushes metric below min_depth → output clamps up to min_depth.
    auto        p = make_calibrated_params(/*amin*/ 0.0f, /*amax*/ 1.0f, /*a*/ 0.5f, /*b*/ -10.0f);
    const float range = p.anchor_max - p.anchor_min;
    // raw=1.0 → normalized=1.0 → metric_anchored = min_depth = 0.1
    // a*x+b = 0.5*0.1 + (-10) = -9.95 → clamp up to min_depth.
    EXPECT_NEAR(convert_raw_to_metric(1.0f, range, p), p.min_depth_m, 1e-4f);
}

TEST(DepthAnythingV2MathTest, NaN_FailsClosed_BothPaths) {
    // Belt-and-braces: NaN guard must fail-closed in BOTH per-frame and
    // calibrated paths, regardless of coefficients.
    auto p_per = make_per_frame_params();
    auto p_cal = make_calibrated_params(/*amin*/ 0.0f, /*amax*/ 1.0f, /*a*/ 100.0f, /*b*/ -50.0f);
    EXPECT_FLOAT_EQ(convert_raw_to_metric(std::numeric_limits<float>::quiet_NaN(), 1.0f, p_per),
                    20.0f);
    EXPECT_FLOAT_EQ(convert_raw_to_metric(std::numeric_limits<float>::quiet_NaN(), 1.0f, p_cal),
                    20.0f);
}

// ═══════════════════════════════════════════════════════════
// Tests that require the ONNX model file (skip if not present)
// ═══════════════════════════════════════════════════════════

class DAv2ModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!model_exists()) {
            GTEST_SKIP() << "DA V2 model not found at " << g_model_path;
        }
    }
};

TEST_F(DAv2ModelTest, ModelLoadsSuccessfully) {
    DepthAnythingV2Estimator estimator(g_model_path);
    EXPECT_TRUE(estimator.is_loaded());
}

TEST_F(DAv2ModelTest, ValidFrameProducesDepthMap) {
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    // Create a synthetic RGB frame (gradient pattern)
    constexpr uint32_t   w = 640, h = 480, c = 3;
    std::vector<uint8_t> frame(w * h * c);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t idx     = (y * w + x) * c;
            frame[idx + 0] = static_cast<uint8_t>(x % 256);  // R: horizontal gradient
            frame[idx + 1] = static_cast<uint8_t>(y % 256);  // G: vertical gradient
            frame[idx + 2] = 128;                            // B: constant
        }
    }

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();

    const auto& map = result.value();
    EXPECT_GT(map.width, 0u);
    EXPECT_GT(map.height, 0u);
    EXPECT_EQ(map.data.size(), static_cast<size_t>(map.width) * map.height);
    EXPECT_FLOAT_EQ(map.scale, 1.0f);
    EXPECT_FLOAT_EQ(map.confidence, 0.7f);
}

TEST_F(DAv2ModelTest, SourceDimensionsSetCorrectly) {
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    constexpr uint32_t   w = 1920, h = 1080, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();

    const auto& map = result.value();
    // source_width/source_height must be the INPUT frame dimensions
    // (for fusion bbox mapping), not the model output dimensions
    EXPECT_EQ(map.source_width, w);
    EXPECT_EQ(map.source_height, h);
}

TEST_F(DAv2ModelTest, DepthValuesPositiveAndBounded) {
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    // Use a gradient frame (not uniform) to ensure depth variation in output
    constexpr uint32_t   w = 320, h = 240, c = 3;
    std::vector<uint8_t> frame(w * h * c);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t idx     = (y * w + x) * c;
            frame[idx + 0] = static_cast<uint8_t>(x % 256);
            frame[idx + 1] = static_cast<uint8_t>(y % 256);
            frame[idx + 2] = 128;
        }
    }

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();

    const auto& map   = result.value();
    float       min_d = map.data[0];
    float       max_d = map.data[0];
    for (float d : map.data) {
        EXPECT_GE(d, 0.1f) << "Depth must be >= 0.1m";
        EXPECT_LE(d, 20.0f) << "Depth must be <= max_depth_m (20.0)";
        min_d = std::min(min_d, d);
        max_d = std::max(max_d, d);
    }

    // Non-uniform input must produce meaningful depth variation — the linear mapping
    // should spread values across [0.1, max_depth], not collapse to a narrow range.
    EXPECT_LT(min_d, max_d) << "Depth map should have variation on non-uniform input";
    EXPECT_GT(max_d - min_d, 1.0f) << "Depth range should span at least 1m on gradient input";
}

TEST_F(DAv2ModelTest, KnownSceneLeftRightDepthDiffers) {
    // "Known scene" golden test: a half-black / half-white frame should produce
    // substantially different mean depths for each half. This catches conversion
    // formula bugs (e.g., normalization that collapses output to a single value)
    // at the unit test level without needing Gazebo.
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    constexpr uint32_t   w = 320, h = 240, c = 3;
    std::vector<uint8_t> frame(w * h * c);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t  idx    = (y * w + x) * c;
            uint8_t val    = (x < w / 2) ? 20 : 235;  // Dark left, bright right
            frame[idx + 0] = val;
            frame[idx + 1] = val;
            frame[idx + 2] = val;
        }
    }

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();

    const auto& map   = result.value();
    uint32_t    mid_x = map.width / 2;
    double      sum_l = 0.0;
    double      sum_r = 0.0;
    uint32_t    cnt_l = 0;
    uint32_t    cnt_r = 0;

    for (uint32_t y = 0; y < map.height; ++y) {
        for (uint32_t x = 0; x < map.width; ++x) {
            float d = map.data[y * map.width + x];
            if (x < mid_x) {
                sum_l += d;
                ++cnt_l;
            } else {
                sum_r += d;
                ++cnt_r;
            }
        }
    }

    ASSERT_GT(cnt_l, 0u);
    ASSERT_GT(cnt_r, 0u);
    float mean_l = static_cast<float>(sum_l / cnt_l);
    float mean_r = static_cast<float>(sum_r / cnt_r);

    // The two halves must have meaningfully different mean depths.
    // Exact values depend on the model, but 2m difference is conservative.
    float diff = std::abs(mean_l - mean_r);
    EXPECT_GT(diff, 2.0f) << "Half-black/half-white scene should produce >2m mean depth "
                             "difference between halves (got left="
                          << mean_l << ", right=" << mean_r << ", diff=" << diff << ")";
}

TEST_F(DAv2ModelTest, RGBAFrameWorks) {
    DepthAnythingV2Estimator estimator(g_model_path, 518, 20.0f);

    constexpr uint32_t   w = 320, h = 240, c = 4;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok()) << result.error();
    EXPECT_GT(result.value().data.size(), 0u);
}

TEST_F(DAv2ModelTest, MaxDepthConfigAffectsOutput) {
    // Same frame, different max_depth — output range should differ
    constexpr uint32_t   w = 320, h = 240, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    DepthAnythingV2Estimator est_20(g_model_path, 518, 20.0f);
    DepthAnythingV2Estimator est_50(g_model_path, 518, 50.0f);

    auto r20 = est_20.estimate(frame.data(), w, h, c);
    auto r50 = est_50.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(r20.is_ok());
    ASSERT_TRUE(r50.is_ok());

    // Max depth in map with max_depth=20 should be <= 20
    float max_20 = *std::max_element(r20.value().data.begin(), r20.value().data.end());
    float max_50 = *std::max_element(r50.value().data.begin(), r50.value().data.end());

    EXPECT_LE(max_20, 20.0f);
    EXPECT_LE(max_50, 50.0f);
}
