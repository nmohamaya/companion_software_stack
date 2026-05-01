// tests/test_fastsam_backend.cpp
//
// Unit tests for FastSamInferenceBackend (Issue #626).
//
// Coverage:
//   - Default construction + name()
//   - init() with empty / non-existent model path → returns false gracefully
//   - infer() input validation: null frame, zero dimensions → R::err
//   - infer() when model is not loaded → R::ok with 0 detections (graceful no-op)
//   - infer() timestamp is populated on every valid call
//   - Non-copyable / movable type-trait assertions
//   - HAL factory creation: backend="fastsam" creates FastSamInferenceBackend
//   - Config-driven construction: thresholds, input_size, mask_channels clamped
//   - use_cuda=true with missing model → graceful failure (not a crash)
//
// Tests run on CPU-only builds (HAS_OPENCV undefined) as well as OpenCV builds.
// In both cases the model is not present on disk, so the OpenCV path exercises
// the "model file missing" graceful failure rather than real inference.

#include "hal/fastsam_inference_backend.h"
#include "hal/hal_factory.h"
#include "util/config.h"

#include <cstdio>
#include <fstream>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::hal;

// ── Temp config helper ────────────────────────────────────────────────────────

static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_fastsam_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_fastsam_" + std::to_string(getpid()) + ".json";
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

// ── Type-trait assertions ─────────────────────────────────────────────────────

// FastSamInferenceBackend must not be copyable (holds cv::dnn::Net resource)
// but must be movable (factory returns unique_ptr, stored by value in process code).
static_assert(!std::is_copy_constructible_v<FastSamInferenceBackend>,
              "FastSamInferenceBackend must not be copy-constructible");
static_assert(!std::is_copy_assignable_v<FastSamInferenceBackend>,
              "FastSamInferenceBackend must not be copy-assignable");
static_assert(std::is_move_constructible_v<FastSamInferenceBackend>,
              "FastSamInferenceBackend must be move-constructible");
static_assert(std::is_move_assignable_v<FastSamInferenceBackend>,
              "FastSamInferenceBackend must be move-assignable");

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST(FastSamBackend, DefaultConstructionName) {
    FastSamInferenceBackend backend;
    EXPECT_EQ(backend.name(), "FastSamInferenceBackend");
}

// init() with an empty model path must return false and not crash.
// This tests the early-exit guard in load_model() that runs on both
// HAS_OPENCV and non-HAS_OPENCV builds.
TEST(FastSamBackend, InitEmptyModelPathReturnsFalse) {
    FastSamInferenceBackend backend;
    EXPECT_FALSE(backend.init("", 1024));
}

// init() with a path that does not exist on disk must return false gracefully.
TEST(FastSamBackend, InitNonexistentModelReturnsFalse) {
    FastSamInferenceBackend backend;
    EXPECT_FALSE(backend.init("/nonexistent/path/fastsam_s.onnx", 1024));
}

// Passing a null frame pointer must always yield R::err — this guard runs
// before any #ifdef HAS_OPENCV block, so it is backend-independent.
TEST(FastSamBackend, InferNullFrameReturnsError) {
    FastSamInferenceBackend backend;
    auto                    result = backend.infer(nullptr, 640, 480, 3, 0);
    EXPECT_TRUE(result.is_err());
}

// Zero width → R::err.
TEST(FastSamBackend, InferZeroWidthReturnsError) {
    FastSamInferenceBackend backend;
    std::vector<uint8_t>    frame(480 * 3, 128);
    auto                    result = backend.infer(frame.data(), 0, 480, 3, 0);
    EXPECT_TRUE(result.is_err());
}

// Zero height → R::err.
TEST(FastSamBackend, InferZeroHeightReturnsError) {
    FastSamInferenceBackend backend;
    std::vector<uint8_t>    frame(640 * 3, 128);
    auto                    result = backend.infer(frame.data(), 640, 0, 3, 0);
    EXPECT_TRUE(result.is_err());
}

// Zero channels → R::err.
TEST(FastSamBackend, InferZeroChannelsReturnsError) {
    FastSamInferenceBackend backend;
    std::vector<uint8_t>    frame(640 * 480, 128);
    auto                    result = backend.infer(frame.data(), 640, 480, 0, 0);
    EXPECT_TRUE(result.is_err());
}

// When the model was never loaded (or the model file is absent), infer() on a
// valid frame must return R::ok with 0 detections — not an error.  The backend
// must degrade silently so P2 SAM thread keeps running without crashing the
// MaskDepthProjector pipeline when fastsam_s.onnx is missing.
TEST(FastSamBackend, InferWhenModelNotLoadedReturnsEmptyOk) {
    FastSamInferenceBackend backend;
    // Deliberately skip init() — model_loaded_ stays false.
    std::vector<uint8_t> frame(640 * 480 * 3, 128);
    auto                 result = backend.infer(frame.data(), 640, 480, 3, 0);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().detections.empty());
}

// Even on a no-op (model absent), the timestamp field must be non-zero so that
// downstream consumers can track freshness and detect stale frames.
TEST(FastSamBackend, InferTimestampPopulatedOnNoOp) {
    FastSamInferenceBackend backend;
    std::vector<uint8_t>    frame(320 * 240 * 3, 64);
    auto                    result = backend.infer(frame.data(), 320, 240, 3, 0);
    ASSERT_TRUE(result.is_ok());
    EXPECT_GT(result.value().timestamp_ns, uint64_t{0});
}

// After a failed init(), a subsequent infer() with valid data must still work
// (return R::ok with 0 detections).  Tests that the internal state is coherent
// after the failure path.
TEST(FastSamBackend, InferAfterFailedInitReturnsEmptyOk) {
    FastSamInferenceBackend backend;
    EXPECT_FALSE(backend.init("/no/such/model.onnx", 1024));

    std::vector<uint8_t> frame(640 * 480 * 3, 200);
    auto                 result = backend.infer(frame.data(), 640, 480, 3, 0);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().detections.empty());
}

// ── HAL factory tests ─────────────────────────────────────────────────────────

// Factory must return a FastSamInferenceBackend when the config requests
// backend="fastsam".  Uses the HAL factory create_inference_backend() path.
TEST(FastSamBackend, HalFactoryCreation) {
    auto          path = create_temp_config(R"({
        "perception": {
            "path_a": {
                "sam": {
                    "backend": "fastsam",
                    "model_path": "models/fastsam_s.onnx"
                }
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto backend = drone::hal::create_inference_backend(cfg, "perception.path_a.sam");
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->name(), "FastSamInferenceBackend");
}

// ── Config-driven construction tests ─────────────────────────────────────────

// Default config (no keys set) must construct without crashing and leave the
// backend in a known state (model not loaded because the default ONNX path
// does not exist in the test environment).
TEST(FastSamBackend, ConfigDefaultsDoNotCrash) {
    auto          path = create_temp_config(R"({
        "perception": { "path_a": { "sam": { "backend": "fastsam" } } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
    EXPECT_EQ(backend.name(), "FastSamInferenceBackend");
    // The default model path ("models/fastsam_s.onnx") won't exist, so init
    // returns false.  That is the expected graceful-failure path.
    EXPECT_FALSE(backend.init("", 0));  // 0/empty → use stored defaults
}

// use_cuda=true in config must be accepted without crashing, even when no GPU
// is present or the model is absent.  The canary fallback must degrade cleanly.
TEST(FastSamBackend, ConfigUseCudaTrueNoModelNoOp) {
    auto          path = create_temp_config(R"({
        "perception": {
            "path_a": {
                "sam": {
                    "backend": "fastsam",
                    "use_cuda": true,
                    "model_path": "/nonexistent/fastsam_s.onnx"
                }
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
    // init() will fail (model absent) — but must NOT crash or throw.
    EXPECT_FALSE(backend.init("", 0));
    // Subsequent infer() must still be safe (graceful no-op).
    std::vector<uint8_t> frame(320 * 240 * 3, 0);
    auto                 result = backend.infer(frame.data(), 320, 240, 3, 0);
    ASSERT_TRUE(result.is_ok());
    EXPECT_TRUE(result.value().detections.empty());
}

// PR #633 P2 review: previous version asserted name() (proves nothing).
// Now asserts the actually-clamped values via the new accessors.
TEST(FastSamBackend, ConfigClampsConfidenceThresholdToZeroOne) {
    auto          path = create_temp_config(R"({
        "perception": {
            "path_a": {
                "sam": {
                    "confidence_threshold": 5.0,
                    "nms_threshold": -1.0
                }
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
    EXPECT_FLOAT_EQ(backend.confidence_threshold(), 1.0f)
        << "confidence_threshold=5.0 must clamp to 1.0";
    EXPECT_FLOAT_EQ(backend.nms_threshold(), 0.0f) << "nms_threshold=-1.0 must clamp to 0.0";
}

TEST(FastSamBackend, ConfigClampsInputSizeBelowMin) {
    for (int extreme : {0, 1, 50, 127}) {
        auto path = create_temp_config(R"({"perception": {"path_a": {"sam": {"input_size": )" +
                                       std::to_string(extreme) + R"(}}}})");
        drone::Config cfg;
        ASSERT_TRUE(cfg.load(path)) << "failed for input_size=" << extreme;
        FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
        EXPECT_EQ(backend.input_size(), 128) << "input_size=" << extreme << " must clamp up to 128";
    }
}

TEST(FastSamBackend, ConfigClampsInputSizeAboveMax) {
    for (int extreme : {2049, 99999}) {
        auto path = create_temp_config(R"({"perception": {"path_a": {"sam": {"input_size": )" +
                                       std::to_string(extreme) + R"(}}}})");
        drone::Config cfg;
        ASSERT_TRUE(cfg.load(path)) << "failed for input_size=" << extreme;
        FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
        EXPECT_EQ(backend.input_size(), 2048)
            << "input_size=" << extreme << " must clamp down to 2048";
    }
}

TEST(FastSamBackend, ConfigClampsMaskChannelsToValidRange) {
    {  // below min
        auto path =
            create_temp_config(R"({"perception": {"path_a": {"sam": {"mask_channels": -1}}}})");
        drone::Config cfg;
        ASSERT_TRUE(cfg.load(path));
        FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
        EXPECT_EQ(backend.mask_channels(), 1) << "mask_channels=-1 must clamp up to 1";
    }
    {  // above max
        auto path =
            create_temp_config(R"({"perception": {"path_a": {"sam": {"mask_channels": 9999}}}})");
        drone::Config cfg;
        ASSERT_TRUE(cfg.load(path));
        FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
        EXPECT_EQ(backend.mask_channels(), 128) << "mask_channels=9999 must clamp down to 128";
    }
}

// PR #633 P2 review: validate use_cuda config wiring (was untested).
// FastSAM reads from the canonical SAM_USE_CUDA constant
// (perception.path_a.sam.use_cuda) — same key the docstring claims.
TEST(FastSamBackend, ConfigUseCudaTrueIsRead) {
    auto          path = create_temp_config(R"({
        "perception": {
            "path_a": {
                "sam": {
                    "backend": "fastsam",
                    "use_cuda": true,
                    "model_path": "models/nonexistent.onnx"
                }
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
    EXPECT_TRUE(backend.use_cuda_configured());
}

TEST(FastSamBackend, ConfigUseCudaDefaultsFalse) {
    auto          path = create_temp_config(R"({
        "perception": {
            "path_a": {
                "sam": {
                    "backend": "fastsam",
                    "model_path": "models/nonexistent.onnx"
                }
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    FastSamInferenceBackend backend(cfg, "perception.path_a.sam");
    EXPECT_FALSE(backend.use_cuda_configured())
        << "default must be false to preserve CPU-only legacy behaviour";
}

// PR #633 P2 review: path traversal guard.  init() with a path
// containing '..' must fail gracefully (not a crash, not a load).
TEST(FastSamBackend, InitRejectsPathTraversal) {
    FastSamInferenceBackend backend;
    EXPECT_FALSE(backend.init("models/../../../etc/passwd", 0))
        << "model paths containing '..' must be rejected";
}

// PR #633 P2 review: malformed CV_Assert removal — the detection-output
// type-check used to abort the SAM thread via CV_Assert on an unexpected
// type.  Now returns Err so the caller can degrade gracefully.  We can't
// trigger this without a real ONNX run, but we can pin the contract:
// infer() with no model loaded must always return ok with empty
// detections, never throw.
TEST(FastSamBackend, InferOnUnloadedReturnsOkEmpty) {
    FastSamInferenceBackend backend;
    std::vector<uint8_t>    frame(320 * 240 * 3, 0);
    auto                    result = backend.infer(frame.data(), 320, 240, 3, 0);
    ASSERT_TRUE(result.is_ok())
        << "infer() with unloaded model must return ok (graceful no-op), not throw";
    EXPECT_TRUE(result.value().detections.empty());
}
