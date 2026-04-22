// tests/test_inference_backend.cpp
// Unit tests for IInferenceBackend HAL interface + SimulatedInferenceBackend.
#include "hal/hal_factory.h"
#include "hal/iinference_backend.h"
#include "hal/simulated_inference_backend.h"
#include "util/config.h"

#include <cstdio>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::hal;

// ── Temp config helper ──

static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_infer_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_infer_" + std::to_string(getpid()) + ".json";
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

TEST(InferenceBackend, SimulatedName) {
    SimulatedInferenceBackend backend;
    EXPECT_EQ(backend.name(), "SimulatedInferenceBackend");
}

TEST(InferenceBackend, InitReturnsTrue) {
    SimulatedInferenceBackend backend;
    EXPECT_TRUE(backend.init("dummy_model.onnx", 640));
}

TEST(InferenceBackend, InferReturnsDetections) {
    SimulatedInferenceBackend backend(3);
    ASSERT_TRUE(backend.init("model.onnx", 640));

    std::vector<uint8_t> frame(640 * 480 * 3, 128);
    auto                 result = backend.infer(frame.data(), 640, 480, 3, 0);

    ASSERT_TRUE(result.is_ok());
    const auto& output = result.value();
    EXPECT_EQ(output.detections.size(), 3u);

    for (const auto& det : output.detections) {
        EXPECT_GE(det.confidence, 0.0f);
        EXPECT_LE(det.confidence, 1.0f);
        EXPECT_GT(det.bbox.w, 0.0f);
        EXPECT_GT(det.bbox.h, 0.0f);
    }
}

TEST(InferenceBackend, NullFrameReturnsError) {
    SimulatedInferenceBackend backend;
    auto                      result = backend.infer(nullptr, 640, 480, 3, 0);
    EXPECT_TRUE(result.is_err());
}

TEST(InferenceBackend, ZeroDimensionsReturnsError) {
    SimulatedInferenceBackend backend;
    std::vector<uint8_t>      frame(100, 0);
    auto                      result = backend.infer(frame.data(), 0, 0, 3, 0);
    EXPECT_TRUE(result.is_err());
}

TEST(InferenceBackend, FactorySimulated) {
    auto          path = create_temp_config(R"({
        "perception": { "inference_backend": { "backend": "simulated" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto backend = drone::hal::create_inference_backend(cfg);
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->name(), "SimulatedInferenceBackend");
}

TEST(InferenceBackend, FactoryUnknownThrows) {
    auto          path = create_temp_config(R"({
        "perception": { "inference_backend": { "backend": "nonexistent" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_inference_backend(cfg), std::runtime_error);
}
