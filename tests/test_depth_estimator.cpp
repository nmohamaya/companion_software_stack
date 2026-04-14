// tests/test_depth_estimator.cpp
// Unit tests for IDepthEstimator, SimulatedDepthEstimator, and UKF depth map integration.
// Issue #430 — ML depth estimation infrastructure.
#include "hal/idepth_estimator.h"
#include "hal/simulated_depth_estimator.h"
#include "perception/fusion_engine.h"
#include "perception/ukf_fusion_engine.h"
#include "util/config.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::hal;

// ═══════════════════════════════════════════════════════════
// SimulatedDepthEstimator tests
// ═══════════════════════════════════════════════════════════

TEST(SimulatedDepthEstimatorTest, NameReturnsExpected) {
    SimulatedDepthEstimator estimator;
    EXPECT_EQ(estimator.name(), "SimulatedDepthEstimator");
}

TEST(SimulatedDepthEstimatorTest, DefaultConstruction) {
    SimulatedDepthEstimator estimator;  // 10.0m depth, 0.5m noise

    // Create a small test frame
    constexpr uint32_t   w = 4, h = 4, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok());

    auto& map = result.value();
    EXPECT_EQ(map.width, w);
    EXPECT_EQ(map.height, h);
    EXPECT_EQ(map.data.size(), static_cast<size_t>(w * h));
    EXPECT_FLOAT_EQ(map.scale, 1.0f);
    EXPECT_FLOAT_EQ(map.confidence, 0.5f);

    // All depths should be roughly around 10.0m (±noise)
    for (float d : map.data) {
        EXPECT_GT(d, 0.1f);   // Clamped minimum
        EXPECT_LT(d, 20.0f);  // Reasonable upper bound with 0.5 std noise
    }
}

TEST(SimulatedDepthEstimatorTest, ExplicitParameters) {
    SimulatedDepthEstimator estimator(5.0f, 0.0f);  // 5m, zero noise

    constexpr uint32_t   w = 2, h = 2, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok());

    auto& map = result.value();
    // With zero noise, all pixels should be exactly 5.0m
    for (float d : map.data) {
        EXPECT_FLOAT_EQ(d, 5.0f);
    }
}

TEST(SimulatedDepthEstimatorTest, ConfigConstruction) {
    // Write a temp config file
    std::string path = "/tmp/test_depth_" + std::to_string(getpid()) + ".json";
    {
        std::ofstream ofs(path);
        ofs << R"({
            "perception": {
                "depth_estimator": {
                    "backend": "simulated",
                    "default_depth_m": 15.0,
                    "noise_std_m": 0.0
                }
            }
        })";
    }

    drone::Config cfg;
    cfg.load(path);
    SimulatedDepthEstimator estimator(cfg, "perception.depth_estimator");

    constexpr uint32_t   w = 2, h = 2, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok());

    for (float d : result.value().data) {
        EXPECT_FLOAT_EQ(d, 15.0f);
    }

    std::remove(path.c_str());
}

TEST(SimulatedDepthEstimatorTest, NullFrameReturnsError) {
    SimulatedDepthEstimator estimator;
    auto                    result = estimator.estimate(nullptr, 640, 480, 3);
    EXPECT_FALSE(result.is_ok());
}

TEST(SimulatedDepthEstimatorTest, ZeroDimensionsReturnError) {
    SimulatedDepthEstimator estimator;
    std::vector<uint8_t>    frame(100, 128);

    auto r1 = estimator.estimate(frame.data(), 0, 480, 3);
    EXPECT_FALSE(r1.is_ok());

    auto r2 = estimator.estimate(frame.data(), 640, 0, 3);
    EXPECT_FALSE(r2.is_ok());
}

TEST(SimulatedDepthEstimatorTest, ZeroChannelsReturnsError) {
    SimulatedDepthEstimator estimator;
    std::vector<uint8_t>    frame(100, 128);

    auto result = estimator.estimate(frame.data(), 10, 10, 0);
    EXPECT_FALSE(result.is_ok());
}

TEST(SimulatedDepthEstimatorTest, SingleChannelFrame) {
    SimulatedDepthEstimator estimator(8.0f, 0.0f);

    constexpr uint32_t   w = 3, h = 3, c = 1;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().data.size(), 9u);
}

TEST(SimulatedDepthEstimatorTest, RGBAFrame) {
    SimulatedDepthEstimator estimator(8.0f, 0.0f);

    constexpr uint32_t   w = 2, h = 2, c = 4;
    std::vector<uint8_t> frame(w * h * c, 128);

    auto result = estimator.estimate(frame.data(), w, h, c);
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().data.size(), 4u);
}

TEST(SimulatedDepthEstimatorTest, DepthsClamped) {
    // Very low default depth + high noise could produce negative values
    // Verify they are clamped to 0.1f minimum
    SimulatedDepthEstimator estimator(0.2f, 1.0f);

    constexpr uint32_t   w = 8, h = 8, c = 3;
    std::vector<uint8_t> frame(w * h * c, 128);

    // Run several times to hit edge cases
    for (int i = 0; i < 10; ++i) {
        auto result = estimator.estimate(frame.data(), w, h, c);
        ASSERT_TRUE(result.is_ok());
        for (float d : result.value().data) {
            EXPECT_GE(d, 0.1f) << "Depth must be clamped to >= 0.1";
        }
    }
}

// ═══════════════════════════════════════════════════════════
// DepthMap struct tests
// ═══════════════════════════════════════════════════════════

TEST(DepthMapTest, DefaultConstruction) {
    DepthMap map;
    EXPECT_TRUE(map.data.empty());
    EXPECT_EQ(map.width, 0u);
    EXPECT_EQ(map.height, 0u);
    EXPECT_EQ(map.timestamp_ns, 0u);
    EXPECT_FLOAT_EQ(map.scale, 1.0f);
    EXPECT_FLOAT_EQ(map.confidence, 0.0f);
}

TEST(DepthMapTest, MoveSemantics) {
    DepthMap a;
    a.data   = {1.0f, 2.0f, 3.0f, 4.0f};
    a.width  = 2;
    a.height = 2;

    DepthMap b = std::move(a);
    EXPECT_EQ(b.data.size(), 4u);
    EXPECT_EQ(b.width, 2u);
    EXPECT_FLOAT_EQ(b.data[0], 1.0f);
}

// ═══════════════════════════════════════════════════════════
// UKF depth map integration tests
// ═══════════════════════════════════════════════════════════

static drone::perception::CalibrationData make_depth_test_calib() {
    drone::perception::CalibrationData calib;
    calib.camera_intrinsics       = Eigen::Matrix3f::Identity();
    calib.camera_intrinsics(0, 0) = 500.0f;  // fx
    calib.camera_intrinsics(1, 1) = 500.0f;  // fy
    calib.camera_intrinsics(0, 2) = 320.0f;  // cx
    calib.camera_intrinsics(1, 2) = 240.0f;  // cy
    calib.camera_height_m         = 1.5f;
    return calib;
}

TEST(UKFDepthFusionTest, SetDepthMapStoresData) {
    auto                               calib = make_depth_test_calib();
    drone::perception::UKFFusionEngine engine(calib);

    DepthMap depth;
    depth.width  = 64;
    depth.height = 48;
    depth.scale  = 1.0f;
    depth.data.assign(64 * 48, 8.0f);

    // set_depth_map should not throw
    engine.set_depth_map(depth);
}

TEST(UKFDepthFusionTest, EmptyDepthMapIsIgnored) {
    auto                               calib = make_depth_test_calib();
    drone::perception::UKFFusionEngine engine(calib);

    DepthMap empty;
    engine.set_depth_map(empty);  // Should not crash

    // Fuse with empty tracked list — no crash
    drone::perception::TrackedObjectList tracked;
    tracked.timestamp_ns = 1000;
    auto result          = engine.fuse(tracked);
    EXPECT_TRUE(result.objects.empty());
}

TEST(UKFDepthFusionTest, DepthMapUsedInFusion) {
    // Verify ML depth map is actually used in Tier 3.5 of estimate_depth().
    // Force past Tiers 1-3 by:
    //   - bbox_h = 0 → skips Tier 1 (needs bbox_h > 10) and Tier 2 (same guard)
    //   - position_2d.y() = cy (240) → ray_down = 0 → skips Tier 3
    // This lands in Tier 3.5 which reads the ML depth map.
    auto calib        = make_depth_test_calib();
    calib.depth_scale = 1.0f;  // Disable conservative scaling so ML depth passes through unscaled
    drone::perception::UKFFusionEngine engine(calib);

    // Create depth map with uniform known depth
    constexpr uint32_t dw = 64, dh = 48;
    constexpr float    known_depth = 12.0f;
    DepthMap           depth;
    depth.width  = dw;
    depth.height = dh;
    depth.scale  = 1.0f;
    depth.data.assign(dw * dh, known_depth);
    engine.set_depth_map(std::move(depth));

    // Create a tracked object at the horizon with zero bbox height
    drone::perception::TrackedObjectList tracked;
    tracked.timestamp_ns   = 1000;
    tracked.frame_sequence = 1;

    drone::perception::TrackedObject obj;
    obj.track_id     = 1;
    obj.class_id     = drone::perception::ObjectClass::PERSON;
    obj.confidence   = 0.8f;
    obj.position_2d  = {320.0f, 240.0f};  // At cy (horizon) — ray_down = 0
    obj.velocity_2d  = {0.0f, 0.0f};
    obj.timestamp_ns = 1000;
    obj.bbox_h       = 0.0f;  // Zero height — forces past Tier 1 & 2
    tracked.objects.push_back(obj);

    auto result = engine.fuse(tracked);

    ASSERT_EQ(result.objects.size(), 1u);
    // Fused depth (position_3d.x() in body-frame FRD) should be near the
    // ML depth of 12m. UKF initial state uses this depth directly, so the
    // first fuse() output should be close. Allow ±3m for UKF initialization.
    EXPECT_NEAR(result.objects[0].position_3d.x(), known_depth, 3.0f);
    EXPECT_TRUE(std::isfinite(result.objects[0].position_3d.x()));
}

TEST(UKFDepthFusionTest, ResetClearsDepthMap) {
    auto                               calib = make_depth_test_calib();
    drone::perception::UKFFusionEngine engine(calib);

    DepthMap depth;
    depth.width  = 32;
    depth.height = 32;
    depth.scale  = 1.0f;
    depth.data.assign(32 * 32, 5.0f);

    engine.set_depth_map(depth);
    engine.reset();

    // After reset, fusing should work without using the cleared depth map
    drone::perception::TrackedObjectList tracked;
    tracked.timestamp_ns = 2000;
    auto result          = engine.fuse(tracked);
    EXPECT_TRUE(result.objects.empty());
}
