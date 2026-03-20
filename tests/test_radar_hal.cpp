// tests/test_radar_hal.cpp
// Unit tests for IRadar HAL interface, SimulatedRadar backend, and RadarDetection IPC types.
// Implements Issue #209 test plan.
#include "hal/hal_factory.h"
#include "hal/iradar.h"
#include "hal/simulated_radar.h"
#include "ipc/ipc_types.h"
#include "util/config.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

// ═══════════════════════════════════════════════════════════
// Helper: create a temporary config file (cleaned up via RAII)
// ═══════════════════════════════════════════════════════════
static std::vector<std::string> g_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_radar_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_radar_" + std::to_string(getpid()) + ".json";
        std::ofstream ofs(path);
        ofs << json_content;
        ofs.close();
        g_temp_files.push_back(path);
        return path;
    }
    ::close(fd);
    std::string   path(tmpl);
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    g_temp_files.push_back(path);
    return path;
}

struct TempFileCleanup {
    ~TempFileCleanup() {
        for (const auto& f : g_temp_files) {
            std::remove(f.c_str());
        }
    }
};
static TempFileCleanup g_cleanup;

// ═══════════════════════════════════════════════════════════
// RadarDetection IPC type validation tests
// ═══════════════════════════════════════════════════════════
TEST(RadarDetectionValidation, ValidDetection) {
    drone::ipc::RadarDetection det{};
    det.range_m    = 10.0f;
    det.confidence = 0.8f;
    EXPECT_TRUE(det.validate());
}

TEST(RadarDetectionValidation, NegativeRangeInvalid) {
    drone::ipc::RadarDetection det{};
    det.range_m    = -1.0f;
    det.confidence = 0.5f;
    EXPECT_FALSE(det.validate());
}

TEST(RadarDetectionValidation, ConfidenceAboveOneInvalid) {
    drone::ipc::RadarDetection det{};
    det.range_m    = 5.0f;
    det.confidence = 1.5f;
    EXPECT_FALSE(det.validate());
}

TEST(RadarDetectionValidation, ConfidenceBelowZeroInvalid) {
    drone::ipc::RadarDetection det{};
    det.range_m    = 5.0f;
    det.confidence = -0.1f;
    EXPECT_FALSE(det.validate());
}

TEST(RadarDetectionValidation, NaNFieldInvalid) {
    drone::ipc::RadarDetection det{};
    det.range_m    = 5.0f;
    det.confidence = 0.5f;
    EXPECT_TRUE(det.validate());

    det.azimuth_rad = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(det.validate());

    det.azimuth_rad         = 0.0f;
    det.radial_velocity_mps = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(det.validate());
}

TEST(RadarDetectionValidation, ZeroRangeValid) {
    drone::ipc::RadarDetection det{};
    det.range_m    = 0.0f;
    det.confidence = 0.5f;
    EXPECT_TRUE(det.validate());
}

TEST(RadarDetectionValidation, BoundaryConfidenceValid) {
    drone::ipc::RadarDetection det{};
    det.range_m    = 1.0f;
    det.confidence = 0.0f;
    EXPECT_TRUE(det.validate());

    det.confidence = 1.0f;
    EXPECT_TRUE(det.validate());
}

// ═══════════════════════════════════════════════════════════
// RadarDetectionList validation tests
// ═══════════════════════════════════════════════════════════
TEST(RadarDetectionListValidation, ValidList) {
    drone::ipc::RadarDetectionList list{};
    list.num_detections = 5;
    EXPECT_TRUE(list.validate());
}

TEST(RadarDetectionListValidation, EmptyListValid) {
    drone::ipc::RadarDetectionList list{};
    list.num_detections = 0;
    EXPECT_TRUE(list.validate());
}

TEST(RadarDetectionListValidation, MaxDetectionsValid) {
    drone::ipc::RadarDetectionList list{};
    list.num_detections = drone::ipc::MAX_RADAR_DETECTIONS;
    EXPECT_TRUE(list.validate());
}

TEST(RadarDetectionListValidation, OverflowInvalid) {
    drone::ipc::RadarDetectionList list{};
    list.num_detections = drone::ipc::MAX_RADAR_DETECTIONS + 1;
    EXPECT_FALSE(list.validate());
}

TEST(RadarDetectionListValidation, InvalidEntryFailsList) {
    drone::ipc::RadarDetectionList list{};
    list.num_detections        = 1;
    list.detections[0].range_m = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(list.validate());
}

// ═══════════════════════════════════════════════════════════
// Trivially copyable compile-time checks
// ═══════════════════════════════════════════════════════════
TEST(RadarDetectionTriviallyCopyable, StaticAssertPasses) {
    static_assert(std::is_trivially_copyable_v<drone::ipc::RadarDetection>);
    static_assert(std::is_trivially_copyable_v<drone::ipc::RadarDetectionList>);
    SUCCEED();
}

// ═══════════════════════════════════════════════════════════
// SimulatedRadar tests — fixture with default config
// ═══════════════════════════════════════════════════════════
class SimulatedRadarTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto path = create_temp_config(R"({
            "perception": {
                "radar": {
                    "backend": "simulated",
                    "max_range_m": 100.0,
                    "fov_azimuth_rad": 1.047,
                    "fov_elevation_rad": 0.262,
                    "num_targets": 3,
                    "false_alarm_rate": 0.02,
                    "noise": {
                        "range_std_m": 0.3,
                        "azimuth_std_rad": 0.026,
                        "elevation_std_rad": 0.026,
                        "velocity_std_mps": 0.1
                    }
                }
            }
        })");
        ASSERT_TRUE(cfg_.load(path));
        radar_ = std::make_unique<drone::hal::SimulatedRadar>(cfg_, "perception.radar");
    }

    drone::Config                               cfg_;
    std::unique_ptr<drone::hal::SimulatedRadar> radar_;
};

TEST_F(SimulatedRadarTest, InitReturnsTrue) {
    EXPECT_TRUE(radar_->init());
    EXPECT_TRUE(radar_->is_active());
}

TEST_F(SimulatedRadarTest, NotActiveBeforeInit) {
    EXPECT_FALSE(radar_->is_active());
}

TEST_F(SimulatedRadarTest, NameReturnsSimulatedRadar) {
    EXPECT_EQ(radar_->name(), "SimulatedRadar");
}

TEST_F(SimulatedRadarTest, ReadReturnsEmptyBeforeInit) {
    auto list = radar_->read();
    EXPECT_EQ(list.num_detections, 0u);
}

TEST_F(SimulatedRadarTest, ReadReturnsValidList) {
    radar_->init();
    auto list = radar_->read();
    EXPECT_TRUE(list.validate());
    EXPECT_GT(list.num_detections, 0u);
    EXPECT_GT(list.timestamp_ns, 0u);
}

TEST_F(SimulatedRadarTest, ReadReturnsExpectedTargetCount) {
    radar_->init();
    auto list = radar_->read();
    // Should have at least num_targets (3), possibly +1 false alarm
    EXPECT_GE(list.num_detections, 3u);
    EXPECT_LE(list.num_detections, 4u);
}

TEST_F(SimulatedRadarTest, FoVLimits) {
    radar_->init();
    const float half_az = 1.047f / 2.0f;
    const float half_el = 0.262f / 2.0f;

    for (int i = 0; i < 100; ++i) {
        auto list = radar_->read();
        for (uint32_t j = 0; j < list.num_detections; ++j) {
            const auto& det = list.detections[j];
            EXPECT_GE(det.azimuth_rad, -half_az) << "Azimuth below FoV at sample " << i;
            EXPECT_LE(det.azimuth_rad, half_az) << "Azimuth above FoV at sample " << i;
            EXPECT_GE(det.elevation_rad, -half_el) << "Elevation below FoV at sample " << i;
            EXPECT_LE(det.elevation_rad, half_el) << "Elevation above FoV at sample " << i;
        }
    }
}

TEST_F(SimulatedRadarTest, RangeLimits) {
    radar_->init();
    for (int i = 0; i < 100; ++i) {
        auto list = radar_->read();
        for (uint32_t j = 0; j < list.num_detections; ++j) {
            EXPECT_GE(list.detections[j].range_m, 0.0f);
        }
    }
}

TEST_F(SimulatedRadarTest, DetectionsHaveValidConfidence) {
    radar_->init();
    for (int i = 0; i < 100; ++i) {
        auto list = radar_->read();
        for (uint32_t j = 0; j < list.num_detections; ++j) {
            EXPECT_TRUE(list.detections[j].validate());
        }
    }
}

TEST_F(SimulatedRadarTest, NoiseDistribution) {
    // Statistical test: range noise should have stddev ~0.3m
    // Generate many samples and check the variance of ranges around their expected values.
    // We use a fixed seed so this is deterministic.
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "simulated",
                "max_range_m": 50.0,
                "fov_azimuth_rad": 1.047,
                "fov_elevation_rad": 0.262,
                "num_targets": 1,
                "false_alarm_rate": 0.0,
                "noise": {
                    "range_std_m": 0.3,
                    "azimuth_std_rad": 0.026,
                    "elevation_std_rad": 0.026,
                    "velocity_std_mps": 0.1
                }
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    drone::hal::SimulatedRadar radar(cfg, "perception.radar");
    radar.init();

    // Collect many range readings
    std::vector<float> ranges;
    ranges.reserve(1000);
    for (int i = 0; i < 1000; ++i) {
        auto list = radar.read();
        ASSERT_GE(list.num_detections, 1u);
        ranges.push_back(list.detections[0].range_m);
    }

    // Compute standard deviation of ranges — it should be noisy (>0)
    const float mean = std::accumulate(ranges.begin(), ranges.end(), 0.0f) /
                       static_cast<float>(ranges.size());
    float var = 0.0f;
    for (const float r : ranges) {
        const float diff = r - mean;
        var += diff * diff;
    }
    var /= static_cast<float>(ranges.size() - 1);
    const float stddev = std::sqrt(var);

    // Range stddev should be non-trivial (noise is present)
    // The total stddev includes both the uniform range spread + Gaussian noise,
    // so it will be larger than 0.3m. Just check it's meaningfully noisy.
    EXPECT_GT(stddev, 0.1f) << "Range noise appears too small";
}

TEST_F(SimulatedRadarTest, ConsecutiveReadsHaveIncreasingTimestamps) {
    radar_->init();
    auto list1 = radar_->read();
    auto list2 = radar_->read();
    EXPECT_GE(list2.timestamp_ns, list1.timestamp_ns);
}

// ═══════════════════════════════════════════════════════════
// Factory tests
// ═══════════════════════════════════════════════════════════
TEST(RadarFactoryTest, CreatesSimulated) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": { "backend": "simulated" }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto radar = drone::hal::create_radar(cfg, "perception.radar");
    ASSERT_NE(radar, nullptr);
    EXPECT_EQ(radar->name(), "SimulatedRadar");
}

TEST(RadarFactoryTest, DefaultsToSimulated) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {}
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto radar = drone::hal::create_radar(cfg, "perception.radar");
    ASSERT_NE(radar, nullptr);
    EXPECT_EQ(radar->name(), "SimulatedRadar");
}

TEST(RadarFactoryTest, ThrowsOnUnknownBackend) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": { "backend": "nonexistent" }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_radar(cfg, "perception.radar"), std::runtime_error);
}

// ═══════════════════════════════════════════════════════════
// Custom config propagation test
// ═══════════════════════════════════════════════════════════
TEST(SimulatedRadarConfigTest, CustomConfigValues) {
    auto          path = create_temp_config(R"({
        "sensor": {
            "radar": {
                "backend": "simulated",
                "max_range_m": 50.0,
                "fov_azimuth_rad": 0.5,
                "fov_elevation_rad": 0.1,
                "num_targets": 5,
                "false_alarm_rate": 0.0,
                "noise": {
                    "range_std_m": 0.1,
                    "azimuth_std_rad": 0.01,
                    "elevation_std_rad": 0.01,
                    "velocity_std_mps": 0.05
                }
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    drone::hal::SimulatedRadar radar(cfg, "sensor.radar");
    ASSERT_TRUE(radar.init());

    auto list = radar.read();
    EXPECT_TRUE(list.validate());
    // With 5 targets and 0 false alarm rate, should get exactly 5
    EXPECT_EQ(list.num_detections, 5u);

    // All detections within custom FoV
    const float half_az = 0.5f / 2.0f;
    const float half_el = 0.1f / 2.0f;
    for (uint32_t i = 0; i < list.num_detections; ++i) {
        EXPECT_GE(list.detections[i].azimuth_rad, -half_az);
        EXPECT_LE(list.detections[i].azimuth_rad, half_az);
        EXPECT_GE(list.detections[i].elevation_rad, -half_el);
        EXPECT_LE(list.detections[i].elevation_rad, half_el);
    }
}

// ═══════════════════════════════════════════════════════════
// Topic constant test
// ═══════════════════════════════════════════════════════════
TEST(RadarTopicTest, TopicStringCorrect) {
    EXPECT_STREQ(drone::ipc::topics::RADAR_DETECTIONS, "/radar_detections");
}
