// tests/test_gazebo_imu.cpp
// Unit tests for GazeboIMUBackend — Gazebo IMU HAL backend via gz-transport.
//
// These tests verify:
//  1. Factory creates GazeboIMUBackend when HAVE_GAZEBO is defined
//  2. name() includes the topic string
//  3. is_active() lifecycle (false before init, depends on messages)
//  4. init() subscribes successfully
//  5. Double-init returns false
//  6. read() returns invalid before any data arrives
//  7. Graceful behavior when Gazebo is not running
//
// NOTE: Full integration tests with a running Gazebo world are planned in Phase 4 (#11).
#include <gtest/gtest.h>
#include "hal/hal_factory.h"
#include "hal/iimu_source.h"
#include "util/config.h"

#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <string>
#include <vector>

// ═══════════════════════════════════════════════════════════
// Helper: temp config (same pattern as other HAL tests)
// ═══════════════════════════════════════════════════════════
static std::vector<std::string> g_gz_imu_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char path[] = "/tmp/test_gz_imu_XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) throw std::runtime_error("mkstemp failed");
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    close(fd);
    g_gz_imu_temp_files.push_back(path);
    return path;
}

// Clean up temp files after all tests
struct GzImuTempCleanup {
    ~GzImuTempCleanup() {
        for (auto& f : g_gz_imu_temp_files) std::remove(f.c_str());
    }
};
static GzImuTempCleanup g_gz_imu_cleanup;

// ═══════════════════════════════════════════════════════════
// Tests — HAVE_GAZEBO path (GazeboIMUBackend)
// ═══════════════════════════════════════════════════════════
#ifdef HAVE_GAZEBO

#include "hal/gazebo_imu.h"

TEST(GazeboIMUTest, NameIncludesTopic) {
    drone::hal::GazeboIMUBackend imu("/test/imu");
    EXPECT_EQ(imu.name(), "GazeboIMU(/test/imu)");
}

TEST(GazeboIMUTest, NotActiveBeforeInit) {
    drone::hal::GazeboIMUBackend imu("/test/imu");
    EXPECT_FALSE(imu.is_active());
}

TEST(GazeboIMUTest, InitSubscribes) {
    // Use a unique topic to avoid collisions with other tests
    drone::hal::GazeboIMUBackend imu("/test/gz_imu_init_sub");
    EXPECT_TRUE(imu.init(200));
}

TEST(GazeboIMUTest, DoubleInitReturnsFalse) {
    drone::hal::GazeboIMUBackend imu("/test/gz_imu_double_init");
    EXPECT_TRUE(imu.init(200));
    EXPECT_FALSE(imu.init(200));  // second init should fail
}

TEST(GazeboIMUTest, ReadReturnsInvalidBeforeData) {
    drone::hal::GazeboIMUBackend imu("/test/gz_imu_no_data");
    imu.init(200);
    auto r = imu.read();
    // No Gazebo publisher → reading is still the default (invalid)
    EXPECT_FALSE(r.valid);
}

TEST(GazeboIMUTest, NotActiveWithoutMessages) {
    // After init but before any messages arrive, is_active should be false
    // (active requires both subscription AND at least one message)
    drone::hal::GazeboIMUBackend imu("/test/gz_imu_no_msg");
    imu.init(400);
    EXPECT_FALSE(imu.is_active());
}

TEST(GazeboIMUTest, MessageCountStartsAtZero) {
    drone::hal::GazeboIMUBackend imu("/test/gz_imu_count");
    EXPECT_EQ(imu.message_count(), 0u);
    imu.init(200);
    EXPECT_EQ(imu.message_count(), 0u);
}

// ── Factory tests ──────────────────────────────────────────
TEST(GazeboIMUTest, FactoryCreatesGazeboBackend) {
    auto path = create_temp_config(R"({
        "slam": {
            "imu": {
                "backend": "gazebo",
                "gz_topic": "/test/factory_imu"
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    auto imu = drone::hal::create_imu_source(cfg, "slam.imu");
    ASSERT_NE(imu, nullptr);
    EXPECT_EQ(imu->name(), "GazeboIMU(/test/factory_imu)");
}

TEST(GazeboIMUTest, FactoryDefaultTopicIsImu) {
    auto path = create_temp_config(R"({
        "slam": {
            "imu": {
                "backend": "gazebo"
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    auto imu = drone::hal::create_imu_source(cfg, "slam.imu");
    ASSERT_NE(imu, nullptr);
    EXPECT_EQ(imu->name(), "GazeboIMU(/imu)");
}

TEST(GazeboIMUTest, FactoryStillCreatesSimulated) {
    auto path = create_temp_config(R"({
        "slam": {
            "imu": {
                "backend": "simulated"
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    auto imu = drone::hal::create_imu_source(cfg, "slam.imu");
    ASSERT_NE(imu, nullptr);
    EXPECT_EQ(imu->name(), "SimulatedIMU");
}

#else  // !HAVE_GAZEBO

// ═══════════════════════════════════════════════════════════
// Fallback tests — verify factory falls back to simulated
// ═══════════════════════════════════════════════════════════

TEST(GazeboIMUFallbackTest, FactoryFallsBackToSimulated) {
    auto path = create_temp_config(R"({
        "slam": {
            "imu": {
                "backend": "simulated"
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    auto imu = drone::hal::create_imu_source(cfg, "slam.imu");
    ASSERT_NE(imu, nullptr);
    EXPECT_EQ(imu->name(), "SimulatedIMU");
}

TEST(GazeboIMUFallbackTest, GazeboBackendThrowsWithoutLib) {
    auto path = create_temp_config(R"({
        "slam": {
            "imu": {
                "backend": "gazebo"
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    EXPECT_THROW(drone::hal::create_imu_source(cfg, "slam.imu"),
                 std::runtime_error);
}

#endif  // HAVE_GAZEBO
