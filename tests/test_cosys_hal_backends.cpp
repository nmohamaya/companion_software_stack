// tests/test_cosys_hal_backends.cpp
// Tests for Cosys-AirSim HAL backends (Issue #434).
//
// Two categories of tests:
// 1. Factory rejection tests (always run) — verify that requesting
//    "cosys_airsim" backend throws when HAVE_COSYS_AIRSIM is not defined.
// 2. Backend construction tests (only with HAVE_COSYS_AIRSIM) — verify
//    that backends can be constructed and destroyed without crashing.
#include "hal/hal_factory.h"
#include "util/config.h"
#include "util/config_keys.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

// ═══════════════════════════════════════════════════════════
// Helper: create a unique temporary config file
// ═══════════════════════════════════════════════════════════
static std::vector<std::string> g_cosys_temp_files;
static std::atomic<int>         g_cosys_temp_counter{0};

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_cosys_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);  // 5 = strlen(".json")
    if (fd < 0) {
        // Fallback: use PID + counter for uniqueness across tests in same process
        int         seq  = g_cosys_temp_counter.fetch_add(1);
        std::string path = "/tmp/test_cosys_" + std::to_string(getpid()) + "_" +
                           std::to_string(seq) + ".json";
        std::ofstream ofs(path);
        ofs << json_content;
        ofs.close();
        g_cosys_temp_files.push_back(path);
        return path;
    }
    ::close(fd);
    std::string   path(tmpl);
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    g_cosys_temp_files.push_back(path);
    return path;
}

struct CosysTempFileCleanup {
    ~CosysTempFileCleanup() {
        for (const auto& f : g_cosys_temp_files) {
            std::remove(f.c_str());
        }
    }
};
static CosysTempFileCleanup g_cosys_cleanup;

// ═══════════════════════════════════════════════════════════
// Config key validation tests (always run)
// ═══════════════════════════════════════════════════════════

TEST(CosysConfigKeysTest, HostKeyIsValid) {
    const std::string key = drone::cfg_key::cosys_airsim::HOST;
    EXPECT_FALSE(key.empty());
    EXPECT_EQ(key.substr(0, 13), "cosys_airsim.");
}

TEST(CosysConfigKeysTest, PortKeyIsValid) {
    const std::string key = drone::cfg_key::cosys_airsim::PORT;
    EXPECT_FALSE(key.empty());
    EXPECT_EQ(key.substr(0, 13), "cosys_airsim.");
}

TEST(CosysConfigKeysTest, CameraNameKeyIsValid) {
    const std::string key = drone::cfg_key::cosys_airsim::CAMERA_NAME;
    EXPECT_FALSE(key.empty());
    EXPECT_EQ(key.substr(0, 13), "cosys_airsim.");
}

TEST(CosysConfigKeysTest, RadarNameKeyIsValid) {
    const std::string key = drone::cfg_key::cosys_airsim::RADAR_NAME;
    EXPECT_FALSE(key.empty());
    EXPECT_EQ(key.substr(0, 13), "cosys_airsim.");
}

TEST(CosysConfigKeysTest, ImuNameKeyIsValid) {
    const std::string key = drone::cfg_key::cosys_airsim::IMU_NAME;
    EXPECT_FALSE(key.empty());
    EXPECT_EQ(key.substr(0, 13), "cosys_airsim.");
}

TEST(CosysConfigKeysTest, VehicleNameKeyIsValid) {
    const std::string key = drone::cfg_key::cosys_airsim::VEHICLE_NAME;
    EXPECT_FALSE(key.empty());
    EXPECT_EQ(key.substr(0, 13), "cosys_airsim.");
}

TEST(CosysConfigKeysTest, SectionKeyIsValid) {
    const std::string key = drone::cfg_key::cosys_airsim::SECTION;
    EXPECT_EQ(key, "cosys_airsim");
}

// ═══════════════════════════════════════════════════════════
// Factory rejection tests (always run — SDK not available in CI)
// When HAVE_COSYS_AIRSIM is NOT defined, the factory must throw
// std::runtime_error for the "cosys_airsim" backend.
// ═══════════════════════════════════════════════════════════

#ifndef HAVE_COSYS_AIRSIM

TEST(CosysFactoryTest, CameraThrowsWithoutSdk) {
    auto          path = create_temp_config(R"({
        "video_capture": { "mission_cam": { "backend": "cosys_airsim" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_camera(cfg, "video_capture.mission_cam"), std::runtime_error);
}

TEST(CosysFactoryTest, RadarThrowsWithoutSdk) {
    auto          path = create_temp_config(R"({
        "perception": { "radar": { "backend": "cosys_airsim" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_radar(cfg, "perception.radar"), std::runtime_error);
}

TEST(CosysFactoryTest, ImuThrowsWithoutSdk) {
    auto          path = create_temp_config(R"({
        "slam": { "imu": { "backend": "cosys_airsim" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_imu_source(cfg, "slam.imu"), std::runtime_error);
}

TEST(CosysFactoryTest, DepthThrowsWithoutSdk) {
    auto          path = create_temp_config(R"({
        "perception": { "depth_estimator": { "backend": "cosys_airsim" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_depth_estimator(cfg, "perception.depth_estimator"),
                 std::runtime_error);
}

#endif  // !HAVE_COSYS_AIRSIM

// ═══════════════════════════════════════════════════════════
// CosysRpcClient tests (only with HAVE_COSYS_AIRSIM)
// ═══════════════════════════════════════════════════════════

#ifdef HAVE_COSYS_AIRSIM

TEST(CosysRpcClientTest, ConstructionSetsEndpoint) {
    drone::hal::CosysRpcClient client("10.0.0.5", 9999);
    EXPECT_EQ(client.endpoint(), "10.0.0.5:9999");
}

TEST(CosysRpcClientTest, InitiallyNotConnected) {
    drone::hal::CosysRpcClient client("127.0.0.1", 41451);
    EXPECT_FALSE(client.is_connected());
}

TEST(CosysRpcClientTest, DisconnectSetsNotConnected) {
    drone::hal::CosysRpcClient client("127.0.0.1", 41451);
    client.disconnect();
    EXPECT_FALSE(client.is_connected());
}

// NOTE: Tests below touch the network — AirSim's confirmConnection() blocks
// indefinitely when no server is running, so these are integration tests.
// They're gated to only run when AIRSIM_SERVER_RUNNING=1 env var is set.
// Run scenario-based integration tests (run_scenario_cosys.sh) for live
// AirSim testing.

static bool airsim_server_available() {
    const char* env = std::getenv("AIRSIM_SERVER_RUNNING");
    return env != nullptr && std::string(env) == "1";
}

TEST(CosysRpcClientTest, ConnectStubReturnsFalse) {
    if (!airsim_server_available())
        GTEST_SKIP() << "Requires live AirSim server (set AIRSIM_SERVER_RUNNING=1)";
    drone::hal::CosysRpcClient client("127.0.0.1", 99);  // Wrong port — fast fail
    EXPECT_FALSE(client.connect());
    EXPECT_FALSE(client.is_connected());
}

TEST(CosysRpcClientTest, ReconnectFailsAfterMaxRetries) {
    if (!airsim_server_available())
        GTEST_SKIP() << "Requires live AirSim server (set AIRSIM_SERVER_RUNNING=1)";
    drone::hal::CosysRpcClient client("127.0.0.1", 99);
    EXPECT_FALSE(client.reconnect());
    EXPECT_FALSE(client.is_connected());
}

TEST(SharedClientTest, FactoryCreatesSingleInstance) {
    if (!airsim_server_available())
        GTEST_SKIP() << "Requires live AirSim server (set AIRSIM_SERVER_RUNNING=1)";
    auto          path = create_temp_config(R"({
        "cosys_airsim": { "host": "127.0.0.1", "port": 41451 }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    const auto& client1 = drone::hal::detail::get_shared_cosys_client(cfg);
    auto*       raw_ptr = client1.get();
    const auto& client2 = drone::hal::detail::get_shared_cosys_client(cfg);
    EXPECT_EQ(raw_ptr, client2.get());
    EXPECT_EQ(client1.get(), client2.get());
}

#endif  // HAVE_COSYS_AIRSIM

// ═══════════════════════════════════════════════════════════
// Backend construction tests (only with HAVE_COSYS_AIRSIM + live server)
// ═══════════════════════════════════════════════════════════

#ifdef HAVE_COSYS_AIRSIM

TEST(CosysBackendTest, CameraConstructAndDestruct) {
    if (!airsim_server_available())
        GTEST_SKIP() << "Requires live AirSim server (set AIRSIM_SERVER_RUNNING=1)";
    auto          path = create_temp_config(R"({
        "video_capture": { "mission_cam": { "backend": "cosys_airsim" } },
        "cosys_airsim": { "host": "127.0.0.1", "port": 41451 }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto camera = drone::hal::create_camera(cfg, "video_capture.mission_cam");
    ASSERT_NE(camera, nullptr);
    EXPECT_NE(camera->name().find("CosysCamera"), std::string::npos);
}

TEST(CosysBackendTest, RadarConstructAndDestruct) {
    if (!airsim_server_available())
        GTEST_SKIP() << "Requires live AirSim server (set AIRSIM_SERVER_RUNNING=1)";
    auto          path = create_temp_config(R"({
        "perception": { "radar": { "backend": "cosys_airsim" } },
        "cosys_airsim": { "host": "127.0.0.1", "port": 41451 }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto radar = drone::hal::create_radar(cfg, "perception.radar");
    ASSERT_NE(radar, nullptr);
    EXPECT_NE(radar->name().find("CosysRadar"), std::string::npos);
}

TEST(CosysBackendTest, ImuConstructAndDestruct) {
    if (!airsim_server_available())
        GTEST_SKIP() << "Requires live AirSim server (set AIRSIM_SERVER_RUNNING=1)";
    auto          path = create_temp_config(R"({
        "slam": { "imu": { "backend": "cosys_airsim" } },
        "cosys_airsim": { "host": "127.0.0.1", "port": 41451 }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto imu = drone::hal::create_imu_source(cfg, "slam.imu");
    ASSERT_NE(imu, nullptr);
    EXPECT_NE(imu->name().find("CosysIMU"), std::string::npos);
}

TEST(CosysBackendTest, DepthConstructAndDestruct) {
    if (!airsim_server_available())
        GTEST_SKIP() << "Requires live AirSim server (set AIRSIM_SERVER_RUNNING=1)";
    auto          path = create_temp_config(R"({
        "perception": { "depth_estimator": { "backend": "cosys_airsim" } },
        "cosys_airsim": { "host": "127.0.0.1", "port": 41451 }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto depth = drone::hal::create_depth_estimator(cfg, "perception.depth_estimator");
    ASSERT_NE(depth, nullptr);
    EXPECT_NE(depth->name().find("CosysDepth"), std::string::npos);
}

TEST(CosysBackendTest, DepthEstimateReturnsErrorUntilSdkConnected) {
    if (!airsim_server_available())
        GTEST_SKIP() << "Requires live AirSim server (set AIRSIM_SERVER_RUNNING=1)";
    auto          path = create_temp_config(R"({
        "perception": { "depth_estimator": { "backend": "cosys_airsim" } },
        "cosys_airsim": { "host": "127.0.0.1", "port": 41451 }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto depth = drone::hal::create_depth_estimator(cfg, "perception.depth_estimator");
    ASSERT_NE(depth, nullptr);

    constexpr uint32_t   kWidth    = 64;
    constexpr uint32_t   kHeight   = 48;
    constexpr uint32_t   kChannels = 3;
    std::vector<uint8_t> frame(kWidth * kHeight * kChannels, 128);

    auto result = depth->estimate(frame.data(), kWidth, kHeight, kChannels);
    ASSERT_TRUE(result.is_err());
}

#endif  // HAVE_COSYS_AIRSIM
