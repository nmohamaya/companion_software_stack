// tests/test_gazebo_radar.cpp
// Unit tests for GazeboRadarBackend — Gazebo radar HAL via gpu_lidar + odometry.
//
// These tests verify:
//  1. Factory creates GazeboRadarBackend when HAVE_GAZEBO is defined
//  2. ray_to_detection() produces correct range/azimuth/elevation/Doppler
//  3. ray_index_to_angles() maps indices to correct FOV angles
//  4. SNR inversely proportional to range
//  5. Lifecycle: name(), is_active(), init(), read()
//  6. Graceful fallback when HAVE_GAZEBO is not defined
//
// NOTE: Full integration tests with a running Gazebo world are in Tier 2 scenarios.
// Issue #212.
#include "hal/hal_factory.h"
#include "hal/iradar.h"
#include "util/config.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

// ═══════════════════════════════════════════════════════════
// Helper: temp config
// ═══════════════════════════════════════════════════════════
static std::vector<std::string> g_gz_radar_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char path[] = "/tmp/test_gz_radar_XXXXXX";
    int  fd     = mkstemp(path);
    if (fd < 0) throw std::runtime_error("mkstemp failed");
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    close(fd);
    g_gz_radar_temp_files.push_back(path);
    return path;
}

struct GzRadarTempCleanup {
    ~GzRadarTempCleanup() {
        for (auto& f : g_gz_radar_temp_files) std::remove(f.c_str());
    }
};
static GzRadarTempCleanup g_gz_radar_cleanup;

// ═══════════════════════════════════════════════════════════
// Tests — HAVE_GAZEBO path (GazeboRadarBackend)
// ═══════════════════════════════════════════════════════════
#ifdef HAVE_GAZEBO

#include "hal/gazebo_radar.h"

TEST(GazeboRadarTest, NameIncludesTopic) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "gazebo",
                "gz_scan_topic": "/test/radar_scan"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    drone::hal::GazeboRadarBackend radar(cfg, "perception.radar");
    EXPECT_EQ(radar.name(), "GazeboRadar(/test/radar_scan)");
}

TEST(GazeboRadarTest, NotActiveBeforeInit) {
    auto          path = create_temp_config(R"({
        "perception": { "radar": { "backend": "gazebo" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    drone::hal::GazeboRadarBackend radar(cfg, "perception.radar");
    EXPECT_FALSE(radar.is_active());
}

TEST(GazeboRadarTest, InitSubscribes) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "gazebo",
                "gz_scan_topic": "/test/gz_radar_init",
                "gz_odom_topic": "/test/gz_radar_odom_init"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    drone::hal::GazeboRadarBackend radar(cfg, "perception.radar");
    EXPECT_TRUE(radar.init());
}

TEST(GazeboRadarTest, DoubleInitReturnsFalse) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "gazebo",
                "gz_scan_topic": "/test/gz_radar_double",
                "gz_odom_topic": "/test/gz_radar_odom_double"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    drone::hal::GazeboRadarBackend radar(cfg, "perception.radar");
    EXPECT_TRUE(radar.init());
    EXPECT_FALSE(radar.init());  // second init should fail
}

TEST(GazeboRadarTest, ReadReturnsEmptyBeforeData) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "gazebo",
                "gz_scan_topic": "/test/gz_radar_no_data",
                "gz_odom_topic": "/test/gz_radar_odom_no_data"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    drone::hal::GazeboRadarBackend radar(cfg, "perception.radar");
    radar.init();
    auto list = radar.read();
    EXPECT_EQ(list.num_detections, 0u);
}

TEST(GazeboRadarTest, MessageCountStartsAtZero) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "gazebo",
                "gz_scan_topic": "/test/gz_radar_count",
                "gz_odom_topic": "/test/gz_radar_odom_count"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    drone::hal::GazeboRadarBackend radar(cfg, "perception.radar");
    EXPECT_EQ(radar.scan_message_count(), 0u);
    EXPECT_EQ(radar.odom_message_count(), 0u);
    radar.init();
    EXPECT_EQ(radar.scan_message_count(), 0u);
    EXPECT_EQ(radar.odom_message_count(), 0u);
}

// ── Static helper tests (no Gazebo needed) ────────────────

TEST(GazeboRadarTest, RayToDetectionZeroVelocity) {
    // Target at 10m range, 0 azimuth, 0 elevation, no body velocity
    auto det = drone::hal::GazeboRadarBackend::ray_to_detection(10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                                0.0f);

    EXPECT_FLOAT_EQ(det.range_m, 10.0f);
    EXPECT_FLOAT_EQ(det.azimuth_rad, 0.0f);
    EXPECT_FLOAT_EQ(det.elevation_rad, 0.0f);
    EXPECT_FLOAT_EQ(det.radial_velocity_mps, 0.0f);
    EXPECT_GT(det.snr_db, 0.0f);
    EXPECT_GT(det.confidence, 0.0f);
}

TEST(GazeboRadarTest, DopplerProjectionForward) {
    // Target directly ahead (az=0, el=0), body moving forward at 5 m/s
    // Radial direction = (1, 0, 0), so Doppler = vx = 5.0
    auto det = drone::hal::GazeboRadarBackend::ray_to_detection(20.0f, 0.0f, 0.0f, 5.0f, 0.0f,
                                                                0.0f);

    EXPECT_NEAR(det.radial_velocity_mps, 5.0f, 1e-5f);
}

TEST(GazeboRadarTest, DopplerProjectionOblique) {
    // Target at 45° azimuth (az=π/4, el=0), body moving forward at 10 m/s
    // Radial direction = (cos(π/4), sin(π/4), 0) = (0.707, 0.707, 0)
    // Doppler = vx * 0.707 = 10 * 0.707 ≈ 7.07
    const float az = static_cast<float>(M_PI) / 4.0f;
    auto det = drone::hal::GazeboRadarBackend::ray_to_detection(15.0f, az, 0.0f, 10.0f, 0.0f, 0.0f);

    const float expected = 10.0f * std::cos(az);
    EXPECT_NEAR(det.radial_velocity_mps, expected, 1e-4f);
}

TEST(GazeboRadarTest, DopplerProjectionVertical) {
    // Target at 30° elevation (az=0, el=π/6), body moving up at 3 m/s
    // Radial direction z-component = sin(π/6) = 0.5
    // Doppler = vz * 0.5 = 1.5
    const float el = static_cast<float>(M_PI) / 6.0f;
    auto det = drone::hal::GazeboRadarBackend::ray_to_detection(10.0f, 0.0f, el, 0.0f, 0.0f, 3.0f);

    EXPECT_NEAR(det.radial_velocity_mps, 3.0f * std::sin(el), 1e-4f);
}

TEST(GazeboRadarTest, SNRDecreasesWithRange) {
    auto det_close = drone::hal::GazeboRadarBackend::ray_to_detection(5.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                                      0.0f);
    auto det_far   = drone::hal::GazeboRadarBackend::ray_to_detection(50.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                                      0.0f);

    EXPECT_GT(det_close.snr_db, det_far.snr_db);
    EXPECT_GT(det_close.confidence, det_far.confidence);
}

TEST(GazeboRadarTest, FOVMappingSingleRay) {
    // Single horizontal ray → azimuth should be 0
    auto [az, el] = drone::hal::GazeboRadarBackend::ray_index_to_angles(0, 0, 1, 1, -0.5f, 0.5f,
                                                                        -0.1f, 0.1f);
    EXPECT_FLOAT_EQ(az, 0.0f);
    EXPECT_FLOAT_EQ(el, 0.0f);
}

TEST(GazeboRadarTest, FOVMappingMultipleRays) {
    // 32 horizontal rays, -0.5236 to +0.5236 rad
    const float h_min   = -0.5236f;
    const float h_max   = 0.5236f;
    const int   h_count = 32;

    // First ray
    auto [az0, el0] = drone::hal::GazeboRadarBackend::ray_index_to_angles(0, 0, h_count, 1, h_min,
                                                                          h_max, 0.0f, 0.0f);
    EXPECT_NEAR(az0, h_min, 1e-5f);

    // Last ray
    auto [az31, el31] = drone::hal::GazeboRadarBackend::ray_index_to_angles(
        h_count - 1, 0, h_count, 1, h_min, h_max, 0.0f, 0.0f);
    EXPECT_NEAR(az31, h_max, 1e-4f);

    // Middle ray
    auto [az_mid, el_mid] = drone::hal::GazeboRadarBackend::ray_index_to_angles(
        h_count / 2, 0, h_count, 1, h_min, h_max, 0.0f, 0.0f);
    EXPECT_NEAR(az_mid, 0.0f, 0.02f);  // approximately center
}

TEST(GazeboRadarTest, FOVMappingVertical) {
    // 8 vertical rays, -0.1309 to +0.1309 rad
    const float v_min   = -0.1309f;
    const float v_max   = 0.1309f;
    const int   v_count = 8;

    auto [az0, el0] = drone::hal::GazeboRadarBackend::ray_index_to_angles(0, 0, 1, v_count, 0.0f,
                                                                          0.0f, v_min, v_max);
    EXPECT_NEAR(el0, v_min, 1e-5f);

    auto [az7, el7] = drone::hal::GazeboRadarBackend::ray_index_to_angles(
        0, v_count - 1, 1, v_count, 0.0f, 0.0f, v_min, v_max);
    EXPECT_NEAR(el7, v_max, 1e-4f);
}

// ── Factory tests ──────────────────────────────────────────

TEST(GazeboRadarTest, FactoryCreatesGazeboBackend) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "gazebo",
                "gz_scan_topic": "/test/factory_radar_scan",
                "gz_odom_topic": "/test/factory_radar_odom"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto radar = drone::hal::create_radar(cfg, "perception.radar");
    ASSERT_NE(radar, nullptr);
    EXPECT_EQ(radar->name(), "GazeboRadar(/test/factory_radar_scan)");
}

TEST(GazeboRadarTest, FactoryStillCreatesSimulated) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "simulated"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto radar = drone::hal::create_radar(cfg, "perception.radar");
    ASSERT_NE(radar, nullptr);
    EXPECT_EQ(radar->name(), "SimulatedRadar");
}

#else  // !HAVE_GAZEBO

// ═══════════════════════════════════════════════════════════
// Fallback tests — verify factory falls back to simulated
// ═══════════════════════════════════════════════════════════

TEST(GazeboRadarFallbackTest, FactoryCreatesSimulatedBackend) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "simulated"
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    auto radar = drone::hal::create_radar(cfg, "perception.radar");
    ASSERT_NE(radar, nullptr);
    EXPECT_EQ(radar->name(), "SimulatedRadar");
}

TEST(GazeboRadarFallbackTest, GazeboBackendThrowsWithoutLib) {
    auto          path = create_temp_config(R"({
        "perception": {
            "radar": {
                "backend": "gazebo"
            }
        }
    })");
    drone::Config cfg;
    cfg.load(path);
    EXPECT_THROW(drone::hal::create_radar(cfg, "perception.radar"), std::runtime_error);
}

#endif  // HAVE_GAZEBO
