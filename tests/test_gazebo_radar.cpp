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
#include <stdexcept>
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
    // Single horizontal ray → azimuth should be midpoint of min/max
    auto [az, el] = drone::hal::GazeboRadarBackend::ray_index_to_angles(0, 0, 1, 1, -0.5f, 0.5f,
                                                                        -0.1f, 0.1f);
    EXPECT_FLOAT_EQ(az, 0.0f);  // midpoint of (-0.5, 0.5)
    EXPECT_FLOAT_EQ(el, 0.0f);  // midpoint of (-0.1, 0.1)

    // Non-symmetric FOV: midpoint should be (min+max)/2, not 0
    auto [az2, el2] = drone::hal::GazeboRadarBackend::ray_index_to_angles(0, 0, 1, 1, 0.1f, 0.5f,
                                                                          -0.2f, 0.0f);
    EXPECT_FLOAT_EQ(az2, 0.3f);   // midpoint of (0.1, 0.5)
    EXPECT_FLOAT_EQ(el2, -0.1f);  // midpoint of (-0.2, 0.0)
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


// ═══════════════════════════════════════════════════════════════════
// Issue #816 PR2 — HAL attitude-aware ground gate + mount compensation.
// FIRST ground-filter coverage for the HAL (none existed before #816): the
// HAL's flat `drone_alt + range·sin(el)` gate shared the exact P1 bug fixed in
// the fusion engine.  These tests exercise the public statics directly (same
// pattern as ray_to_detection above).
// ═══════════════════════════════════════════════════════════════════

namespace {
constexpr float kD2R816 = 3.14159265358979323846f / 180.0f;
}

TEST(GazeboRadarGroundGate, MountCompensationTiltsRayIntoBodyFrame) {
    // A boresight sensor ray (az=0, el=0) through the −5° mount must come out
    // pointing 5° DOWN in the body frame.
    const auto mount  = drone::util::quat_from_rpy(0.0f, -0.087f, 0.0f);
    auto [az_b, el_b] = drone::hal::GazeboRadarBackend::sensor_to_body_angles(0.0f, 0.0f, mount);
    EXPECT_NEAR(az_b, 0.0f, 1e-4f);
    EXPECT_NEAR(el_b, -0.087f, 1e-3f) << "mount pitch must tilt the emitted body elevation down";

    // Identity mount = angles unchanged.
    auto [az_i, el_i] = drone::hal::GazeboRadarBackend::sensor_to_body_angles(
        0.3f, -0.2f, Eigen::Quaternionf::Identity());
    EXPECT_NEAR(az_i, 0.3f, 1e-5f);
    EXPECT_NEAR(el_i, -0.2f, 1e-5f);
}

TEST(GazeboRadarGroundGate, RealObstacleKeptAcrossPitchSweep) {
    // THE safety property at the HAL layer: a real 1.7 m obstacle at 15 m,
    // pitch swept ±25° (beyond the airframe envelope) — the gate must NEVER
    // reject it.  Body-frame geometry: el_body points at the obstacle top from
    // altitude 4.48 m; the gate must recover its true world altitude under any
    // attitude.
    const float alt = 4.48f, h = 1.7f, r = 15.0f;
    const float slant   = std::sqrt(r * r + (h - alt) * (h - alt));
    const float el_body = std::asin((h - alt) / slant);  // body el when level
    for (int pitch = -25; pitch <= 25; ++pitch) {
        const auto body_to_world =
            drone::util::quat_from_rpy(0.0f, static_cast<float>(pitch) * kD2R816, 0.0f);
        // The body-frame elevation of a FIXED world point changes with pitch:
        // el_b(pitch) = el_world − pitch (small-angle at az=0).
        const float el_b = el_body - static_cast<float>(pitch) * kD2R816;
        EXPECT_FALSE(drone::hal::GazeboRadarBackend::ground_gate_should_reject(
            true, true, alt, slant, 0.0f, el_b, body_to_world, 0.5f, 0.02f))
            << "REAL obstacle dropped by HAL gate at pitch " << pitch << "°";
    }
}

TEST(GazeboRadarGroundGate, GroundRejectedLevelFlight) {
    // Level flight, ground return at 12 m slant (inside the confident range):
    // world altitude ≈ 0 → must be rejected against a 0.5 m floor.
    const float alt = 4.48f, slant = 12.0f;
    const float el_b = std::asin(-alt / slant);  // ray hits ground at 12 m
    EXPECT_TRUE(drone::hal::GazeboRadarBackend::ground_gate_should_reject(
        true, true, alt, slant, 0.0f, el_b, Eigen::Quaternionf::Identity(), 0.5f, 0.02f))
        << "near ground must be rejected in level flight";
}

TEST(GazeboRadarGroundGate, NoAttitudeNeverRejects) {
    // Fail-safe: attitude unknown ⇒ never suppress, even an obviously-ground
    // return.  (The old HAL gate rejected with altitude alone — the unsafe
    // path this replaces.)
    const float alt = 4.48f, slant = 12.0f;
    const float el_b = std::asin(-alt / slant);
    EXPECT_FALSE(drone::hal::GazeboRadarBackend::ground_gate_should_reject(
        true, /*has_attitude=*/false, alt, slant, 0.0f, el_b, Eigen::Quaternionf::Identity(), 0.5f,
        0.02f))
        << "no attitude ⇒ fail-safe keep";
    EXPECT_FALSE(drone::hal::GazeboRadarBackend::ground_gate_should_reject(
        /*has_altitude=*/false, true, alt, slant, 0.0f, el_b, Eigen::Quaternionf::Identity(), 0.5f,
        0.02f))
        << "no altitude ⇒ fail-safe keep";
}

TEST(GazeboRadarGroundGate, MarginBiasesFalseAccept) {
    // A return floating just at the floor must be KEPT: the margin
    // (range·sin(attitude_uncertainty)) absorbs plausible attitude error.
    const float alt = 4.48f, slant = 12.0f, floor_m = 0.5f;
    // Ray to a point exactly AT the floor altitude.
    const float el_b = std::asin((floor_m - alt) / slant);
    EXPECT_FALSE(drone::hal::GazeboRadarBackend::ground_gate_should_reject(
        true, true, alt, slant, 0.0f, el_b, Eigen::Quaternionf::Identity(), floor_m, 0.02f))
        << "a floor-height return is within the margin — keep (false-accept bias)";
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
    ASSERT_TRUE(cfg.load(path));
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
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_radar(cfg, "perception.radar"), std::runtime_error);
}

#endif  // HAVE_GAZEBO
