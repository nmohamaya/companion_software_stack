// tests/test_cosys_camera_config.cpp
// Unit tests for CosysCameraBackend config-lookup precedence (Issue #499, Bug #1).
//
// Bug #1 background:
//   AirSim silently auto-creates a requested camera with default 256x144
//   CaptureSettings when the name is not declared in server-side settings.json.
//   The dev profile shipped with "cosys_airsim.camera_name":"front_center"
//   while settings.json only declared "mission_cam" — so YOLO received
//   256x144 thumbnails, producing 0 detections for 39 inference frames.
//
// These tests verify the name-resolution logic (no RPC required):
//   1. Per-section <section>.camera_name overrides top-level
//   2. Top-level cosys_airsim.camera_name is used when per-section is absent
//   3. Defaults ("front_center" / "Drone0") apply when neither key is set
#ifdef HAVE_COSYS_AIRSIM

#include "hal/cosys_camera.h"
#include "util/config.h"

#include <atomic>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

namespace {

// Track temp files so we can unlink them on process exit.
std::vector<std::string>& temp_files() {
    static std::vector<std::string> files;
    return files;
}

// Write JSON to a unique temp file and return the path. Uses mkstemps() where
// available; falls back to PID+counter naming in sandboxed environments.
std::string write_temp_config(const std::string& json_content) {
    char        tmpl[] = "/tmp/test_cosys_camera_cfg_XXXXXX.json";
    int         fd     = ::mkstemps(tmpl, 5);  // 5 = strlen(".json")
    std::string path;
    if (fd < 0) {
        static std::atomic<int> counter{0};
        path = "/tmp/test_cosys_camera_cfg_" + std::to_string(::getpid()) + "_" +
               std::to_string(counter.fetch_add(1)) + ".json";
    } else {
        ::close(fd);
        path.assign(tmpl);
    }
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    temp_files().push_back(path);
    return path;
}

struct TempFileCleanup {
    ~TempFileCleanup() {
        for (const auto& f : temp_files()) std::remove(f.c_str());
    }
};
TempFileCleanup g_cleanup;

}  // namespace

// ═══════════════════════════════════════════════════════════
// resolve_camera_name — precedence ladder
// ═══════════════════════════════════════════════════════════

TEST(CosysCameraConfigTest, PerSectionOverridesTopLevel) {
    const auto    path = write_temp_config(R"({
        "cosys_airsim": { "camera_name": "front_center", "vehicle_name": "Drone0" },
        "video_capture": {
            "mission_cam": {
                "camera_name": "mission_cam",
                "vehicle_name": "Hero"
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));

    EXPECT_EQ(drone::hal::CosysCameraBackend::resolve_camera_name(cfg, "video_capture.mission_cam"),
              "mission_cam");
    EXPECT_EQ(
        drone::hal::CosysCameraBackend::resolve_vehicle_name(cfg, "video_capture.mission_cam"),
        "Hero");
}

TEST(CosysCameraConfigTest, TopLevelUsedWhenPerSectionAbsent) {
    const auto    path = write_temp_config(R"({
        "cosys_airsim": { "camera_name": "mission_cam", "vehicle_name": "Drone0" },
        "video_capture": {
            "mission_cam": { "backend": "cosys_airsim" }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));

    // No per-section camera_name → falls back to cosys_airsim.camera_name
    EXPECT_EQ(drone::hal::CosysCameraBackend::resolve_camera_name(cfg, "video_capture.mission_cam"),
              "mission_cam");
    EXPECT_EQ(
        drone::hal::CosysCameraBackend::resolve_vehicle_name(cfg, "video_capture.mission_cam"),
        "Drone0");
}

TEST(CosysCameraConfigTest, DefaultUsedWhenNeitherSet) {
    // Completely empty config — no cosys_airsim section, no per-section overrides.
    const auto    path = write_temp_config(R"({})");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));

    EXPECT_EQ(drone::hal::CosysCameraBackend::resolve_camera_name(cfg, "video_capture.mission_cam"),
              "front_center");
    EXPECT_EQ(
        drone::hal::CosysCameraBackend::resolve_vehicle_name(cfg, "video_capture.mission_cam"),
        "Drone0");
}

TEST(CosysCameraConfigTest, EmptySectionFallsThroughToTopLevel) {
    // If a caller passes an empty section (e.g. legacy call sites), the
    // per-section lookup must be skipped cleanly — top-level wins.
    const auto    path = write_temp_config(R"({
        "cosys_airsim": { "camera_name": "mission_cam" }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));

    EXPECT_EQ(drone::hal::CosysCameraBackend::resolve_camera_name(cfg, ""), "mission_cam");
}

TEST(CosysCameraConfigTest, EmptyPerSectionValueTreatedAsAbsent) {
    // An explicit empty string at the per-section key must not shadow the
    // top-level value — otherwise a JSON typo could silently disable the
    // top-level default.
    const auto    path = write_temp_config(R"({
        "cosys_airsim": { "camera_name": "mission_cam" },
        "video_capture": {
            "mission_cam": { "camera_name": "" }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));

    EXPECT_EQ(drone::hal::CosysCameraBackend::resolve_camera_name(cfg, "video_capture.mission_cam"),
              "mission_cam");
}

#endif  // HAVE_COSYS_AIRSIM
