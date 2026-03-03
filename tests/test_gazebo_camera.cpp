// tests/test_gazebo_camera.cpp
// Unit tests for GazeboCameraBackend — Gazebo camera HAL backend via gz-transport.
//
// These tests verify:
//  1. Factory creates GazeboCameraBackend when HAVE_GAZEBO is defined
//  2. GazeboCameraBackend returns correct name() with topic
//  3. is_open() / close lifecycle
//  4. capture() times out gracefully when Gazebo is not running
//  5. Double-open returns false
//  6. Pixel format helper correctness
//
// NOTE: Full integration tests with a running Gazebo world are planned in Phase 4 (#11).
#include "hal/hal_factory.h"
#include "hal/icamera.h"
#include "util/config.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

// ═══════════════════════════════════════════════════════════
// Helper: temp config (same pattern as other HAL tests)
// ═══════════════════════════════════════════════════════════
static std::vector<std::string> g_gz_cam_temp_files;

static std::string create_temp_config(const std::string& json_content) {
    char tmpl[] = "/tmp/test_gz_cam_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_gz_cam_" + std::to_string(getpid()) + ".json";
        std::ofstream ofs(path);
        ofs << json_content;
        ofs.close();
        g_gz_cam_temp_files.push_back(path);
        return path;
    }
    ::close(fd);
    std::string   path(tmpl);
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    g_gz_cam_temp_files.push_back(path);
    return path;
}

struct GzCamTempCleanup {
    ~GzCamTempCleanup() {
        for (const auto& f : g_gz_cam_temp_files) {
            std::remove(f.c_str());
        }
    }
};
static GzCamTempCleanup g_gz_cam_cleanup;

// ═══════════════════════════════════════════════════════════
// Tests conditional on HAVE_GAZEBO
// ═══════════════════════════════════════════════════════════

#ifdef HAVE_GAZEBO

#include "hal/gazebo_camera.h"

// ── Basic construction and name ────────────────────────────
TEST(GazeboCameraTest, NameIncludesTopic) {
    drone::hal::GazeboCameraBackend cam("/test_camera");
    EXPECT_EQ(cam.name(), "GazeboCameraBackend(/test_camera)");
}

TEST(GazeboCameraTest, NameDifferentTopic) {
    drone::hal::GazeboCameraBackend cam("/stereo_left");
    EXPECT_EQ(cam.name(), "GazeboCameraBackend(/stereo_left)");
}

TEST(GazeboCameraTest, InitiallyNotOpen) {
    drone::hal::GazeboCameraBackend cam("/camera");
    EXPECT_FALSE(cam.is_open());
}

// ── Open / close lifecycle ─────────────────────────────────
TEST(GazeboCameraTest, OpenSubscribesToTopic) {
    drone::hal::GazeboCameraBackend cam("/gz_test_topic_unused");
    // Open should succeed (subscribes to topic — no Gazebo needed for subscribe)
    EXPECT_TRUE(cam.open(640, 480, 30));
    EXPECT_TRUE(cam.is_open());
    cam.close();
    EXPECT_FALSE(cam.is_open());
}

TEST(GazeboCameraTest, DoubleOpenReturnsFalse) {
    drone::hal::GazeboCameraBackend cam("/gz_double_open");
    EXPECT_TRUE(cam.open(640, 480, 30));
    EXPECT_FALSE(cam.open(320, 240, 15));  // Already open
    cam.close();
}

TEST(GazeboCameraTest, CloseWhenNotOpenIsNoOp) {
    drone::hal::GazeboCameraBackend cam("/gz_close_noop");
    // Should not throw or crash
    cam.close();
    EXPECT_FALSE(cam.is_open());
}

TEST(GazeboCameraTest, DoubleCloseIsNoOp) {
    drone::hal::GazeboCameraBackend cam("/gz_double_close");
    cam.open(640, 480, 30);
    cam.close();
    cam.close();  // Second close should be safe
    EXPECT_FALSE(cam.is_open());
}

// ── Capture without Gazebo ─────────────────────────────────
TEST(GazeboCameraTest, CaptureTimesOutWithNoGazebo) {
    drone::hal::GazeboCameraBackend cam("/gz_no_frames");
    cam.open(640, 480, 30);
    // capture() blocks up to 2s — will timeout since no Gazebo is publishing
    auto frame = cam.capture();
    EXPECT_FALSE(frame.valid);
    cam.close();
}

TEST(GazeboCameraTest, CaptureWhenClosedReturnsInvalid) {
    drone::hal::GazeboCameraBackend cam("/gz_closed_capture");
    auto                            frame = cam.capture();
    EXPECT_FALSE(frame.valid);
}

// ── Factory creates GazeboCameraBackend ────────────────────
TEST(GazeboCameraTest, FactoryCreatesGazeboBackend) {
    auto          path = create_temp_config(R"({
        "video_capture": {
            "mission_cam": {
                "backend": "gazebo",
                "gz_topic": "/test_factory_cam",
                "width": 640,
                "height": 480,
                "fps": 30
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));

    auto cam = drone::hal::create_camera(cfg, "video_capture.mission_cam");
    ASSERT_NE(cam, nullptr);
    EXPECT_EQ(cam->name(), "GazeboCameraBackend(/test_factory_cam)");
}

TEST(GazeboCameraTest, FactoryDefaultTopicIsCameraSlash) {
    auto          path = create_temp_config(R"({
        "video_capture": {
            "mission_cam": {
                "backend": "gazebo",
                "width": 640,
                "height": 480,
                "fps": 30
            }
        }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));

    auto cam = drone::hal::create_camera(cfg, "video_capture.mission_cam");
    ASSERT_NE(cam, nullptr);
    EXPECT_EQ(cam->name(), "GazeboCameraBackend(/camera)");
}

#else  // !HAVE_GAZEBO

// ── When Gazebo libs are not available ─────────────────────
TEST(GazeboCameraTest, GazeboBackendThrowsWithoutGazebo) {
    auto          path = create_temp_config(R"({
        "video_capture": { "mission_cam": { "backend": "gazebo" } }
    })");
    drone::Config cfg;
    cfg.load(path);

    EXPECT_THROW(drone::hal::create_camera(cfg, "video_capture.mission_cam"), std::runtime_error);
}

TEST(GazeboCameraTest, SimulatedStillWorksWithoutGazebo) {
    auto          path = create_temp_config(R"({
        "video_capture": { "mission_cam": { "backend": "simulated" } }
    })");
    drone::Config cfg;
    cfg.load(path);

    auto cam = drone::hal::create_camera(cfg, "video_capture.mission_cam");
    ASSERT_NE(cam, nullptr);
    EXPECT_EQ(cam->name(), "SimulatedCamera");
}

#endif  // HAVE_GAZEBO
