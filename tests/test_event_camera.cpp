// tests/test_event_camera.cpp
// Unit tests for IEventCamera HAL interface, PixelFormat, and SimulatedEventCamera.
#include "hal/hal_factory.h"
#include "hal/ievent_camera.h"
#include "hal/pixel_format.h"
#include "hal/simulated_event_camera.h"
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
    char tmpl[] = "/tmp/test_evcam_XXXXXX.json";
    int  fd     = mkstemps(tmpl, 5);
    if (fd < 0) {
        std::string   path = "/tmp/test_evcam_" + std::to_string(getpid()) + ".json";
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

// ── PixelFormat tests ──

TEST(PixelFormat, ChannelCounts) {
    EXPECT_EQ(pixel_format_channels(PixelFormat::GRAY8), 1);
    EXPECT_EQ(pixel_format_channels(PixelFormat::RGB8), 3);
    EXPECT_EQ(pixel_format_channels(PixelFormat::BGR8), 3);
    EXPECT_EQ(pixel_format_channels(PixelFormat::RGBA8), 4);
    EXPECT_EQ(pixel_format_channels(PixelFormat::THERMAL_8), 1);
    EXPECT_EQ(pixel_format_channels(PixelFormat::THERMAL_16), 1);
    EXPECT_EQ(pixel_format_channels(PixelFormat::EVENT_CD), 0);
}

// ── SimulatedEventCamera tests ──

TEST(EventCamera, SimulatedName) {
    SimulatedEventCamera cam;
    EXPECT_EQ(cam.name(), "SimulatedEventCamera");
}

TEST(EventCamera, OpenClose) {
    SimulatedEventCamera cam;
    EXPECT_FALSE(cam.is_open());

    EXPECT_TRUE(cam.open(640, 480));
    EXPECT_TRUE(cam.is_open());

    cam.close();
    EXPECT_FALSE(cam.is_open());
}

TEST(EventCamera, OpenZeroDimensionsFails) {
    SimulatedEventCamera cam;
    EXPECT_FALSE(cam.open(0, 480));
    EXPECT_FALSE(cam.open(640, 0));
}

TEST(EventCamera, ReadEventsWhenClosed) {
    SimulatedEventCamera cam;
    auto                 batch = cam.read_events();
    EXPECT_FALSE(batch.valid);
    EXPECT_TRUE(batch.events.empty());
}

TEST(EventCamera, ReadEventsProducesEvents) {
    SimulatedEventCamera cam(50);
    ASSERT_TRUE(cam.open(320, 240));

    auto batch = cam.read_events();
    ASSERT_TRUE(batch.valid);
    EXPECT_EQ(batch.events.size(), 50u);
    EXPECT_LT(batch.batch_start_ns, batch.batch_end_ns);

    for (const auto& ev : batch.events) {
        EXPECT_LT(ev.x, 320);
        EXPECT_LT(ev.y, 240);
        EXPECT_TRUE(ev.polarity == 1 || ev.polarity == -1);
    }
}

TEST(EventCamera, PixelFormatIsEventCD) {
    SimulatedEventCamera cam;
    EXPECT_EQ(cam.pixel_format(), PixelFormat::EVENT_CD);
}

TEST(EventCamera, FactorySimulated) {
    auto          path = create_temp_config(R"({
        "perception": { "event_camera": { "backend": "simulated" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    auto cam = drone::hal::create_event_camera(cfg);
    ASSERT_NE(cam, nullptr);
    EXPECT_EQ(cam->name(), "SimulatedEventCamera");
}

TEST(EventCamera, FactoryUnknownThrows) {
    auto          path = create_temp_config(R"({
        "perception": { "event_camera": { "backend": "nonexistent" } }
    })");
    drone::Config cfg;
    ASSERT_TRUE(cfg.load(path));
    EXPECT_THROW(drone::hal::create_event_camera(cfg), std::runtime_error);
}
