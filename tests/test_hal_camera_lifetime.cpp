// tests/test_hal_camera_lifetime.cpp
// Lifetime safety tests for CapturedFrame owned data (Issue #452).
// Verifies that CapturedFrame::data (now std::vector<uint8_t>) survives
// beyond subsequent capture() calls — the key safety property that the
// old raw-pointer design lacked.

#include "hal/icamera.h"
#include "hal/simulated_camera.h"

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

namespace {

class CameraLifetimeTest : public ::testing::Test {
protected:
    drone::hal::SimulatedCamera cam_;
};

// ── Frame data survives beyond next capture() ──────────────────
// This is the core safety test: with the old raw-pointer design,
// frame1.data would become a dangling pointer after capture() #2.
TEST_F(CameraLifetimeTest, FrameDataSurvivesBeyondNextCapture) {
    cam_.open(64, 64, 10);

    auto frame1 = cam_.capture();
    ASSERT_TRUE(frame1.valid);
    ASSERT_FALSE(frame1.data.empty());

    // Save a copy of frame1's data for comparison
    const std::vector<uint8_t> frame1_snapshot(frame1.data.begin(), frame1.data.end());

    // Capture again — this must NOT invalidate frame1.data
    auto frame2 = cam_.capture();
    ASSERT_TRUE(frame2.valid);

    // frame1.data must still be accessible and unchanged
    ASSERT_EQ(frame1.data.size(), frame1_snapshot.size());
    EXPECT_EQ(frame1.data, frame1_snapshot);
}

// ── Frame data size matches width * height * channels ──────────
TEST_F(CameraLifetimeTest, FrameDataSizeMatchesDimensions) {
    constexpr uint32_t kWidth  = 128;
    constexpr uint32_t kHeight = 96;
    constexpr int      kFps    = 10;

    cam_.open(kWidth, kHeight, kFps);
    auto frame = cam_.capture();
    ASSERT_TRUE(frame.valid);

    const size_t expected_size = static_cast<size_t>(frame.width) * frame.height * frame.channels;
    EXPECT_EQ(frame.data.size(), expected_size);
}

// ── Invalid frame has empty data ───────────────────────────────
TEST_F(CameraLifetimeTest, InvalidFrameHasEmptyData) {
    // Camera not opened — capture returns invalid frame
    auto frame = cam_.capture();
    EXPECT_FALSE(frame.valid);
    EXPECT_TRUE(frame.data.empty());
}

// ── Multiple frames are independent ────────────────────────────
// Each frame must own its own data independently.
TEST_F(CameraLifetimeTest, MultipleFramesAreIndependent) {
    cam_.open(32, 32, 10);

    auto frame1 = cam_.capture();
    auto frame2 = cam_.capture();
    auto frame3 = cam_.capture();

    ASSERT_TRUE(frame1.valid);
    ASSERT_TRUE(frame2.valid);
    ASSERT_TRUE(frame3.valid);

    // All frames must have data of the correct size
    const size_t expected = static_cast<size_t>(frame1.width) * frame1.height * frame1.channels;
    EXPECT_EQ(frame1.data.size(), expected);
    EXPECT_EQ(frame2.data.size(), expected);
    EXPECT_EQ(frame3.data.size(), expected);

    // Data pointers must be distinct (each frame owns its own allocation)
    EXPECT_NE(frame1.data.data(), frame2.data.data());
    EXPECT_NE(frame2.data.data(), frame3.data.data());
    EXPECT_NE(frame1.data.data(), frame3.data.data());
}

// ── Grayscale frame data size ──────────────────────────────────
TEST_F(CameraLifetimeTest, GrayscaleFrameDataSize) {
    // SimulatedCamera uses 1 channel for small resolutions (stereo mode)
    cam_.open(640, 480, 10);
    auto frame = cam_.capture();
    ASSERT_TRUE(frame.valid);
    EXPECT_EQ(frame.channels, 1u);

    const size_t expected_size = static_cast<size_t>(frame.width) * frame.height * frame.channels;
    EXPECT_EQ(frame.data.size(), expected_size);
}

// ── Default CapturedFrame has empty data ───────────────────────
TEST(CapturedFrameDefault, DefaultConstructedHasEmptyData) {
    drone::hal::CapturedFrame frame;
    EXPECT_TRUE(frame.data.empty());
    EXPECT_FALSE(frame.valid);
    EXPECT_EQ(frame.width, 0u);
    EXPECT_EQ(frame.height, 0u);
}

// ── Frame data survives after camera close ─────────────────────
TEST_F(CameraLifetimeTest, FrameDataSurvivesAfterClose) {
    cam_.open(64, 64, 10);
    auto frame = cam_.capture();
    ASSERT_TRUE(frame.valid);
    ASSERT_FALSE(frame.data.empty());

    const size_t data_size = frame.data.size();

    // Close the camera — frame data must remain valid (owned by frame)
    cam_.close();

    EXPECT_EQ(frame.data.size(), data_size);
    // Verify we can still read the data without crashing
    uint8_t sum = 0;
    for (uint8_t byte : frame.data) {
        sum ^= byte;
    }
    (void)sum;  // just verifying access doesn't crash
}

}  // namespace
