// tests/test_hal.cpp
// Unit tests for the Hardware Abstraction Layer (HAL).
// Tests interfaces, simulated backends, and factory creation.
#include <gtest/gtest.h>
#include "hal/hal_factory.h"
#include "hal/icamera.h"
#include "hal/ifc_link.h"
#include "hal/igcs_link.h"
#include "hal/igimbal.h"
#include "hal/iimu_source.h"
#include "util/config.h"

#include <fstream>
#include <cstdio>

// ═══════════════════════════════════════════════════════════
// Helper: create a temporary config file
// ═══════════════════════════════════════════════════════════
static std::string create_temp_config(const std::string& json_content) {
    std::string path = "/tmp/test_hal_config.json";
    std::ofstream ofs(path);
    ofs << json_content;
    ofs.close();
    return path;
}

// ═══════════════════════════════════════════════════════════
// Factory Tests
// ═══════════════════════════════════════════════════════════
class HALFactoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto path = create_temp_config(R"({
            "video_capture": {
                "mission_cam": { "backend": "simulated", "width": 320, "height": 240, "fps": 10 },
                "stereo_cam":  { "backend": "simulated", "width": 160, "height": 120, "fps": 10 }
            },
            "comms": {
                "mavlink": { "backend": "simulated", "serial_port": "/dev/test", "baud_rate": 115200 },
                "gcs":     { "backend": "simulated", "udp_port": 14550 }
            },
            "payload_manager": {
                "gimbal": { "backend": "simulated" }
            },
            "slam": {
                "imu": { "backend": "simulated" }
            }
        })");
        cfg_.load(path);
    }

    drone::Config cfg_;
};

TEST_F(HALFactoryTest, CreateCameraReturnsNonNull) {
    auto cam = drone::hal::create_camera(cfg_, "video_capture.mission_cam");
    ASSERT_NE(cam, nullptr);
    EXPECT_EQ(cam->name(), "SimulatedCamera");
}

TEST_F(HALFactoryTest, CreateFCLinkReturnsNonNull) {
    auto fc = drone::hal::create_fc_link(cfg_, "comms.mavlink");
    ASSERT_NE(fc, nullptr);
    EXPECT_EQ(fc->name(), "SimulatedFCLink");
}

TEST_F(HALFactoryTest, CreateGCSLinkReturnsNonNull) {
    auto gcs = drone::hal::create_gcs_link(cfg_, "comms.gcs");
    ASSERT_NE(gcs, nullptr);
    EXPECT_EQ(gcs->name(), "SimulatedGCSLink");
}

TEST_F(HALFactoryTest, CreateGimbalReturnsNonNull) {
    auto gimbal = drone::hal::create_gimbal(cfg_, "payload_manager.gimbal");
    ASSERT_NE(gimbal, nullptr);
    EXPECT_EQ(gimbal->name(), "SimulatedGimbal");
}

TEST_F(HALFactoryTest, CreateIMUSourceReturnsNonNull) {
    auto imu = drone::hal::create_imu_source(cfg_, "slam.imu");
    ASSERT_NE(imu, nullptr);
    EXPECT_EQ(imu->name(), "SimulatedIMU");
}

TEST_F(HALFactoryTest, UnknownBackendThrows) {
    auto path = create_temp_config(R"({
        "video_capture": { "mission_cam": { "backend": "nonexistent" } }
    })");
    drone::Config bad_cfg;
    bad_cfg.load(path);
    EXPECT_THROW(drone::hal::create_camera(bad_cfg, "video_capture.mission_cam"),
                 std::runtime_error);
}

TEST_F(HALFactoryTest, MissingBackendDefaultsToSimulated) {
    auto path = create_temp_config(R"({
        "video_capture": { "mission_cam": { "width": 640 } }
    })");
    drone::Config no_backend_cfg;
    no_backend_cfg.load(path);
    auto cam = drone::hal::create_camera(no_backend_cfg, "video_capture.mission_cam");
    ASSERT_NE(cam, nullptr);
    EXPECT_EQ(cam->name(), "SimulatedCamera");
}

// ═══════════════════════════════════════════════════════════
// SimulatedCamera Tests
// ═══════════════════════════════════════════════════════════
class SimulatedCameraTest : public ::testing::Test {
protected:
    void SetUp() override {
        cam_ = std::make_unique<drone::hal::SimulatedCamera>();
    }
    std::unique_ptr<drone::hal::SimulatedCamera> cam_;
};

TEST_F(SimulatedCameraTest, OpenAndClose) {
    EXPECT_FALSE(cam_->is_open());
    EXPECT_TRUE(cam_->open(640, 480, 30));
    EXPECT_TRUE(cam_->is_open());
    cam_->close();
    EXPECT_FALSE(cam_->is_open());
}

TEST_F(SimulatedCameraTest, CaptureReturnsValidFrame) {
    cam_->open(320, 240, 10);
    auto frame = cam_->capture();
    EXPECT_TRUE(frame.valid);
    EXPECT_EQ(frame.width, 320u);
    EXPECT_EQ(frame.height, 240u);
    EXPECT_GT(frame.timestamp_ns, 0u);
    EXPECT_EQ(frame.sequence, 0u);
    EXPECT_NE(frame.data, nullptr);
}

TEST_F(SimulatedCameraTest, SequenceIncrementsEachCapture) {
    cam_->open(64, 64, 10);
    auto f1 = cam_->capture();
    auto f2 = cam_->capture();
    auto f3 = cam_->capture();
    EXPECT_EQ(f1.sequence, 0u);
    EXPECT_EQ(f2.sequence, 1u);
    EXPECT_EQ(f3.sequence, 2u);
}

TEST_F(SimulatedCameraTest, CaptureWhenClosedReturnsInvalid) {
    auto frame = cam_->capture();
    EXPECT_FALSE(frame.valid);
}

// ═══════════════════════════════════════════════════════════
// SimulatedFCLink Tests
// ═══════════════════════════════════════════════════════════
class SimulatedFCLinkTest : public ::testing::Test {
protected:
    void SetUp() override {
        fc_ = std::make_unique<drone::hal::SimulatedFCLink>();
    }
    std::unique_ptr<drone::hal::SimulatedFCLink> fc_;
};

TEST_F(SimulatedFCLinkTest, OpenAndConnect) {
    EXPECT_FALSE(fc_->is_connected());
    EXPECT_TRUE(fc_->open("/dev/test", 115200));
    EXPECT_TRUE(fc_->is_connected());
    fc_->close();
    EXPECT_FALSE(fc_->is_connected());
}

TEST_F(SimulatedFCLinkTest, SendTrajectoryWhenConnected) {
    fc_->open("/dev/test", 115200);
    EXPECT_TRUE(fc_->send_trajectory(1.0f, 2.0f, 3.0f, 0.5f));
}

TEST_F(SimulatedFCLinkTest, SendTrajectoryWhenDisconnectedFails) {
    EXPECT_FALSE(fc_->send_trajectory(1.0f, 2.0f, 3.0f, 0.5f));
}

TEST_F(SimulatedFCLinkTest, ArmDisarm) {
    fc_->open("/dev/test", 115200);
    fc_->send_arm(true);
    auto state = fc_->receive_state();
    EXPECT_TRUE(state.armed);

    fc_->send_arm(false);
    state = fc_->receive_state();
    EXPECT_FALSE(state.armed);
}

TEST_F(SimulatedFCLinkTest, ReceiveStateHasValidTimestamp) {
    fc_->open("/dev/test", 115200);
    auto state = fc_->receive_state();
    EXPECT_GT(state.timestamp_ns, 0u);
    EXPECT_GT(state.battery_percent, 0.0f);
    EXPECT_GT(state.satellites, 0);
}

TEST_F(SimulatedFCLinkTest, ModeChange) {
    fc_->open("/dev/test", 115200);
    fc_->send_mode(2);  // AUTO
    auto state = fc_->receive_state();
    EXPECT_EQ(state.flight_mode, 2);
}

// ═══════════════════════════════════════════════════════════
// SimulatedGCSLink Tests
// ═══════════════════════════════════════════════════════════
TEST(SimulatedGCSLinkTest, OpenAndConnect) {
    drone::hal::SimulatedGCSLink gcs;
    EXPECT_FALSE(gcs.is_connected());
    EXPECT_TRUE(gcs.open("0.0.0.0", 14550));
    EXPECT_TRUE(gcs.is_connected());
    gcs.close();
    EXPECT_FALSE(gcs.is_connected());
}

TEST(SimulatedGCSLinkTest, SendTelemetryWhenConnected) {
    drone::hal::SimulatedGCSLink gcs;
    gcs.open("0.0.0.0", 14550);
    EXPECT_TRUE(gcs.send_telemetry(47.0f, 8.0f, 100.0f, 85.0f, 1));
}

TEST(SimulatedGCSLinkTest, SendTelemetryWhenDisconnectedFails) {
    drone::hal::SimulatedGCSLink gcs;
    EXPECT_FALSE(gcs.send_telemetry(47.0f, 8.0f, 100.0f, 85.0f, 1));
}

TEST(SimulatedGCSLinkTest, PollCommandInitiallyNoCommand) {
    drone::hal::SimulatedGCSLink gcs;
    gcs.open("0.0.0.0", 14550);
    auto cmd = gcs.poll_command();
    // RTL comes after 120s, so immediate poll should be invalid
    EXPECT_FALSE(cmd.valid);
}

// ═══════════════════════════════════════════════════════════
// SimulatedGimbal Tests
// ═══════════════════════════════════════════════════════════
class SimulatedGimbalTest : public ::testing::Test {
protected:
    void SetUp() override {
        gimbal_ = std::make_unique<drone::hal::SimulatedGimbal>();
    }
    std::unique_ptr<drone::hal::SimulatedGimbal> gimbal_;
};

TEST_F(SimulatedGimbalTest, InitAndState) {
    EXPECT_FALSE(gimbal_->is_initialised());
    EXPECT_TRUE(gimbal_->init());
    EXPECT_TRUE(gimbal_->is_initialised());
}

TEST_F(SimulatedGimbalTest, SetTargetAndUpdate) {
    gimbal_->init();
    gimbal_->set_target(-45.0f, 90.0f);
    // Single large step
    gimbal_->update(10.0f);
    auto state = gimbal_->state();
    EXPECT_NEAR(state.pitch, -45.0f, 1.0f);
    EXPECT_NEAR(state.yaw, 90.0f, 1.0f);
}

TEST_F(SimulatedGimbalTest, SlewRateLimiting) {
    gimbal_->init();
    gimbal_->set_target(-90.0f, 0.0f);
    // 60 deg/s max, 0.1s step = 6 deg max
    gimbal_->update(0.1f);
    auto state = gimbal_->state();
    EXPECT_GT(state.pitch, -7.0f);  // didn't jump to -90
    EXPECT_LE(state.pitch, 0.0f);   // moved toward target
}

TEST_F(SimulatedGimbalTest, CaptureImage) {
    gimbal_->init();
    auto ts = gimbal_->capture_image();
    EXPECT_GT(ts, 0u);
}

TEST_F(SimulatedGimbalTest, Recording) {
    gimbal_->init();
    EXPECT_FALSE(gimbal_->is_recording());
    gimbal_->start_recording();
    EXPECT_TRUE(gimbal_->is_recording());
    gimbal_->stop_recording();
    EXPECT_FALSE(gimbal_->is_recording());
}

TEST_F(SimulatedGimbalTest, PitchClampedToRange) {
    gimbal_->init();
    gimbal_->set_target(-200.0f, 0.0f);  // below -90 min
    gimbal_->update(100.0f);  // large dt to reach target
    auto state = gimbal_->state();
    EXPECT_GE(state.pitch, -90.0f);  // clamped
}

// ═══════════════════════════════════════════════════════════
// SimulatedIMU Tests
// ═══════════════════════════════════════════════════════════
TEST(SimulatedIMUTest, InitAndRead) {
    drone::hal::SimulatedIMU imu;
    EXPECT_FALSE(imu.is_active());
    EXPECT_TRUE(imu.init(400));
    EXPECT_TRUE(imu.is_active());

    auto reading = imu.read();
    EXPECT_TRUE(reading.valid);
    EXPECT_GT(reading.timestamp, 0.0);
    // Gravity ~9.81 on z-axis
    EXPECT_NEAR(reading.accel.z(), 9.81, 0.5);
}

TEST(SimulatedIMUTest, ReadWhenNotInitReturnsInvalid) {
    drone::hal::SimulatedIMU imu;
    auto reading = imu.read();
    EXPECT_FALSE(reading.valid);
}

TEST(SimulatedIMUTest, ConsecutiveReadsHaveIncreasingTimestamps) {
    drone::hal::SimulatedIMU imu;
    imu.init(400);
    auto r1 = imu.read();
    auto r2 = imu.read();
    EXPECT_GE(r2.timestamp, r1.timestamp);
}
