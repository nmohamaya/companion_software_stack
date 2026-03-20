// tests/test_ipc_validation.cpp
// Unit tests for IPC struct validate() methods (Issues #179, #181, #185).
#include "ipc/ipc_types.h"

#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// VideoFrame validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, VideoFrameValidDimensions) {
    auto f      = std::make_unique<VideoFrame>();
    f->width    = 1920;
    f->height   = 1080;
    f->channels = 3;
    EXPECT_TRUE(f->validate());
}

TEST(IpcValidation, VideoFrameZeroWidth) {
    auto f      = std::make_unique<VideoFrame>();
    f->width    = 0;
    f->height   = 1080;
    f->channels = 3;
    EXPECT_FALSE(f->validate());
}

TEST(IpcValidation, VideoFrameOversizedDimensions) {
    auto f      = std::make_unique<VideoFrame>();
    f->width    = 3840;
    f->height   = 2160;
    f->channels = 3;
    // 3840*2160*3 = 24,883,200 > sizeof(pixel_data) = 6,220,800
    EXPECT_FALSE(f->validate());
}

TEST(IpcValidation, VideoFrameSmallValidDimensions) {
    auto f      = std::make_unique<VideoFrame>();
    f->width    = 640;
    f->height   = 480;
    f->channels = 3;
    EXPECT_TRUE(f->validate());
}

// ═══════════════════════════════════════════════════════════
// StereoFrame validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, StereoFrameValid) {
    StereoFrame f{};
    f.width  = 640;
    f.height = 480;
    EXPECT_TRUE(f.validate());
}

TEST(IpcValidation, StereoFrameOversized) {
    StereoFrame f{};
    f.width  = 1280;
    f.height = 720;
    EXPECT_FALSE(f.validate());
}

// ═══════════════════════════════════════════════════════════
// DetectedObject validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, DetectedObjectValid) {
    DetectedObject o{};
    o.confidence = 0.85f;
    o.position_x = 1.0f;
    o.position_y = 2.0f;
    o.position_z = 3.0f;
    o.velocity_x = 0.0f;
    o.velocity_y = 0.0f;
    o.velocity_z = 0.0f;
    EXPECT_TRUE(o.validate());
}

TEST(IpcValidation, DetectedObjectNaNPosition) {
    DetectedObject o{};
    o.confidence = 0.5f;
    o.position_x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(o.validate());
}

TEST(IpcValidation, DetectedObjectInfVelocity) {
    DetectedObject o{};
    o.confidence = 0.5f;
    o.position_x = 1.0f;
    o.position_y = 2.0f;
    o.position_z = 3.0f;
    o.velocity_x = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(o.validate());
}

TEST(IpcValidation, DetectedObjectConfidenceOutOfRange) {
    DetectedObject o{};
    o.confidence = 1.5f;  // > 1.0
    o.position_x = 1.0f;
    o.position_y = 2.0f;
    o.position_z = 3.0f;
    EXPECT_FALSE(o.validate());
}

TEST(IpcValidation, DetectedObjectNegativeConfidence) {
    DetectedObject o{};
    o.confidence = -0.1f;
    o.position_x = 1.0f;
    o.position_y = 2.0f;
    o.position_z = 3.0f;
    EXPECT_FALSE(o.validate());
}

// ═══════════════════════════════════════════════════════════
// DetectedObjectList validation (#179)
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, DetectedObjectListValid) {
    DetectedObjectList list{};
    list.num_objects           = 1;
    list.objects[0]            = {};
    list.objects[0].confidence = 0.9f;
    list.objects[0].position_x = 1.0f;
    list.objects[0].position_y = 2.0f;
    list.objects[0].position_z = 3.0f;
    EXPECT_TRUE(list.validate());
}

TEST(IpcValidation, DetectedObjectListNumObjectsExceedsMax) {
    DetectedObjectList list{};
    list.num_objects = MAX_DETECTED_OBJECTS + 1;
    EXPECT_FALSE(list.validate());
}

TEST(IpcValidation, DetectedObjectListCorruptedNumObjects) {
    DetectedObjectList list{};
    list.num_objects = 1000;  // way beyond array size
    EXPECT_FALSE(list.validate());
}

TEST(IpcValidation, DetectedObjectListInvalidObject) {
    DetectedObjectList list{};
    list.num_objects           = 1;
    list.objects[0].confidence = 0.5f;
    list.objects[0].position_x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(list.validate());
}

TEST(IpcValidation, DetectedObjectListEmpty) {
    DetectedObjectList list{};
    list.num_objects = 0;
    EXPECT_TRUE(list.validate());
}

// ═══════════════════════════════════════════════════════════
// Pose validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, PoseValid) {
    Pose p{};
    p.translation[0] = 1.0;
    p.translation[1] = 2.0;
    p.translation[2] = 3.0;
    p.quaternion[0]  = 1.0;
    p.quality        = 2;
    EXPECT_TRUE(p.validate());
}

TEST(IpcValidation, PoseNaNTranslation) {
    Pose p{};
    p.quaternion[0]  = 1.0;
    p.translation[1] = std::numeric_limits<double>::quiet_NaN();
    p.quality        = 2;
    EXPECT_FALSE(p.validate());
}

TEST(IpcValidation, PoseGroundTruthQuality) {
    Pose p{};
    p.translation[0] = 1.0;
    p.quaternion[0]  = 1.0;
    p.quality        = 3;  // ground truth (Gazebo backends)
    EXPECT_TRUE(p.validate());
}

TEST(IpcValidation, PoseInvalidQuality) {
    Pose p{};
    p.quaternion[0] = 1.0;
    p.quality       = 5;
    EXPECT_FALSE(p.validate());
}

// ═══════════════════════════════════════════════════════════
// TrajectoryCmd validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, TrajectoryCmdValidStop) {
    TrajectoryCmd t{};
    t.valid = false;  // stop command — always valid
    EXPECT_TRUE(t.validate());
}

TEST(IpcValidation, TrajectoryCmdValidTarget) {
    TrajectoryCmd t{};
    t.valid      = true;
    t.target_x   = 10.0f;
    t.target_y   = 20.0f;
    t.target_z   = 5.0f;
    t.target_yaw = 1.57f;
    EXPECT_TRUE(t.validate());
}

TEST(IpcValidation, TrajectoryCmdNaNTarget) {
    TrajectoryCmd t{};
    t.valid    = true;
    t.target_x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(t.validate());
}

// ═══════════════════════════════════════════════════════════
// FCState validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, FCStateValid) {
    FCState s{};
    s.battery_voltage   = 16.8f;
    s.battery_remaining = 85.0f;
    s.roll              = 0.0f;
    s.pitch             = 0.0f;
    s.yaw               = 0.0f;
    s.vx                = 0.0f;
    s.vy                = 0.0f;
    s.vz                = 0.0f;
    EXPECT_TRUE(s.validate());
}

TEST(IpcValidation, FCStateBatteryVoltageNaN) {
    FCState s{};
    s.battery_voltage   = std::numeric_limits<float>::quiet_NaN();
    s.battery_remaining = 85.0f;
    EXPECT_FALSE(s.validate());
}

TEST(IpcValidation, FCStateBatteryVoltageNegative) {
    FCState s{};
    s.battery_voltage   = -1.0f;
    s.battery_remaining = 85.0f;
    EXPECT_FALSE(s.validate());
}

TEST(IpcValidation, FCStateBatteryRemainingOver100) {
    FCState s{};
    s.battery_voltage   = 16.8f;
    s.battery_remaining = 150.0f;
    EXPECT_FALSE(s.validate());
}

// ═══════════════════════════════════════════════════════════
// GCSCommand validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, GCSCommandValid) {
    GCSCommand c{};
    c.command = GCSCommandType::RTL;
    EXPECT_TRUE(c.validate());
}

TEST(IpcValidation, GCSCommandInvalidEnum) {
    GCSCommand c{};
    // Force an out-of-range enum value
    c.command = static_cast<GCSCommandType>(255);
    EXPECT_FALSE(c.validate());
}

// ═══════════════════════════════════════════════════════════
// IpcWaypoint validation (#177, #178)
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, IpcWaypointValid) {
    IpcWaypoint w{};
    w.x      = 10.0f;
    w.y      = 20.0f;
    w.z      = 5.0f;
    w.radius = 2.0f;
    w.speed  = 3.0f;
    EXPECT_TRUE(w.validate());
}

TEST(IpcValidation, IpcWaypointNaNCoordinate) {
    IpcWaypoint w{};
    w.x      = std::numeric_limits<float>::quiet_NaN();
    w.y      = 20.0f;
    w.z      = 5.0f;
    w.radius = 2.0f;
    w.speed  = 3.0f;
    EXPECT_FALSE(w.validate());
}

TEST(IpcValidation, IpcWaypointInfCoordinate) {
    IpcWaypoint w{};
    w.x      = 10.0f;
    w.y      = 20.0f;
    w.z      = std::numeric_limits<float>::infinity();
    w.radius = 2.0f;
    w.speed  = 3.0f;
    EXPECT_FALSE(w.validate());
}

TEST(IpcValidation, IpcWaypointZeroSpeed) {
    IpcWaypoint w{};
    w.x      = 10.0f;
    w.y      = 20.0f;
    w.z      = 5.0f;
    w.radius = 2.0f;
    w.speed  = 0.0f;
    EXPECT_FALSE(w.validate());
}

TEST(IpcValidation, IpcWaypointNegativeRadius) {
    IpcWaypoint w{};
    w.x      = 10.0f;
    w.y      = 20.0f;
    w.z      = 5.0f;
    w.radius = -1.0f;
    w.speed  = 3.0f;
    EXPECT_FALSE(w.validate());
}

TEST(IpcValidation, IpcWaypointNegativeSpeed) {
    IpcWaypoint w{};
    w.x      = 10.0f;
    w.y      = 20.0f;
    w.z      = 5.0f;
    w.radius = 2.0f;
    w.speed  = -5.0f;
    EXPECT_FALSE(w.validate());
}

// ═══════════════════════════════════════════════════════════
// MissionUpload validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, MissionUploadValid) {
    MissionUpload u{};
    u.num_waypoints = 2;
    u.waypoints[0]  = {1, 2, 3, 0, 2, 3, false};
    u.waypoints[1]  = {4, 5, 6, 0, 2, 3, false};
    EXPECT_TRUE(u.validate());
}

TEST(IpcValidation, MissionUploadTooManyWaypoints) {
    MissionUpload u{};
    u.num_waypoints = kMaxUploadWaypoints + 1;
    EXPECT_FALSE(u.validate());
}

TEST(IpcValidation, MissionUploadInvalidWaypoint) {
    MissionUpload u{};
    u.num_waypoints       = 1;
    u.waypoints[0].x      = std::numeric_limits<float>::quiet_NaN();
    u.waypoints[0].radius = 2.0f;
    u.waypoints[0].speed  = 3.0f;
    EXPECT_FALSE(u.validate());
}

TEST(IpcValidation, MissionUploadEmpty) {
    MissionUpload u{};
    u.num_waypoints = 0;
    EXPECT_TRUE(u.validate());
}

// ═══════════════════════════════════════════════════════════
// SystemHealth validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, SystemHealthValid) {
    SystemHealth h{};
    h.cpu_usage_percent    = 45.0f;
    h.memory_usage_percent = 60.0f;
    h.max_temp_c           = 55.0f;
    h.power_watts          = 15.0f;
    h.thermal_zone         = 1;
    h.num_processes        = 7;
    EXPECT_TRUE(h.validate());
}

TEST(IpcValidation, SystemHealthInvalidThermalZone) {
    SystemHealth h{};
    h.cpu_usage_percent    = 45.0f;
    h.memory_usage_percent = 60.0f;
    h.max_temp_c           = 55.0f;
    h.power_watts          = 15.0f;
    h.thermal_zone         = 10;  // invalid
    EXPECT_FALSE(h.validate());
}

TEST(IpcValidation, SystemHealthNaNTemp) {
    SystemHealth h{};
    h.max_temp_c = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(h.validate());
}

// ═══════════════════════════════════════════════════════════
// FCCommand validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, FCCommandValid) {
    FCCommand c{};
    c.command = FCCommandType::TAKEOFF;
    c.param1  = 10.0f;
    EXPECT_TRUE(c.validate());
}

TEST(IpcValidation, FCCommandInvalidEnum) {
    FCCommand c{};
    c.command = static_cast<FCCommandType>(255);
    c.param1  = 10.0f;
    EXPECT_FALSE(c.validate());
}

TEST(IpcValidation, FCCommandNaNParam) {
    FCCommand c{};
    c.command = FCCommandType::TAKEOFF;
    c.param1  = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(c.validate());
}

// ═══════════════════════════════════════════════════════════
// PayloadCommand validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, PayloadCommandValid) {
    PayloadCommand p{};
    p.gimbal_pitch = -90.0f;
    p.gimbal_yaw   = 0.0f;
    EXPECT_TRUE(p.validate());
}

TEST(IpcValidation, PayloadCommandNaNPitch) {
    PayloadCommand p{};
    p.gimbal_pitch = std::numeric_limits<float>::quiet_NaN();
    p.gimbal_yaw   = 0.0f;
    EXPECT_FALSE(p.validate());
}

TEST(IpcValidation, PayloadCommandInfYaw) {
    PayloadCommand p{};
    p.gimbal_pitch = 0.0f;
    p.gimbal_yaw   = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(p.validate());
}

// ═══════════════════════════════════════════════════════════
// PayloadStatus validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, PayloadStatusValid) {
    PayloadStatus s{};
    s.gimbal_pitch = -45.0f;
    s.gimbal_yaw   = 90.0f;
    EXPECT_TRUE(s.validate());
}

TEST(IpcValidation, PayloadStatusNaNPitch) {
    PayloadStatus s{};
    s.gimbal_pitch = std::numeric_limits<float>::quiet_NaN();
    s.gimbal_yaw   = 0.0f;
    EXPECT_FALSE(s.validate());
}

// ═══════════════════════════════════════════════════════════
// MissionStatus validation
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, MissionStatusValid) {
    MissionStatus m{};
    m.progress_percent = 50.0f;
    m.target_x         = 10.0f;
    m.target_y         = 20.0f;
    m.target_z         = 5.0f;
    m.battery_percent  = 80.0f;
    EXPECT_TRUE(m.validate());
}

TEST(IpcValidation, MissionStatusNaNProgress) {
    MissionStatus m{};
    m.progress_percent = std::numeric_limits<float>::quiet_NaN();
    m.target_x         = 10.0f;
    m.target_y         = 20.0f;
    m.target_z         = 5.0f;
    m.battery_percent  = 80.0f;
    EXPECT_FALSE(m.validate());
}

TEST(IpcValidation, MissionStatusInfTarget) {
    MissionStatus m{};
    m.progress_percent = 50.0f;
    m.target_x         = std::numeric_limits<float>::infinity();
    m.target_y         = 20.0f;
    m.target_z         = 5.0f;
    m.battery_percent  = 80.0f;
    EXPECT_FALSE(m.validate());
}

// ═══════════════════════════════════════════════════════════
// Trivially copyable static assertions still hold
// ═══════════════════════════════════════════════════════════
TEST(IpcValidation, StructsRemainTriviallyCopyable) {
    static_assert(std::is_trivially_copyable_v<VideoFrame>);
    static_assert(std::is_trivially_copyable_v<StereoFrame>);
    static_assert(std::is_trivially_copyable_v<DetectedObject>);
    static_assert(std::is_trivially_copyable_v<DetectedObjectList>);
    static_assert(std::is_trivially_copyable_v<Pose>);
    static_assert(std::is_trivially_copyable_v<TrajectoryCmd>);
    static_assert(std::is_trivially_copyable_v<FCState>);
    static_assert(std::is_trivially_copyable_v<GCSCommand>);
    static_assert(std::is_trivially_copyable_v<IpcWaypoint>);
    static_assert(std::is_trivially_copyable_v<MissionUpload>);
    static_assert(std::is_trivially_copyable_v<FCCommand>);
    static_assert(std::is_trivially_copyable_v<PayloadCommand>);
    static_assert(std::is_trivially_copyable_v<PayloadStatus>);
    static_assert(std::is_trivially_copyable_v<SystemHealth>);
    SUCCEED();  // static_asserts are the real test
}
