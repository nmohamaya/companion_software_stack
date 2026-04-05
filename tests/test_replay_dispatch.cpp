// tests/test_replay_dispatch.cpp
// Unit tests for replay dispatch logic (Issue #352).
// Tests the reusable dispatch_record() and calculate_replay_delay() functions
// from common/recorder/include/recorder/replay_dispatch.h.
#include "recorder/replay_dispatch.h"

#include <cmath>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::ipc;
using namespace drone::recorder;

// ═══════════════════════════════════════════════════════════
// Test handler — records which callbacks were invoked
// ═══════════════════════════════════════════════════════════
class TestHandler : public IReplayHandler {
public:
    int  pose_count       = 0;
    int  detections_count = 0;
    int  fc_state_count   = 0;
    int  trajectory_count = 0;
    int  health_count     = 0;
    int  payload_count    = 0;
    Pose last_pose{};

    void on_pose(const Pose& msg) override {
        ++pose_count;
        last_pose = msg;
    }
    void on_detections(const DetectedObjectList& /*msg*/) override { ++detections_count; }
    void on_fc_state(const FCState& /*msg*/) override { ++fc_state_count; }
    void on_trajectory(const TrajectoryCmd& /*msg*/) override { ++trajectory_count; }
    void on_health(const SystemHealth& /*msg*/) override { ++health_count; }
    void on_payload(const PayloadStatus& /*msg*/) override { ++payload_count; }

    [[nodiscard]] int total() const {
        return pose_count + detections_count + fc_state_count + trajectory_count + health_count +
               payload_count;
    }
};

// ═══════════════════════════════════════════════════════════
// Helper: create a valid RecordEntry for a given message type
// ═══════════════════════════════════════════════════════════
template<typename T>
RecordEntry make_entry(WireMessageType msg_type, const T& msg) {
    static_assert(std::is_trivially_copyable_v<T>);
    RecordEntry entry;
    entry.header.wire_header.magic        = kWireMagic;
    entry.header.wire_header.version      = kWireVersion;
    entry.header.wire_header.msg_type     = msg_type;
    entry.header.wire_header.payload_size = static_cast<uint32_t>(sizeof(T));
    entry.header.wire_header.timestamp_ns = 1000;
    entry.header.topic_name_len           = 5;
    entry.topic_name                      = "/test";
    entry.payload.resize(sizeof(T));
    const auto* src = reinterpret_cast<const uint8_t*>(&msg);
    std::copy(src, src + sizeof(T), entry.payload.data());
    return entry;
}

// ═══════════════════════════════════════════════════════════
// Dispatch: each message type
// ═══════════════════════════════════════════════════════════
TEST(ReplayDispatch, DispatchesSlamPose) {
    TestHandler handler;
    Pose        pose{};
    pose.translation[0] = 1.0;
    pose.translation[1] = 2.0;
    pose.translation[2] = 3.0;

    auto entry  = make_entry(WireMessageType::SLAM_POSE, pose);
    auto result = dispatch_record(entry, handler);

    EXPECT_TRUE(result.dispatched);
    EXPECT_EQ(result.msg_type, WireMessageType::SLAM_POSE);
    EXPECT_EQ(handler.pose_count, 1);
    EXPECT_DOUBLE_EQ(handler.last_pose.translation[0], 1.0);
    EXPECT_DOUBLE_EQ(handler.last_pose.translation[1], 2.0);
    EXPECT_DOUBLE_EQ(handler.last_pose.translation[2], 3.0);
}

TEST(ReplayDispatch, DispatchesDetections) {
    TestHandler        handler;
    DetectedObjectList dets{};

    auto result = dispatch_record(make_entry(WireMessageType::DETECTIONS, dets), handler);

    EXPECT_TRUE(result.dispatched);
    EXPECT_EQ(handler.detections_count, 1);
}

TEST(ReplayDispatch, DispatchesFCState) {
    TestHandler handler;
    FCState     fc{};
    fc.armed = true;

    auto result = dispatch_record(make_entry(WireMessageType::FC_STATE, fc), handler);

    EXPECT_TRUE(result.dispatched);
    EXPECT_EQ(handler.fc_state_count, 1);
}

TEST(ReplayDispatch, DispatchesTrajectoryCmd) {
    TestHandler   handler;
    TrajectoryCmd traj{};
    traj.valid = true;

    auto result = dispatch_record(make_entry(WireMessageType::TRAJECTORY_CMD, traj), handler);

    EXPECT_TRUE(result.dispatched);
    EXPECT_EQ(handler.trajectory_count, 1);
}

TEST(ReplayDispatch, DispatchesSystemHealth) {
    TestHandler  handler;
    SystemHealth health{};

    auto result = dispatch_record(make_entry(WireMessageType::SYSTEM_HEALTH, health), handler);

    EXPECT_TRUE(result.dispatched);
    EXPECT_EQ(handler.health_count, 1);
}

TEST(ReplayDispatch, DispatchesPayloadStatus) {
    TestHandler   handler;
    PayloadStatus payload{};

    auto result = dispatch_record(make_entry(WireMessageType::PAYLOAD_STATUS, payload), handler);

    EXPECT_TRUE(result.dispatched);
    EXPECT_EQ(handler.payload_count, 1);
}

// ═══════════════════════════════════════════════════════════
// Dispatch: error cases
// ═══════════════════════════════════════════════════════════
TEST(ReplayDispatch, WrongPayloadSizeNotDispatched) {
    TestHandler handler;
    RecordEntry entry;
    entry.header.wire_header.magic    = kWireMagic;
    entry.header.wire_header.version  = kWireVersion;
    entry.header.wire_header.msg_type = WireMessageType::SLAM_POSE;
    // Wrong size — too small
    entry.payload.resize(4, 0);

    auto result = dispatch_record(entry, handler);

    EXPECT_FALSE(result.dispatched);
    EXPECT_EQ(result.msg_type, WireMessageType::SLAM_POSE);
    EXPECT_EQ(handler.total(), 0);
}

TEST(ReplayDispatch, UnknownMessageTypeNotDispatched) {
    TestHandler handler;
    RecordEntry entry;
    entry.header.wire_header.magic    = kWireMagic;
    entry.header.wire_header.version  = kWireVersion;
    entry.header.wire_header.msg_type = WireMessageType::UNKNOWN;
    entry.payload.resize(32, 0);

    auto result = dispatch_record(entry, handler);

    EXPECT_FALSE(result.dispatched);
    EXPECT_EQ(result.msg_type, WireMessageType::UNKNOWN);
    EXPECT_EQ(handler.total(), 0);
}

TEST(ReplayDispatch, EmptyPayloadNotDispatched) {
    TestHandler handler;
    RecordEntry entry;
    entry.header.wire_header.magic    = kWireMagic;
    entry.header.wire_header.version  = kWireVersion;
    entry.header.wire_header.msg_type = WireMessageType::FC_STATE;
    // Empty payload

    auto result = dispatch_record(entry, handler);

    EXPECT_FALSE(result.dispatched);
    EXPECT_EQ(handler.total(), 0);
}

// ═══════════════════════════════════════════════════════════
// Dispatch: template (non-virtual) callback struct
// ═══════════════════════════════════════════════════════════
TEST(ReplayDispatch, TemplateCallbacksWork) {
    struct PlainCallbacks {
        int  count = 0;
        void on_pose(const Pose&) { ++count; }
        void on_detections(const DetectedObjectList&) { ++count; }
        void on_fc_state(const FCState&) { ++count; }
        void on_trajectory(const TrajectoryCmd&) { ++count; }
        void on_health(const SystemHealth&) { ++count; }
        void on_payload(const PayloadStatus&) { ++count; }
    };

    PlainCallbacks cb;
    Pose           pose{};
    auto           result = dispatch_record(make_entry(WireMessageType::SLAM_POSE, pose), cb);

    EXPECT_TRUE(result.dispatched);
    EXPECT_EQ(cb.count, 1);
}

// ═══════════════════════════════════════════════════════════
// Replay delay calculation
// ═══════════════════════════════════════════════════════════
TEST(ReplayDelay, NormalSpeed) {
    // 1 second delta at 1x speed = 1 second delay
    auto delay = calculate_replay_delay(2'000'000'000, 1'000'000'000, 1.0f);
    EXPECT_EQ(delay.count(), 1'000'000'000);
}

TEST(ReplayDelay, DoubleSpeed) {
    // 1 second delta at 2x speed = 0.5 second delay
    auto delay = calculate_replay_delay(2'000'000'000, 1'000'000'000, 2.0f);
    EXPECT_EQ(delay.count(), 500'000'000);
}

TEST(ReplayDelay, ZeroSpeedNoDelay) {
    auto delay = calculate_replay_delay(2'000'000'000, 1'000'000'000, 0.0f);
    EXPECT_EQ(delay.count(), 0);
}

TEST(ReplayDelay, NegativeSpeedNoDelay) {
    auto delay = calculate_replay_delay(2'000'000'000, 1'000'000'000, -1.0f);
    EXPECT_EQ(delay.count(), 0);
}

TEST(ReplayDelay, NaNSpeedNoDelay) {
    auto delay = calculate_replay_delay(2'000'000'000, 1'000'000'000, std::nanf(""));
    EXPECT_EQ(delay.count(), 0);
}

TEST(ReplayDelay, OutOfOrderTimestampClampsToZero) {
    // record_ts < first_ts — should clamp delta to 0, not go negative
    auto delay = calculate_replay_delay(500'000'000, 1'000'000'000, 1.0f);
    EXPECT_EQ(delay.count(), 0);
}

TEST(ReplayDelay, SameTimestampZeroDelay) {
    auto delay = calculate_replay_delay(1'000'000'000, 1'000'000'000, 1.0f);
    EXPECT_EQ(delay.count(), 0);
}

TEST(ReplayDelay, HalfSpeed) {
    // 1 second delta at 0.5x speed = 2 second delay
    auto delay = calculate_replay_delay(2'000'000'000, 1'000'000'000, 0.5f);
    EXPECT_EQ(delay.count(), 2'000'000'000);
}
