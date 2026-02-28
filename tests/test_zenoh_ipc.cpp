// tests/test_zenoh_ipc.cpp
// Unit tests for the Zenoh IPC backend and message bus factory.
//
// Four categories:
//   1. MessageBusFactory tests (always compiled — no Zenoh dependency)
//   2. Zenoh topic mapping tests (HAVE_ZENOH — ZenohMessageBus header required)
//   3. Zenoh pub/sub round-trip tests (HAVE_ZENOH — requires running Zenoh session)
//   4. Phase C — SHM provider zero-copy tests (HAVE_ZENOH)
//
// Build:
//   cmake -B build                          → runs category 1 only
//   cmake -B build -DENABLE_ZENOH=ON        → runs categories 1 + 2 + 3 + 4
#include <gtest/gtest.h>

#include "ipc/shm_message_bus.h"
#include "ipc/shm_types.h"
#include "ipc/message_bus_factory.h"

#ifdef HAVE_ZENOH
#include "ipc/zenoh_message_bus.h"
#include "ipc/zenoh_publisher.h"
#include "ipc/zenoh_subscriber.h"
#include "ipc/zenoh_session.h"
#endif

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Category 1: Topic mapping tests (always compiled)
// ═══════════════════════════════════════════════════════════

#ifdef HAVE_ZENOH

TEST(ZenohTopicMapping, VideoMissionCam) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/drone_mission_cam"),
              "drone/video/frame");
}

TEST(ZenohTopicMapping, VideoStereoCam) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/drone_stereo_cam"),
              "drone/video/stereo_frame");
}

TEST(ZenohTopicMapping, DetectedObjects) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/detected_objects"),
              "drone/perception/detections");
}

TEST(ZenohTopicMapping, SlamPose) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/slam_pose"),
              "drone/slam/pose");
}

TEST(ZenohTopicMapping, MissionStatus) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/mission_status"),
              "drone/mission/status");
}

TEST(ZenohTopicMapping, TrajectoryCmd) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/trajectory_cmd"),
              "drone/mission/trajectory");
}

TEST(ZenohTopicMapping, PayloadCommands) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/payload_commands"),
              "drone/mission/payload_command");
}

TEST(ZenohTopicMapping, FCCommands) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/fc_commands"),
              "drone/comms/fc_command");
}

TEST(ZenohTopicMapping, FCState) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/fc_state"),
              "drone/comms/fc_state");
}

TEST(ZenohTopicMapping, GCSCommands) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/gcs_commands"),
              "drone/comms/gcs_command");
}

TEST(ZenohTopicMapping, PayloadStatus) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/payload_status"),
              "drone/payload/status");
}

TEST(ZenohTopicMapping, SystemHealth) {
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/system_health"),
              "drone/monitor/health");
}

TEST(ZenohTopicMapping, AllTwelveSegmentsMapped) {
    // Verify every shm_names constant has a mapping
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::VIDEO_MISSION_CAM), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::VIDEO_STEREO_CAM), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::DETECTED_OBJECTS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::SLAM_POSE), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::MISSION_STATUS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::TRAJECTORY_CMD), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::PAYLOAD_COMMANDS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::FC_COMMANDS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::FC_STATE), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::GCS_COMMANDS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::PAYLOAD_STATUS), "");
    EXPECT_NE(ZenohMessageBus::to_key_expr(shm_names::SYSTEM_HEALTH), "");
}

TEST(ZenohTopicMapping, PassthroughZenohKeyExpr) {
    // Already a Zenoh key expression — no leading '/'
    EXPECT_EQ(ZenohMessageBus::to_key_expr("drone/test/data"),
              "drone/test/data");
}

TEST(ZenohTopicMapping, FallbackUnmappedShmName) {
    // Unknown SHM name — fallback: strip '/' and replace '_' with '/'
    EXPECT_EQ(ZenohMessageBus::to_key_expr("/some_unknown_topic"),
              "some/unknown/topic");
}

#endif  // HAVE_ZENOH

// ═══════════════════════════════════════════════════════════
// Category 1b: MessageBusFactory tests (always compiled)
// ═══════════════════════════════════════════════════════════

TEST(MessageBusFactory, DefaultCreatesShmBus) {
    auto bus = create_message_bus();
    EXPECT_TRUE(std::holds_alternative<std::unique_ptr<ShmMessageBus>>(bus));
}

TEST(MessageBusFactory, ExplicitShmCreatesShmBus) {
    auto bus = create_message_bus("shm");
    EXPECT_TRUE(std::holds_alternative<std::unique_ptr<ShmMessageBus>>(bus));
}

TEST(MessageBusFactory, ZenohRequestWithoutBuild) {
    // When HAVE_ZENOH is not defined, requesting "zenoh" should fall back
    // to SHM (with a warning). When HAVE_ZENOH IS defined, it should
    // create a ZenohMessageBus.
    auto bus = create_message_bus("zenoh");
#ifdef HAVE_ZENOH
    EXPECT_TRUE(
        std::holds_alternative<std::unique_ptr<ZenohMessageBus>>(bus));
#else
    EXPECT_TRUE(
        std::holds_alternative<std::unique_ptr<ShmMessageBus>>(bus));
#endif
}

TEST(MessageBusFactory, BusAdvertiseViaVariant) {
    auto bus = create_message_bus("shm");
    auto pub = bus_advertise<ShmPose>(bus, "/test_factory_pub_" +
                                     std::to_string(getpid()));
    ASSERT_NE(pub, nullptr);
    EXPECT_TRUE(pub->is_ready());
}

TEST(MessageBusFactory, BusSubscribeViaVariant) {
    // Create publisher first so subscriber can connect
    auto bus = create_message_bus("shm");
    auto topic = "/test_factory_sub_" + std::to_string(getpid());
    auto pub = bus_advertise<ShmPose>(bus, topic);
    ASSERT_NE(pub, nullptr);

    auto sub = bus_subscribe<ShmPose>(bus, topic, 5, 50);
    ASSERT_NE(sub, nullptr);
    EXPECT_TRUE(sub->is_connected());
}

// ═══════════════════════════════════════════════════════════
// Category 2: Zenoh pub/sub round-trip tests (HAVE_ZENOH only)
// ═══════════════════════════════════════════════════════════

#ifdef HAVE_ZENOH

// Small trivially-copyable test payload
struct ZenohTestPayload {
    uint64_t id{0};
    float value{0.0f};
    char tag[16]{};
};

TEST(ZenohSession, Opens) {
    auto& session = ZenohSession::instance();
    // Accessing session() triggers lazy open
    auto& s = session.session();
    EXPECT_TRUE(session.is_open());
    (void)s;
}

TEST(ZenohPublisher, Constructs) {
    ZenohPublisher<ZenohTestPayload> pub("drone/test/construct_pub");
    EXPECT_TRUE(pub.is_ready());
    EXPECT_EQ(pub.topic_name(), "drone/test/construct_pub");
}

TEST(ZenohSubscriber, Constructs) {
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/construct_sub");
    EXPECT_EQ(sub.topic_name(), "drone/test/construct_sub");
    // No data yet
    ZenohTestPayload msg;
    EXPECT_FALSE(sub.receive(msg));
    EXPECT_FALSE(sub.is_connected());
}

// ---------------------------------------------------------------------------
// Polling helpers — replace fixed sleep_for() with bounded retry loops
// so tests are not flaky under CI load.
// ---------------------------------------------------------------------------

/// Poll subscriber until a message is received or timeout is reached.
template <typename T>
bool poll_receive(drone::ipc::ZenohSubscriber<T>& sub, T& out,
                  uint64_t* ts = nullptr,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
    constexpr auto kInterval = std::chrono::milliseconds(5);
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (sub.receive(out, ts)) return true;
        std::this_thread::sleep_for(kInterval);
    }
    return false;
}

/// Publish (with retries) until the subscriber receives a message.
/// Handles the Zenoh discovery window during which messages are dropped
/// because the publisher and subscriber have not yet matched.
template <typename T>
bool publish_until_received(
        drone::ipc::ZenohPublisher<T>& pub, const T& msg,
        drone::ipc::ZenohSubscriber<T>& sub, T& out,
        uint64_t* ts = nullptr,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
    constexpr auto kPollInterval  = std::chrono::milliseconds(5);
    constexpr auto kRetryInterval = std::chrono::milliseconds(50);
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        pub.publish(msg);
        auto batch_end = std::min(
            std::chrono::steady_clock::now() + kRetryInterval, deadline);
        while (std::chrono::steady_clock::now() < batch_end) {
            if (sub.receive(out, ts)) return true;
            std::this_thread::sleep_for(kPollInterval);
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Round-trip tests
// ---------------------------------------------------------------------------

TEST(ZenohPubSub, SmallMessageRoundTrip) {
    ZenohPublisher<ZenohTestPayload> pub("drone/test/small_rt");
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/small_rt");

    ZenohTestPayload sent{42, 3.14f, {}};
    std::strncpy(sent.tag, "hello", sizeof(sent.tag));

    ZenohTestPayload received;
    uint64_t ts = 0;
    ASSERT_TRUE(publish_until_received(pub, sent, sub, received, &ts));
    EXPECT_EQ(received.id, 42u);
    EXPECT_FLOAT_EQ(received.value, 3.14f);
    EXPECT_STREQ(received.tag, "hello");
    EXPECT_GT(ts, 0u);
    EXPECT_TRUE(sub.is_connected());
}

TEST(ZenohPubSub, ShmPoseRoundTrip) {
    ZenohPublisher<ShmPose> pub("drone/test/pose_rt");
    ZenohSubscriber<ShmPose> sub("drone/test/pose_rt");

    ShmPose sent{};
    sent.timestamp_ns = 123456789;
    sent.translation[0] = 1.0;
    sent.translation[1] = 2.0;
    sent.translation[2] = 3.0;
    sent.quaternion[0] = 1.0;  // w
    sent.quality = 2;

    ShmPose received{};
    ASSERT_TRUE(publish_until_received(pub, sent, sub, received));
    EXPECT_EQ(received.timestamp_ns, 123456789u);
    EXPECT_DOUBLE_EQ(received.translation[0], 1.0);
    EXPECT_DOUBLE_EQ(received.translation[1], 2.0);
    EXPECT_DOUBLE_EQ(received.translation[2], 3.0);
    EXPECT_EQ(received.quality, 2u);
}

TEST(ZenohPubSub, LargeVideoFrameRoundTrip) {
    ZenohPublisher<ShmVideoFrame> pub("drone/test/video_rt");
    ZenohSubscriber<ShmVideoFrame> sub("drone/test/video_rt");

    // Heap-allocate — ShmVideoFrame is ~6 MB, too large for the stack.
    auto sent = std::make_unique<ShmVideoFrame>();
    sent->timestamp_ns = 999;
    sent->width = 1920;
    sent->height = 1080;
    sent->channels = 3;
    sent->sequence_number = 7;
    sent->pixel_data[0] = 0xAA;
    sent->pixel_data[1] = 0xBB;
    sent->pixel_data[sizeof(sent->pixel_data) - 1] = 0xFF;

    auto received = std::make_unique<ShmVideoFrame>();
    ASSERT_TRUE(publish_until_received(pub, *sent, sub, *received));
    EXPECT_EQ(received->width, 1920u);
    EXPECT_EQ(received->height, 1080u);
    EXPECT_EQ(received->channels, 3u);
    EXPECT_EQ(received->sequence_number, 7u);
    EXPECT_EQ(received->pixel_data[0], 0xAA);
    EXPECT_EQ(received->pixel_data[1], 0xBB);
    EXPECT_EQ(received->pixel_data[sizeof(received->pixel_data) - 1], 0xFF);
}

TEST(ZenohPubSub, MultipleTopics) {
    ZenohPublisher<ZenohTestPayload> pub1("drone/test/multi/a");
    ZenohPublisher<ZenohTestPayload> pub2("drone/test/multi/b");
    ZenohPublisher<ZenohTestPayload> pub3("drone/test/multi/c");
    ZenohSubscriber<ZenohTestPayload> sub1("drone/test/multi/a");
    ZenohSubscriber<ZenohTestPayload> sub2("drone/test/multi/b");
    ZenohSubscriber<ZenohTestPayload> sub3("drone/test/multi/c");

    ZenohTestPayload m1{1, 1.0f, {}};
    ZenohTestPayload m2{2, 2.0f, {}};
    ZenohTestPayload m3{3, 3.0f, {}};
    ZenohTestPayload r1{}, r2{}, r3{};

    // Poll each topic pair with publish retries to handle discovery latency.
    ASSERT_TRUE(publish_until_received(pub1, m1, sub1, r1));
    ASSERT_TRUE(publish_until_received(pub2, m2, sub2, r2));
    ASSERT_TRUE(publish_until_received(pub3, m3, sub3, r3));
    EXPECT_EQ(r1.id, 1u);
    EXPECT_EQ(r2.id, 2u);
    EXPECT_EQ(r3.id, 3u);
}

TEST(ZenohPubSub, NoData) {
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/no_data");
    ZenohTestPayload msg;
    EXPECT_FALSE(sub.receive(msg));
    EXPECT_FALSE(sub.is_connected());
}

TEST(ZenohPubSub, SequenceIncrementsOnPublish) {
    ZenohPublisher<ZenohTestPayload> pub("drone/test/seq");
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/seq");

    // First message — handles discovery latency via retry loop.
    ZenohTestPayload m1{10, 0.0f, {}};
    ZenohTestPayload r1;
    uint64_t ts1 = 0;
    ASSERT_TRUE(publish_until_received(pub, m1, sub, r1, &ts1));
    EXPECT_EQ(r1.id, 10u);

    // Second message — discovery already done, poll-receive is sufficient.
    ZenohTestPayload m2{20, 0.0f, {}};
    pub.publish(m2);
    ZenohTestPayload r2;
    uint64_t ts2 = 0;
    ASSERT_TRUE(poll_receive(sub, r2, &ts2));
    EXPECT_EQ(r2.id, 20u);
    EXPECT_GE(ts2, ts1);
}

TEST(ZenohMessageBus, AdvertiseCreatesPublisher) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmPose>("/slam_pose");
    ASSERT_NE(pub, nullptr);
    EXPECT_TRUE(pub->is_ready());
    // Should have mapped to Zenoh key expression
    EXPECT_EQ(pub->topic_name(), "drone/slam/pose");
}

TEST(ZenohMessageBus, SubscribeCreatesSubscriber) {
    ZenohMessageBus bus;
    auto sub = bus.subscribe<ShmPose>("/slam_pose");
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(sub->topic_name(), "drone/slam/pose");
}

TEST(ZenohMessageBus, SubscribeLazyCreatesSubscriber) {
    ZenohMessageBus bus;
    auto sub = bus.subscribe_lazy<ShmPose>("/slam_pose");
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(sub->topic_name(), "drone/slam/pose");
}

TEST(ZenohMessageBus, RoundTripViaFactory) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ZenohTestPayload>("/detected_objects");
    auto sub = bus.subscribe<ZenohTestPayload>("/detected_objects");
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);

    ZenohTestPayload sent{99, 2.71f, {}};
    ZenohTestPayload received;
    // Manual publish-and-poll loop (IPublisher/ISubscriber interfaces).
    bool ok = false;
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(5000);
    while (std::chrono::steady_clock::now() < deadline) {
        pub->publish(sent);
        auto batch = std::min(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(50),
            deadline);
        while (std::chrono::steady_clock::now() < batch) {
            if (sub->receive(received)) { ok = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (ok) break;
    }
    ASSERT_TRUE(ok);
    EXPECT_EQ(received.id, 99u);
    EXPECT_FLOAT_EQ(received.value, 2.71f);
}

// ═══════════════════════════════════════════════════════════
// Category 3: Per-channel migration round-trip tests (Phase B)
//
// Verify each of the 10 low-bandwidth channels works end-to-end
// through ZenohMessageBus with legacy SHM topic names.
// ═══════════════════════════════════════════════════════════

// ---------------------------------------------------------------------------
// Interface-level polling helper (works with IPublisher/ISubscriber)
// ---------------------------------------------------------------------------

template <typename T>
static bool publish_until_received_iface(
    IPublisher<T>& pub, const T& msg,
    ISubscriber<T>& sub, T& out,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
{
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        pub.publish(msg);
        auto batch = std::min(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(50),
            deadline);
        while (std::chrono::steady_clock::now() < batch) {
            if (sub.receive(out)) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    return false;
}

// --- Channel 1: ShmPose (P3 → P4, P5) -----------------------

TEST(ZenohMigration, Pose_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmPose>(shm_names::SLAM_POSE);
    auto sub = bus.subscribe<ShmPose>(shm_names::SLAM_POSE);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(pub->topic_name(), "drone/slam/pose");
    EXPECT_EQ(sub->topic_name(), "drone/slam/pose");

    ShmPose sent{};
    sent.timestamp_ns = 100;
    sent.translation[0] = 10.0;
    sent.translation[1] = 20.0;
    sent.translation[2] = 30.0;
    sent.quaternion[0] = 1.0;
    sent.velocity[0] = 1.5;
    sent.quality = 2;

    ShmPose received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 100u);
    EXPECT_DOUBLE_EQ(received.translation[0], 10.0);
    EXPECT_DOUBLE_EQ(received.translation[1], 20.0);
    EXPECT_DOUBLE_EQ(received.translation[2], 30.0);
    EXPECT_DOUBLE_EQ(received.quaternion[0], 1.0);
    EXPECT_DOUBLE_EQ(received.velocity[0], 1.5);
    EXPECT_EQ(received.quality, 2u);
}

// --- Channel 2: ShmFCState (P5 → P4, P7) --------------------

TEST(ZenohMigration, FCState_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmFCState>(shm_names::FC_STATE);
    auto sub = bus.subscribe<ShmFCState>(shm_names::FC_STATE);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(pub->topic_name(), "drone/comms/fc_state");

    ShmFCState sent{};
    sent.timestamp_ns = 200;
    sent.gps_lat = 37.7749f;
    sent.gps_lon = -122.4194f;
    sent.battery_voltage = 11.8f;
    sent.battery_remaining = 72.0f;
    sent.armed = true;
    sent.connected = true;
    sent.satellites_visible = 12;
    sent.flight_mode = 6;

    ShmFCState received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 200u);
    EXPECT_FLOAT_EQ(received.gps_lat, 37.7749f);
    EXPECT_FLOAT_EQ(received.gps_lon, -122.4194f);
    EXPECT_FLOAT_EQ(received.battery_voltage, 11.8f);
    EXPECT_FLOAT_EQ(received.battery_remaining, 72.0f);
    EXPECT_TRUE(received.armed);
    EXPECT_TRUE(received.connected);
    EXPECT_EQ(received.satellites_visible, 12);
    EXPECT_EQ(received.flight_mode, 6);
}

// --- Channel 3: ShmFCCommand (P4 → P5) ----------------------

TEST(ZenohMigration, FCCommand_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmFCCommand>(shm_names::FC_COMMANDS);
    auto sub = bus.subscribe<ShmFCCommand>(shm_names::FC_COMMANDS);
    EXPECT_EQ(pub->topic_name(), "drone/comms/fc_command");

    ShmFCCommand sent{};
    sent.timestamp_ns = 300;
    sent.command = FCCommandType::TAKEOFF;
    sent.param1 = 5.0f;
    sent.sequence_id = 42;
    sent.valid = true;

    ShmFCCommand received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 300u);
    EXPECT_EQ(received.command, FCCommandType::TAKEOFF);
    EXPECT_FLOAT_EQ(received.param1, 5.0f);
    EXPECT_EQ(received.sequence_id, 42u);
    EXPECT_TRUE(received.valid);
}

// --- Channel 4: ShmMissionStatus (P4 → P5, P7) --------------

TEST(ZenohMigration, MissionStatus_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmMissionStatus>(shm_names::MISSION_STATUS);
    auto sub = bus.subscribe<ShmMissionStatus>(shm_names::MISSION_STATUS);
    EXPECT_EQ(pub->topic_name(), "drone/mission/status");

    ShmMissionStatus sent{};
    sent.timestamp_ns = 400;
    sent.state = MissionState::NAVIGATE;
    sent.current_waypoint = 3;
    sent.total_waypoints = 10;
    sent.progress_percent = 30.0f;
    sent.target_x = 100.0f;
    sent.target_y = 200.0f;
    sent.target_z = 50.0f;
    sent.battery_percent = 85.0f;
    sent.mission_active = true;

    ShmMissionStatus received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 400u);
    EXPECT_EQ(received.state, MissionState::NAVIGATE);
    EXPECT_EQ(received.current_waypoint, 3u);
    EXPECT_EQ(received.total_waypoints, 10u);
    EXPECT_FLOAT_EQ(received.progress_percent, 30.0f);
    EXPECT_FLOAT_EQ(received.target_x, 100.0f);
    EXPECT_TRUE(received.mission_active);
}

// --- Channel 5: ShmTrajectoryCmd (P4 → P5) -------------------

TEST(ZenohMigration, TrajectoryCmd_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmTrajectoryCmd>(shm_names::TRAJECTORY_CMD);
    auto sub = bus.subscribe<ShmTrajectoryCmd>(shm_names::TRAJECTORY_CMD);
    EXPECT_EQ(pub->topic_name(), "drone/mission/trajectory");

    ShmTrajectoryCmd sent{};
    sent.timestamp_ns = 500;
    sent.target_x = 10.0f;
    sent.target_y = 20.0f;
    sent.target_z = 5.0f;
    sent.target_yaw = 1.57f;
    sent.velocity_x = 2.0f;
    sent.velocity_y = 0.0f;
    sent.velocity_z = -0.5f;
    sent.yaw_rate = 0.1f;
    sent.coordinate_frame = 8;
    sent.valid = true;

    ShmTrajectoryCmd received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 500u);
    EXPECT_FLOAT_EQ(received.target_x, 10.0f);
    EXPECT_FLOAT_EQ(received.target_yaw, 1.57f);
    EXPECT_FLOAT_EQ(received.velocity_x, 2.0f);
    EXPECT_EQ(received.coordinate_frame, 8);
    EXPECT_TRUE(received.valid);
}

// --- Channel 6: ShmGCSCommand (P5 → P4) ---------------------

TEST(ZenohMigration, GCSCommand_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmGCSCommand>(shm_names::GCS_COMMANDS);
    auto sub = bus.subscribe<ShmGCSCommand>(shm_names::GCS_COMMANDS);
    EXPECT_EQ(pub->topic_name(), "drone/comms/gcs_command");

    ShmGCSCommand sent{};
    sent.timestamp_ns = 600;
    sent.command = GCSCommandType::RTL;
    sent.param1 = 0.0f;
    sent.param2 = 0.0f;
    sent.param3 = 0.0f;
    sent.sequence_id = 7;
    sent.valid = true;

    ShmGCSCommand received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 600u);
    EXPECT_EQ(received.command, GCSCommandType::RTL);
    EXPECT_EQ(received.sequence_id, 7u);
    EXPECT_TRUE(received.valid);
}

// --- Channel 7: ShmDetectedObjectList (P2 → P4) -------------

TEST(ZenohMigration, DetectedObjects_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmDetectedObjectList>(shm_names::DETECTED_OBJECTS);
    auto sub = bus.subscribe<ShmDetectedObjectList>(shm_names::DETECTED_OBJECTS);
    EXPECT_EQ(pub->topic_name(), "drone/perception/detections");

    ShmDetectedObjectList sent{};
    sent.timestamp_ns = 700;
    sent.frame_sequence = 42;
    sent.num_objects = 2;
    sent.objects[0].track_id = 1;
    sent.objects[0].class_id = ObjectClass::PERSON;
    sent.objects[0].confidence = 0.95f;
    sent.objects[0].position_x = 5.0f;
    sent.objects[0].position_y = 3.0f;
    sent.objects[0].position_z = 0.0f;
    sent.objects[1].track_id = 2;
    sent.objects[1].class_id = ObjectClass::VEHICLE_CAR;
    sent.objects[1].confidence = 0.87f;

    ShmDetectedObjectList received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 700u);
    EXPECT_EQ(received.frame_sequence, 42u);
    EXPECT_EQ(received.num_objects, 2u);
    EXPECT_EQ(received.objects[0].track_id, 1u);
    EXPECT_EQ(received.objects[0].class_id, ObjectClass::PERSON);
    EXPECT_FLOAT_EQ(received.objects[0].confidence, 0.95f);
    EXPECT_FLOAT_EQ(received.objects[0].position_x, 5.0f);
    EXPECT_EQ(received.objects[1].track_id, 2u);
    EXPECT_EQ(received.objects[1].class_id, ObjectClass::VEHICLE_CAR);
}

// --- Channel 8: ShmPayloadCommand (P4 → P6) -----------------

TEST(ZenohMigration, PayloadCommand_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmPayloadCommand>(shm_names::PAYLOAD_COMMANDS);
    auto sub = bus.subscribe<ShmPayloadCommand>(shm_names::PAYLOAD_COMMANDS);
    EXPECT_EQ(pub->topic_name(), "drone/mission/payload_command");

    ShmPayloadCommand sent{};
    sent.timestamp_ns = 800;
    sent.action = PayloadAction::GIMBAL_POINT;
    sent.gimbal_pitch = -30.0f;
    sent.gimbal_yaw = 45.0f;
    sent.sequence_id = 100;
    sent.valid = true;

    ShmPayloadCommand received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 800u);
    EXPECT_EQ(received.action, PayloadAction::GIMBAL_POINT);
    EXPECT_FLOAT_EQ(received.gimbal_pitch, -30.0f);
    EXPECT_FLOAT_EQ(received.gimbal_yaw, 45.0f);
    EXPECT_EQ(received.sequence_id, 100u);
    EXPECT_TRUE(received.valid);
}

// --- Channel 9: ShmPayloadStatus (P6 → P4, P7) --------------

TEST(ZenohMigration, PayloadStatus_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmPayloadStatus>(shm_names::PAYLOAD_STATUS);
    auto sub = bus.subscribe<ShmPayloadStatus>(shm_names::PAYLOAD_STATUS);
    EXPECT_EQ(pub->topic_name(), "drone/payload/status");

    ShmPayloadStatus sent{};
    sent.timestamp_ns = 900;
    sent.gimbal_pitch = -15.0f;
    sent.gimbal_yaw = 90.0f;
    sent.images_captured = 42;
    sent.recording_video = true;
    sent.gimbal_stabilized = true;
    sent.num_plugins_active = 3;

    ShmPayloadStatus received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 900u);
    EXPECT_FLOAT_EQ(received.gimbal_pitch, -15.0f);
    EXPECT_FLOAT_EQ(received.gimbal_yaw, 90.0f);
    EXPECT_EQ(received.images_captured, 42u);
    EXPECT_TRUE(received.recording_video);
    EXPECT_TRUE(received.gimbal_stabilized);
    EXPECT_EQ(received.num_plugins_active, 3);
}

// --- Channel 10: ShmSystemHealth (P7 → P4) -------------------

TEST(ZenohMigration, SystemHealth_RoundTrip) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ShmSystemHealth>(shm_names::SYSTEM_HEALTH);
    auto sub = bus.subscribe<ShmSystemHealth>(shm_names::SYSTEM_HEALTH);
    EXPECT_EQ(pub->topic_name(), "drone/monitor/health");

    ShmSystemHealth sent{};
    sent.timestamp_ns = 1000;
    sent.cpu_usage_percent = 45.0f;
    sent.memory_usage_percent = 62.0f;
    sent.disk_usage_percent = 38.0f;
    sent.max_temp_c = 68.0f;
    sent.gpu_temp_c = 55.0f;
    sent.cpu_temp_c = 65.0f;
    sent.total_healthy = 7;
    sent.total_degraded = 0;
    sent.total_dead = 0;
    sent.power_watts = 12.5f;
    sent.thermal_zone = 1;

    ShmSystemHealth received{};
    ASSERT_TRUE(publish_until_received_iface(*pub, sent, *sub, received));
    EXPECT_EQ(received.timestamp_ns, 1000u);
    EXPECT_FLOAT_EQ(received.cpu_usage_percent, 45.0f);
    EXPECT_FLOAT_EQ(received.memory_usage_percent, 62.0f);
    EXPECT_FLOAT_EQ(received.disk_usage_percent, 38.0f);
    EXPECT_FLOAT_EQ(received.max_temp_c, 68.0f);
    EXPECT_EQ(received.total_healthy, 7u);
    EXPECT_EQ(received.total_dead, 0u);
    EXPECT_FLOAT_EQ(received.power_watts, 12.5f);
    EXPECT_EQ(received.thermal_zone, 1);
}

// --- Multi-channel: 5 channels active simultaneously ----------

TEST(ZenohMigration, MultiChannel_Simultaneous) {
    ZenohMessageBus bus;

    auto pose_pub   = bus.advertise<ShmPose>(shm_names::SLAM_POSE);
    auto fc_pub     = bus.advertise<ShmFCState>(shm_names::FC_STATE);
    auto traj_pub   = bus.advertise<ShmTrajectoryCmd>(shm_names::TRAJECTORY_CMD);
    auto health_pub = bus.advertise<ShmSystemHealth>(shm_names::SYSTEM_HEALTH);
    auto status_pub = bus.advertise<ShmMissionStatus>(shm_names::MISSION_STATUS);

    auto pose_sub   = bus.subscribe<ShmPose>(shm_names::SLAM_POSE);
    auto fc_sub     = bus.subscribe<ShmFCState>(shm_names::FC_STATE);
    auto traj_sub   = bus.subscribe<ShmTrajectoryCmd>(shm_names::TRAJECTORY_CMD);
    auto health_sub = bus.subscribe<ShmSystemHealth>(shm_names::SYSTEM_HEALTH);
    auto status_sub = bus.subscribe<ShmMissionStatus>(shm_names::MISSION_STATUS);

    ShmPose pose_s{};          pose_s.timestamp_ns = 1;  pose_s.quality = 2;
    ShmFCState fc_s{};         fc_s.timestamp_ns = 2;    fc_s.armed = true;
    ShmTrajectoryCmd traj_s{}; traj_s.timestamp_ns = 3;  traj_s.valid = true;
    ShmSystemHealth hlth_s{};  hlth_s.timestamp_ns = 4;  hlth_s.total_healthy = 7;
    ShmMissionStatus ms_s{};   ms_s.timestamp_ns = 5;    ms_s.mission_active = true;

    ShmPose pose_r{};
    ShmFCState fc_r{};
    ShmTrajectoryCmd traj_r{};
    ShmSystemHealth hlth_r{};
    ShmMissionStatus ms_r{};

    ASSERT_TRUE(publish_until_received_iface(*pose_pub, pose_s, *pose_sub, pose_r));
    ASSERT_TRUE(publish_until_received_iface(*fc_pub, fc_s, *fc_sub, fc_r));
    ASSERT_TRUE(publish_until_received_iface(*traj_pub, traj_s, *traj_sub, traj_r));
    ASSERT_TRUE(publish_until_received_iface(*health_pub, hlth_s, *health_sub, hlth_r));
    ASSERT_TRUE(publish_until_received_iface(*status_pub, ms_s, *status_sub, ms_r));

    EXPECT_EQ(pose_r.quality, 2u);
    EXPECT_TRUE(fc_r.armed);
    EXPECT_TRUE(traj_r.valid);
    EXPECT_EQ(hlth_r.total_healthy, 7u);
    EXPECT_TRUE(ms_r.mission_active);
}

// --- High-rate: Pose at 200 Hz for 2 seconds -----------------

TEST(ZenohMigration, HighRate_Pose) {
    ZenohPublisher<ShmPose> pub("drone/test/highrate_pose");
    ZenohSubscriber<ShmPose> sub("drone/test/highrate_pose");

    // Wait for discovery
    {
        ShmPose warm{};
        warm.timestamp_ns = 0;
        ShmPose warmr{};
        ASSERT_TRUE(publish_until_received(pub, warm, sub, warmr));
    }

    // Publish 400 messages at 200 Hz (5 ms interval) for 2 seconds
    constexpr int total_msgs = 400;
    for (int i = 1; i <= total_msgs; ++i) {
        ShmPose msg{};
        msg.timestamp_ns = static_cast<uint64_t>(i);
        msg.quality = 2;
        pub.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Poll until the subscriber holds the final published message.
    // ZenohSubscriber keeps the latest value (non-consuming receive),
    // so we poll repeatedly until the final message (ts == total_msgs)
    // appears or a generous timeout expires.
    ShmPose last{};
    bool got_final = false;
    auto poll_deadline = std::chrono::steady_clock::now()
                       + std::chrono::milliseconds(3000);
    while (std::chrono::steady_clock::now() < poll_deadline) {
        if (sub.receive(last) &&
            last.timestamp_ns == static_cast<uint64_t>(total_msgs)) {
            got_final = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(got_final)
        << "Timed out waiting for final message (ts=" << total_msgs
        << "); last seen ts=" << last.timestamp_ns;
    EXPECT_EQ(last.quality, 2u);
}

// --- Factory integration: bus_subscribe_optional ---------------

TEST(ZenohMigration, FactorySubscribeOptional) {
    // Use a unique key so this test is isolated from any other test that
    // may publish on GCS_COMMANDS.
    std::string unique_key = std::string("/factory_opt_test_") + std::to_string(::getpid());
    auto bus = create_message_bus("zenoh");
    auto sub = bus_subscribe_optional<ShmGCSCommand>(bus, unique_key);
    ASSERT_NE(sub, nullptr);
    // No publisher exists — receive should return false but not crash
    ShmGCSCommand cmd{};
    EXPECT_FALSE(sub->receive(cmd));
}


// ═══════════════════════════════════════════════════════════
// Category 4: Phase C — SHM provider zero-copy tests
// ═══════════════════════════════════════════════════════════

// --- SHM Provider tests -----------------------------------------------

TEST(ZenohShmProvider, ProviderCreatesSuccessfully) {
    auto* provider = ZenohSession::instance().shm_provider();
    ASSERT_NE(provider, nullptr);
}

TEST(ZenohShmProvider, AllocAndWriteBuffer) {
    auto* provider = ZenohSession::instance().shm_provider();
    ASSERT_NE(provider, nullptr);

    constexpr std::size_t buf_size = 4096;
    auto result = provider->alloc_gc_defrag_blocking(buf_size);
    auto* buf = std::get_if<zenoh::ZShmMut>(&result);
    ASSERT_NE(buf, nullptr);
    EXPECT_EQ(buf->len(), buf_size);

    // Write a pattern and verify
    std::memset(buf->data(), 0xCD, buf_size);
    EXPECT_EQ(buf->data()[0], 0xCD);
    EXPECT_EQ(buf->data()[buf_size - 1], 0xCD);
}

TEST(ZenohShmProvider, AllocLargeVideoFrameBuffer) {
    auto* provider = ZenohSession::instance().shm_provider();
    ASSERT_NE(provider, nullptr);

    // Allocate a buffer the size of ShmVideoFrame (~6.2 MB)
    auto result = provider->alloc_gc_defrag_blocking(sizeof(ShmVideoFrame));
    auto* buf = std::get_if<zenoh::ZShmMut>(&result);
    ASSERT_NE(buf, nullptr)
        << "Failed to allocate " << sizeof(ShmVideoFrame) << " bytes from SHM pool";
    EXPECT_EQ(buf->len(), sizeof(ShmVideoFrame));
}

// --- Size-aware publish path tests ------------------------------------

TEST(ZenohShmPublish, SmallMessageUsesBytes) {
    // ShmPose is small (~400 bytes) — should use bytes path, not SHM
    static_assert(sizeof(ShmPose) <= kShmPublishThreshold,
                  "ShmPose should be below SHM threshold");

    ZenohPublisher<ShmPose> pub("drone/test/shm_small_path");
    ZenohSubscriber<ShmPose> sub("drone/test/shm_small_path");

    ShmPose sent{};
    sent.timestamp_ns = 42;
    sent.quality = 2;

    ShmPose received{};
    ASSERT_TRUE(publish_until_received(pub, sent, sub, received));
    EXPECT_EQ(received.timestamp_ns, 42u);
    EXPECT_EQ(received.quality, 2u);
}

TEST(ZenohShmPublish, LargeVideoFrameUsesShmPath) {
    // ShmVideoFrame is ~6.2 MB — should use SHM provider path
    static_assert(sizeof(ShmVideoFrame) > kShmPublishThreshold,
                  "ShmVideoFrame should be above SHM threshold");

    ZenohPublisher<ShmVideoFrame> pub("drone/test/shm_video_path");
    ZenohSubscriber<ShmVideoFrame> sub("drone/test/shm_video_path");

    // Record baseline counters before publish
    const auto shm_before = pub.shm_publish_count();
    const auto bytes_before = pub.bytes_publish_count();

    // Heap-allocate — too large for the stack
    auto sent = std::make_unique<ShmVideoFrame>();
    sent->timestamp_ns = 12345;
    sent->width = 1920;
    sent->height = 1080;
    sent->channels = 3;
    sent->sequence_number = 99;
    // Write sentinel bytes at key positions
    sent->pixel_data[0] = 0xDE;
    sent->pixel_data[1] = 0xAD;
    sent->pixel_data[sizeof(sent->pixel_data) / 2] = 0xBE;
    sent->pixel_data[sizeof(sent->pixel_data) - 1] = 0xEF;

    auto received = std::make_unique<ShmVideoFrame>();
    ASSERT_TRUE(publish_until_received(pub, *sent, sub, *received));
    EXPECT_EQ(received->timestamp_ns, 12345u);
    EXPECT_EQ(received->width, 1920u);
    EXPECT_EQ(received->height, 1080u);
    EXPECT_EQ(received->channels, 3u);
    EXPECT_EQ(received->sequence_number, 99u);
    EXPECT_EQ(received->pixel_data[0], 0xDE);
    EXPECT_EQ(received->pixel_data[1], 0xAD);
    EXPECT_EQ(received->pixel_data[sizeof(received->pixel_data) / 2], 0xBE);
    EXPECT_EQ(received->pixel_data[sizeof(received->pixel_data) - 1], 0xEF);

    // Verify SHM path was actually used (not the bytes fallback)
    EXPECT_GT(pub.shm_publish_count(), shm_before)
        << "Expected SHM publish path for ShmVideoFrame";
    EXPECT_EQ(pub.bytes_publish_count(), bytes_before)
        << "Bytes path should not be used for ShmVideoFrame";
}

TEST(ZenohShmPublish, StereoFrameUsesShmPath) {
    // ShmStereoFrame is ~614 KB — should use SHM provider path
    static_assert(sizeof(ShmStereoFrame) > kShmPublishThreshold,
                  "ShmStereoFrame should be above SHM threshold");

    ZenohPublisher<ShmStereoFrame> pub("drone/test/shm_stereo_path");
    ZenohSubscriber<ShmStereoFrame> sub("drone/test/shm_stereo_path");

    const auto shm_before = pub.shm_publish_count();
    const auto bytes_before = pub.bytes_publish_count();

    auto sent = std::make_unique<ShmStereoFrame>();
    sent->timestamp_ns = 77777;
    sent->width = 640;
    sent->height = 480;
    sent->sequence_number = 42;
    sent->left_data[0] = 0xAA;
    sent->right_data[0] = 0xBB;
    sent->left_data[sizeof(sent->left_data) - 1] = 0xCC;
    sent->right_data[sizeof(sent->right_data) - 1] = 0xDD;

    auto received = std::make_unique<ShmStereoFrame>();
    ASSERT_TRUE(publish_until_received(pub, *sent, sub, *received));
    EXPECT_EQ(received->timestamp_ns, 77777u);
    EXPECT_EQ(received->width, 640u);
    EXPECT_EQ(received->height, 480u);
    EXPECT_EQ(received->left_data[0], 0xAA);
    EXPECT_EQ(received->right_data[0], 0xBB);
    EXPECT_EQ(received->left_data[sizeof(received->left_data) - 1], 0xCC);
    EXPECT_EQ(received->right_data[sizeof(received->right_data) - 1], 0xDD);

    // Verify SHM path was actually used
    EXPECT_GT(pub.shm_publish_count(), shm_before)
        << "Expected SHM publish path for ShmStereoFrame";
    EXPECT_EQ(pub.bytes_publish_count(), bytes_before)
        << "Bytes path should not be used for ShmStereoFrame";
}

// --- Factory integration with video channels --------------------------

TEST(ZenohShmPublish, FactoryVideoRoundTrip) {
    // End-to-end: factory → advertise ShmVideoFrame → subscribe → roundtrip
    auto bus = create_message_bus("zenoh");
    auto pub = bus_advertise<ShmVideoFrame>(bus, shm_names::VIDEO_MISSION_CAM);
    auto sub = bus_subscribe<ShmVideoFrame>(bus, shm_names::VIDEO_MISSION_CAM);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);

    auto sent = std::make_unique<ShmVideoFrame>();
    sent->timestamp_ns = 555;
    sent->width = 1920;
    sent->height = 1080;
    sent->channels = 3;
    sent->pixel_data[100] = 0x42;

    auto received = std::make_unique<ShmVideoFrame>();
    bool ok = false;
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(5000);
    while (std::chrono::steady_clock::now() < deadline) {
        pub->publish(*sent);
        auto batch = std::min(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(50),
            deadline);
        while (std::chrono::steady_clock::now() < batch) {
            if (sub->receive(*received)) { ok = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (ok) break;
    }
    ASSERT_TRUE(ok) << "Factory video round-trip timed out";
    EXPECT_EQ(received->timestamp_ns, 555u);
    EXPECT_EQ(received->width, 1920u);
    EXPECT_EQ(received->pixel_data[100], 0x42);
}

TEST(ZenohShmPublish, FactoryStereoRoundTrip) {
    auto bus = create_message_bus("zenoh");
    auto pub = bus_advertise<ShmStereoFrame>(bus, shm_names::VIDEO_STEREO_CAM);
    auto sub = bus_subscribe<ShmStereoFrame>(bus, shm_names::VIDEO_STEREO_CAM);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);

    auto sent = std::make_unique<ShmStereoFrame>();
    sent->timestamp_ns = 888;
    sent->width = 640;
    sent->height = 480;
    sent->left_data[42] = 0x99;

    auto received = std::make_unique<ShmStereoFrame>();
    bool ok = false;
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(5000);
    while (std::chrono::steady_clock::now() < deadline) {
        pub->publish(*sent);
        auto batch = std::min(
            std::chrono::steady_clock::now() + std::chrono::milliseconds(50),
            deadline);
        while (std::chrono::steady_clock::now() < batch) {
            if (sub->receive(*received)) { ok = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (ok) break;
    }
    ASSERT_TRUE(ok) << "Factory stereo round-trip timed out";
    EXPECT_EQ(received->timestamp_ns, 888u);
    EXPECT_EQ(received->left_data[42], 0x99);
}

// --- Sustained video publish test (simulates 30 Hz camera) -----------

TEST(ZenohShmPublish, SustainedVideoPublish) {
    ZenohPublisher<ShmVideoFrame> pub("drone/test/shm_sustained_video");
    ZenohSubscriber<ShmVideoFrame> sub("drone/test/shm_sustained_video");

    auto frame = std::make_unique<ShmVideoFrame>();
    frame->width = 1920;
    frame->height = 1080;
    frame->channels = 3;

    auto received = std::make_unique<ShmVideoFrame>();

    // Warm up discovery
    {
        frame->timestamp_ns = 0;
        ASSERT_TRUE(publish_until_received(pub, *frame, sub, *received));
    }

    // Track distinct frames received via polling during publish window
    std::atomic<int> frames_seen{0};
    uint64_t last_seen_ts = 0;

    // Publish 30 frames (simulating 1 second at 30 Hz)
    constexpr int total_frames = 30;
    for (int i = 1; i <= total_frames; ++i) {
        frame->timestamp_ns = static_cast<uint64_t>(i);
        frame->sequence_number = static_cast<uint64_t>(i);
        frame->pixel_data[0] = static_cast<uint8_t>(i & 0xFF);
        pub.publish(*frame);

        // Poll between publishes to count distinct frames
        if (sub.receive(*received) &&
            received->timestamp_ns != last_seen_ts) {
            last_seen_ts = received->timestamp_ns;
            frames_seen.fetch_add(1, std::memory_order_relaxed);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // Wait for final frame to arrive
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(3000);
    bool got_final = false;
    while (std::chrono::steady_clock::now() < deadline) {
        if (sub.receive(*received)) {
            if (received->timestamp_ns != last_seen_ts) {
                last_seen_ts = received->timestamp_ns;
                frames_seen.fetch_add(1, std::memory_order_relaxed);
            }
            if (received->timestamp_ns == static_cast<uint64_t>(total_frames)) {
                got_final = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(got_final)
        << "Did not receive final video frame (ts=" << total_frames
        << "); last seen ts=" << received->timestamp_ns;
    EXPECT_EQ(received->sequence_number,
              static_cast<uint64_t>(total_frames));

    // Verify we saw a meaningful number of distinct frames (not just the last)
    // With 33ms publish interval and 10ms poll interval, we should see
    // at least half the frames. This validates sustained delivery, not just
    // final-frame arrival.
    EXPECT_GE(frames_seen.load(), total_frames / 2)
        << "Expected to observe at least " << total_frames / 2
        << " distinct frames, but only saw " << frames_seen.load();

    // Verify SHM path was used for all publishes
    EXPECT_GE(pub.shm_publish_count(),
              static_cast<uint64_t>(total_frames))
        << "Expected at least " << total_frames << " SHM publishes";
}

// --- SHM pool configuration test --------------------------------------

TEST(ZenohShmProvider, PoolSizeConfiguration) {
    // Verify default pool size constant
    EXPECT_EQ(kDefaultShmPoolBytes, 32u * 1024 * 1024);
    EXPECT_EQ(kShmPublishThreshold, 64u * 1024);

    // Provider should exist with default pool
    EXPECT_NE(ZenohSession::instance().shm_provider(), nullptr);
    EXPECT_GT(ZenohSession::instance().shm_pool_bytes(), 0u);
}


#endif  // HAVE_ZENOH
