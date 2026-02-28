// tests/test_zenoh_ipc.cpp
// Unit tests for the Zenoh IPC backend and message bus factory.
//
// Two categories:
//   1. Topic mapping + factory (always compiled — no Zenoh dependency)
//   2. Zenoh pub/sub round-trip (compiled only with HAVE_ZENOH)
//
// Build:
//   cmake -B build                          → runs category 1 only
//   cmake -B build -DENABLE_ZENOH=ON        → runs categories 1 + 2
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

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

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

TEST(ZenohPubSub, SmallMessageRoundTrip) {
    ZenohPublisher<ZenohTestPayload> pub("drone/test/small_rt");
    ZenohSubscriber<ZenohTestPayload> sub("drone/test/small_rt");

    // Give Zenoh time to match publisher ↔ subscriber
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohTestPayload sent{42, 3.14f, {}};
    std::strncpy(sent.tag, "hello", sizeof(sent.tag));
    pub.publish(sent);

    // Wait for delivery
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohTestPayload received;
    uint64_t ts = 0;
    ASSERT_TRUE(sub.receive(received, &ts));
    EXPECT_EQ(received.id, 42u);
    EXPECT_FLOAT_EQ(received.value, 3.14f);
    EXPECT_STREQ(received.tag, "hello");
    EXPECT_GT(ts, 0u);
    EXPECT_TRUE(sub.is_connected());
}

TEST(ZenohPubSub, ShmPoseRoundTrip) {
    ZenohPublisher<ShmPose> pub("drone/test/pose_rt");
    ZenohSubscriber<ShmPose> sub("drone/test/pose_rt");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ShmPose sent{};
    sent.timestamp_ns = 123456789;
    sent.translation[0] = 1.0;
    sent.translation[1] = 2.0;
    sent.translation[2] = 3.0;
    sent.quaternion[0] = 1.0;  // w
    sent.quality = 2;
    pub.publish(sent);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ShmPose received{};
    ASSERT_TRUE(sub.receive(received));
    EXPECT_EQ(received.timestamp_ns, 123456789u);
    EXPECT_DOUBLE_EQ(received.translation[0], 1.0);
    EXPECT_DOUBLE_EQ(received.translation[1], 2.0);
    EXPECT_DOUBLE_EQ(received.translation[2], 3.0);
    EXPECT_EQ(received.quality, 2u);
}

TEST(ZenohPubSub, LargeVideoFrameRoundTrip) {
    ZenohPublisher<ShmVideoFrame> pub("drone/test/video_rt");
    ZenohSubscriber<ShmVideoFrame> sub("drone/test/video_rt");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ShmVideoFrame sent{};
    sent.timestamp_ns = 999;
    sent.width = 1920;
    sent.height = 1080;
    sent.channels = 3;
    sent.sequence_number = 7;
    // Fill first few bytes of pixel data
    sent.pixel_data[0] = 0xAA;
    sent.pixel_data[1] = 0xBB;
    sent.pixel_data[sizeof(sent.pixel_data) - 1] = 0xFF;
    pub.publish(sent);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    ShmVideoFrame received{};
    ASSERT_TRUE(sub.receive(received));
    EXPECT_EQ(received.width, 1920u);
    EXPECT_EQ(received.height, 1080u);
    EXPECT_EQ(received.channels, 3u);
    EXPECT_EQ(received.sequence_number, 7u);
    EXPECT_EQ(received.pixel_data[0], 0xAA);
    EXPECT_EQ(received.pixel_data[1], 0xBB);
    EXPECT_EQ(received.pixel_data[sizeof(received.pixel_data) - 1], 0xFF);
}

TEST(ZenohPubSub, MultipleTopics) {
    ZenohPublisher<ZenohTestPayload> pub1("drone/test/multi/a");
    ZenohPublisher<ZenohTestPayload> pub2("drone/test/multi/b");
    ZenohPublisher<ZenohTestPayload> pub3("drone/test/multi/c");
    ZenohSubscriber<ZenohTestPayload> sub1("drone/test/multi/a");
    ZenohSubscriber<ZenohTestPayload> sub2("drone/test/multi/b");
    ZenohSubscriber<ZenohTestPayload> sub3("drone/test/multi/c");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohTestPayload m1{1, 1.0f, {}};
    ZenohTestPayload m2{2, 2.0f, {}};
    ZenohTestPayload m3{3, 3.0f, {}};
    pub1.publish(m1);
    pub2.publish(m2);
    pub3.publish(m3);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohTestPayload r1, r2, r3;
    ASSERT_TRUE(sub1.receive(r1));
    ASSERT_TRUE(sub2.receive(r2));
    ASSERT_TRUE(sub3.receive(r3));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohTestPayload m1{10, 0.0f, {}};
    pub.publish(m1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ZenohTestPayload r1;
    uint64_t ts1 = 0;
    ASSERT_TRUE(sub.receive(r1, &ts1));
    EXPECT_EQ(r1.id, 10u);

    ZenohTestPayload m2{20, 0.0f, {}};
    pub.publish(m2);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ZenohTestPayload r2;
    uint64_t ts2 = 0;
    ASSERT_TRUE(sub.receive(r2, &ts2));
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

TEST(ZenohMessageBus, RoundTripViaFactory) {
    ZenohMessageBus bus;
    auto pub = bus.advertise<ZenohTestPayload>("/detected_objects");
    auto sub = bus.subscribe<ZenohTestPayload>("/detected_objects");
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohTestPayload sent{99, 2.71f, {}};
    pub->publish(sent);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ZenohTestPayload received;
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.id, 99u);
    EXPECT_FLOAT_EQ(received.value, 2.71f);
}

#endif  // HAVE_ZENOH
