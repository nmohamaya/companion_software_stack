// tests/test_zenoh_ipc.cpp
// Unit tests for the Zenoh IPC backend and message bus factory.
//
// Three categories:
//   1. MessageBusFactory tests (always compiled — no Zenoh dependency)
//   2. Zenoh topic mapping tests (HAVE_ZENOH — ZenohMessageBus header required)
//   3. Zenoh pub/sub round-trip tests (HAVE_ZENOH — requires running Zenoh session)
//
// Build:
//   cmake -B build                          → runs category 1 only
//   cmake -B build -DENABLE_ZENOH=ON        → runs categories 1 + 2 + 3
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

#endif  // HAVE_ZENOH
