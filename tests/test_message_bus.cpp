// tests/test_message_bus.cpp
// Tests for IPublisher/ISubscriber/ShmMessageBus/ShmServiceChannel interfaces.
#include <gtest/gtest.h>

#include "ipc/ipublisher.h"
#include "ipc/isubscriber.h"
#include "ipc/shm_publisher.h"
#include "ipc/shm_subscriber.h"
#include "ipc/shm_message_bus.h"
#include "ipc/iservice_channel.h"
#include "ipc/shm_service_channel.h"
#include "ipc/shm_types.h"

#include <cstring>
#include <memory>
#include <thread>
#include <chrono>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Helper: unique SHM names to avoid test collisions
// ═══════════════════════════════════════════════════════════
static std::string unique_name(const std::string& base) {
    static int counter = 0;
    return base + "_mbus_" + std::to_string(getpid()) + "_" +
           std::to_string(counter++);
}

// ═══════════════════════════════════════════════════════════
// Test data struct (trivially copyable)
// ═══════════════════════════════════════════════════════════
struct TestPayload {
    uint64_t id{0};
    float value{0.0f};
    char tag[16]{};
};

// ═══════════════════════════════════════════════════════════
// ShmPublisher tests
// ═══════════════════════════════════════════════════════════

TEST(ShmPublisherTest, CreateAndPublish) {
    auto name = unique_name("/test_pub");
    ShmPublisher<TestPayload> pub(name);
    ASSERT_TRUE(pub.is_ready());
    EXPECT_EQ(pub.topic_name(), name);

    TestPayload msg{42, 3.14f, "hello"};
    pub.publish(msg);  // should not crash
}

TEST(ShmPublisherTest, TopicNameStored) {
    auto name = unique_name("/test_pub_name");
    ShmPublisher<TestPayload> pub(name);
    EXPECT_EQ(pub.topic_name(), name);
}

TEST(ShmPublisherTest, ImplementsIPublisher) {
    auto name = unique_name("/test_pub_iface");
    auto pub = std::make_unique<ShmPublisher<TestPayload>>(name);
    IPublisher<TestPayload>* iface = pub.get();
    EXPECT_TRUE(iface->is_ready());
    EXPECT_EQ(iface->topic_name(), name);
}

// ═══════════════════════════════════════════════════════════
// ShmSubscriber tests
// ═══════════════════════════════════════════════════════════

TEST(ShmSubscriberTest, LazyConstructAndConnect) {
    auto name = unique_name("/test_sub_lazy");
    // Create a publisher first so the SHM topic exists
    ShmPublisher<TestPayload> pub(name);
    ASSERT_TRUE(pub.is_ready());

    ShmSubscriber<TestPayload> sub;
    EXPECT_FALSE(sub.is_connected());
    EXPECT_TRUE(sub.connect(name));
    EXPECT_TRUE(sub.is_connected());
}

TEST(ShmSubscriberTest, ReceivePublishedData) {
    auto name = unique_name("/test_sub_recv");
    ShmPublisher<TestPayload> pub(name);

    ShmSubscriber<TestPayload> sub;
    sub.connect(name);
    ASSERT_TRUE(sub.is_connected());

    TestPayload sent{99, 2.718f, {}};
    std::strncpy(sent.tag, "test", sizeof(sent.tag));
    pub.publish(sent);

    TestPayload received{};
    ASSERT_TRUE(sub.receive(received));
    EXPECT_EQ(received.id, 99u);
    EXPECT_FLOAT_EQ(received.value, 2.718f);
    EXPECT_STREQ(received.tag, "test");
}

TEST(ShmSubscriberTest, ImplementsISubscriber) {
    auto name = unique_name("/test_sub_iface");
    ShmPublisher<TestPayload> pub(name);

    auto sub = std::make_unique<ShmSubscriber<TestPayload>>(name, 1, 10);
    ISubscriber<TestPayload>* iface = sub.get();
    EXPECT_TRUE(iface->is_connected());
    EXPECT_EQ(iface->topic_name(), name);
}

TEST(ShmSubscriberTest, LazyConnectFails) {
    ShmSubscriber<TestPayload> sub;
    EXPECT_FALSE(sub.connect("/nonexistent_topic_12345"));
    EXPECT_FALSE(sub.is_connected());
}

// ═══════════════════════════════════════════════════════════
// ShmMessageBus tests
// ═══════════════════════════════════════════════════════════

TEST(ShmMessageBusTest, AdvertiseAndSubscribe) {
    auto name = unique_name("/test_bus");
    ShmMessageBus bus;

    auto pub = bus.advertise<TestPayload>(name);
    ASSERT_TRUE(pub->is_ready());

    auto sub = bus.subscribe<TestPayload>(name, 5, 10);
    ASSERT_TRUE(sub->is_connected());

    TestPayload sent{123, 1.0f, {}};
    pub->publish(sent);

    TestPayload received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.id, 123u);
}

TEST(ShmMessageBusTest, AdvReturnsIPublisher) {
    auto name = unique_name("/test_bus_type");
    ShmMessageBus bus;
    std::unique_ptr<IPublisher<TestPayload>> pub = bus.advertise<TestPayload>(name);
    EXPECT_NE(pub, nullptr);
    EXPECT_TRUE(pub->is_ready());
}

TEST(ShmMessageBusTest, SubReturnsISubscriber) {
    auto name = unique_name("/test_bus_subtype");
    ShmMessageBus bus;
    auto pub = bus.advertise<TestPayload>(name);
    std::unique_ptr<ISubscriber<TestPayload>> sub = bus.subscribe<TestPayload>(name, 1, 10);
    EXPECT_NE(sub, nullptr);
}

TEST(ShmMessageBusTest, LazySubscriber) {
    auto name = unique_name("/test_bus_lazy");
    ShmMessageBus bus;
    auto pub = bus.advertise<TestPayload>(name);

    auto sub = bus.subscribe_lazy<TestPayload>();
    EXPECT_FALSE(sub->is_connected());
    EXPECT_TRUE(sub->connect(name));
    EXPECT_TRUE(sub->is_connected());
}

TEST(ShmMessageBusTest, MultiplePublishOverwrites) {
    auto name = unique_name("/test_bus_overwrite");
    ShmMessageBus bus;
    auto pub = bus.advertise<TestPayload>(name);
    auto sub = bus.subscribe<TestPayload>(name, 1, 10);

    for (int i = 0; i < 10; ++i) {
        TestPayload msg{static_cast<uint64_t>(i), static_cast<float>(i), {}};
        pub->publish(msg);
    }

    TestPayload latest{};
    ASSERT_TRUE(sub->receive(latest));
    EXPECT_EQ(latest.id, 9u);  // latest-value semantics
}

TEST(ShmMessageBusTest, WithShmTypes) {
    auto name = unique_name("/test_bus_pose");
    ShmMessageBus bus;
    auto pub = bus.advertise<ShmPose>(name);
    ASSERT_TRUE(pub->is_ready());

    auto sub = bus.subscribe<ShmPose>(name, 1, 10);
    ASSERT_TRUE(sub->is_connected());

    ShmPose pose{};
    pose.timestamp_ns = 12345678;
    pose.translation[0] = 1.0;
    pose.translation[1] = 2.0;
    pose.translation[2] = 3.0;
    pose.quality = 2;
    pub->publish(pose);

    ShmPose received{};
    ASSERT_TRUE(sub->receive(received));
    EXPECT_EQ(received.timestamp_ns, 12345678u);
    EXPECT_DOUBLE_EQ(received.translation[0], 1.0);
    EXPECT_EQ(received.quality, 2u);
}

// ═══════════════════════════════════════════════════════════
// IServiceChannel tests
// ═══════════════════════════════════════════════════════════

struct TestRequest {
    uint32_t command{0};
    float param{0.0f};
};

struct TestResponse {
    uint32_t result{0};
    bool success{false};
};

TEST(ShmServiceChannelTest, ClientSendServerReceive) {
    auto req_name = unique_name("/test_svc_req");
    auto resp_name = unique_name("/test_svc_resp");

    // Create server first (creates resp SHM)
    ShmServiceServer<TestRequest, TestResponse> server(req_name, resp_name);

    // Then create client (creates req SHM, connects to resp SHM)
    ShmServiceClient<TestRequest, TestResponse> client(req_name, resp_name);

    // Send a request
    TestRequest req{42, 3.14f};
    auto correlation_id = client.send_request(req);
    EXPECT_GE(correlation_id, 1u);

    // Server should receive it
    auto received = server.poll_request();
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->correlation_id, correlation_id);
    EXPECT_EQ(received->payload.command, 42u);
    EXPECT_FLOAT_EQ(received->payload.param, 3.14f);
}

TEST(ShmServiceChannelTest, ServerRespondClientReceive) {
    auto req_name = unique_name("/test_svc_rr_req");
    auto resp_name = unique_name("/test_svc_rr_resp");

    ShmServiceServer<TestRequest, TestResponse> server(req_name, resp_name);
    ShmServiceClient<TestRequest, TestResponse> client(req_name, resp_name);

    TestRequest req{1, 0.0f};
    auto id = client.send_request(req);

    auto srv_req = server.poll_request();
    ASSERT_TRUE(srv_req.has_value());

    // Server sends response
    TestResponse resp{100, true};
    server.send_response(id, ServiceStatus::OK, resp);

    // Client should receive it
    auto cli_resp = client.poll_response(id);
    ASSERT_TRUE(cli_resp.has_value());
    EXPECT_EQ(cli_resp->correlation_id, id);
    EXPECT_EQ(cli_resp->status, ServiceStatus::OK);
    EXPECT_EQ(cli_resp->payload.result, 100u);
    EXPECT_TRUE(cli_resp->payload.success);
}

TEST(ShmServiceChannelTest, WrongCorrelationIdReturnsNull) {
    auto req_name = unique_name("/test_svc_wid_req");
    auto resp_name = unique_name("/test_svc_wid_resp");

    ShmServiceServer<TestRequest, TestResponse> server(req_name, resp_name);
    ShmServiceClient<TestRequest, TestResponse> client(req_name, resp_name);

    TestRequest req{1, 0.0f};
    auto id = client.send_request(req);

    server.poll_request();
    TestResponse resp{1, true};
    server.send_response(id, ServiceStatus::OK, resp);

    // Poll with wrong ID
    auto wrong = client.poll_response(id + 999);
    EXPECT_FALSE(wrong.has_value());
}

TEST(ShmServiceChannelTest, AwaitResponseTimeout) {
    auto req_name = unique_name("/test_svc_to_req");
    auto resp_name = unique_name("/test_svc_to_resp");

    ShmServiceServer<TestRequest, TestResponse> server(req_name, resp_name);
    ShmServiceClient<TestRequest, TestResponse> client(req_name, resp_name);

    TestRequest req{1, 0.0f};
    auto id = client.send_request(req);
    // Don't send any response — should timeout
    auto result = client.await_response(id, std::chrono::milliseconds(50));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->status, ServiceStatus::TIMEOUT);
}

TEST(ShmServiceChannelTest, ServiceStatusEnum) {
    EXPECT_EQ(static_cast<uint8_t>(ServiceStatus::OK), 0);
    EXPECT_EQ(static_cast<uint8_t>(ServiceStatus::REJECTED), 1);
    EXPECT_EQ(static_cast<uint8_t>(ServiceStatus::TIMEOUT), 2);
    EXPECT_EQ(static_cast<uint8_t>(ServiceStatus::ERROR), 3);
}

TEST(ShmServiceChannelTest, ImplementsIServiceClient) {
    auto req_name = unique_name("/test_svc_cli_iface_req");
    auto resp_name = unique_name("/test_svc_cli_iface_resp");

    ShmServiceServer<TestRequest, TestResponse> server(req_name, resp_name);
    auto client = std::make_unique<ShmServiceClient<TestRequest, TestResponse>>(
        req_name, resp_name);
    IServiceClient<TestRequest, TestResponse>* iface = client.get();
    auto id = iface->send_request(TestRequest{1, 0.0f});
    EXPECT_GE(id, 1u);
}

TEST(ShmServiceChannelTest, ImplementsIServiceServer) {
    auto req_name = unique_name("/test_svc_srv_iface_req");
    auto resp_name = unique_name("/test_svc_srv_iface_resp");

    // Client creates req SHM; we need it to exist for server
    ShmServiceClient<TestRequest, TestResponse> client(req_name, resp_name);
    auto server = std::make_unique<ShmServiceServer<TestRequest, TestResponse>>(
        req_name, resp_name);
    IServiceServer<TestRequest, TestResponse>* iface = server.get();
    auto req = iface->poll_request();
    // No request sent yet, so should be empty
    EXPECT_FALSE(req.has_value());
}

TEST(ShmServiceChannelTest, MultipleRequestsSequential) {
    auto req_name = unique_name("/test_svc_multi_req");
    auto resp_name = unique_name("/test_svc_multi_resp");

    ShmServiceServer<TestRequest, TestResponse> server(req_name, resp_name);
    ShmServiceClient<TestRequest, TestResponse> client(req_name, resp_name);

    for (int i = 0; i < 5; ++i) {
        TestRequest req{static_cast<uint32_t>(i), 0.0f};
        auto id = client.send_request(req);

        auto srv_req = server.poll_request();
        ASSERT_TRUE(srv_req.has_value());
        EXPECT_EQ(srv_req->payload.command, static_cast<uint32_t>(i));

        TestResponse resp{static_cast<uint32_t>(i * 10), true};
        server.send_response(id, ServiceStatus::OK, resp);

        auto cli_resp = client.poll_response(id);
        ASSERT_TRUE(cli_resp.has_value());
        EXPECT_EQ(cli_resp->payload.result, static_cast<uint32_t>(i * 10));
    }
}
