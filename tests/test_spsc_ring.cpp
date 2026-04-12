// tests/test_spsc_ring.cpp
// Unit tests for SPSCRing lock-free single-producer single-consumer ring buffer.
#include "util/spsc_ring.h"

#include <atomic>
#include <thread>

#include <gtest/gtest.h>

using drone::SPSCRing;

TEST(SPSCRingTest, PushAndPop) {
    SPSCRing<int, 4> ring;

    EXPECT_TRUE(ring.try_push(10));
    EXPECT_TRUE(ring.try_push(20));
    EXPECT_TRUE(ring.try_push(30));

    auto v1 = ring.try_pop();
    ASSERT_TRUE(v1.has_value());
    EXPECT_EQ(*v1, 10);

    auto v2 = ring.try_pop();
    ASSERT_TRUE(v2.has_value());
    EXPECT_EQ(*v2, 20);

    auto v3 = ring.try_pop();
    ASSERT_TRUE(v3.has_value());
    EXPECT_EQ(*v3, 30);
}

TEST(SPSCRingTest, PopEmptyReturnsNullopt) {
    SPSCRing<int, 4> ring;
    auto             v = ring.try_pop();
    EXPECT_FALSE(v.has_value());
}

TEST(SPSCRingTest, FullRingRejectsPush) {
    SPSCRing<int, 4> ring;  // capacity = 4
    EXPECT_TRUE(ring.try_push(1));
    EXPECT_TRUE(ring.try_push(2));
    EXPECT_TRUE(ring.try_push(3));
    EXPECT_TRUE(ring.try_push(4));
    EXPECT_FALSE(ring.try_push(5));  // full
}

TEST(SPSCRingTest, WrapAround) {
    SPSCRing<int, 4> ring;

    // Fill and drain multiple times to test wrap-around
    for (int round = 0; round < 10; ++round) {
        for (int i = 0; i < 4; ++i) {
            EXPECT_TRUE(ring.try_push(round * 100 + i));
        }
        for (int i = 0; i < 4; ++i) {
            auto v = ring.try_pop();
            ASSERT_TRUE(v.has_value());
            EXPECT_EQ(*v, round * 100 + i);
        }
    }
}

TEST(SPSCRingTest, Available) {
    SPSCRing<int, 8> ring;
    EXPECT_EQ(ring.available(), 0u);

    ring.try_push(1);
    ring.try_push(2);
    EXPECT_EQ(ring.available(), 2u);

    ring.try_pop();
    EXPECT_EQ(ring.available(), 1u);
}

TEST(SPSCRingTest, ConcurrentProducerConsumer) {
    SPSCRing<uint64_t, 1024> ring;
    constexpr uint64_t       COUNT = 100000;
    std::atomic<bool>        done{false};

    std::thread producer([&]() {
        for (uint64_t i = 0; i < COUNT; ++i) {
            while (!ring.try_push(i)) {
                std::this_thread::yield();
            }
        }
        done.store(true, std::memory_order_release);
    });

    std::thread consumer([&]() {
        uint64_t expected = 0;
        while (expected < COUNT) {
            auto v = ring.try_pop();
            if (v.has_value()) {
                EXPECT_EQ(*v, expected);
                ++expected;
            } else if (done.load(std::memory_order_acquire)) {
                // Drain remaining
                auto remaining = ring.try_pop();
                if (!remaining.has_value()) break;
                EXPECT_EQ(*remaining, expected);
                ++expected;
            }
        }
        EXPECT_EQ(expected, COUNT);
    });

    producer.join();
    consumer.join();
}

// Test with struct type
struct TestStruct {
    int    id;
    double value;
    char   tag[8];
};

TEST(SPSCRingTest, StructPayload) {
    SPSCRing<TestStruct, 4> ring;

    TestStruct s1{1, 3.14, "hello"};
    EXPECT_TRUE(ring.try_push(s1));

    auto v = ring.try_pop();
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(v->id, 1);
    EXPECT_DOUBLE_EQ(v->value, 3.14);
    EXPECT_STREQ(v->tag, "hello");
}
