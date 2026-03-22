// tests/test_triple_buffer.cpp
// Unit tests for TripleBuffer lock-free single-producer single-consumer latest-value handoff.
#include "util/triple_buffer.h"

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using drone::TripleBuffer;

TEST(TripleBufferTest, WriteAndRead) {
    TripleBuffer<int> buf;
    buf.write(42);

    auto v = buf.read();
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 42);
}

TEST(TripleBufferTest, ReadReturnsNulloptWhenEmpty) {
    TripleBuffer<int> buf;
    auto              v = buf.read();
    EXPECT_FALSE(v.has_value());
}

TEST(TripleBufferTest, OverwriteKeepsLatest) {
    TripleBuffer<int> buf;
    buf.write(10);
    buf.write(20);
    buf.write(30);

    auto v = buf.read();
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 30);
}

TEST(TripleBufferTest, ReadClearsNewFlag) {
    TripleBuffer<int> buf;
    buf.write(1);

    auto v1 = buf.read();
    ASSERT_TRUE(v1.has_value());
    EXPECT_EQ(*v1, 1);

    // Second read without intervening write returns nullopt
    auto v2 = buf.read();
    EXPECT_FALSE(v2.has_value());
}

TEST(TripleBufferTest, WriteCountTracking) {
    TripleBuffer<int> buf;
    EXPECT_EQ(buf.write_count(), 0u);

    buf.write(1);
    EXPECT_EQ(buf.write_count(), 1u);

    buf.write(2);
    buf.write(3);
    EXPECT_EQ(buf.write_count(), 3u);
}

TEST(TripleBufferTest, ReadCountTracking) {
    TripleBuffer<int> buf;
    EXPECT_EQ(buf.read_count(), 0u);

    buf.write(1);
    (void)buf.read();
    EXPECT_EQ(buf.read_count(), 1u);

    // Failed read (no new data) does not increment
    (void)buf.read();
    EXPECT_EQ(buf.read_count(), 1u);

    buf.write(2);
    (void)buf.read();
    EXPECT_EQ(buf.read_count(), 2u);
}

TEST(TripleBufferTest, MoveSemantics) {
    TripleBuffer<std::vector<std::string>> buf;

    std::vector<std::string> payload = {"hello", "world", "test"};
    buf.write(std::move(payload));

    auto v = buf.read();
    ASSERT_TRUE(v.has_value());
    ASSERT_EQ(v->size(), 3u);
    EXPECT_EQ((*v)[0], "hello");
    EXPECT_EQ((*v)[1], "world");
    EXPECT_EQ((*v)[2], "test");
}

TEST(TripleBufferTest, NonTriviallyCopyable) {
    // std::vector<std::string> is non-trivially copyable — TripleBuffer
    // must handle it correctly (unlike Zenoh SHM zero-copy).
    TripleBuffer<std::vector<std::string>> buf;

    for (int i = 0; i < 100; ++i) {
        std::vector<std::string> v;
        v.push_back("item_" + std::to_string(i));
        buf.write(std::move(v));
    }

    auto v = buf.read();
    ASSERT_TRUE(v.has_value());
    ASSERT_EQ(v->size(), 1u);
    EXPECT_EQ((*v)[0], "item_99");
}

TEST(TripleBufferTest, ConcurrentProducerConsumer) {
    TripleBuffer<uint64_t> buf;
    constexpr uint64_t     COUNT = 100000;
    std::atomic<bool>      done{false};
    std::atomic<uint64_t>  last_read{0};

    std::thread producer([&]() {
        for (uint64_t i = 1; i <= COUNT; ++i) {
            buf.write(i);
        }
        done.store(true, std::memory_order_release);
    });

    std::thread consumer([&]() {
        uint64_t prev = 0;
        while (true) {
            auto v = buf.read();
            if (v.has_value()) {
                // Values must be monotonically increasing (latest-value semantics
                // means we may skip values, but never go backwards)
                EXPECT_GT(*v, prev);
                prev = *v;
            } else if (done.load(std::memory_order_acquire)) {
                // Producer finished — try one final read to drain latest
                auto final_v = buf.read();
                if (final_v.has_value()) {
                    EXPECT_GT(*final_v, prev);
                    prev = *final_v;
                }
                break;
            } else {
                std::this_thread::yield();
            }
        }
        last_read.store(prev, std::memory_order_relaxed);
    });

    producer.join();
    consumer.join();

    // Last value read should be the final written value
    EXPECT_EQ(last_read.load(), COUNT);
    EXPECT_EQ(buf.write_count(), COUNT);
    EXPECT_GT(buf.read_count(), 0u);
}

TEST(TripleBufferTest, HighContentionStress) {
    // Producer at max rate, consumer at ~30Hz cadence
    TripleBuffer<uint64_t> buf;
    std::atomic<bool>      done{false};

    std::thread producer([&]() {
        for (uint64_t i = 1; i <= 50000; ++i) {
            buf.write(i);
        }
        done.store(true, std::memory_order_release);
    });

    std::thread consumer([&]() {
        uint64_t prev     = 0;
        uint64_t read_cnt = 0;
        while (!done.load(std::memory_order_acquire) || true) {
            auto v = buf.read();
            if (v.has_value()) {
                // Monotonically increasing — no corruption
                EXPECT_GT(*v, prev);
                prev = *v;
                ++read_cnt;
            }
            // Simulate ~30Hz consumer (33ms period)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (done.load(std::memory_order_acquire)) {
                // Drain final value
                auto final_v = buf.read();
                if (final_v.has_value()) {
                    EXPECT_GT(*final_v, prev);
                }
                break;
            }
        }
        // Consumer should have read far fewer values than producer wrote
        // (latest-value semantics — skipping is expected)
        EXPECT_GT(read_cnt, 0u);
    });

    producer.join();
    consumer.join();

    EXPECT_EQ(buf.write_count(), 50000u);
}
