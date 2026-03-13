// tests/test_shm_ipc.cpp
// Unit tests for ShmWriter / ShmReader SeqLock-based IPC.
#include "ipc/ipc_types.h"
#include "ipc/shm_reader.h"
#include "ipc/shm_writer.h"

#include <chrono>
#include <cstring>
#include <thread>

#include <gtest/gtest.h>

// ── Simple trivially-copyable test payload ──────────────────
struct TestPayload {
    uint64_t sequence;
    double   values[4];
    uint32_t checksum;
};

static constexpr const char* TEST_SHM_NAME = "/drone_test_shm";

class ShmIPCTest : public ::testing::Test {
protected:
    void TearDown() override {
        // Clean up any leftover SHM segment
        shm_unlink(TEST_SHM_NAME);
    }
};

// ── Basic write + read ──────────────────────────────────────
TEST_F(ShmIPCTest, WriteAndRead) {
    ShmWriter<TestPayload> writer;
    ASSERT_TRUE(writer.create(TEST_SHM_NAME));

    TestPayload out;
    out.sequence  = 42;
    out.values[0] = 1.0;
    out.values[1] = 2.0;
    out.values[2] = 3.0;
    out.values[3] = 4.0;
    out.checksum  = 12345;
    writer.write(out);

    ShmReader<TestPayload> reader;
    ASSERT_TRUE(reader.open(TEST_SHM_NAME));
    ASSERT_TRUE(reader.is_open());

    TestPayload in{};
    ASSERT_TRUE(reader.read(in));
    EXPECT_EQ(in.sequence, 42u);
    EXPECT_DOUBLE_EQ(in.values[0], 1.0);
    EXPECT_DOUBLE_EQ(in.values[1], 2.0);
    EXPECT_DOUBLE_EQ(in.values[2], 3.0);
    EXPECT_DOUBLE_EQ(in.values[3], 4.0);
    EXPECT_EQ(in.checksum, 12345u);
}

// ── Multiple sequential writes ──────────────────────────────
TEST_F(ShmIPCTest, MultipleWrites) {
    ShmWriter<TestPayload> writer;
    ASSERT_TRUE(writer.create(TEST_SHM_NAME));
    ShmReader<TestPayload> reader;
    ASSERT_TRUE(reader.open(TEST_SHM_NAME));

    for (uint64_t i = 0; i < 100; ++i) {
        TestPayload out{};
        out.sequence = i;
        out.checksum = static_cast<uint32_t>(i * 7);
        writer.write(out);

        TestPayload in{};
        ASSERT_TRUE(reader.read(in));
        EXPECT_EQ(in.sequence, i);
        EXPECT_EQ(in.checksum, static_cast<uint32_t>(i * 7));
    }
}

// ── Reader fails on non-existent SHM ────────────────────────
TEST_F(ShmIPCTest, ReaderFailsNonExistent) {
    ShmReader<TestPayload> reader;
    EXPECT_FALSE(reader.open("/drone_test_nonexistent_shm"));
    EXPECT_FALSE(reader.is_open());
}

// ── Reader returns false when no data written ───────────────
TEST_F(ShmIPCTest, ReadBeforeWrite) {
    ShmWriter<TestPayload> writer;
    ASSERT_TRUE(writer.create(TEST_SHM_NAME));
    ShmReader<TestPayload> reader;
    ASSERT_TRUE(reader.open(TEST_SHM_NAME));

    // seq is 0 (even), so read should succeed but data is zero-initialized
    TestPayload in{};
    EXPECT_TRUE(reader.read(in));
    EXPECT_EQ(in.sequence, 0u);
}

// ── Move constructor transfers ownership correctly ──────────
TEST_F(ShmIPCTest, WriterMoveConstruct) {
    ShmWriter<TestPayload> writer1;
    ASSERT_TRUE(writer1.create(TEST_SHM_NAME));

    TestPayload out{};
    out.sequence = 99;
    writer1.write(out);

    // Move writer1 into writer2
    ShmWriter<TestPayload> writer2(std::move(writer1));

    // writer2 should still work
    out.sequence = 100;
    writer2.write(out);

    ShmReader<TestPayload> reader;
    ASSERT_TRUE(reader.open(TEST_SHM_NAME));
    TestPayload in{};
    ASSERT_TRUE(reader.read(in));
    EXPECT_EQ(in.sequence, 100u);
}

TEST_F(ShmIPCTest, ReaderMoveConstruct) {
    ShmWriter<TestPayload> writer;
    ASSERT_TRUE(writer.create(TEST_SHM_NAME));
    TestPayload out{};
    out.sequence = 77;
    writer.write(out);

    ShmReader<TestPayload> reader1;
    ASSERT_TRUE(reader1.open(TEST_SHM_NAME));

    // Move reader1 into reader2
    ShmReader<TestPayload> reader2(std::move(reader1));
    EXPECT_TRUE(reader2.is_open());

    TestPayload in{};
    ASSERT_TRUE(reader2.read(in));
    EXPECT_EQ(in.sequence, 77u);
}

// ── Concurrent writer + reader stress test ──────────────────
TEST_F(ShmIPCTest, ConcurrentWriteRead) {
    ShmWriter<TestPayload> writer;
    ASSERT_TRUE(writer.create(TEST_SHM_NAME));
    ShmReader<TestPayload> reader;
    ASSERT_TRUE(reader.open(TEST_SHM_NAME));

    constexpr int    NUM_WRITES = 10000;
    std::atomic<int> writes_done{0};
    std::atomic<int> successful_reads{0};
    std::atomic<int> torn_reads{0};

    // Writer thread — write sequentially, let reader keep up
    std::thread writer_thread([&]() {
        for (int i = 0; i < NUM_WRITES; ++i) {
            TestPayload out{};
            out.sequence = static_cast<uint64_t>(i);
            out.checksum = static_cast<uint32_t>(i * 3);
            writer.write(out);
            writes_done.store(i + 1, std::memory_order_release);
        }
    });

    // Reader thread — spin-read until all writes are done AND drained
    std::thread reader_thread([&]() {
        while (writes_done.load(std::memory_order_acquire) < NUM_WRITES ||
               successful_reads.load() == 0) {
            TestPayload in{};
            if (reader.read(in)) {
                // Verify consistency: checksum should match sequence
                EXPECT_EQ(in.checksum, static_cast<uint32_t>(in.sequence * 3));
                successful_reads.fetch_add(1);
            } else {
                torn_reads.fetch_add(1);
            }
            // Stop once we've read enough after writer is done
            if (writes_done.load(std::memory_order_acquire) >= NUM_WRITES &&
                successful_reads.load() > 0) {
                break;
            }
        }
    });

    writer_thread.join();
    reader_thread.join();

    EXPECT_GT(successful_reads.load(), 0);
}

// ── create_non_owning: segment survives writer destruction ──
TEST_F(ShmIPCTest, ShmWriterNonOwning_SegmentSurvivesDestruction) {
    constexpr const char* NON_OWNING_SHM = "/drone_test_non_owning";
    // Ensure clean state
    shm_unlink(NON_OWNING_SHM);

    {
        ShmWriter<TestPayload> writer;
        ASSERT_TRUE(writer.create_non_owning(NON_OWNING_SHM));

        TestPayload out{};
        out.sequence = 42;
        writer.write(out);
    }
    // Writer destroyed — segment must still exist

    int fd = shm_open(NON_OWNING_SHM, O_RDONLY, 0);
    ASSERT_GE(fd, 0) << "SHM segment should survive non-owning writer destruction";
    close(fd);

    // Verify data is still readable
    ShmReader<TestPayload> reader;
    ASSERT_TRUE(reader.open(NON_OWNING_SHM));
    TestPayload in{};
    ASSERT_TRUE(reader.read(in));
    EXPECT_EQ(in.sequence, 42u);

    shm_unlink(NON_OWNING_SHM);
}

// ── Test with actual SHM types (Pose) ────────────────────
TEST_F(ShmIPCTest, ShmPoseRoundTrip) {
    constexpr const char* POSE_SHM = "/drone_test_pose";

    ShmWriter<drone::ipc::Pose> writer;
    ASSERT_TRUE(writer.create(POSE_SHM));

    drone::ipc::Pose pose_out{};
    pose_out.timestamp_ns   = 123456789ULL;
    pose_out.translation[0] = 1.0;
    pose_out.translation[1] = 2.0;
    pose_out.translation[2] = 3.0;
    pose_out.quaternion[0]  = 1.0;  // w
    pose_out.quality        = 2;
    writer.write(pose_out);

    ShmReader<drone::ipc::Pose> reader;
    ASSERT_TRUE(reader.open(POSE_SHM));

    drone::ipc::Pose pose_in{};
    ASSERT_TRUE(reader.read(pose_in));
    EXPECT_EQ(pose_in.timestamp_ns, 123456789ULL);
    EXPECT_DOUBLE_EQ(pose_in.translation[0], 1.0);
    EXPECT_DOUBLE_EQ(pose_in.translation[1], 2.0);
    EXPECT_DOUBLE_EQ(pose_in.translation[2], 3.0);
    EXPECT_EQ(pose_in.quality, 2u);

    shm_unlink(POSE_SHM);
}
