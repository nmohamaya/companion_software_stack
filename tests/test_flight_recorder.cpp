// tests/test_flight_recorder.cpp
// Unit tests for the FlightRecorder, RecordRingBuffer, and log file I/O.

#include "ipc/ipc_types.h"
#include "ipc/wire_format.h"
#include "recorder/flight_recorder.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace drone::recorder;
using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Helper: create a temporary directory for test log files
// ═══════════════════════════════════════════════════════════
class FlightRecorderTest : public ::testing::Test {
protected:
    void SetUp() override {
        tmp_dir_ =
            std::filesystem::temp_directory_path() /
            ("flog_test_" + std::to_string(::getpid()) + "_" + std::to_string(test_counter_++));
        std::filesystem::create_directories(tmp_dir_);
    }

    void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

    std::string tmp_path() const { return tmp_dir_.string(); }
    std::string tmp_file(const std::string& name) const { return (tmp_dir_ / name).string(); }

private:
    std::filesystem::path  tmp_dir_;
    static inline uint32_t test_counter_ = 0;
};

// ═══════════════════════════════════════════════════════════
// RecordRingBuffer tests
// ═══════════════════════════════════════════════════════════

TEST(RecordRingBufferTest, PushAndRetrieve) {
    RecordRingBuffer buf(4096);
    EXPECT_EQ(buf.size(), 0u);
    EXPECT_EQ(buf.current_bytes(), 0u);

    RecordEntry entry;
    entry.header.wire_header.magic        = kWireMagic;
    entry.header.wire_header.payload_size = 8;
    entry.header.topic_name_len           = 5;
    entry.topic_name                      = "/test";
    entry.payload.resize(8, 0x42);

    buf.push(entry);
    EXPECT_EQ(buf.size(), 1u);
    EXPECT_EQ(buf.current_bytes(), entry.serialized_size());

    const auto& stored = buf.entries().front();
    EXPECT_EQ(stored.topic_name, "/test");
    EXPECT_EQ(stored.payload.size(), 8u);
    EXPECT_EQ(stored.payload[0], 0x42);
}

TEST(RecordRingBufferTest, WrapDiscards) {
    // Buffer that can hold ~2 entries of 50 bytes each
    const std::size_t entry_size = sizeof(RecordHeader) + 5 + 8;  // ~47 bytes
    RecordRingBuffer  buf(entry_size * 2 + 10);                   // just over 2 entries

    auto make_entry = [](uint32_t seq) {
        RecordEntry e;
        e.header.wire_header.magic        = kWireMagic;
        e.header.wire_header.payload_size = 8;
        e.header.wire_header.sequence     = seq;
        e.header.topic_name_len           = 5;
        e.topic_name                      = "/test";
        e.payload.resize(8, static_cast<uint8_t>(seq));
        return e;
    };

    buf.push(make_entry(1));
    buf.push(make_entry(2));
    EXPECT_EQ(buf.size(), 2u);

    // Third entry should evict the first
    buf.push(make_entry(3));
    EXPECT_EQ(buf.size(), 2u);
    EXPECT_EQ(buf.entries().front().header.wire_header.sequence, 2u);
    EXPECT_EQ(buf.entries().back().header.wire_header.sequence, 3u);
}

TEST(RecordRingBufferTest, OversizedEntryIgnored) {
    RecordRingBuffer buf(10);  // very small buffer

    RecordEntry entry;
    entry.header.wire_header.magic        = kWireMagic;
    entry.header.wire_header.payload_size = 100;
    entry.header.topic_name_len           = 5;
    entry.topic_name                      = "/test";
    entry.payload.resize(100, 0xFF);

    buf.push(entry);
    EXPECT_EQ(buf.size(), 0u);  // should be rejected
}

TEST(RecordRingBufferTest, ClearEmptiesBuffer) {
    RecordRingBuffer buf(4096);

    RecordEntry entry;
    entry.header.wire_header.magic        = kWireMagic;
    entry.header.wire_header.payload_size = 4;
    entry.header.topic_name_len           = 2;
    entry.topic_name                      = "/t";
    entry.payload.resize(4, 0);

    buf.push(entry);
    buf.push(entry);
    EXPECT_EQ(buf.size(), 2u);

    buf.clear();
    EXPECT_EQ(buf.size(), 0u);
    EXPECT_EQ(buf.current_bytes(), 0u);
}

// ═══════════════════════════════════════════════════════════
// Log file I/O tests
// ═══════════════════════════════════════════════════════════

TEST_F(FlightRecorderTest, WriteAndReadLogFile) {
    // Create some entries
    std::vector<RecordEntry> entries;
    for (uint32_t i = 0; i < 5; ++i) {
        RecordEntry e;
        e.header.wire_header.magic        = kWireMagic;
        e.header.wire_header.version      = kWireVersion;
        e.header.wire_header.msg_type     = WireMessageType::FC_STATE;
        e.header.wire_header.payload_size = sizeof(FCState);
        e.header.wire_header.sequence     = i;
        e.header.wire_header.timestamp_ns = 1000000 * (i + 1);
        e.header.topic_name_len           = static_cast<uint16_t>(std::strlen("/fc_state"));
        e.topic_name                      = "/fc_state";

        FCState fc{};
        fc.timestamp_ns      = 1000000 * (i + 1);
        fc.battery_voltage   = 22.0f + static_cast<float>(i) * 0.1f;
        fc.battery_remaining = 90.0f - static_cast<float>(i);
        fc.connected         = true;
        e.payload.resize(sizeof(FCState));
        std::copy(reinterpret_cast<const uint8_t*>(&fc),
                  reinterpret_cast<const uint8_t*>(&fc) + sizeof(FCState), e.payload.data());

        entries.push_back(std::move(e));
    }

    const auto path = tmp_file("test_write_read.flog");
    ASSERT_TRUE(write_log_file(path, entries, 1000000));

    auto result = read_log_file(path);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.file_header.magic, kFlightLogMagic);
    EXPECT_EQ(result.file_header.start_time_ns, 1000000u);
    ASSERT_EQ(result.entries.size(), 5u);

    for (uint32_t i = 0; i < 5; ++i) {
        const auto& e = result.entries[i];
        EXPECT_EQ(e.header.wire_header.sequence, i);
        EXPECT_EQ(e.topic_name, "/fc_state");
        EXPECT_EQ(e.payload.size(), sizeof(FCState));

        FCState fc{};
        std::copy(e.payload.begin(), e.payload.end(), reinterpret_cast<uint8_t*>(&fc));
        EXPECT_TRUE(fc.connected);
        EXPECT_NEAR(fc.battery_voltage, 22.0f + static_cast<float>(i) * 0.1f, 0.001f);
    }
}

TEST_F(FlightRecorderTest, ReadInvalidFileReturnsFailure) {
    auto result = read_log_file(tmp_file("nonexistent.flog"));
    EXPECT_FALSE(result.success);
}

TEST_F(FlightRecorderTest, ReadCorruptedMagicReturnsEmpty) {
    const auto path = tmp_file("corrupt.flog");
    {
        std::ofstream out(path, std::ios::binary);
        uint32_t      bad_magic = 0xDEADBEEF;
        out.write(reinterpret_cast<const char*>(&bad_magic), sizeof(bad_magic));
    }
    auto result = read_log_file(path);
    EXPECT_FALSE(result.success);
}

// ═══════════════════════════════════════════════════════════
// WireHeader serialization roundtrip
// ═══════════════════════════════════════════════════════════

TEST(WireHeaderRoundtripTest, SerializeDeserializePose) {
    Pose pose{};
    pose.timestamp_ns   = 42000000;
    pose.translation[0] = 1.0;
    pose.translation[1] = 2.0;
    pose.translation[2] = 3.0;
    pose.quaternion[0]  = 1.0;
    pose.quality        = 2;

    auto bytes = wire_serialize(pose, WireMessageType::SLAM_POSE, 7, 42000000);
    ASSERT_GE(bytes.size(), sizeof(WireHeader) + sizeof(Pose));

    ASSERT_TRUE(wire_validate(bytes.data(), bytes.size()));

    WireHeader hdr = wire_read_header(bytes.data());
    EXPECT_EQ(hdr.magic, kWireMagic);
    EXPECT_EQ(hdr.msg_type, WireMessageType::SLAM_POSE);
    EXPECT_EQ(hdr.payload_size, sizeof(Pose));
    EXPECT_EQ(hdr.sequence, 7u);
    EXPECT_EQ(hdr.timestamp_ns, 42000000u);

    Pose out{};
    ASSERT_TRUE(wire_deserialize(bytes.data(), bytes.size(), out));
    EXPECT_EQ(out.timestamp_ns, 42000000u);
    EXPECT_DOUBLE_EQ(out.translation[0], 1.0);
    EXPECT_DOUBLE_EQ(out.translation[1], 2.0);
    EXPECT_DOUBLE_EQ(out.translation[2], 3.0);
    EXPECT_EQ(out.quality, 2u);
}

TEST(WireHeaderRoundtripTest, SerializeDeserializeFCState) {
    FCState fc{};
    fc.timestamp_ns      = 99000;
    fc.battery_voltage   = 22.2f;
    fc.battery_remaining = 85.0f;
    fc.armed             = true;
    fc.connected         = true;
    fc.roll              = 0.1f;
    fc.pitch             = 0.2f;
    fc.yaw               = 0.3f;
    fc.vx                = 1.0f;
    fc.vy                = 2.0f;
    fc.vz                = 0.0f;

    auto bytes = wire_serialize(fc, WireMessageType::FC_STATE, 3, 99000);
    ASSERT_TRUE(wire_validate(bytes.data(), bytes.size()));

    FCState out{};
    ASSERT_TRUE(wire_deserialize(bytes.data(), bytes.size(), out));
    EXPECT_EQ(out.timestamp_ns, 99000u);
    EXPECT_FLOAT_EQ(out.battery_voltage, 22.2f);
    EXPECT_TRUE(out.armed);
}

// ═══════════════════════════════════════════════════════════
// FlightRecorder integration tests
// ═══════════════════════════════════════════════════════════

TEST_F(FlightRecorderTest, RecordAndFlush) {
    FlightRecorder recorder(1024 * 1024, tmp_path());

    EXPECT_FALSE(recorder.is_recording());
    recorder.start();
    EXPECT_TRUE(recorder.is_recording());

    // Record some FC states
    for (uint32_t i = 0; i < 10; ++i) {
        FCState fc{};
        fc.timestamp_ns      = 1000 * (i + 1);
        fc.battery_voltage   = 22.0f;
        fc.battery_remaining = 90.0f;
        fc.connected         = true;
        fc.roll              = 0.0f;
        fc.pitch             = 0.0f;
        fc.yaw               = 0.0f;
        fc.vx                = 0.0f;
        fc.vy                = 0.0f;
        fc.vz                = 0.0f;
        recorder.record(topics::FC_STATE, WireMessageType::FC_STATE, fc, i);
    }

    EXPECT_EQ(recorder.entry_count(), 10u);
    EXPECT_GT(recorder.buffered_bytes(), 0u);

    auto path = recorder.flush();
    EXPECT_FALSE(path.empty());
    EXPECT_EQ(recorder.entry_count(), 0u);

    // Verify the flushed file
    auto result = read_log_file(path);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.entries.size(), 10u);

    recorder.stop();
    EXPECT_FALSE(recorder.is_recording());
}

TEST_F(FlightRecorderTest, RecordIgnoredWhenStopped) {
    FlightRecorder recorder(1024 * 1024, tmp_path());
    // Don't call start()

    FCState fc{};
    fc.timestamp_ns      = 1000;
    fc.battery_voltage   = 22.0f;
    fc.battery_remaining = 90.0f;
    fc.connected         = true;
    fc.roll              = 0.0f;
    fc.pitch             = 0.0f;
    fc.yaw               = 0.0f;
    fc.vx                = 0.0f;
    fc.vy                = 0.0f;
    fc.vz                = 0.0f;
    recorder.record(topics::FC_STATE, WireMessageType::FC_STATE, fc, 0);

    EXPECT_EQ(recorder.entry_count(), 0u);
}

TEST_F(FlightRecorderTest, FlushEmptyBufferReturnsEmpty) {
    FlightRecorder recorder(1024 * 1024, tmp_path());
    recorder.start();
    auto path = recorder.flush();
    EXPECT_TRUE(path.empty());
}

TEST_F(FlightRecorderTest, RecordMultipleTypes) {
    FlightRecorder recorder(4 * 1024 * 1024, tmp_path());
    recorder.start();

    // Record a Pose
    Pose pose{};
    pose.timestamp_ns   = 1000;
    pose.translation[0] = 1.0;
    pose.translation[1] = 2.0;
    pose.translation[2] = 3.0;
    pose.quaternion[0]  = 1.0;
    pose.quality        = 2;
    recorder.record(topics::SLAM_POSE, WireMessageType::SLAM_POSE, pose, 0);

    // Record an FCState
    FCState fc{};
    fc.timestamp_ns      = 2000;
    fc.battery_voltage   = 22.0f;
    fc.battery_remaining = 90.0f;
    fc.connected         = true;
    fc.roll              = 0.0f;
    fc.pitch             = 0.0f;
    fc.yaw               = 0.0f;
    fc.vx                = 0.0f;
    fc.vy                = 0.0f;
    fc.vz                = 0.0f;
    recorder.record(topics::FC_STATE, WireMessageType::FC_STATE, fc, 1);

    // Record a SystemHealth
    SystemHealth health{};
    health.timestamp_ns         = 3000;
    health.cpu_usage_percent    = 55.0f;
    health.memory_usage_percent = 40.0f;
    health.max_temp_c           = 65.0f;
    health.power_watts          = 15.0f;
    health.thermal_zone         = 0;
    health.disk_usage_percent   = 30.0f;
    recorder.record(topics::SYSTEM_HEALTH, WireMessageType::SYSTEM_HEALTH, health, 2);

    EXPECT_EQ(recorder.entry_count(), 3u);

    auto path = recorder.flush();
    ASSERT_FALSE(path.empty());

    auto result = read_log_file(path);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.entries.size(), 3u);

    // Verify types
    EXPECT_EQ(result.entries[0].header.wire_header.msg_type, WireMessageType::SLAM_POSE);
    EXPECT_EQ(result.entries[0].topic_name, topics::SLAM_POSE);
    EXPECT_EQ(result.entries[1].header.wire_header.msg_type, WireMessageType::FC_STATE);
    EXPECT_EQ(result.entries[2].header.wire_header.msg_type, WireMessageType::SYSTEM_HEALTH);

    // Verify Pose payload roundtrip
    Pose pose_out{};
    std::copy(result.entries[0].payload.begin(), result.entries[0].payload.end(),
              reinterpret_cast<uint8_t*>(&pose_out));
    EXPECT_DOUBLE_EQ(pose_out.translation[0], 1.0);
    EXPECT_EQ(pose_out.quality, 2u);

    recorder.stop();
}

// ═══════════════════════════════════════════════════════════
// Ring buffer wrapping correctness with real IPC types
// ═══════════════════════════════════════════════════════════

TEST_F(FlightRecorderTest, RingBufferWrapsWithRealTypes) {
    // Calculate the size of one FCState record
    RecordEntry sample;
    sample.header.wire_header.payload_size = sizeof(FCState);
    sample.header.topic_name_len           = static_cast<uint16_t>(std::strlen("/fc_state"));
    sample.topic_name                      = "/fc_state";
    sample.payload.resize(sizeof(FCState));
    const auto one_entry_size = sample.serialized_size();

    // Buffer that can hold exactly 3 entries
    FlightRecorder recorder(one_entry_size * 3, tmp_path());
    recorder.start();

    // Record 5 entries — first 2 should be evicted
    for (uint32_t i = 0; i < 5; ++i) {
        FCState fc{};
        fc.timestamp_ns      = 1000 * (i + 1);
        fc.battery_voltage   = static_cast<float>(i);
        fc.battery_remaining = 50.0f;
        fc.connected         = true;
        fc.roll              = 0.0f;
        fc.pitch             = 0.0f;
        fc.yaw               = 0.0f;
        fc.vx                = 0.0f;
        fc.vy                = 0.0f;
        fc.vz                = 0.0f;
        recorder.record(topics::FC_STATE, WireMessageType::FC_STATE, fc, i);
    }

    // Should have 3 entries (the last 3)
    EXPECT_EQ(recorder.entry_count(), 3u);

    auto path = recorder.flush();
    ASSERT_FALSE(path.empty());

    auto result = read_log_file(path);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.entries.size(), 3u);

    // Verify we kept entries 2, 3, 4 (sequences)
    EXPECT_EQ(result.entries[0].header.wire_header.sequence, 2u);
    EXPECT_EQ(result.entries[1].header.wire_header.sequence, 3u);
    EXPECT_EQ(result.entries[2].header.wire_header.sequence, 4u);

    recorder.stop();
}

// ═══════════════════════════════════════════════════════════
// File header validation
// ═══════════════════════════════════════════════════════════

TEST_F(FlightRecorderTest, FileHeaderFields) {
    FlightRecorder recorder(1024 * 1024, tmp_path());
    recorder.start();

    FCState fc{};
    fc.timestamp_ns      = 42000;
    fc.battery_voltage   = 22.0f;
    fc.battery_remaining = 90.0f;
    fc.connected         = true;
    fc.roll              = 0.0f;
    fc.pitch             = 0.0f;
    fc.yaw               = 0.0f;
    fc.vx                = 0.0f;
    fc.vy                = 0.0f;
    fc.vz                = 0.0f;
    recorder.record(topics::FC_STATE, WireMessageType::FC_STATE, fc, 0);

    auto path = recorder.flush();
    ASSERT_FALSE(path.empty());

    auto result = read_log_file(path);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.file_header.magic, kFlightLogMagic);
    EXPECT_EQ(result.file_header.version, 1u);
    // start_time_ns should be the timestamp of the first record
    EXPECT_GT(result.file_header.start_time_ns, 0u);

    recorder.stop();
}

// ═══════════════════════════════════════════════════════════
// Thread safety: concurrent record calls
// ═══════════════════════════════════════════════════════════

TEST_F(FlightRecorderTest, ConcurrentRecordIsSafe) {
    FlightRecorder recorder(8 * 1024 * 1024, tmp_path());
    recorder.start();

    constexpr int            kThreads   = 4;
    constexpr int            kPerThread = 50;
    std::vector<std::thread> threads;

    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&recorder, t]() {
            for (int i = 0; i < kPerThread; ++i) {
                FCState fc{};
                fc.timestamp_ns      = static_cast<uint64_t>(t * 1000 + i);
                fc.battery_voltage   = 22.0f;
                fc.battery_remaining = 90.0f;
                fc.connected         = true;
                fc.roll              = 0.0f;
                fc.pitch             = 0.0f;
                fc.yaw               = 0.0f;
                fc.vx                = 0.0f;
                fc.vy                = 0.0f;
                fc.vz                = 0.0f;
                recorder.record(topics::FC_STATE, WireMessageType::FC_STATE, fc,
                                static_cast<uint32_t>(t * kPerThread + i));
            }
        });
    }

    for (auto& th : threads) {
        th.join();
    }

    EXPECT_EQ(recorder.entry_count(), static_cast<std::size_t>(kThreads * kPerThread));

    auto path = recorder.flush();
    ASSERT_FALSE(path.empty());

    auto result = read_log_file(path);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.entries.size(), static_cast<std::size_t>(kThreads * kPerThread));

    recorder.stop();
}

// ═══════════════════════════════════════════════════════════
// Struct layout assertions
// ═══════════════════════════════════════════════════════════

TEST(FlightRecorderStructTest, FlightLogFileHeaderSize) {
    EXPECT_EQ(sizeof(FlightLogFileHeader), 24u);
}

TEST(FlightRecorderStructTest, RecordHeaderSize) {
    EXPECT_EQ(sizeof(RecordHeader), 34u);
}

TEST(FlightRecorderStructTest, RecordEntrySerializedSize) {
    RecordEntry entry;
    entry.header.wire_header.payload_size = 100;
    entry.header.topic_name_len           = 10;
    entry.topic_name                      = "0123456789";
    entry.payload.resize(100);

    EXPECT_EQ(entry.serialized_size(), sizeof(RecordHeader) + 10u + 100u);
}
