// tests/test_serializer.cpp
// Tests for ISerializer<T> interface and RawSerializer<T> implementation.
//
// Verifies:
//   - Round-trip fidelity for multiple IPC struct types
//   - Size mismatch rejection
//   - Buffer overrun protection
//   - Serialized size correctness
//   - Wire-format backward compatibility (byte-identical to raw reinterpret_cast)
//
// Part of Epic #284 (Platform Modularity), Issue #294.
#include "ipc/ipc_types.h"
#include "ipc/iserializer.h"
#include "ipc/raw_serializer.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string_view>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Test fixture with factory helper
// ═══════════════════════════════════════════════════════════

/// Simple trivially-copyable test struct (no validate method).
struct SimpleMsg {
    uint64_t id{0};
    float    value{0.0f};
    char     tag[16]{};
};
static_assert(std::is_trivially_copyable_v<SimpleMsg>);

// ═══════════════════════════════════════════════════════════
// RawSerializer — name and basics
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, NameReturnsRaw) {
    RawSerializer<SimpleMsg> ser;
    EXPECT_EQ(ser.name(), "raw");
}

TEST(RawSerializerTest, SerializedSizeMatchesSizeof) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                msg{};
    EXPECT_EQ(ser.serialized_size(msg), sizeof(SimpleMsg));
}

// ═══════════════════════════════════════════════════════════
// Round-trip: SimpleMsg
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, RoundTripSimpleMsg) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                original{};
    original.id    = 42;
    original.value = 3.14f;
    std::strncpy(original.tag, "hello", sizeof(original.tag) - 1);

    auto vec = ser.serialize(original);
    ASSERT_EQ(vec.size(), sizeof(SimpleMsg));

    SimpleMsg decoded{};
    ASSERT_TRUE(ser.deserialize(vec.data(), vec.size(), decoded));
    EXPECT_EQ(decoded.id, 42u);
    EXPECT_FLOAT_EQ(decoded.value, 3.14f);
    EXPECT_STREQ(decoded.tag, "hello");
}

TEST(RawSerializerTest, RoundTripBufferSimpleMsg) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                original{};
    original.id    = 99;
    original.value = -1.5f;

    std::vector<uint8_t> buf(sizeof(SimpleMsg));
    size_t               written = ser.serialize(original, buf.data(), buf.size());
    ASSERT_EQ(written, sizeof(SimpleMsg));

    SimpleMsg decoded{};
    ASSERT_TRUE(ser.deserialize(buf.data(), written, decoded));
    EXPECT_EQ(decoded.id, 99u);
    EXPECT_FLOAT_EQ(decoded.value, -1.5f);
}

// ═══════════════════════════════════════════════════════════
// Round-trip: SlamPose (Pose)
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, RoundTripPose) {
    RawSerializer<Pose> ser;
    Pose                original{};
    original.timestamp_ns   = 1234567890;
    original.translation[0] = 1.0;
    original.translation[1] = 2.0;
    original.translation[2] = 3.0;
    original.quaternion[0]  = 1.0;
    original.quality        = 2;

    auto vec = ser.serialize(original);
    ASSERT_EQ(vec.size(), sizeof(Pose));

    Pose decoded{};
    ASSERT_TRUE(ser.deserialize(vec.data(), vec.size(), decoded));
    EXPECT_EQ(decoded.timestamp_ns, 1234567890u);
    EXPECT_DOUBLE_EQ(decoded.translation[0], 1.0);
    EXPECT_DOUBLE_EQ(decoded.translation[1], 2.0);
    EXPECT_DOUBLE_EQ(decoded.translation[2], 3.0);
    EXPECT_DOUBLE_EQ(decoded.quaternion[0], 1.0);
    EXPECT_EQ(decoded.quality, 2u);
}

// ═══════════════════════════════════════════════════════════
// Round-trip: FCState
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, RoundTripFCState) {
    RawSerializer<FCState> ser;
    FCState                original{};
    original.timestamp_ns       = 9999;
    original.battery_voltage    = 12.6f;
    original.battery_remaining  = 85.0f;
    original.armed              = true;
    original.connected          = true;
    original.gps_fix_type       = 3;
    original.satellites_visible = 12;

    auto vec = ser.serialize(original);
    ASSERT_EQ(vec.size(), sizeof(FCState));

    FCState decoded{};
    ASSERT_TRUE(ser.deserialize(vec.data(), vec.size(), decoded));
    EXPECT_EQ(decoded.timestamp_ns, 9999u);
    EXPECT_FLOAT_EQ(decoded.battery_voltage, 12.6f);
    EXPECT_FLOAT_EQ(decoded.battery_remaining, 85.0f);
    EXPECT_TRUE(decoded.armed);
    EXPECT_TRUE(decoded.connected);
    EXPECT_EQ(decoded.gps_fix_type, 3);
    EXPECT_EQ(decoded.satellites_visible, 12);
}

// ═══════════════════════════════════════════════════════════
// Round-trip: SystemHealth
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, RoundTripSystemHealth) {
    RawSerializer<SystemHealth> ser;
    SystemHealth                original{};
    original.timestamp_ns         = 42000;
    original.cpu_usage_percent    = 55.5f;
    original.memory_usage_percent = 70.0f;
    original.thermal_zone         = 1;

    auto vec = ser.serialize(original);
    ASSERT_EQ(vec.size(), sizeof(SystemHealth));

    SystemHealth decoded{};
    ASSERT_TRUE(ser.deserialize(vec.data(), vec.size(), decoded));
    EXPECT_EQ(decoded.timestamp_ns, 42000u);
    EXPECT_FLOAT_EQ(decoded.cpu_usage_percent, 55.5f);
    EXPECT_FLOAT_EQ(decoded.memory_usage_percent, 70.0f);
    EXPECT_EQ(decoded.thermal_zone, 1);
}

// ═══════════════════════════════════════════════════════════
// Round-trip: DetectedObjectList
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, RoundTripDetectedObjectList) {
    RawSerializer<DetectedObjectList> ser;
    DetectedObjectList                original{};
    original.timestamp_ns          = 77777;
    original.frame_sequence        = 10;
    original.num_objects           = 1;
    original.objects[0].track_id   = 5;
    original.objects[0].confidence = 0.95f;
    original.objects[0].position_x = 10.0f;

    auto vec = ser.serialize(original);
    ASSERT_EQ(vec.size(), sizeof(DetectedObjectList));

    DetectedObjectList decoded{};
    ASSERT_TRUE(ser.deserialize(vec.data(), vec.size(), decoded));
    EXPECT_EQ(decoded.timestamp_ns, 77777u);
    EXPECT_EQ(decoded.frame_sequence, 10u);
    EXPECT_EQ(decoded.num_objects, 1u);
    EXPECT_EQ(decoded.objects[0].track_id, 5u);
    EXPECT_FLOAT_EQ(decoded.objects[0].confidence, 0.95f);
    EXPECT_FLOAT_EQ(decoded.objects[0].position_x, 10.0f);
}

// ═══════════════════════════════════════════════════════════
// Deserialization failure: size mismatch
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, DeserializeSizeMismatchTooSmall) {
    RawSerializer<SimpleMsg> ser;
    std::vector<uint8_t>     bad_data(sizeof(SimpleMsg) - 1, 0);
    SimpleMsg                out{};
    EXPECT_FALSE(ser.deserialize(bad_data.data(), bad_data.size(), out));
}

TEST(RawSerializerTest, DeserializeSizeMismatchTooLarge) {
    RawSerializer<SimpleMsg> ser;
    std::vector<uint8_t>     bad_data(sizeof(SimpleMsg) + 1, 0);
    SimpleMsg                out{};
    EXPECT_FALSE(ser.deserialize(bad_data.data(), bad_data.size(), out));
}

TEST(RawSerializerTest, DeserializeSizeMismatchZero) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                out{};
    EXPECT_FALSE(ser.deserialize(nullptr, 0, out));
}

// ═══════════════════════════════════════════════════════════
// Buffer overrun protection: serialize into too-small buffer
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, SerializeBufferTooSmallReturnsZero) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                msg{};
    msg.id = 1;

    std::vector<uint8_t> small_buf(sizeof(SimpleMsg) - 1, 0xFF);
    size_t               written = ser.serialize(msg, small_buf.data(), small_buf.size());
    EXPECT_EQ(written, 0u);
}

TEST(RawSerializerTest, SerializeBufferZeroSizeReturnsZero) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                msg{};
    EXPECT_EQ(ser.serialize(msg, nullptr, 0), 0u);
}

TEST(RawSerializerTest, SerializeBufferExactSize) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                msg{};
    msg.id = 123;

    std::vector<uint8_t> exact_buf(sizeof(SimpleMsg));
    size_t               written = ser.serialize(msg, exact_buf.data(), exact_buf.size());
    EXPECT_EQ(written, sizeof(SimpleMsg));

    // Verify data is correct
    SimpleMsg decoded{};
    ASSERT_TRUE(ser.deserialize(exact_buf.data(), written, decoded));
    EXPECT_EQ(decoded.id, 123u);
}

// ═══════════════════════════════════════════════════════════
// Wire-format backward compatibility
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, WireFormatByteIdentical) {
    // Verify RawSerializer produces exactly the same bytes as raw
    // reinterpret_cast — this ensures backward compatibility with
    // messages published by the old inline code.
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                msg{};
    msg.id    = 0xDEADBEEF;
    msg.value = 2.718f;
    std::strncpy(msg.tag, "wire_test", sizeof(msg.tag) - 1);

    // Old inline method (what ZenohPublisher used to do)
    const auto*          raw_ptr = reinterpret_cast<const uint8_t*>(&msg);
    std::vector<uint8_t> old_bytes(raw_ptr, raw_ptr + sizeof(SimpleMsg));

    // New serializer method
    auto new_bytes = ser.serialize(msg);

    ASSERT_EQ(old_bytes.size(), new_bytes.size());
    EXPECT_EQ(old_bytes, new_bytes);
}

TEST(RawSerializerTest, WireFormatBufferByteIdentical) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                msg{};
    msg.id    = 0xCAFEBABE;
    msg.value = 1.414f;

    // Old inline SHM method
    const auto*          raw_ptr = reinterpret_cast<const uint8_t*>(&msg);
    std::vector<uint8_t> old_bytes(sizeof(SimpleMsg));
    std::copy(raw_ptr, raw_ptr + sizeof(SimpleMsg), old_bytes.data());

    // New serializer buffer method
    std::vector<uint8_t> new_bytes(sizeof(SimpleMsg));
    size_t               written = ser.serialize(msg, new_bytes.data(), new_bytes.size());
    ASSERT_EQ(written, sizeof(SimpleMsg));

    EXPECT_EQ(old_bytes, new_bytes);
}

// ═══════════════════════════════════════════════════════════
// Polymorphism: use through ISerializer<T>* base pointer
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, PolymorphicUsage) {
    auto serializer = std::make_shared<const RawSerializer<SimpleMsg>>();

    // Use through base interface pointer
    const ISerializer<SimpleMsg>* base = serializer.get();
    EXPECT_EQ(base->name(), "raw");

    SimpleMsg msg{};
    msg.id = 777;
    EXPECT_EQ(base->serialized_size(msg), sizeof(SimpleMsg));

    auto vec = base->serialize(msg);
    ASSERT_EQ(vec.size(), sizeof(SimpleMsg));

    SimpleMsg decoded{};
    ASSERT_TRUE(base->deserialize(vec.data(), vec.size(), decoded));
    EXPECT_EQ(decoded.id, 777u);
}

// ═══════════════════════════════════════════════════════════
// Both serialize overloads produce identical bytes
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, BothSerializeOverloadsMatch) {
    RawSerializer<Pose> ser;
    Pose                msg{};
    msg.timestamp_ns   = 555;
    msg.translation[0] = 100.0;

    auto vec = ser.serialize(msg);

    std::vector<uint8_t> buf(sizeof(Pose));
    size_t               written = ser.serialize(msg, buf.data(), buf.size());
    ASSERT_EQ(written, sizeof(Pose));
    ASSERT_EQ(vec.size(), buf.size());
    EXPECT_EQ(vec, buf);
}

// ═══════════════════════════════════════════════════════════
// Oversized buffer still works (extra space ignored)
// ═══════════════════════════════════════════════════════════

TEST(RawSerializerTest, OversizedBufferWorks) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                msg{};
    msg.id = 42;

    // Buffer is bigger than needed
    std::vector<uint8_t> big_buf(sizeof(SimpleMsg) * 2, 0xAA);
    size_t               written = ser.serialize(msg, big_buf.data(), big_buf.size());
    EXPECT_EQ(written, sizeof(SimpleMsg));

    // Extra bytes should be untouched (still 0xAA)
    for (size_t i = sizeof(SimpleMsg); i < big_buf.size(); ++i) {
        EXPECT_EQ(big_buf[i], 0xAA) << "Extra byte at index " << i << " was modified";
    }

    // Data portion should deserialize correctly
    SimpleMsg decoded{};
    ASSERT_TRUE(ser.deserialize(big_buf.data(), sizeof(SimpleMsg), decoded));
    EXPECT_EQ(decoded.id, 42u);
}

// ── Null pointer safety ───────────────────────────────────

TEST(RawSerializerTest, SerializeNullBufferNonZeroSizeReturnsZero) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                msg{};
    msg.id = 1;

    // Null buf with non-zero size must return 0, not dereference
    EXPECT_EQ(ser.serialize(msg, nullptr, sizeof(SimpleMsg)), 0u);
    EXPECT_EQ(ser.serialize(msg, nullptr, 1024), 0u);
}

TEST(RawSerializerTest, DeserializeNullDataReturnsFailure) {
    RawSerializer<SimpleMsg> ser;
    SimpleMsg                out{};

    // Null data with valid size must return false, not dereference
    EXPECT_FALSE(ser.deserialize(nullptr, sizeof(SimpleMsg), out));
    EXPECT_FALSE(ser.deserialize(nullptr, 0, out));
}
