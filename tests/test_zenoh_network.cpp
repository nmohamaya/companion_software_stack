// tests/test_zenoh_network.cpp
// Phase E — Network transport tests.
//
// Categories:
//   1. Wire format tests (always compiled — no Zenoh dependency)
//   2. ZenohNetworkConfig tests (HAVE_ZENOH only)
//   3. Config-aware factory tests (HAVE_ZENOH only)
//
// These tests verify the wire format header, serialization/deserialization,
// network configuration generation, and config-driven bus creation.
#include "ipc/ipc_types.h"
#include "ipc/message_bus_factory.h"
#include "ipc/wire_format.h"
#include "ipc/zenoh_network_config.h"
#include "ipc/zenoh_session.h"

#include <cstring>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace drone::ipc;

// ═══════════════════════════════════════════════════════════
// Category 1: Wire format tests (always compiled)
// ═══════════════════════════════════════════════════════════

// ── WireHeader layout ────────────────────────────────────

TEST(WireFormat, HeaderSizeIs32Bytes) {
    EXPECT_EQ(sizeof(WireHeader), 32u);
}

TEST(WireFormat, DefaultHeaderHasMagicAndVersion) {
    WireHeader hdr{};
    EXPECT_EQ(hdr.magic, kWireMagic);
    EXPECT_EQ(hdr.version, kWireVersion);
    EXPECT_EQ(hdr.flags, 0);
    EXPECT_EQ(hdr.msg_type, WireMessageType::UNKNOWN);
    EXPECT_EQ(hdr.payload_size, 0u);
    EXPECT_EQ(hdr.timestamp_ns, 0u);
    EXPECT_EQ(hdr.sequence, 0u);
    EXPECT_EQ(hdr.correlation_id, 0u);
}

TEST(WireFormat, MagicIsCorrectDRONLittleEndian) {
    // "DRON" in ASCII = D(0x44) R(0x52) O(0x4F) N(0x4E)
    // Little-endian uint32: 0x4E4F5244
    EXPECT_EQ(kWireMagic, 0x4E4F5244u);

    // Verify byte representation
    uint8_t  bytes[4];
    uint32_t magic = kWireMagic;
    std::memcpy(bytes, &magic, 4);
    EXPECT_EQ(bytes[0], 'D');
    EXPECT_EQ(bytes[1], 'R');
    EXPECT_EQ(bytes[2], 'O');
    EXPECT_EQ(bytes[3], 'N');
}

// ── Serialization round-trip ─────────────────────────────

TEST(WireFormat, SerializeRoundTrip) {
    Pose pose{};
    pose.translation[0] = 1.0;
    pose.translation[1] = 2.0;
    pose.translation[2] = 3.0;
    pose.quaternion[0]  = 1.0;  // w
    pose.quality        = 2;

    auto buf = wire_serialize(pose, WireMessageType::SLAM_POSE, 7, 12345);

    ASSERT_EQ(buf.size(), sizeof(WireHeader) + sizeof(Pose));

    // Validate header
    ASSERT_TRUE(wire_validate(buf.data(), buf.size()));

    WireHeader hdr = wire_read_header(buf.data());
    EXPECT_EQ(hdr.magic, kWireMagic);
    EXPECT_EQ(hdr.version, kWireVersion);
    EXPECT_EQ(hdr.msg_type, WireMessageType::SLAM_POSE);
    EXPECT_EQ(hdr.payload_size, sizeof(Pose));
    EXPECT_EQ(hdr.sequence, 7u);
    EXPECT_EQ(hdr.timestamp_ns, 12345u);

    // Deserialize payload
    Pose out{};
    ASSERT_TRUE(wire_deserialize(buf.data(), buf.size(), out));
    EXPECT_DOUBLE_EQ(out.translation[0], 1.0);
    EXPECT_DOUBLE_EQ(out.translation[1], 2.0);
    EXPECT_DOUBLE_EQ(out.translation[2], 3.0);
    EXPECT_DOUBLE_EQ(out.quaternion[0], 1.0);
    EXPECT_EQ(out.quality, 2u);
}

TEST(WireFormat, SerializeSystemHealth) {
    SystemHealth health{};
    health.cpu_usage_percent    = 55.5f;
    health.memory_usage_percent = 42.0f;
    health.disk_usage_percent   = 75.0f;
    health.cpu_temp_c           = 68.0f;
    health.power_watts          = 11.8f;

    auto buf = wire_serialize(health, WireMessageType::SYSTEM_HEALTH, 100);
    ASSERT_TRUE(wire_validate(buf.data(), buf.size()));

    SystemHealth out{};
    ASSERT_TRUE(wire_deserialize(buf.data(), buf.size(), out));
    EXPECT_FLOAT_EQ(out.cpu_usage_percent, 55.5f);
    EXPECT_FLOAT_EQ(out.memory_usage_percent, 42.0f);
    EXPECT_FLOAT_EQ(out.disk_usage_percent, 75.0f);
    EXPECT_FLOAT_EQ(out.cpu_temp_c, 68.0f);
    EXPECT_FLOAT_EQ(out.power_watts, 11.8f);
}

TEST(WireFormat, SerializeFCState) {
    FCState fc{};
    fc.armed   = true;
    fc.gps_lat = 37.7749f;
    fc.gps_lon = -122.4194f;
    fc.gps_alt = 100.0f;
    fc.yaw     = 270.0f;
    fc.vx      = 5.5f;

    auto buf = wire_serialize(fc, WireMessageType::FC_STATE, 1, 99999);
    ASSERT_TRUE(wire_validate(buf.data(), buf.size()));

    FCState out{};
    ASSERT_TRUE(wire_deserialize(buf.data(), buf.size(), out));
    EXPECT_EQ(out.armed, true);
    EXPECT_FLOAT_EQ(out.gps_lat, 37.7749f);
    EXPECT_FLOAT_EQ(out.gps_lon, -122.4194f);
    EXPECT_FLOAT_EQ(out.gps_alt, 100.0f);
    EXPECT_FLOAT_EQ(out.yaw, 270.0f);
    EXPECT_FLOAT_EQ(out.vx, 5.5f);
}

TEST(WireFormat, SerializeMissionStatus) {
    MissionStatus ms{};
    ms.state            = MissionState::NAVIGATE;
    ms.current_waypoint = 3;
    ms.total_waypoints  = 10;
    ms.progress_percent = 30.0f;

    auto buf = wire_serialize(ms, WireMessageType::MISSION_STATUS, 5);
    ASSERT_TRUE(wire_validate(buf.data(), buf.size()));

    MissionStatus out{};
    ASSERT_TRUE(wire_deserialize(buf.data(), buf.size(), out));
    EXPECT_EQ(out.state, MissionState::NAVIGATE);
    EXPECT_EQ(out.current_waypoint, 3u);
    EXPECT_EQ(out.total_waypoints, 10u);
    EXPECT_FLOAT_EQ(out.progress_percent, 30.0f);
}

// ── Validation failures ──────────────────────────────────

TEST(WireFormat, ValidateRejectsTooSmall) {
    uint8_t tiny[10] = {};
    EXPECT_FALSE(wire_validate(tiny, sizeof(tiny)));
}

TEST(WireFormat, ValidateRejectsBadMagic) {
    Pose pose{};
    auto buf = wire_serialize(pose, WireMessageType::SLAM_POSE);
    // Corrupt magic
    buf[0] = 0xFF;
    EXPECT_FALSE(wire_validate(buf.data(), buf.size()));
}

TEST(WireFormat, ValidateRejectsBadVersion) {
    Pose pose{};
    auto buf = wire_serialize(pose, WireMessageType::SLAM_POSE);
    // Corrupt version byte (offset 4)
    buf[4] = 99;
    EXPECT_FALSE(wire_validate(buf.data(), buf.size()));
}

TEST(WireFormat, ValidateRejectsTruncatedPayload) {
    Pose pose{};
    auto buf = wire_serialize(pose, WireMessageType::SLAM_POSE);
    // Truncate to just the header — payload is missing
    EXPECT_FALSE(wire_validate(buf.data(), sizeof(WireHeader)));
}

TEST(WireFormat, DeserializeSizeMismatch) {
    Pose pose{};
    auto buf = wire_serialize(pose, WireMessageType::SLAM_POSE);

    // Try to deserialize as a different-sized struct
    FCState out{};
    EXPECT_FALSE(wire_deserialize(buf.data(), buf.size(), out));
}

TEST(WireFormat, DeserializeRejectsStructVersionMismatch) {
    // Serialize a valid Pose, then corrupt its embedded version field
    Pose pose{};
    pose.translation[0] = 1.0;
    pose.quaternion[0]  = 1.0;
    pose.quality        = 2;
    auto buf            = wire_serialize(pose, WireMessageType::SLAM_POSE, 1);
    ASSERT_TRUE(wire_validate(buf.data(), buf.size()));

    // Corrupt the struct's version field (first 4 bytes of payload, after WireHeader)
    uint32_t bad_version = 99;
    auto*    ver_dst     = reinterpret_cast<uint8_t*>(&bad_version);
    std::copy(ver_dst, ver_dst + sizeof(bad_version), buf.data() + sizeof(WireHeader));

    Pose out{};
    EXPECT_FALSE(wire_deserialize(buf.data(), buf.size(), out));
}

TEST(WireFormat, DeserializeRejectsZeroStructVersion) {
    FCState fc{};
    fc.battery_voltage   = 16.8f;
    fc.battery_remaining = 85.0f;
    auto buf             = wire_serialize(fc, WireMessageType::FC_STATE, 1);

    // Set struct version to 0
    uint32_t bad_version = 0;
    auto*    ver_dst     = reinterpret_cast<uint8_t*>(&bad_version);
    std::copy(ver_dst, ver_dst + sizeof(bad_version), buf.data() + sizeof(WireHeader));

    FCState out{};
    EXPECT_FALSE(wire_deserialize(buf.data(), buf.size(), out));
}

// ── Key expression mapping ───────────────────────────────

TEST(WireFormat, KeyToWireTypeMapping) {
    EXPECT_EQ(key_to_wire_type("drone/video/frame"), WireMessageType::VIDEO_FRAME);
    EXPECT_EQ(key_to_wire_type("drone/video/stereo_frame"), WireMessageType::STEREO_FRAME);
    EXPECT_EQ(key_to_wire_type("drone/perception/detections"), WireMessageType::DETECTIONS);
    EXPECT_EQ(key_to_wire_type("drone/slam/pose"), WireMessageType::SLAM_POSE);
    EXPECT_EQ(key_to_wire_type("drone/mission/status"), WireMessageType::MISSION_STATUS);
    EXPECT_EQ(key_to_wire_type("drone/mission/trajectory"), WireMessageType::TRAJECTORY_CMD);
    EXPECT_EQ(key_to_wire_type("drone/mission/payload_command"), WireMessageType::PAYLOAD_COMMAND);
    EXPECT_EQ(key_to_wire_type("drone/comms/fc_command"), WireMessageType::FC_COMMAND);
    EXPECT_EQ(key_to_wire_type("drone/comms/fc_state"), WireMessageType::FC_STATE);
    EXPECT_EQ(key_to_wire_type("drone/comms/gcs_command"), WireMessageType::GCS_COMMAND);
    EXPECT_EQ(key_to_wire_type("drone/payload/status"), WireMessageType::PAYLOAD_STATUS);
    EXPECT_EQ(key_to_wire_type("drone/monitor/health"), WireMessageType::SYSTEM_HEALTH);
}

TEST(WireFormat, UnknownKeyReturnsUnknown) {
    EXPECT_EQ(key_to_wire_type("drone/unknown/topic"), WireMessageType::UNKNOWN);
    EXPECT_EQ(key_to_wire_type(""), WireMessageType::UNKNOWN);
    EXPECT_EQ(key_to_wire_type("something/else"), WireMessageType::UNKNOWN);
}

// ── WireMessageType enum values are stable ───────────────

TEST(WireFormat, EnumValuesAreStable) {
    // These exact numeric values are part of the wire protocol and must NOT change.
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::UNKNOWN), 0);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::VIDEO_FRAME), 1);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::STEREO_FRAME), 2);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::DETECTIONS), 10);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::SLAM_POSE), 20);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::MISSION_STATUS), 30);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::TRAJECTORY_CMD), 31);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::PAYLOAD_COMMAND), 32);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::FC_COMMAND), 33);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::FC_STATE), 40);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::GCS_COMMAND), 41);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::PAYLOAD_STATUS), 50);
    EXPECT_EQ(static_cast<uint16_t>(WireMessageType::SYSTEM_HEALTH), 60);
}

// ═══════════════════════════════════════════════════════════
// Category 2: ZenohNetworkConfig tests (HAVE_ZENOH only)
// ═══════════════════════════════════════════════════════════


TEST(ZenohNetworkConfig, MakeDroneDefaultsArePeer) {
    auto cfg = ZenohNetworkConfig::make_drone();
    EXPECT_EQ(cfg.mode, "peer");
    EXPECT_FALSE(cfg.listen_endpoints.empty());
    EXPECT_TRUE(cfg.connect_endpoints.empty());
    EXPECT_TRUE(cfg.multicast_scouting);
    EXPECT_TRUE(cfg.gossip_scouting);

    // Default endpoint should be tcp/127.0.0.1:7447 (localhost for security, #180)
    EXPECT_EQ(cfg.listen_endpoints[0], "tcp/127.0.0.1:7447");
}

TEST(ZenohNetworkConfig, MakeDroneCustomPort) {
    auto cfg = ZenohNetworkConfig::make_drone(8888, "192.168.1.1", "udp");
    ASSERT_EQ(cfg.listen_endpoints.size(), 1u);
    EXPECT_EQ(cfg.listen_endpoints[0], "udp/192.168.1.1:8888");
}

TEST(ZenohNetworkConfig, MakeGCSIsClientMode) {
    auto cfg = ZenohNetworkConfig::make_gcs("10.0.0.1", 7447, "tcp");
    EXPECT_EQ(cfg.mode, "client");
    EXPECT_TRUE(cfg.listen_endpoints.empty());
    ASSERT_EQ(cfg.connect_endpoints.size(), 1u);
    EXPECT_EQ(cfg.connect_endpoints[0], "tcp/10.0.0.1:7447");
    EXPECT_FALSE(cfg.multicast_scouting);
    EXPECT_FALSE(cfg.gossip_scouting);
}

TEST(ZenohNetworkConfig, MakeLocalIsPeerNoListeners) {
    auto cfg = ZenohNetworkConfig::make_local();
    EXPECT_EQ(cfg.mode, "peer");
    EXPECT_TRUE(cfg.listen_endpoints.empty());
    EXPECT_TRUE(cfg.connect_endpoints.empty());
    EXPECT_FALSE(cfg.multicast_scouting);
    EXPECT_FALSE(cfg.gossip_scouting);
}

TEST(ZenohNetworkConfig, ToJsonContainsMode) {
    auto cfg      = ZenohNetworkConfig::make_drone();
    auto json_str = cfg.to_json();
    EXPECT_NE(json_str.find("\"peer\""), std::string::npos);
    EXPECT_NE(json_str.find("listen"), std::string::npos);
}

TEST(ZenohNetworkConfig, ToJsonGCSContainsConnect) {
    auto cfg      = ZenohNetworkConfig::make_gcs("192.168.1.10");
    auto json_str = cfg.to_json();
    EXPECT_NE(json_str.find("\"client\""), std::string::npos);
    EXPECT_NE(json_str.find("192.168.1.10"), std::string::npos);
    EXPECT_NE(json_str.find("connect"), std::string::npos);
}

// ── Config-aware factory with mock config ────────────────

/// Minimal mock config class for testing create_message_bus(cfg).
/// The section() method returns a nlohmann::json value, which satisfies
/// the from_app_config() template's operator[], contains(), etc.
struct MockConfig {
    std::string ipc_backend     = "shm";
    std::size_t shm_pool_mb     = 0;
    bool        network_enabled = false;

    template<typename T>
    T get(const std::string& key, const T& default_val) const {
        if constexpr (std::is_same_v<T, std::string>) {
            if (key == "ipc_backend") return ipc_backend;
            return default_val;
        } else if constexpr (std::is_same_v<T, std::size_t>) {
            if (key == "zenoh.shm_pool_size_mb") return shm_pool_mb;
            return default_val;
        } else if constexpr (std::is_same_v<T, bool>) {
            if (key == "zenoh.network.enabled") return network_enabled;
            return default_val;
        }
        return default_val;
    }

    // section() returns a nlohmann::json object so that from_app_config()
    // compiles. Since our tests only use network_enabled=false, the
    // from_app_config() path is never taken at runtime.
    nlohmann::json section(const std::string& /*key*/) const { return nlohmann::json::object(); }
};

TEST(ConfigAwareFactory, ShmBackendFallsBackToZenoh) {
    MockConfig cfg;
    cfg.ipc_backend = "shm";
    auto bus        = create_message_bus(cfg);
    // "shm" is removed — falls back to Zenoh with error log
    EXPECT_EQ(bus.backend_name(), "zenoh");
}

TEST(ConfigAwareFactory, ZenohBackendFromMockConfig) {
    MockConfig cfg;
    cfg.ipc_backend     = "zenoh";
    cfg.shm_pool_mb     = 0;
    cfg.network_enabled = false;
    auto bus            = create_message_bus(cfg);
    // Should create a ZenohMessageBus (not the legacy ShmMessageBus, which was removed)
    EXPECT_EQ(bus.backend_name(), "zenoh");
}
