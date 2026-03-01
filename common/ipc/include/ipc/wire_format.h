// common/ipc/include/ipc/wire_format.h
// Lightweight wire format for network-transported Zenoh messages.
//
// When a Zenoh message crosses a network boundary (drone ↔ GCS), the
// payload is trivially-copyable raw bytes.  This header defines a
// fixed-size prefix that provides:
//   - Magic number for fast validity checks
//   - Message type ID for deserialization routing
//   - Payload size for framing
//   - Timestamp + sequence number for ordering / loss detection
//
// Design rationale (see issue #50):
//   Protobuf adds ~3 MB dependency + codegen step.  Our structs are already
//   fixed-size and trivially copyable.  A packed header + raw bytes is
//   sufficient for the drone ↔ GCS link and can be upgraded to protobuf
//   later if multi-vendor interop is needed.
//
// This header is backend-agnostic — included unconditionally.
#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

namespace drone::ipc {

/// Magic number: ASCII "DRON" (0x44524F4E), little-endian.
static constexpr uint32_t kWireMagic = 0x4E4F5244;  // "DRON" LE

/// Wire format version.  Increment on breaking layout changes.
static constexpr uint8_t kWireVersion = 1;

/// Message type identifiers for wire format routing.
/// Values are stable — never renumber existing entries.
enum class WireMessageType : uint16_t {
    UNKNOWN           = 0,

    // ── Video (Process 1) ────────────────────────────
    VIDEO_FRAME       = 1,   // ShmVideoFrame
    STEREO_FRAME      = 2,   // ShmStereoFrame

    // ── Perception (Process 2) ───────────────────────
    DETECTIONS        = 10,  // ShmDetectedObjectList

    // ── SLAM (Process 3) ─────────────────────────────
    SLAM_POSE         = 20,  // ShmSlamPose

    // ── Mission Planner (Process 4) ──────────────────
    MISSION_STATUS    = 30,  // ShmMissionStatus
    TRAJECTORY_CMD    = 31,  // ShmTrajectoryCmd
    PAYLOAD_COMMAND   = 32,  // ShmPayloadCommand
    FC_COMMAND        = 33,  // ShmFCCommand

    // ── Comms (Process 5) ────────────────────────────
    FC_STATE          = 40,  // ShmFCState
    GCS_COMMAND       = 41,  // ShmGCSCommand

    // ── Payload Manager (Process 6) ──────────────────
    PAYLOAD_STATUS    = 50,  // ShmPayloadStatus

    // ── System Monitor (Process 7) ───────────────────
    SYSTEM_HEALTH     = 60,  // ShmSystemHealth
};

/// Fixed-size wire header prepended to every network message.
///
/// Layout (24 bytes, packed):
///   [0..3]   magic         — 0x4E4F5244 ("DRON" LE)
///   [4]      version       — wire format version (currently 1)
///   [5]      flags         — reserved (0)
///   [6..7]   msg_type      — WireMessageType enum
///   [8..11]  payload_size  — size of payload following this header
///   [12..19] timestamp_ns  — sender's steady_clock timestamp
///   [20..23] sequence      — per-topic monotonic counter
struct __attribute__((packed)) WireHeader {
    uint32_t        magic        = kWireMagic;
    uint8_t         version      = kWireVersion;
    uint8_t         flags        = 0;
    WireMessageType msg_type     = WireMessageType::UNKNOWN;
    uint32_t        payload_size = 0;
    uint64_t        timestamp_ns = 0;
    uint32_t        sequence     = 0;
};

static_assert(sizeof(WireHeader) == 24,
              "WireHeader must be exactly 24 bytes (packed)");
static_assert(std::is_trivially_copyable_v<WireHeader>,
              "WireHeader must be trivially copyable");

// ─── Serialization helpers ──────────────────────────────────

/// Serialize a trivially-copyable message with a wire header prefix.
///
/// Returns a byte vector containing [WireHeader | payload].
/// The header's payload_size is set to sizeof(T).
///
/// @tparam T         Trivially-copyable message type.
/// @param  msg       The message to serialize.
/// @param  msg_type  Wire message type identifier.
/// @param  seq       Sequence number (caller-managed per topic).
/// @param  ts_ns     Timestamp in nanoseconds (0 = auto from steady_clock).
template <typename T>
std::vector<uint8_t> wire_serialize(
    const T& msg,
    WireMessageType msg_type,
    uint32_t seq = 0,
    uint64_t ts_ns = 0)
{
    static_assert(std::is_trivially_copyable_v<T>,
                  "wire_serialize requires trivially copyable T");

    WireHeader hdr;
    hdr.msg_type     = msg_type;
    hdr.payload_size = static_cast<uint32_t>(sizeof(T));
    hdr.sequence     = seq;
    hdr.timestamp_ns = ts_ns;

    std::vector<uint8_t> buf(sizeof(WireHeader) + sizeof(T));
    std::memcpy(buf.data(), &hdr, sizeof(WireHeader));
    std::memcpy(buf.data() + sizeof(WireHeader), &msg, sizeof(T));
    return buf;
}

/// Validate a wire header at the start of a byte buffer.
///
/// @param  data  Pointer to the received buffer.
/// @param  len   Length of the buffer in bytes.
/// @return true if the header is valid (magic + version + size check).
inline bool wire_validate(const uint8_t* data, std::size_t len) {
    if (len < sizeof(WireHeader)) return false;

    WireHeader hdr;
    std::memcpy(&hdr, data, sizeof(WireHeader));

    if (hdr.magic != kWireMagic) return false;
    if (hdr.version != kWireVersion) return false;
    if (len < sizeof(WireHeader) + hdr.payload_size) return false;

    return true;
}

/// Extract the wire header from a byte buffer (no validation).
inline WireHeader wire_read_header(const uint8_t* data) {
    WireHeader hdr;
    std::memcpy(&hdr, data, sizeof(WireHeader));
    return hdr;
}

/// Deserialize the payload following a wire header.
///
/// @tparam T     Expected payload type (must match header's payload_size).
/// @param  data  Pointer to the full wire message (header + payload).
/// @param  len   Total length of the buffer.
/// @param  out   Output: deserialized message.
/// @return true if deserialization succeeded, false on size mismatch.
template <typename T>
bool wire_deserialize(const uint8_t* data, std::size_t len, T& out) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "wire_deserialize requires trivially copyable T");

    if (!wire_validate(data, len)) return false;

    WireHeader hdr = wire_read_header(data);
    if (hdr.payload_size != sizeof(T)) return false;

    std::memcpy(&out, data + sizeof(WireHeader), sizeof(T));
    return true;
}

/// Map a Zenoh key expression to its wire message type.
///
/// Used by network-aware publishers to stamp the correct type ID
/// in the wire header.  Returns UNKNOWN for unrecognized keys.
inline WireMessageType key_to_wire_type(const std::string& key) {
    // Use a simple if-chain; the mapping is small and called once per pub.
    if (key == "drone/video/frame")               return WireMessageType::VIDEO_FRAME;
    if (key == "drone/video/stereo_frame")         return WireMessageType::STEREO_FRAME;
    if (key == "drone/perception/detections")      return WireMessageType::DETECTIONS;
    if (key == "drone/slam/pose")                  return WireMessageType::SLAM_POSE;
    if (key == "drone/mission/status")             return WireMessageType::MISSION_STATUS;
    if (key == "drone/mission/trajectory_cmd")     return WireMessageType::TRAJECTORY_CMD;
    if (key == "drone/payload/commands")           return WireMessageType::PAYLOAD_COMMAND;
    if (key == "drone/comms/fc_command")           return WireMessageType::FC_COMMAND;
    if (key == "drone/comms/fc_state")             return WireMessageType::FC_STATE;
    if (key == "drone/comms/gcs_command")          return WireMessageType::GCS_COMMAND;
    if (key == "drone/payload/status")             return WireMessageType::PAYLOAD_STATUS;
    if (key == "drone/monitor/health")             return WireMessageType::SYSTEM_HEALTH;
    return WireMessageType::UNKNOWN;
}

}  // namespace drone::ipc
