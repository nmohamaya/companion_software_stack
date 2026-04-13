// common/ipc/include/ipc/iserializer.h
// Abstract serialization interface for IPC message types.
//
// Decouples wire format from transport:  ZenohPublisher/Subscriber delegate
// to ISerializer<T> instead of hardcoding reinterpret_cast + std::copy.
// The default (and currently only) implementation is RawSerializer<T>,
// which produces byte-identical output to the previous inline code.
//
// Virtual dispatch overhead (~1-3 ns per call) is negligible compared to
// Zenoh transport latency (~10-100 us).  This is a deliberate design
// choice: the simpler virtual interface wins over CRTP complexity.
//
// Part of Epic #284 (Platform Modularity), Issue #294.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace drone::ipc {

/// Abstract serializer for IPC message type T.
///
/// Implementations define how a message is converted to/from raw bytes.
/// The interface supports both buffer-write (for SHM zero-copy) and
/// vector-return (for standard bytes path) serialization.
///
/// Serializers are stateless and const-safe, so a single instance can
/// be shared (via shared_ptr<const ISerializer<T>>) across multiple
/// publishers/subscribers for the same message type.
template<typename T>
class ISerializer {
public:
    virtual ~ISerializer() = default;

    ISerializer()                              = default;
    ISerializer(const ISerializer&)            = delete;
    ISerializer& operator=(const ISerializer&) = delete;
    ISerializer(ISerializer&&)                 = default;
    ISerializer& operator=(ISerializer&&)      = default;

    /// Serialize msg into a pre-allocated buffer.
    /// Used by the SHM zero-copy path where the buffer is a Zenoh SHM region.
    /// @param msg       Message to serialize.
    /// @param buf       Destination buffer (caller-owned).
    /// @param buf_size  Size of the destination buffer in bytes.
    /// @return Number of bytes written, or 0 on failure (e.g. buffer too small).
    [[nodiscard]] virtual size_t serialize(const T& msg, uint8_t* buf, size_t buf_size) const = 0;

    /// Serialize msg into a new vector.
    /// Used by the standard bytes path.
    /// @param msg  Message to serialize.
    /// @return Vector of serialized bytes.
    [[nodiscard]] virtual std::vector<uint8_t> serialize(const T& msg) const = 0;

    /// Deserialize from raw bytes into out.
    /// @param data  Source bytes.
    /// @param size  Number of source bytes.
    /// @param out   Destination message (caller-owned).
    /// @return true on success, false on size mismatch or format error.
    [[nodiscard]] virtual bool deserialize(const uint8_t* data, size_t size, T& out) const = 0;

    /// Serialized size for the given message.
    /// For fixed-size formats (e.g. raw), this is constant.
    /// For variable-length formats (e.g. protobuf), it may vary per message.
    /// @param msg  Message to measure.
    /// @return Serialized size in bytes.
    [[nodiscard]] virtual size_t serialized_size(const T& msg) const = 0;

    /// Human-readable name for logging (e.g. "raw", "protobuf").
    [[nodiscard]] virtual std::string_view name() const = 0;
};

}  // namespace drone::ipc
