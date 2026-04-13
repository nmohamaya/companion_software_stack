// common/ipc/include/ipc/raw_serializer.h
// Raw binary serializer — trivially-copyable structs via reinterpret_cast.
//
// Produces byte-identical output to the previous inline serialization in
// ZenohPublisher/Subscriber, ensuring wire-format backward compatibility.
//
// The static_assert on trivially_copyable lives here (not in ZenohPublisher)
// so that future serializers (e.g. ProtobufSerializer) can handle
// non-trivially-copyable types without hitting the constraint.
//
// Part of Epic #284 (Platform Modularity), Issue #294.
#pragma once

#include "ipc/iserializer.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>
#include <vector>

namespace drone::ipc {

/// Raw binary serializer for trivially-copyable IPC structs.
///
/// Uses reinterpret_cast + std::copy — the same mechanism previously
/// hardcoded in ZenohPublisher/Subscriber.  Wire format is the raw
/// object representation: sizeof(T) bytes, host byte order.
template<typename T>
class RawSerializer final : public ISerializer<T> {
    static_assert(std::is_trivially_copyable_v<T>,
                  "RawSerializer requires trivially copyable types");

public:
    /// Serialize msg into a pre-allocated buffer (SHM zero-copy path).
    /// @return sizeof(T) on success, 0 if buf_size < sizeof(T).
    [[nodiscard]] size_t serialize(const T& msg, uint8_t* buf, size_t buf_size) const override {
        if (!buf || buf_size < sizeof(T)) {
            return 0;
        }
        const auto* src = reinterpret_cast<const uint8_t*>(&msg);
        std::copy(src, src + sizeof(T), buf);
        return sizeof(T);
    }

    /// Serialize msg to a new vector (standard bytes path).
    [[nodiscard]] std::vector<uint8_t> serialize(const T& msg) const override {
        const auto* ptr = reinterpret_cast<const uint8_t*>(&msg);
        return std::vector<uint8_t>(ptr, ptr + sizeof(T));
    }

    /// Deserialize from raw bytes.
    /// @return false if size != sizeof(T).
    [[nodiscard]] bool deserialize(const uint8_t* data, size_t size, T& out) const override {
        if (!data || size != sizeof(T)) {
            return false;
        }
        auto* dst = reinterpret_cast<uint8_t*>(&out);
        std::copy(data, data + size, dst);
        return true;
    }

    /// Serialized size is always sizeof(T) for raw format.
    [[nodiscard]] size_t serialized_size([[maybe_unused]] const T& msg) const override {
        return sizeof(T);
    }

    /// Returns "raw".
    [[nodiscard]] std::string_view name() const override { return "raw"; }
};

}  // namespace drone::ipc
