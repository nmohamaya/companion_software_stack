// Flight-controller mode sampled from telemetry, separate from command results.
#pragma once

#include <cstdint>

namespace drone {

enum class FCObservedMode : uint8_t { UNKNOWN, OTHER, RTL, LAND };

struct FCModeObservation {
    uint64_t       timestamp_ns{0};  // Host monotonic time of the mode sample
    FCObservedMode mode{FCObservedMode::UNKNOWN};
    uint8_t        reserved[7]{};

    [[nodiscard]] bool validate() const { return timestamp_ns > 0 && mode <= FCObservedMode::LAND; }
};

}  // namespace drone
