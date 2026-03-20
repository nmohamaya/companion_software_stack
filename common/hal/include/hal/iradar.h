// common/hal/include/hal/iradar.h
// Hardware-abstraction interface for radar sensors.
//
// Implementations: SimulatedRadar (always available), GazeboRadar (compile-guarded).
// Implements Issue #209 — IRadar HAL interface.
#pragma once

#include "ipc/ipc_types.h"

#include <string>

namespace drone::hal {

/// Abstract radar sensor interface.
class IRadar {
public:
    virtual ~IRadar() = default;

    /// Initialise the radar sensor. Returns true on success.
    [[nodiscard]] virtual bool init() = 0;

    /// Read the latest radar scan. Returns empty list if no data available.
    [[nodiscard]] virtual drone::ipc::RadarDetectionList read() = 0;

    /// True after successful init() and before close().
    [[nodiscard]] virtual bool is_active() const = 0;

    /// Human-readable backend name for logging.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
