// common/hal/include/hal/igimbal.h
// HAL interface: Gimbal + payload camera controller.
// Implementations: SimulatedGimbal (built-in), SIYIGimbal (future).
#pragma once
#include <cstdint>
#include <string>

namespace drone::hal {

/// Current gimbal state.
struct GimbalState {
    float pitch{0.0f};  // degrees, -90 (nadir) to +30
    float yaw{0.0f};    // degrees, relative to body frame
    float roll{0.0f};   // degrees
    bool  stabilised{true};
};

/// Abstract gimbal interface.
class IGimbal {
public:
    virtual ~IGimbal() = default;

    /// Initialise the gimbal hardware/simulation.
    [[nodiscard]] virtual bool init() = 0;

    /// Check if initialised.
    [[nodiscard]] virtual bool is_initialised() const = 0;

    /// Set target angles (degrees).
    virtual void set_target(float pitch_deg, float yaw_deg) = 0;

    /// Step the gimbal simulation/control loop (call at control rate).
    virtual void update(float dt_s) = 0;

    /// Get current state.
    [[nodiscard]] virtual GimbalState state() const = 0;

    /// Trigger a still image capture. Returns timestamp_ns.
    [[nodiscard]] virtual uint64_t capture_image() = 0;

    /// Start video recording.
    virtual void start_recording() = 0;

    /// Stop video recording.
    virtual void stop_recording() = 0;

    /// Check if currently recording.
    [[nodiscard]] virtual bool is_recording() const = 0;

    /// Human-readable name.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
