// common/hal/include/hal/ifc_link.h
// HAL interface: Flight Controller communication link.
// Implementations: SimulatedFCLink (built-in), MavlinkV2Link (future).
#pragma once
#include <cstdint>
#include <string>

namespace drone::hal {

/// Flight controller state received from the FC heartbeat.
struct FCState {
    uint64_t timestamp_ns{0};
    float    battery_voltage{0.0f};
    float    battery_current{0.0f};
    float    battery_percent{0.0f};
    float    altitude_rel{0.0f};  // relative (AGL) altitude in meters
    float    ground_speed{0.0f};
    uint8_t  satellites{0};
    uint8_t  flight_mode{0};  // 0=STAB, 1=GUIDED, 2=AUTO, 3=RTL
    bool     armed{false};
};

/// Abstract flight controller link interface.
class IFCLink {
public:
    virtual ~IFCLink() = default;

    /// Open the link (e.g. serial port or simulated).
    virtual bool open(const std::string& port, int baud) = 0;

    /// Close the link.
    virtual void close() = 0;

    /// Check connection status.
    virtual bool is_connected() const = 0;

    /// Send a velocity+yaw trajectory command to the FC.
    virtual bool send_trajectory(float vx, float vy, float vz, float yaw) = 0;

    /// Send arm/disarm command.
    virtual bool send_arm(bool arm) = 0;

    /// Send flight mode change.
    virtual bool send_mode(uint8_t mode) = 0;

    /// Command autonomous takeoff to a target altitude (m AGL).
    virtual bool send_takeoff(float altitude_m) = 0;

    /// Receive the latest FC state (heartbeat).
    virtual FCState receive_state() = 0;

    /// Human-readable name.
    virtual std::string name() const = 0;
};

}  // namespace drone::hal
