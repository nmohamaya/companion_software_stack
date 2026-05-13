// common/hal/include/hal/ifc_link.h
// HAL interface: Flight Controller communication link.
// Implementations: SimulatedFCLink (built-in, Tier 1 unit tests + dev runs),
//                  MavlinkFCLink   (MAVSDK + PX4 SITL or real Pixhawk, Tier 2),
//                  CosysFCLink     (AirSim SimpleFlight RPC,            Tier 3).
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
    /// Relative altitude in meters (above takeoff/spawn point; not terrain-corrected AGL).
    /// MavlinkFCLink sources this from MAVSDK Telemetry::Position::relative_altitude_m
    /// (relative to the arm altitude). CosysFCLink reports altitude above spawn, since
    /// SimpleFlight does not model terrain. Callers that require true AGL must correct
    /// with a terrain model externally.
    float   altitude_rel{0.0f};
    float   ground_speed{0.0f};
    uint8_t satellites{0};
    uint8_t flight_mode{0};  // 0=STAB, 1=GUIDED, 2=AUTO, 3=RTL
    bool    armed{false};
    /// Issue #716 — true once the FC reports preflight checks have passed.
    /// MavlinkFCLink sources this from MAVSDK Telemetry::subscribe_health_all_ok
    /// (EKF2 converged, sensors initialized, GPS lock acquired, etc.).
    /// CosysFCLink sets this to true once the RPC connection is established —
    /// SimpleFlight has no real preflight check and accepts ARM unconditionally.
    /// SimulatedFCLink mirrors that pattern (always armable when connected).
    bool armable{false};
};

/// Abstract flight controller link interface.
class IFCLink {
public:
    virtual ~IFCLink() = default;

    /// Open the link (e.g. serial port or simulated).
    [[nodiscard]] virtual bool open(const std::string& port, int baud) = 0;

    /// Close the link.
    virtual void close() = 0;

    /// Check connection status.
    [[nodiscard]] virtual bool is_connected() const = 0;

    /// Send a velocity+yaw trajectory command to the FC.
    /// @param vx   Velocity north / forward (m/s). Frame per implementation docstring.
    /// @param vy   Velocity east  / right   (m/s). Frame per implementation docstring.
    /// @param vz   Velocity up / down       (m/s). Frame per implementation docstring.
    /// @param yaw  Yaw angle in radians (0 = North, CW positive).
    ///             Backends convert to their native FC unit internally.
    [[nodiscard]] virtual bool send_trajectory(float vx, float vy, float vz, float yaw) = 0;

    /// Send arm/disarm command.
    [[nodiscard]] virtual bool send_arm(bool arm) = 0;

    /// Send flight mode change.
    [[nodiscard]] virtual bool send_mode(uint8_t mode) = 0;

    /// Command autonomous takeoff to a target altitude (m AGL).
    [[nodiscard]] virtual bool send_takeoff(float altitude_m) = 0;

    /// Receive the latest FC state (heartbeat).  Thread-safe — implementations
    /// must serialise concurrent reads internally (typically with a mutex on
    /// a cached snapshot updated by the link's callback / poll thread).
    ///
    /// Issue #716 — implementations MUST populate `FCState::armable`:
    ///   - true when the FC reports preflight checks have passed and ARM
    ///     is safe to send (e.g. PX4 EKF2 converged, sensors initialized).
    ///   - false otherwise (FC still initializing, sensor fault, etc.).
    /// Backends with no real preflight check (Simulated, Cosys SimpleFlight)
    /// should report `armable=true` once connected — leaving it stuck at
    /// `false` will block P4 mission_planner indefinitely in PREFLIGHT.
    [[nodiscard]] virtual FCState receive_state() = 0;

    /// Human-readable name.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
