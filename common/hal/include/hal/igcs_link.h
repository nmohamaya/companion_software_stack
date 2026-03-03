// common/hal/include/hal/igcs_link.h
// HAL interface: Ground Control Station communication link.
// Implementations: SimulatedGCSLink (built-in), UDPGCSLink (future).
#pragma once
#include <cstdint>
#include <string>

namespace drone::hal {

/// GCS command types.
enum class GCSCommandType : uint8_t {
    NONE      = 0,
    RTL       = 1,
    LAND      = 2,
    MISSION   = 3,
    PARAM_SET = 4,
};

/// A command received from the GCS.
struct GCSCommand {
    GCSCommandType type{GCSCommandType::NONE};
    float          param1{0.0f};
    float          param2{0.0f};
    uint64_t       timestamp_ns{0};
    bool           valid{false};
};

/// Abstract GCS link interface.
class IGCSLink {
public:
    virtual ~IGCSLink() = default;

    /// Open the GCS link (e.g. UDP socket or simulated).
    virtual bool open(const std::string& addr, int port) = 0;

    /// Close the link.
    virtual void close() = 0;

    /// Check connection status.
    virtual bool is_connected() const = 0;

    /// Send telemetry to the GCS.
    virtual bool send_telemetry(float lat, float lon, float alt, float battery, uint8_t state) = 0;

    /// Poll for incoming GCS commands.
    virtual GCSCommand poll_command() = 0;

    /// Human-readable name.
    virtual std::string name() const = 0;
};

}  // namespace drone::hal
