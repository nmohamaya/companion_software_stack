// process5_comms/include/comms/mavlink_sim.h
// Simulated MAVLink communication interface.
// On real hardware this wraps serial / UDP MAVLink v2; here we stub it.

#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <spdlog/spdlog.h>

namespace drone::comms {

// Simulated flight-controller heartbeat data
struct FCHeartbeat {
    uint64_t timestamp_ns{0};
    float    battery_voltage{16.4f};
    float    battery_current{8.5f};
    float    battery_percent{100.0f};
    float    altitude_rel{0.0f};
    float    ground_speed{0.0f};
    uint8_t  satellites{12};
    uint8_t  flight_mode{0};  // 0=STAB, 1=GUIDED, 2=AUTO, 3=RTL
    bool     armed{false};
};

// ── Simulated MAVLink link ──────────────────────────────────
class MavlinkSim {
public:
    [[nodiscard]] bool open(const std::string& port, int baud) {
        (void)port;
        (void)baud;
        spdlog::info("[MavlinkSim] Opened simulated link {}@{}", port, baud);
        connected_  = true;
        start_time_ = std::chrono::steady_clock::now();
        return true;
    }

    [[nodiscard]] bool is_connected() const { return connected_; }

    // Send trajectory command to FC (simulated)
    [[nodiscard]] bool send_trajectory(float vx, float vy, float vz, float yaw) {
        if (!connected_) return false;
        spdlog::debug("[MavlinkSim] SET_POSITION_TARGET_LOCAL_NED "
                      "vx={:.2f} vy={:.2f} vz={:.2f} yaw={:.2f}",
                      vx, vy, vz, yaw);
        last_cmd_vx_ = vx;
        last_cmd_vy_ = vy;
        last_cmd_vz_ = vz;
        return true;
    }

    // Send arm / disarm
    [[nodiscard]] bool send_arm(bool arm) {
        spdlog::info("[MavlinkSim] {} command sent", arm ? "ARM" : "DISARM");
        heartbeat_.armed = arm;
        return true;
    }

    // Send mode change
    [[nodiscard]] bool send_mode(uint8_t mode) {
        spdlog::info("[MavlinkSim] MODE change to {}", mode);
        heartbeat_.flight_mode = mode;
        return true;
    }

    // Receive heartbeat (simulated)
    [[nodiscard]] FCHeartbeat receive_heartbeat() {
        auto   now     = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time_).count();

        heartbeat_.timestamp_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        // Simulate battery drain
        heartbeat_.battery_percent = std::max(0.0f, 100.0f - static_cast<float>(elapsed) * 0.05f);
        heartbeat_.battery_voltage = 12.0f + heartbeat_.battery_percent * 0.048f;
        heartbeat_.ground_speed =
            std::sqrt(last_cmd_vx_ * last_cmd_vx_ + last_cmd_vy_ * last_cmd_vy_);
        heartbeat_.altitude_rel = 100.0f + last_cmd_vz_ * 0.1f;
        heartbeat_.satellites   = 12 + static_cast<uint8_t>(std::sin(elapsed * 0.1) * 3);

        return heartbeat_;
    }

private:
    bool                                  connected_{false};
    FCHeartbeat                           heartbeat_{};
    float                                 last_cmd_vx_{0}, last_cmd_vy_{0}, last_cmd_vz_{0};
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace drone::comms
