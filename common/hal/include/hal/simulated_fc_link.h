// common/hal/include/hal/simulated_fc_link.h
// Simulated flight controller link backend.
// Mirrors the behaviour of comms/mavlink_sim.h behind the IFCLink interface.
#pragma once
#include "hal/ifc_link.h"
#include <chrono>
#include <cmath>
#include <spdlog/spdlog.h>

namespace drone::hal {

class SimulatedFCLink : public IFCLink {
public:
    bool open(const std::string& port, int baud) override {
        (void)port; (void)baud;
        spdlog::info("[SimulatedFCLink] Opened simulated link {}@{}", port, baud);
        connected_ = true;
        start_time_ = std::chrono::steady_clock::now();
        return true;
    }

    void close() override {
        connected_ = false;
        spdlog::info("[SimulatedFCLink] Closed");
    }

    bool is_connected() const override { return connected_; }

    bool send_trajectory(float vx, float vy, float vz, float yaw) override {
        if (!connected_) return false;
        spdlog::debug("[SimulatedFCLink] SET_POSITION_TARGET vx={:.2f} vy={:.2f} vz={:.2f} yaw={:.2f}",
                      vx, vy, vz, yaw);
        last_vx_ = vx; last_vy_ = vy; last_vz_ = vz;
        return true;
    }

    bool send_arm(bool arm) override {
        spdlog::info("[SimulatedFCLink] {} command", arm ? "ARM" : "DISARM");
        state_.armed = arm;
        return true;
    }

    bool send_mode(uint8_t mode) override {
        spdlog::info("[SimulatedFCLink] MODE change to {}", mode);
        state_.flight_mode = mode;
        return true;
    }

    FCState receive_state() override {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time_).count();

        state_.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count());
        // Simulate battery drain
        state_.battery_percent = std::max(0.0f, 100.0f - static_cast<float>(elapsed) * 0.05f);
        state_.battery_voltage = 12.0f + state_.battery_percent * 0.048f;
        state_.ground_speed = std::sqrt(last_vx_ * last_vx_ + last_vy_ * last_vy_);
        state_.altitude_msl = 100.0f + last_vz_ * 0.1f;
        state_.satellites = static_cast<uint8_t>(
            12 + static_cast<int>(std::sin(elapsed * 0.1) * 3));

        return state_;
    }

    std::string name() const override { return "SimulatedFCLink"; }

private:
    bool connected_{false};
    FCState state_{};
    float last_vx_{0}, last_vy_{0}, last_vz_{0};
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace drone::hal
