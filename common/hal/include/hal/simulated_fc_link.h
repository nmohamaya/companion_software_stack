// common/hal/include/hal/simulated_fc_link.h
// Simulated flight controller link backend.
// Mirrors the behaviour of comms/mavlink_sim.h behind the IFCLink interface.
// Thread-safe: all mutable state guarded by mutex.
#pragma once
#include "hal/ifc_link.h"
#include "util/ilogger.h"

#include <chrono>
#include <cmath>
#include <mutex>

namespace drone::hal {

class SimulatedFCLink : public IFCLink {
public:
    bool open(const std::string& port, int baud) override {
        std::lock_guard<std::mutex> lock(mtx_);
        (void)port;
        (void)baud;
        DRONE_LOG_INFO("[SimulatedFCLink] Opened simulated link {}@{}", port, baud);
        connected_  = true;
        start_time_ = std::chrono::steady_clock::now();
        return true;
    }

    void close() override {
        std::lock_guard<std::mutex> lock(mtx_);
        connected_ = false;
        DRONE_LOG_INFO("[SimulatedFCLink] Closed");
    }

    bool is_connected() const override {
        std::lock_guard<std::mutex> lock(mtx_);
        return connected_;
    }

    bool send_trajectory(float vx, float vy, float vz, float yaw) override {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!connected_) return false;
        DRONE_LOG_DEBUG(
            "[SimulatedFCLink] SET_POSITION_TARGET vx={:.2f} vy={:.2f} vz={:.2f} yaw={:.2f}", vx,
            vy, vz, yaw);
        last_vx_ = vx;
        last_vy_ = vy;
        last_vz_ = vz;
        return true;
    }

    bool send_arm(bool arm) override {
        std::lock_guard<std::mutex> lock(mtx_);
        DRONE_LOG_INFO("[SimulatedFCLink] {} command", arm ? "ARM" : "DISARM");
        state_.armed = arm;
        return true;
    }

    bool send_mode(uint8_t mode) override {
        std::lock_guard<std::mutex> lock(mtx_);
        DRONE_LOG_INFO("[SimulatedFCLink] MODE change to {}", mode);
        state_.flight_mode = mode;
        return true;
    }

    bool send_takeoff(float altitude_m) override {
        std::lock_guard<std::mutex> lock(mtx_);
        DRONE_LOG_INFO("[SimulatedFCLink] TAKEOFF to {:.1f}m", altitude_m);
        state_.altitude_rel = altitude_m;  // Simulate instant takeoff
        state_.flight_mode  = 2;           // AUTO/Takeoff
        return true;
    }

    FCState receive_state() override {
        std::lock_guard<std::mutex> lock(mtx_);
        auto                        now = std::chrono::steady_clock::now();
        double elapsed                  = std::chrono::duration<double>(now - start_time_).count();

        state_.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        // Simulate battery drain
        state_.battery_percent = std::max(0.0f, 100.0f - static_cast<float>(elapsed) * 0.05f);
        state_.battery_voltage = 12.0f + state_.battery_percent * 0.048f;
        state_.ground_speed    = std::sqrt(last_vx_ * last_vx_ + last_vy_ * last_vy_);
        state_.altitude_rel    = 100.0f + last_vz_ * 0.1f;
        state_.satellites =
            static_cast<uint8_t>(12 + static_cast<int>(std::sin(elapsed * 0.1) * 3));

        return state_;
    }

    std::string name() const override { return "SimulatedFCLink"; }

private:
    mutable std::mutex                    mtx_;
    bool                                  connected_{false};
    FCState                               state_{};
    float                                 last_vx_{0}, last_vy_{0}, last_vz_{0};
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace drone::hal
