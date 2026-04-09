// common/hal/include/hal/simulated_gcs_link.h
// Simulated GCS link backend.
// Mirrors the behaviour of comms/gcs_link.h behind the IGCSLink interface.
// Thread-safe: all mutable state guarded by mutex.
#pragma once
#include "hal/igcs_link.h"
#include "util/ilogger.h"

#include <chrono>
#include <mutex>

namespace drone::hal {

class SimulatedGCSLink : public IGCSLink {
public:
    bool open(const std::string& addr, int port) override {
        std::lock_guard<std::mutex> lock(mtx_);
        (void)addr;
        (void)port;
        DRONE_LOG_INFO("[SimulatedGCSLink] Simulated UDP link {}:{}", addr, port);
        connected_  = true;
        start_time_ = std::chrono::steady_clock::now();
        return true;
    }

    void close() override {
        std::lock_guard<std::mutex> lock(mtx_);
        connected_ = false;
        DRONE_LOG_INFO("[SimulatedGCSLink] Closed");
    }

    bool is_connected() const override {
        std::lock_guard<std::mutex> lock(mtx_);
        return connected_;
    }

    bool send_telemetry(float lat, float lon, float alt, float battery, uint8_t state) override {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!connected_) return false;
        telem_count_++;
        if (telem_count_ % 50 == 0) {
            DRONE_LOG_DEBUG("[SimulatedGCSLink] Telemetry #{}: pos=({:.4f},{:.4f},{:.1f}) "
                            "batt={:.0f}% state={}",
                            telem_count_, lat, lon, alt, battery, state);
        }
        return true;
    }

    GCSCommand poll_command() override {
        std::lock_guard<std::mutex> lock(mtx_);
        GCSCommand                  cmd{};
        auto                        now = std::chrono::steady_clock::now();
        double elapsed                  = std::chrono::duration<double>(now - start_time_).count();

        // Simulate an RTL command after 300 seconds (scenario timeout is 180 s,
        // but with obstacle avoidance the mission can take longer than 120 s.
        // 300 s gives enough headroom for any test scenario while still providing
        // a GCS watchdog for runaway processes.)
        if (!rtl_sent_ && elapsed > 300.0) {
            cmd.type         = GCSCommandType::RTL;
            cmd.timestamp_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                    .count());
            cmd.valid = true;
            rtl_sent_ = true;
            DRONE_LOG_INFO("[SimulatedGCSLink] Simulated RTL command from GCS");
        }
        return cmd;
    }

    std::string name() const override { return "SimulatedGCSLink"; }

private:
    mutable std::mutex                    mtx_;
    bool                                  connected_{false};
    bool                                  rtl_sent_{false};
    uint64_t                              telem_count_{0};
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace drone::hal
