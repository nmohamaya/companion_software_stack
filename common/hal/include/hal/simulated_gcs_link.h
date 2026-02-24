// common/hal/include/hal/simulated_gcs_link.h
// Simulated GCS link backend.
// Mirrors the behaviour of comms/gcs_link.h behind the IGCSLink interface.
#pragma once
#include "hal/igcs_link.h"
#include <chrono>
#include <spdlog/spdlog.h>

namespace drone::hal {

class SimulatedGCSLink : public IGCSLink {
public:
    bool open(const std::string& addr, int port) override {
        (void)addr; (void)port;
        spdlog::info("[SimulatedGCSLink] Simulated UDP link {}:{}", addr, port);
        connected_ = true;
        start_time_ = std::chrono::steady_clock::now();
        return true;
    }

    void close() override {
        connected_ = false;
        spdlog::info("[SimulatedGCSLink] Closed");
    }

    bool is_connected() const override { return connected_; }

    bool send_telemetry(float lat, float lon, float alt,
                        float battery, uint8_t state) override {
        if (!connected_) return false;
        telem_count_++;
        if (telem_count_ % 50 == 0) {
            spdlog::debug("[SimulatedGCSLink] Telemetry #{}: pos=({:.4f},{:.4f},{:.1f}) "
                          "batt={:.0f}% state={}",
                          telem_count_, lat, lon, alt, battery, state);
        }
        return true;
    }

    GCSCommand poll_command() override {
        GCSCommand cmd{};
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time_).count();

        // Simulate an RTL command after 120 seconds
        if (!rtl_sent_ && elapsed > 120.0) {
            cmd.type = GCSCommandType::RTL;
            cmd.timestamp_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    now.time_since_epoch()).count());
            cmd.valid = true;
            rtl_sent_ = true;
            spdlog::info("[SimulatedGCSLink] Simulated RTL command from GCS");
        }
        return cmd;
    }

    std::string name() const override { return "SimulatedGCSLink"; }

private:
    bool connected_{false};
    bool rtl_sent_{false};
    uint64_t telem_count_{0};
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace drone::hal
