// process5_comms/include/comms/gcs_link.h
// Simulated Ground Control Station link (UDP telemetry).

#pragma once
#include <cstdint>
#include <cstring>
#include <chrono>
#include <atomic>
#include <spdlog/spdlog.h>

namespace drone::comms {

enum class GCSMessageType : uint8_t {
    HEARTBEAT   = 0,
    MISSION_CMD = 1,
    RTL_CMD     = 2,
    LAND_CMD    = 3,
    PARAM_SET   = 4,
};

struct GCSMessage {
    GCSMessageType type{GCSMessageType::HEARTBEAT};
    float param1{0};
    float param2{0};
    uint64_t timestamp_ns{0};
    bool valid{false};
};

// ── Simulated GCS link ──────────────────────────────────────
class GCSLink {
public:
    bool open(const std::string& addr, int port) {
        (void)addr; (void)port;
        spdlog::info("[GCSLink] Simulated UDP link {}:{}", addr, port);
        connected_ = true;
        start_time_ = std::chrono::steady_clock::now();
        return true;
    }

    bool is_connected() const { return connected_; }

    // Send telemetry to GCS (simulated — just logs)
    bool send_telemetry(float lat, float lon, float alt,
                        float battery, uint8_t state) {
        if (!connected_) return false;
        telem_count_++;
        if (telem_count_ % 50 == 0) {
            spdlog::debug("[GCSLink] Telemetry #{}: pos=({:.4f},{:.4f},{:.1f}) "
                          "batt={:.0f}% state={}",
                          telem_count_, lat, lon, alt, battery, state);
        }
        return true;
    }

    // Poll for incoming GCS commands (simulated — empty most of the time)
    GCSMessage poll_command() {
        GCSMessage msg{};
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time_).count();

        // Simulate an RTL command after 120 seconds
        if (!rtl_sent_ && elapsed > 120.0) {
            msg.type = GCSMessageType::RTL_CMD;
            msg.timestamp_ns = std::chrono::duration_cast<
                std::chrono::nanoseconds>(now.time_since_epoch()).count();
            msg.valid = true;
            rtl_sent_ = true;
            spdlog::info("[GCSLink] Simulated RTL command from GCS");
        }
        return msg;
    }

private:
    bool connected_{false};
    bool rtl_sent_{false};
    uint64_t telem_count_{0};
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace drone::comms
