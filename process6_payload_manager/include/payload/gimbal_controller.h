// process6_payload_manager/include/payload/gimbal_controller.h
// Simulated gimbal + camera payload controller.

#pragma once
#include "util/ilogger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace drone::payload {

struct GimbalState {
    float pitch{0.0f};  // degrees, -90 (nadir) to +30
    float yaw{0.0f};    // degrees, relative to body frame
    float roll{0.0f};
    bool  stabilised{true};
};

// ── Simulated gimbal controller ──────────────────────────────
class GimbalController {
public:
    [[nodiscard]] bool init() {
        DRONE_LOG_INFO("[Gimbal] Initialised (simulated)");
        initialised_ = true;
        return true;
    }

    [[nodiscard]] bool is_initialised() const { return initialised_; }

    // Set target angles — smooth move simulated
    void set_target(float pitch_deg, float yaw_deg) {
        target_pitch_ = std::clamp(pitch_deg, -90.0f, 30.0f);
        target_yaw_   = std::clamp(yaw_deg, -180.0f, 180.0f);
    }

    // Step the simulated gimbal toward target (call at control rate)
    void update(float dt_s) {
        constexpr float RATE     = 60.0f;  // deg/s max slew rate
        float           max_step = RATE * dt_s;

        float dp = target_pitch_ - state_.pitch;
        state_.pitch += std::clamp(dp, -max_step, max_step);

        float dy = target_yaw_ - state_.yaw;
        state_.yaw += std::clamp(dy, -max_step, max_step);
    }

    [[nodiscard]] const GimbalState& state() const { return state_; }

    // Trigger camera capture (simulated)
    [[nodiscard]] uint64_t capture_image() {
        capture_count_++;
        auto ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();
        DRONE_LOG_INFO("[Gimbal] Image captured #{} pitch={:.1f} yaw={:.1f}", capture_count_,
                       state_.pitch, state_.yaw);
        return static_cast<uint64_t>(ts);
    }

    // Start / stop video recording (simulated)
    void start_recording() {
        recording_ = true;
        DRONE_LOG_INFO("[Gimbal] Recording started");
    }
    void stop_recording() {
        recording_ = false;
        DRONE_LOG_INFO("[Gimbal] Recording stopped");
    }
    [[nodiscard]] bool is_recording() const { return recording_; }

private:
    bool        initialised_{false};
    bool        recording_{false};
    uint32_t    capture_count_{0};
    float       target_pitch_{0.0f};
    float       target_yaw_{0.0f};
    GimbalState state_{};
};

}  // namespace drone::payload
