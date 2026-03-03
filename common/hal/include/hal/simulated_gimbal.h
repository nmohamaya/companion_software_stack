// common/hal/include/hal/simulated_gimbal.h
// Simulated gimbal backend.
// Mirrors the behaviour of payload/gimbal_controller.h behind the IGimbal interface.
#pragma once
#include "hal/igimbal.h"

#include <algorithm>
#include <chrono>

#include <spdlog/spdlog.h>

namespace drone::hal {

class SimulatedGimbal : public IGimbal {
public:
    bool init() override {
        spdlog::info("[SimulatedGimbal] Initialised");
        initialised_ = true;
        return true;
    }

    bool is_initialised() const override { return initialised_; }

    void set_target(float pitch_deg, float yaw_deg) override {
        target_pitch_ = std::clamp(pitch_deg, -90.0f, 30.0f);
        target_yaw_   = std::clamp(yaw_deg, -180.0f, 180.0f);
    }

    void update(float dt_s) override {
        constexpr float RATE     = 60.0f;  // deg/s max slew rate
        float           max_step = RATE * dt_s;

        float dp = target_pitch_ - state_.pitch;
        state_.pitch += std::clamp(dp, -max_step, max_step);

        float dy = target_yaw_ - state_.yaw;
        state_.yaw += std::clamp(dy, -max_step, max_step);
    }

    GimbalState state() const override { return state_; }

    uint64_t capture_image() override {
        capture_count_++;
        auto now = std::chrono::steady_clock::now();
        auto ts  = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        spdlog::info("[SimulatedGimbal] Image captured #{} pitch={:.1f} yaw={:.1f}", capture_count_,
                     state_.pitch, state_.yaw);
        return ts;
    }

    void start_recording() override {
        recording_ = true;
        spdlog::info("[SimulatedGimbal] Recording started");
    }

    void stop_recording() override {
        recording_ = false;
        spdlog::info("[SimulatedGimbal] Recording stopped");
    }

    bool is_recording() const override { return recording_; }

    std::string name() const override { return "SimulatedGimbal"; }

private:
    bool        initialised_{false};
    bool        recording_{false};
    uint32_t    capture_count_{0};
    float       target_pitch_{0.0f};
    float       target_yaw_{0.0f};
    GimbalState state_{};
};

}  // namespace drone::hal
