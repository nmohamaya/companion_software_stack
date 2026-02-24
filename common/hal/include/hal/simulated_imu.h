// common/hal/include/hal/simulated_imu.h
// Simulated IMU backend: generates noisy synthetic IMU readings.
#pragma once
#include "hal/iimu_source.h"
#include <chrono>
#include <random>
#include <spdlog/spdlog.h>

namespace drone::hal {

class SimulatedIMU : public IIMUSource {
public:
    bool init(int rate_hz) override {
        rate_hz_ = rate_hz;
        active_ = true;
        spdlog::info("[SimulatedIMU] Initialised at {} Hz", rate_hz_);
        return true;
    }

    ImuReading read() override {
        ImuReading r;
        if (!active_) return r;

        r.timestamp = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        r.accel = Eigen::Vector3d(
            accel_noise_(rng_),
            accel_noise_(rng_),
            9.81 + accel_noise_(rng_));
        r.gyro = Eigen::Vector3d(
            gyro_noise_(rng_),
            gyro_noise_(rng_),
            0.5 + gyro_noise_(rng_));
        r.valid = true;

        return r;
    }

    bool is_active() const override { return active_; }

    std::string name() const override { return "SimulatedIMU"; }

private:
    int rate_hz_{400};
    bool active_{false};
    std::mt19937 rng_{99};
    std::normal_distribution<double> accel_noise_{0.0, 0.05};
    std::normal_distribution<double> gyro_noise_{0.0, 0.002};
};

}  // namespace drone::hal
