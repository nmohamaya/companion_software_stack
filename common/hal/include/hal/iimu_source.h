// common/hal/include/hal/iimu_source.h
// HAL interface: IMU data source for SLAM/VIO.
// Implementations: SimulatedIMU (built-in), RealIMU (future).
#pragma once
#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace drone::hal {

/// A single IMU measurement (accelerometer + gyroscope).
struct ImuReading {
    double          timestamp{0.0};
    Eigen::Vector3d accel{Eigen::Vector3d::Zero()};  // m/s²
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};   // rad/s
    bool            valid{false};
};

/// Abstract IMU source interface.
class IIMUSource {
public:
    virtual ~IIMUSource() = default;

    /// Initialise the IMU at the given sample rate (Hz).
    [[nodiscard]] virtual bool init(int rate_hz) = 0;

    /// Read the latest IMU sample (non-blocking).
    [[nodiscard]] virtual ImuReading read() = 0;

    /// Check whether the IMU is initialised and producing data.
    [[nodiscard]] virtual bool is_active() const = 0;

    /// Human-readable name.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace drone::hal
