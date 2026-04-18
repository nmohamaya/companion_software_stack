// process3_slam_vio_nav/include/slam/ivio_interface.h
// Pure interface for VIO backends — extracted to break circular includes
// between ivio_backend.h (SimulatedVIOBackend + factory) and
// swvio_backend.h (SlidingWindowVIOBackend).
//
// All VIO backends inherit from IVIOBackend. The factory function
// create_vio_backend() lives in ivio_backend.h.
#pragma once

#include "ipc/ipc_types.h"
#include "slam/types.h"
#include "slam/vio_types.h"

#include <memory>
#include <string>
#include <vector>

namespace drone::slam {

class IVIOBackend {
public:
    virtual ~IVIOBackend() = default;

    /// Process one stereo frame + buffered IMU data and produce a VIO output.
    ///
    /// @param frame        The stereo frame from the camera.
    /// @param imu_samples  IMU readings accumulated since the last call.
    /// @return Ok(VIOOutput) on success, Err(VIOError) on critical failure.
    [[nodiscard]] virtual VIOResult<VIOOutput> process_frame(
        const drone::ipc::StereoFrame& frame, const std::vector<ImuSample>& imu_samples) = 0;

    /// Current pipeline health.
    [[nodiscard]] virtual VIOHealth health() const = 0;

    /// Human-readable name for logging.
    [[nodiscard]] virtual std::string name() const = 0;

    /// Set trajectory target for simulated navigation.
    /// Only meaningful for simulated backends — real/Gazebo backends get pose
    /// from actual sensors and ignore this. Default implementation is a no-op.
    virtual void set_trajectory_target(float /*x*/, float /*y*/, float /*z*/, float /*yaw*/) {}
};

}  // namespace drone::slam
