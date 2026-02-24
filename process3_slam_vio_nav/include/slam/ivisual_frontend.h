// process3_slam_vio_nav/include/slam/ivisual_frontend.h
// Strategy interface for visual odometry frontends.
//
// The visual frontend extracts features from stereo frames, matches them
// across time, and produces a relative pose estimate.  Swapping the
// implementation lets us move from simulation to ORB-SLAM3, VINS-Mono,
// MSCKF, etc. without touching the process orchestration code.
#pragma once

#include "slam/types.h"
#include "ipc/shm_types.h"
#include <string>
#include <memory>
#include <random>
#include <cmath>
#include <stdexcept>

namespace drone::slam {

/// Abstract visual frontend — processes stereo frames and outputs poses.
class IVisualFrontend {
public:
    virtual ~IVisualFrontend() = default;

    /// Process one stereo frame pair and return the estimated pose.
    /// @param frame  The stereo image pair from SHM.
    /// @return The estimated 6-DOF pose.
    virtual Pose process_frame(const drone::ipc::ShmStereoFrame& frame) = 0;

    /// Human-readable name for logging.
    virtual std::string name() const = 0;
};

// ─────────────────────────────────────────────────────────────
// Simulated frontend — generates a circular trajectory with noise.
// This is the current behaviour extracted into a strategy object.
// ─────────────────────────────────────────────────────────────

class SimulatedVisualFrontend final : public IVisualFrontend {
public:
    Pose process_frame(const drone::ipc::ShmStereoFrame& /*frame*/) override {
        t_ += dt_;

        Pose p;
        p.timestamp = t_;
        p.position = Eigen::Vector3d(
            5.0 * std::cos(t_ * 0.5) + noise_(rng_),
            5.0 * std::sin(t_ * 0.5) + noise_(rng_),
            2.0 + 0.1 * std::sin(t_) + noise_(rng_));
        double yaw = t_ * 0.5 + M_PI / 2.0;
        p.orientation = Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        p.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        p.quality = 2;  // good
        return p;
    }

    std::string name() const override { return "SimulatedVisualFrontend"; }

private:
    double t_ = 0.0;
    double dt_ = 0.033;  // 30 Hz
    std::mt19937 rng_{42};
    std::normal_distribution<double> noise_{0.0, 0.01};
};

/// Factory — creates the appropriate frontend based on config.
inline std::unique_ptr<IVisualFrontend> create_visual_frontend(
    const std::string& backend = "simulated")
{
    if (backend == "simulated") {
        return std::make_unique<SimulatedVisualFrontend>();
    }
    throw std::runtime_error("Unknown visual frontend: " + backend);
}

}  // namespace drone::slam
