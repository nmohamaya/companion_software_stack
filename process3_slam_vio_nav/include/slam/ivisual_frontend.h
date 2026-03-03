// process3_slam_vio_nav/include/slam/ivisual_frontend.h
// Strategy interface for visual odometry frontends.
//
// The visual frontend extracts features from stereo frames, matches them
// across time, and produces a relative pose estimate.  Swapping the
// implementation lets us move from simulation to ORB-SLAM3, VINS-Mono,
// MSCKF, etc. without touching the process orchestration code.
#pragma once

#include "ipc/shm_types.h"
#include "slam/types.h"

#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#ifdef HAVE_GAZEBO
#include <atomic>
#include <mutex>

#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <spdlog/spdlog.h>
#endif

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
        p.timestamp   = t_;
        p.position    = Eigen::Vector3d(5.0 * std::cos(t_ * 0.5) + noise_(rng_),
                                        5.0 * std::sin(t_ * 0.5) + noise_(rng_),
                                        2.0 + 0.1 * std::sin(t_) + noise_(rng_));
        double yaw    = t_ * 0.5 + M_PI / 2.0;
        p.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        p.covariance  = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        p.quality     = 2;  // good
        return p;
    }

    std::string name() const override { return "SimulatedVisualFrontend"; }

private:
    double                           t_  = 0.0;
    double                           dt_ = 0.033;  // 30 Hz
    std::mt19937                     rng_{42};
    std::normal_distribution<double> noise_{0.0, 0.01};
};

// ─────────────────────────────────────────────────────────────
// Gazebo ground-truth frontend — reads odometry from gz-transport.
//
// Subscribes to the OdometryPublisher topic (default:
// /model/<model>/odometry) and returns the ground-truth pose.
// ─────────────────────────────────────────────────────────────
#ifdef HAVE_GAZEBO

class GazeboVisualFrontend final : public IVisualFrontend {
public:
    /// @param gz_topic  Gazebo odometry topic
    ///                  (e.g. "/model/x500_companion_0/odometry")
    explicit GazeboVisualFrontend(std::string gz_topic) : gz_topic_(std::move(gz_topic)) {
        bool ok = node_.Subscribe(gz_topic_, &GazeboVisualFrontend::on_odom, this);
        if (!ok) {
            spdlog::error("[GazeboVIO] Failed to subscribe to '{}'", gz_topic_);
        } else {
            spdlog::info("[GazeboVIO] Subscribed to '{}'", gz_topic_);
        }
    }

    ~GazeboVisualFrontend() override { node_.Unsubscribe(gz_topic_); }

    // Non-copyable, non-movable (owns gz::transport::Node)
    GazeboVisualFrontend(const GazeboVisualFrontend&)            = delete;
    GazeboVisualFrontend& operator=(const GazeboVisualFrontend&) = delete;

    Pose process_frame(const drone::ipc::ShmStereoFrame& /*frame*/) override {
        std::lock_guard<std::mutex> lk(mtx_);
        return cached_;
    }

    std::string name() const override { return "GazeboVisualFrontend(" + gz_topic_ + ")"; }

private:
    void on_odom(const gz::msgs::Odometry& msg) {
        Pose p;
        // Use monotonic counter as timestamp (same units as SimulatedFrontend)
        p.timestamp = count_.fetch_add(1, std::memory_order_relaxed) * 0.01;

        if (msg.has_pose()) {
            const auto& pose = msg.pose();
            if (pose.has_position()) {
                // Gazebo world frame: X=East, Y=North, Z=Up
                // Our internal frame: X=North, Y=East, Z=Up
                // (matches MavlinkFCLink which sends vx→north, vy→east)
                p.position = Eigen::Vector3d(pose.position().y(),   // our North = Gazebo Y
                                             pose.position().x(),   // our East  = Gazebo X
                                             pose.position().z());  // Up = Up
            }
            if (pose.has_orientation()) {
                p.orientation = Eigen::Quaterniond(pose.orientation().w(), pose.orientation().x(),
                                                   pose.orientation().y(), pose.orientation().z());
            }
        }
        p.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
        p.quality    = 3;  // excellent (ground truth)

        std::lock_guard<std::mutex> lk(mtx_);
        cached_ = p;
    }

    std::string           gz_topic_;
    gz::transport::Node   node_;
    mutable std::mutex    mtx_;
    Pose                  cached_;
    std::atomic<uint64_t> count_{0};
};

#endif  // HAVE_GAZEBO

/// Factory — creates the appropriate frontend based on config.
inline std::unique_ptr<IVisualFrontend> create_visual_frontend(
    const std::string&                  backend  = "simulated",
    [[maybe_unused]] const std::string& gz_topic = "/model/x500_companion_0/odometry") {
    if (backend == "simulated") {
        return std::make_unique<SimulatedVisualFrontend>();
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        return std::make_unique<GazeboVisualFrontend>(gz_topic);
    }
#endif
    throw std::runtime_error("Unknown visual frontend: " + backend);
}

}  // namespace drone::slam
