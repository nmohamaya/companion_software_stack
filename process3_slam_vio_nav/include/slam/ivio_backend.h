// process3_slam_vio_nav/include/slam/ivio_backend.h
// Strategy interface for the VIO backend — the top-level pipeline that
// fuses visual features with IMU pre-integration to produce poses.
//
// The backend receives:
//   1. A stereo frame (from the camera thread)
//   2. Buffered IMU samples since the last frame
// and produces a VIOOutput (pose + health + diagnostics).
//
// Error handling:
//   - Returns VIOResult<VIOOutput> with structured diagnostics
//   - Each sub-component (feature extractor, stereo matcher, pre-integrator)
//     reports its own diagnostics which are merged into FrameDiagnostics
//   - Health state transitions (INITIALIZING→NOMINAL→DEGRADED→LOST)
//     are logged with the reason for the transition
#pragma once

#include "ipc/ipc_types.h"
#include "slam/ifeature_extractor.h"
#include "slam/imu_preintegrator.h"
#include "slam/istereo_matcher.h"
#include "slam/ivio_interface.h"
#include "slam/types.h"
#include "slam/vio_types.h"
#include "util/diagnostic.h"
#include "util/ilogger.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

// ── Gazebo ground-truth headers (only when building with Gazebo support) ──
#ifdef HAVE_GAZEBO
#include <mutex>

#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#endif  // HAVE_GAZEBO

// Forward-declare CosysRpcClient so the factory signature compiles whether or
// not HAVE_COSYS_AIRSIM is defined. Actual type is used only inside the
// HAVE_COSYS_AIRSIM guarded class.
namespace drone::hal {
class CosysRpcClient;
}

// Pull the full CosysRpcClient + MultirotorRpcLibClient headers BEFORE opening
// our namespace. If the include lands inside `namespace drone::slam { ... }`
// further down, AirSim's transitive `<locale>` inclusion is re-declared as
// `drone::slam::std::codecvt`, which breaks anything that later uses <locale>
// (observed on gcc-13 / libstdc++ 13). Keep this above the namespace block.
#ifdef HAVE_COSYS_AIRSIM
#include "hal/cosys_rpc_client.h"
#endif

namespace drone::slam {

// IVIOBackend interface is defined in slam/ivio_interface.h (extracted to
// break circular includes with swvio_backend.h).

// ─────────────────────────────────────────────────────────────
// Simulated VIO backend
//
// Wires together the simulated sub-components:
//   SimulatedFeatureExtractor → SimulatedStereoMatcher → ImuPreintegrator
//
// Pose generation:
//   - Steers toward the latest trajectory target set via set_trajectory_target().
//   - When no target has been set, hovers at the initial position (0, 0, 0).
//   - Uses first-order dynamics: position moves toward target at configurable
//     speed (sim_speed_mps), clamped to not overshoot.
//   - IMU pre-integration is computed and validated but the pose is
//     generated independently (no actual fusion yet — that's Phase 2B)
//   - This lets Tier 1 scenarios validate actual waypoint navigation,
//     mission completion, and RTL logic without Gazebo.
//
// Health transitions:
//   INITIALIZING → NOMINAL after min_init_frames_ frames
//   NOMINAL ↔ DEGRADED based on feature count / match rate thresholds
//   Any → LOST if feature extraction or stereo matching fails hard
// ─────────────────────────────────────────────────────────────

class SimulatedVIOBackend final : public IVIOBackend {
public:
    explicit SimulatedVIOBackend(StereoCalibration calib = {}, ImuNoiseParams imu_params = {},
                                 uint64_t min_init_frames = 5, float sim_speed_mps = 3.0f,
                                 double good_trace_max = 0.1, double degraded_trace_max = 1.0)
        : calib_(calib)
        , preintegrator_(imu_params)
        , extractor_(std::make_unique<SimulatedFeatureExtractor>())
        , matcher_(std::make_unique<SimulatedStereoMatcher>(calib))
        , min_init_frames_(min_init_frames)
        , sim_speed_mps_(sim_speed_mps)
        , good_trace_max_(good_trace_max)
        , degraded_trace_max_(degraded_trace_max) {}

    VIOResult<VIOOutput> process_frame(const drone::ipc::StereoFrame& frame,
                                       const std::vector<ImuSample>&  imu_samples) override {

        auto                          pipeline_start = std::chrono::steady_clock::now();
        const uint64_t                seq            = frame.sequence_number;
        drone::util::FrameDiagnostics diag(seq);

        VIOOutput output;
        output.frame_id = seq;

        // ── 1. Feature extraction ───────────────────────────
        auto feat_result = extractor_->extract(frame, diag);
        if (feat_result.is_err()) {
            transition_health(VIOHealth::LOST,
                              "Feature extraction failed: " + feat_result.error().message);
            output.health = health_;
            diag.log_summary("VIO");
            return VIOResult<VIOOutput>::err(std::move(feat_result.error()));
        }

        auto& feat          = feat_result.value();
        output.num_features = static_cast<int>(feat.features.size());

        // ── 2. Stereo matching ──────────────────────────────
        auto match_result = matcher_->match(frame, feat.features, diag);
        if (match_result.is_err()) {
            transition_health(VIOHealth::DEGRADED,
                              "Stereo matching failed: " + match_result.error().message);
            // Not fatal — we can still produce a pose from IMU only
            output.num_stereo_matches = 0;
            diag.add_warning("VIOBackend", "Proceeding without stereo matches");
        } else {
            output.num_stereo_matches = static_cast<int>(match_result.value().matches.size());
        }

        // ── 3. IMU pre-integration ──────────────────────────
        preintegrator_.reset();
        for (const auto& s : imu_samples) {
            preintegrator_.add_sample(s);
        }
        output.imu_samples_used = static_cast<int>(imu_samples.size());

        if (preintegrator_.has_samples()) {
            auto preint_result = preintegrator_.integrate(diag);
            if (preint_result.is_err()) {
                diag.add_warning("VIOBackend",
                                 "IMU pre-integration failed: " + preint_result.error().message);
                // Not fatal for simulated — we generate pose independently
            } else {
                auto& pm = preint_result.value();
                diag.add_metric("VIOBackend", "preint_dt", pm.dt);
                diag.add_metric("VIOBackend", "preint_delta_pos", pm.delta_position.norm());
                // Extract position covariance trace for quality assessment
                output.position_trace = pm.covariance.block<3, 3>(0, 0).trace();
                diag.add_metric("VIOBackend", "position_trace", output.position_trace);
            }
        } else {
            diag.add_metric("VIOBackend", "imu_samples", 0);
        }

        // ── 4. Generate pose (simulated trajectory) ─────────
        // In Phase 2B this will be replaced by actual VIO fusion
        output.pose = generate_simulated_pose(seq);

        // ── 5. Health state machine ─────────────────────────
        ++frame_count_;
        update_health(output, diag);
        output.health = health_;

        // ── 6. Pipeline timing ──────────────────────────────
        auto pipeline_end = std::chrono::steady_clock::now();
        output.total_ms =
            std::chrono::duration<double, std::milli>(pipeline_end - pipeline_start).count();
        diag.add_timing("VIOBackend", output.total_ms);

        // ── 7. Log diagnostics (every N frames or on warnings/errors)
        if (diag.has_errors() || diag.has_warnings() || frame_count_ % 300 == 0) {
            diag.log_summary("VIO");
        }

        return VIOResult<VIOOutput>::ok(std::move(output));
    }

    [[nodiscard]] VIOHealth health() const override { return health_; }

    [[nodiscard]] std::string name() const override { return "SimulatedVIOBackend"; }

    void set_trajectory_target(float x, float y, float z, float yaw) override {
        target_x_.store(x, std::memory_order_release);
        target_y_.store(y, std::memory_order_release);
        target_z_.store(z, std::memory_order_release);
        target_yaw_.store(yaw, std::memory_order_release);
        has_target_.store(true, std::memory_order_release);
    }

private:
    // ── Simulated pose generation (target-following dynamics) ──
    Pose generate_simulated_pose(uint64_t /*seq*/) {
        constexpr double dt = 0.033;  // ~30Hz frame rate

        // Steer toward target waypoint (set by P4 trajectory commands via P3 main)
        if (has_target_.load(std::memory_order_acquire)) {
            Eigen::Vector3d target(target_x_.load(std::memory_order_acquire),
                                   target_y_.load(std::memory_order_acquire),
                                   target_z_.load(std::memory_order_acquire));

            Eigen::Vector3d dir  = target - current_pos_;
            double          dist = dir.norm();
            if (dist > 0.01) {
                double step = std::min(static_cast<double>(sim_speed_mps_) * dt, dist);
                current_pos_ += dir.normalized() * step;
            }
            current_yaw_ = static_cast<double>(target_yaw_.load(std::memory_order_acquire));
        }
        // If no target set yet, stay at current_pos_ (initially 0,0,0)

        Pose p;
        // Use steady_clock so the timestamp is in the same epoch as
        // FaultManager's now_ns — prevents false "pose stale" faults.
        p.timestamp =
            std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
                .count();
        p.position = current_pos_ + Eigen::Vector3d(noise_(rng_), noise_(rng_), noise_(rng_));
        p.orientation =
            Eigen::Quaterniond(Eigen::AngleAxisd(current_yaw_, Eigen::Vector3d::UnitZ()));
        p.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        // Map VIOHealth → Pose quality (higher = better)
        switch (health_) {
            case VIOHealth::LOST: p.quality = 0; break;          // lost
            case VIOHealth::INITIALIZING: p.quality = 1; break;  // degraded
            case VIOHealth::DEGRADED: p.quality = 1; break;      // degraded
            case VIOHealth::NOMINAL: [[fallthrough]];
            default: p.quality = 2; break;  // good
        }
        return p;
    }

    // ── Health state machine ─────────────────────────────────
    void update_health(const VIOOutput& output, drone::util::FrameDiagnostics& diag) {
        VIOHealth new_health = health_;

        if (frame_count_ < min_init_frames_) {
            new_health = VIOHealth::INITIALIZING;
        } else if (output.position_trace >= 0.0) {
            // Covariance-based quality (preferred when IMU data available)
            if (output.position_trace <= good_trace_max_) {
                new_health = VIOHealth::NOMINAL;
            } else if (output.position_trace <= degraded_trace_max_) {
                new_health = VIOHealth::DEGRADED;
                diag.add_warning("VIOBackend", "Degraded: position trace " +
                                                   std::to_string(output.position_trace) +
                                                   " > good_max " +
                                                   std::to_string(good_trace_max_));
            } else {
                new_health = VIOHealth::LOST;
                diag.add_warning("VIOBackend",
                                 "Lost: position trace " + std::to_string(output.position_trace) +
                                     " > degraded_max " + std::to_string(degraded_trace_max_));
            }
        } else {
            // Fallback: feature/match-based heuristic (no IMU data)
            if (output.num_features < kMinFeaturesNominal) {
                new_health = VIOHealth::DEGRADED;
                diag.add_warning("VIOBackend", "Degraded: only " +
                                                   std::to_string(output.num_features) +
                                                   " features (need " +
                                                   std::to_string(kMinFeaturesNominal) + ")");
            } else if (output.num_stereo_matches < kMinMatchesNominal && output.num_features > 0) {
                new_health = VIOHealth::DEGRADED;
            } else {
                new_health = VIOHealth::NOMINAL;
            }
        }

        transition_health(new_health, "");
    }

    void transition_health(VIOHealth new_health, const std::string& reason) {
        if (new_health != health_) {
            DRONE_LOG_INFO("[VIO] Health: {} → {}{}", vio_health_name(health_),
                           vio_health_name(new_health), reason.empty() ? "" : " (" + reason + ")");
            health_ = new_health;
        }
    }

    // ── Configuration / thresholds ──────────────────────────
    StereoCalibration                  calib_;
    ImuPreintegrator                   preintegrator_;
    std::unique_ptr<IFeatureExtractor> extractor_;
    std::unique_ptr<IStereoMatcher>    matcher_;
    uint64_t                           min_init_frames_;

    // ── Runtime state ───────────────────────────────────────
    VIOHealth                        health_      = VIOHealth::INITIALIZING;
    uint64_t                         frame_count_ = 0;
    std::mt19937                     rng_{42};
    std::normal_distribution<double> noise_{0.0, 0.01};
    float                            sim_speed_mps_;

    // ── Covariance quality thresholds ───────────────────────
    double good_trace_max_     = 0.1;  // trace(P_pos) <= this → NOMINAL
    double degraded_trace_max_ = 1.0;  // trace(P_pos) <= this → DEGRADED, else LOST

    // ── Target-following state (set from trajectory commands) ──
    std::atomic<float> target_x_{0.0f};
    std::atomic<float> target_y_{0.0f};
    std::atomic<float> target_z_{0.0f};
    std::atomic<float> target_yaw_{0.0f};
    std::atomic<bool>  has_target_{false};
    Eigen::Vector3d    current_pos_ = Eigen::Vector3d::Zero();
    double             current_yaw_ = 0.0;

    // ── Fallback thresholds (used when no IMU data) ─────────
    static constexpr int kMinFeaturesNominal = 30;
    static constexpr int kMinMatchesNominal  = 15;
};

// ─────────────────────────────────────────────────────────────
// Gazebo ground-truth VIO backend
//
// Reads the drone's exact position from the Gazebo odometry topic via
// gz-transport and wraps it as a VIOBackend.  The stereo frame and IMU
// samples are intentionally ignored — Gazebo provides perfect ground
// truth so no vision/IMU fusion is needed.
//
// MODULARITY NOTE: This class is entirely gated behind HAVE_GAZEBO.
// It is registered under the name "gazebo" in create_vio_backend().
// To switch back to simulated (or a real VIO algorithm), change
// slam.vio.backend in the config.  No code changes are required.
//
// Frame convention:
//   Gazebo world: X=East, Y=North, Z=Up
//   Our internal:  X=North, Y=East, Z=Up
//   (vx→north, vy→east matches MavlinkFCLink's NED conversion)
// ─────────────────────────────────────────────────────────────
#ifdef HAVE_GAZEBO

class GazeboVIOBackend final : public IVIOBackend {
public:
    /// @param gz_topic  Gazebo odometry topic, e.g.
    ///                  "/model/x500_companion_0/odometry"
    explicit GazeboVIOBackend(std::string gz_topic)
        : gz_topic_(std::move(gz_topic)), health_(VIOHealth::INITIALIZING) {
        bool ok = node_.Subscribe(gz_topic_, &GazeboVIOBackend::on_odom, this);
        if (!ok) {
            DRONE_LOG_ERROR("[GazeboVIOBackend] Failed to subscribe to '{}'", gz_topic_);
        } else {
            DRONE_LOG_INFO("[GazeboVIOBackend] Subscribed to ground-truth odometry: '{}'",
                           gz_topic_);
        }
    }

    ~GazeboVIOBackend() override { node_.Unsubscribe(gz_topic_); }

    // Non-copyable, non-movable (owns gz::transport::Node)
    GazeboVIOBackend(const GazeboVIOBackend&)            = delete;
    GazeboVIOBackend& operator=(const GazeboVIOBackend&) = delete;

    VIOResult<VIOOutput> process_frame(const drone::ipc::StereoFrame& frame,
                                       const std::vector<ImuSample>& /*imu_samples*/) override {
        VIOOutput output;
        output.frame_id = frame.sequence_number;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!pose_valid_) {
                // No odometry received yet — return degraded but not fatal
                output.health = VIOHealth::INITIALIZING;
                return VIOResult<VIOOutput>::ok(std::move(output));
            }
            output.pose = cached_pose_;
        }

        output.health             = health_.load(std::memory_order_relaxed);
        output.num_features       = -1;  // N/A — ground truth, no feature extraction
        output.num_stereo_matches = -1;
        output.imu_samples_used   = 0;
        return VIOResult<VIOOutput>::ok(std::move(output));
    }

    [[nodiscard]] VIOHealth health() const override {
        return health_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::string name() const override {
        return "GazeboVIOBackend(" + gz_topic_ + ")";
    }

private:
    void on_odom(const gz::msgs::Odometry& msg) {
        Pose p;
        // Use steady_clock so the timestamp is in the same epoch as
        // FaultManager's now_ns — prevents false "pose stale" faults.
        p.timestamp =
            std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
                .count();

        if (msg.has_pose()) {
            const auto& pose = msg.pose();
            if (pose.has_position()) {
                // Remap Gazebo world frame → our internal frame:
                //   our X (North) = Gazebo Y
                //   our Y (East)  = Gazebo X
                //   our Z (Up)    = Gazebo Z
                p.position = Eigen::Vector3d(pose.position().y(),   // North
                                             pose.position().x(),   // East
                                             pose.position().z());  // Up
            }
            if (pose.has_orientation()) {
                Eigen::Quaterniond gz_q(pose.orientation().w(), pose.orientation().x(),
                                        pose.orientation().y(), pose.orientation().z());
                // Gazebo uses ENU: yaw=0 means facing East (+Gz_X = our +Y).
                // Our frame: yaw=0 means facing North (+our_X = +Gz_Y).
                // With position remap (North=Gz_Y, East=Gz_X), the heading
                // transforms as: our_yaw = π/2 - gz_ENU_yaw.
                // For a yaw-only rotation this equals R_z(+π/2) * gz_q.inverse().
                static const Eigen::Quaterniond k_enu_to_our(
                    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
                p.orientation = k_enu_to_our * gz_q.conjugate();
            }
        }
        p.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
        p.quality    = 3;  // excellent — ground truth

        {
            std::lock_guard<std::mutex> lk(mtx_);
            cached_pose_ = p;
            pose_valid_  = true;
        }
        odom_count_.fetch_add(1, std::memory_order_relaxed);
        health_.store(VIOHealth::NOMINAL, std::memory_order_relaxed);
    }

    std::string            gz_topic_;
    gz::transport::Node    node_;
    mutable std::mutex     mtx_;
    Pose                   cached_pose_;
    bool                   pose_valid_{false};
    std::atomic<VIOHealth> health_;
    std::atomic<uint64_t>  odom_count_{0};
};

// ─────────────────────────────────────────────────────────────
// Gazebo full-pipeline VIO backend
//
// Combines the best of both backends:
//   - Runs the full VIO pipeline (feature extraction, stereo matching,
//     IMU pre-integration) on Gazebo-rendered frames, giving code path
//     coverage of the entire VIO stack in Tier 2 simulation.
//   - Uses Gazebo ground-truth odometry for the final pose, since real
//     VIO fusion does not exist yet (Phase 2B).
//
// This backend is registered as "gazebo_full_vio" in the factory.
// Use it when you want to exercise the pipeline code on Gazebo frames
// while still getting a reliable pose for downstream consumers.
// ─────────────────────────────────────────────────────────────

class GazeboFullVIOBackend final : public IVIOBackend {
public:
    explicit GazeboFullVIOBackend(StereoCalibration calib, ImuNoiseParams imu_params,
                                  std::string gz_topic, uint64_t min_init_frames = 5,
                                  double good_trace_max = 0.1, double degraded_trace_max = 1.0)
        : calib_(std::move(calib))
        , preintegrator_(imu_params)
        , extractor_(std::make_unique<SimulatedFeatureExtractor>())
        , matcher_(std::make_unique<SimulatedStereoMatcher>(calib_))
        , gz_topic_(std::move(gz_topic))
        , min_init_frames_(min_init_frames)
        , good_trace_max_(good_trace_max)
        , degraded_trace_max_(degraded_trace_max) {
        bool ok = node_.Subscribe(gz_topic_, &GazeboFullVIOBackend::on_odom, this);
        if (!ok) {
            DRONE_LOG_ERROR("[GazeboFullVIOBackend] Failed to subscribe to '{}'", gz_topic_);
        } else {
            DRONE_LOG_INFO("[GazeboFullVIOBackend] Subscribed to ground-truth odometry: '{}'",
                           gz_topic_);
        }
    }

    ~GazeboFullVIOBackend() override { node_.Unsubscribe(gz_topic_); }

    // Non-copyable, non-movable (owns gz::transport::Node)
    GazeboFullVIOBackend(const GazeboFullVIOBackend&)            = delete;
    GazeboFullVIOBackend& operator=(const GazeboFullVIOBackend&) = delete;

    VIOResult<VIOOutput> process_frame(const drone::ipc::StereoFrame& frame,
                                       const std::vector<ImuSample>&  imu_samples) override {
        auto                          pipeline_start = std::chrono::steady_clock::now();
        const uint64_t                seq            = frame.sequence_number;
        drone::util::FrameDiagnostics diag(seq);

        VIOOutput output;
        output.frame_id = seq;

        // ── 1. Feature extraction ───────────────────────────
        auto feat_result = extractor_->extract(frame, diag);
        if (feat_result.is_err()) {
            transition_health(VIOHealth::LOST,
                              "Feature extraction failed: " + feat_result.error().message);
            output.health = health_.load(std::memory_order_acquire);
            diag.log_summary("VIO");
            return VIOResult<VIOOutput>::err(std::move(feat_result.error()));
        }

        auto& feat          = feat_result.value();
        output.num_features = static_cast<int>(feat.features.size());

        // ── 2. Stereo matching ──────────────────────────────
        auto match_result = matcher_->match(frame, feat.features, diag);
        if (match_result.is_err()) {
            transition_health(VIOHealth::DEGRADED,
                              "Stereo matching failed: " + match_result.error().message);
            output.num_stereo_matches = 0;
            diag.add_warning("VIOBackend", "Proceeding without stereo matches");
        } else {
            output.num_stereo_matches = static_cast<int>(match_result.value().matches.size());
        }

        // ── 3. IMU pre-integration ──────────────────────────
        preintegrator_.reset();
        for (const auto& s : imu_samples) {
            preintegrator_.add_sample(s);
        }
        output.imu_samples_used = static_cast<int>(imu_samples.size());

        if (preintegrator_.has_samples()) {
            auto preint_result = preintegrator_.integrate(diag);
            if (preint_result.is_err()) {
                diag.add_warning("VIOBackend",
                                 "IMU pre-integration failed: " + preint_result.error().message);
            } else {
                auto& pm = preint_result.value();
                diag.add_metric("VIOBackend", "preint_dt", pm.dt);
                diag.add_metric("VIOBackend", "preint_delta_pos", pm.delta_position.norm());
                // Extract position covariance trace for quality assessment
                output.position_trace = pm.covariance.block<3, 3>(0, 0).trace();
                diag.add_metric("VIOBackend", "position_trace", output.position_trace);
            }
        } else {
            diag.add_metric("VIOBackend", "imu_samples", 0);
        }

        // ── 4. Get pose from Gazebo ground truth ────────────
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!pose_valid_) {
                output.health = VIOHealth::INITIALIZING;
                ++frame_count_;
                auto pipeline_end = std::chrono::steady_clock::now();
                output.total_ms =
                    std::chrono::duration<double, std::milli>(pipeline_end - pipeline_start).count();
                return VIOResult<VIOOutput>::ok(std::move(output));
            }
            output.pose = cached_pose_;
        }

        // ── 5. Health state machine ─────────────────────────
        ++frame_count_;
        update_health(output, diag);
        output.health = health_.load(std::memory_order_acquire);

        // ── 6. Pipeline timing ──────────────────────────────
        auto pipeline_end = std::chrono::steady_clock::now();
        output.total_ms =
            std::chrono::duration<double, std::milli>(pipeline_end - pipeline_start).count();
        diag.add_timing("VIOBackend", output.total_ms);

        // ── 7. Log diagnostics periodically ─────────────────
        if (diag.has_errors() || diag.has_warnings() || frame_count_ % 300 == 0) {
            diag.log_summary("VIO");
        }

        return VIOResult<VIOOutput>::ok(std::move(output));
    }

    [[nodiscard]] VIOHealth health() const override {
        return health_.load(std::memory_order_acquire);
    }

    [[nodiscard]] std::string name() const override {
        return "GazeboFullVIOBackend(" + gz_topic_ + ")";
    }

private:
    void on_odom(const gz::msgs::Odometry& msg) {
        Pose p;
        p.timestamp =
            std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
                .count();

        if (msg.has_pose()) {
            const auto& pose = msg.pose();
            if (pose.has_position()) {
                // Remap Gazebo world frame → our internal frame:
                //   our X (North) = Gazebo Y
                //   our Y (East)  = Gazebo X
                //   our Z (Up)    = Gazebo Z
                p.position = Eigen::Vector3d(pose.position().y(),   // North
                                             pose.position().x(),   // East
                                             pose.position().z());  // Up
            }
            if (pose.has_orientation()) {
                Eigen::Quaterniond              gz_q(pose.orientation().w(), pose.orientation().x(),
                                                     pose.orientation().y(), pose.orientation().z());
                static const Eigen::Quaterniond k_enu_to_our(
                    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
                p.orientation = k_enu_to_our * gz_q.conjugate();
            }
        }
        p.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
        p.quality    = 3;  // excellent — ground truth

        {
            std::lock_guard<std::mutex> lk(mtx_);
            cached_pose_ = p;
            pose_valid_  = true;
        }
        // Health is driven by the pipeline state machine in update_health(),
        // not by odometry arrival. Don't set health_ here to avoid racing
        // with process_frame() and overwriting pipeline-driven transitions.
    }

    void update_health(const VIOOutput& output, drone::util::FrameDiagnostics& diag) {
        VIOHealth new_health = health_.load(std::memory_order_acquire);

        if (frame_count_ < min_init_frames_) {
            new_health = VIOHealth::INITIALIZING;
        } else if (output.position_trace >= 0.0) {
            // Covariance-based quality (preferred when IMU data available)
            if (output.position_trace <= good_trace_max_) {
                new_health = VIOHealth::NOMINAL;
            } else if (output.position_trace <= degraded_trace_max_) {
                new_health = VIOHealth::DEGRADED;
                diag.add_warning("VIOBackend", "Degraded: position trace " +
                                                   std::to_string(output.position_trace) +
                                                   " > good_max " +
                                                   std::to_string(good_trace_max_));
            } else {
                new_health = VIOHealth::LOST;
                diag.add_warning("VIOBackend",
                                 "Lost: position trace " + std::to_string(output.position_trace) +
                                     " > degraded_max " + std::to_string(degraded_trace_max_));
            }
        } else {
            // Fallback: feature/match-based heuristic (no IMU data)
            if (output.num_features < kMinFeaturesNominal) {
                new_health = VIOHealth::DEGRADED;
                diag.add_warning("VIOBackend", "Degraded: only " +
                                                   std::to_string(output.num_features) +
                                                   " features (need " +
                                                   std::to_string(kMinFeaturesNominal) + ")");
            } else if (output.num_stereo_matches < kMinMatchesNominal && output.num_features > 0) {
                new_health = VIOHealth::DEGRADED;
            } else {
                new_health = VIOHealth::NOMINAL;
            }
        }

        transition_health(new_health, "");
    }

    void transition_health(VIOHealth new_health, const std::string& reason) {
        VIOHealth current = health_.load(std::memory_order_acquire);
        if (new_health != current) {
            DRONE_LOG_INFO("[VIO] Health: {} → {}{}", vio_health_name(current),
                           vio_health_name(new_health), reason.empty() ? "" : " (" + reason + ")");
            health_.store(new_health, std::memory_order_release);
        }
    }

    // ── Pipeline components ─────────────────────────────────
    StereoCalibration                  calib_;
    ImuPreintegrator                   preintegrator_;
    std::unique_ptr<IFeatureExtractor> extractor_;
    std::unique_ptr<IStereoMatcher>    matcher_;

    // ── Gazebo ground-truth state ───────────────────────────
    std::string         gz_topic_;
    gz::transport::Node node_;
    mutable std::mutex  mtx_;
    Pose                cached_pose_;
    bool                pose_valid_{false};

    // ── Health state machine ────────────────────────────────
    std::atomic<VIOHealth> health_{VIOHealth::INITIALIZING};
    uint64_t               frame_count_ = 0;
    uint64_t               min_init_frames_;

    // ── Covariance quality thresholds ───────────────────────
    double good_trace_max_     = 0.1;
    double degraded_trace_max_ = 1.0;

    // ── Fallback thresholds (used when no IMU data) ─────────
    static constexpr int kMinFeaturesNominal = 30;
    static constexpr int kMinMatchesNominal  = 15;
};

#endif  // HAVE_GAZEBO

// ─────────────────────────────────────────────────────────────
// Cosys-AirSim ground-truth VIO backend
//
// Polls Cosys-AirSim's simGetGroundTruthKinematics() at ~50 Hz and exposes
// the pose to the VIO pipeline. Analogous to GazeboVIOBackend but uses RPC
// instead of gz-transport. Ground truth — perfect pose, no feature extraction.
//
// Frame convention:
//   AirSim NED: X=North, Y=East, Z=Down
//   Our internal: X=North, Y=East, Z=Up
//   → X passes through, Y passes through, Z is negated
// ─────────────────────────────────────────────────────────────
#ifdef HAVE_COSYS_AIRSIM
// cosys_rpc_client.h is pulled in above the namespace block — see note there.

class CosysVIOBackend final : public IVIOBackend {
public:
    /// @param client        Shared RPC client (manages connection lifecycle)
    /// @param vehicle_name  AirSim vehicle name (e.g. "Drone0", "x500")
    /// @param poll_hz       Pose polling rate (default 50 Hz)
    explicit CosysVIOBackend(std::shared_ptr<drone::hal::CosysRpcClient> client,
                             std::string vehicle_name, int poll_hz = 50)
        : client_(std::move(client))
        , vehicle_name_(std::move(vehicle_name))
        , poll_interval_(std::chrono::milliseconds(std::max(1, 1000 / std::max(1, poll_hz))))
        , health_(VIOHealth::INITIALIZING)
        , active_(true) {
        DRONE_LOG_INFO("[CosysVIOBackend] Started polling ground-truth kinematics for '{}' @ {} Hz",
                       vehicle_name_, poll_hz);
        poll_thread_ = std::thread(&CosysVIOBackend::poll_loop, this);
    }

    ~CosysVIOBackend() override {
        active_.store(false, std::memory_order_release);
        if (poll_thread_.joinable()) poll_thread_.join();
    }

    // Non-copyable, non-movable (owns polling thread)
    CosysVIOBackend(const CosysVIOBackend&)            = delete;
    CosysVIOBackend& operator=(const CosysVIOBackend&) = delete;

    VIOResult<VIOOutput> process_frame(const drone::ipc::StereoFrame& frame,
                                       const std::vector<ImuSample>& /*imu_samples*/) override {
        VIOOutput output;
        output.frame_id = frame.sequence_number;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!pose_valid_) {
                output.health = VIOHealth::INITIALIZING;
                return VIOResult<VIOOutput>::ok(std::move(output));
            }
            output.pose = cached_pose_;
        }

        output.health             = health_.load(std::memory_order_relaxed);
        output.num_features       = -1;  // N/A — ground truth
        output.num_stereo_matches = -1;
        output.imu_samples_used   = 0;
        return VIOResult<VIOOutput>::ok(std::move(output));
    }

    [[nodiscard]] VIOHealth health() const override {
        return health_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::string name() const override {
        return "CosysVIOBackend(" + vehicle_name_ + "@" + client_->endpoint() + ")";
    }

private:
    void poll_loop() {
        while (active_.load(std::memory_order_acquire)) {
            try {
                msr::airlib::Kinematics::State kin{};
                bool                           got = client_->with_client(
                    [&](auto& rpc) { kin = rpc.simGetGroundTruthKinematics(vehicle_name_); });
                if (!got) {
                    std::this_thread::sleep_for(poll_interval_);
                    continue;
                }

                Pose p;
                // Timestamp: steady_clock (same epoch as FaultManager for staleness checks)
                p.timestamp = std::chrono::duration<double>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();

                // AirSim NED → our internal (X=N, Y=E, Z=Up): negate Z
                p.position = Eigen::Vector3d(static_cast<double>(kin.pose.position.x()),
                                             static_cast<double>(kin.pose.position.y()),
                                             -static_cast<double>(kin.pose.position.z()));

                // Quaternion: NED → ENU (negate Y, Z axes via conjugate around X-axis swap)
                // For ground truth the simplest correct form is to negate Z component of
                // the rotation axis, which is equivalent to:
                Eigen::Quaterniond q_ned(kin.pose.orientation.w(), kin.pose.orientation.x(),
                                         kin.pose.orientation.y(), kin.pose.orientation.z());
                // NED → ENU-like (Z flipped): q_enu = R_x(pi) * q_ned * R_x(pi)^-1 simplification:
                // Equivalent to negating y and z components of the quaternion (yaw sign flip + roll flip)
                p.orientation = Eigen::Quaterniond(q_ned.w(), q_ned.x(), -q_ned.y(), -q_ned.z());

                p.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
                p.quality    = 3;  // excellent — ground truth

                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    cached_pose_ = p;
                    pose_valid_  = true;
                }
                odom_count_.fetch_add(1, std::memory_order_relaxed);
                health_.store(VIOHealth::NOMINAL, std::memory_order_relaxed);

            } catch (const std::exception& e) {
                DRONE_LOG_WARN("[CosysVIOBackend] RPC error: {}", e.what());
            }

            std::this_thread::sleep_for(poll_interval_);
        }
    }

    std::shared_ptr<drone::hal::CosysRpcClient> client_;
    std::string                                 vehicle_name_;
    std::chrono::milliseconds                   poll_interval_;
    mutable std::mutex                          mtx_;
    Pose                                        cached_pose_;
    bool                                        pose_valid_{false};
    std::atomic<VIOHealth>                      health_;
    std::atomic<uint64_t>                       odom_count_{0};
    std::atomic<bool>                           active_;
    std::thread                                 poll_thread_;
};

#endif  // HAVE_COSYS_AIRSIM

}  // namespace drone::slam

// ── SWVIO backend (included here so the factory can instantiate it) ──
// Must come after IVIOBackend is fully defined to avoid circular includes.
// Included outside the namespace block because swvio_backend.h opens its own
// drone::slam namespace — nesting would cause double-namespace errors.
#include "slam/swvio_backend.h"

namespace drone::slam {

// ── Factory ────────────────────────────────────────────────

/// Create a VIO backend by name.
///
/// Supported backends:
///   "simulated"       — target-following dynamics (default; works without hardware)
///   "swvio"           — sliding-window VIO with IMU propagation (Phase 1)
///   "gazebo"          — Gazebo ground-truth odometry via gz-transport (HAVE_GAZEBO only)
///   "gazebo_full_vio" — full pipeline on Gazebo frames + ground-truth pose (HAVE_GAZEBO only)
///   "cosys_airsim"    — Cosys-AirSim ground-truth kinematics via RPC (HAVE_COSYS_AIRSIM only)
///
/// To add a real VIO algorithm (e.g. MSCKF), add a new class implementing
/// IVIOBackend and register it here.  The rest of the stack is unchanged.
inline std::unique_ptr<IVIOBackend> create_vio_backend(
    const std::string& backend = "simulated", const StereoCalibration& calib = {},
    const ImuNoiseParams& imu_params = {},
    const std::string& gz_topic = "/model/x500_companion_0/odometry", float sim_speed_mps = 3.0f,
    double good_trace_max = 0.1, double degraded_trace_max = 1.0,
    std::shared_ptr<drone::hal::CosysRpcClient> cosys_client       = nullptr,
    const std::string&                          cosys_vehicle_name = "Drone0") {

    // Validate quality thresholds — good must be <= degraded for the
    // health state machine to make sense (NOMINAL < DEGRADED < LOST).
    if (good_trace_max < 0.0 || degraded_trace_max < 0.0) {
        DRONE_LOG_WARN("[VIOBackend] Negative quality thresholds (good={}, degraded={}) — "
                       "clamping to defaults (0.1, 1.0)",
                       good_trace_max, degraded_trace_max);
        good_trace_max     = 0.1;
        degraded_trace_max = 1.0;
    } else if (good_trace_max > degraded_trace_max) {
        DRONE_LOG_WARN("[VIOBackend] good_trace_max ({}) > degraded_trace_max ({}) — "
                       "swapping to preserve NOMINAL < DEGRADED ordering",
                       good_trace_max, degraded_trace_max);
        std::swap(good_trace_max, degraded_trace_max);
    }

#ifndef HAVE_GAZEBO
    (void)gz_topic;  // only used by Gazebo backend
#endif

    if (backend == "simulated") {
        return std::make_unique<SimulatedVIOBackend>(calib, imu_params, 5, sim_speed_mps,
                                                     good_trace_max, degraded_trace_max);
    }
#ifdef HAVE_GAZEBO
    if (backend == "gazebo") {
        (void)calib;       // ground truth — calibration not used
        (void)imu_params;  // ground truth — IMU params not used
        return std::make_unique<GazeboVIOBackend>(gz_topic);
    }
    if (backend == "gazebo_full_vio") {
        return std::make_unique<GazeboFullVIOBackend>(calib, imu_params, gz_topic, 5,
                                                      good_trace_max, degraded_trace_max);
    }
#endif
#ifdef HAVE_COSYS_AIRSIM
    if (backend == "cosys_airsim") {
        if (!cosys_client) {
            throw std::runtime_error("[VIOBackend] 'cosys_airsim' backend requires a shared "
                                     "CosysRpcClient (pass via cosys_client parameter)");
        }
        (void)calib;       // ground truth — calibration not used
        (void)imu_params;  // ground truth — IMU params not used
        return std::make_unique<CosysVIOBackend>(cosys_client, cosys_vehicle_name);
    }
#else
    (void)cosys_client;
    (void)cosys_vehicle_name;
#endif
    if (backend == "swvio") {
        SWVIOParams swvio_params;
        swvio_params.good_trace_max     = good_trace_max;
        swvio_params.degraded_trace_max = degraded_trace_max;
        return std::make_unique<SlidingWindowVIOBackend>(calib, imu_params, swvio_params);
    }
    throw std::runtime_error("[VIOBackend] Unknown backend: '" + backend +
                             "' (available: simulated, swvio"
#ifdef HAVE_GAZEBO
                             ", gazebo, gazebo_full_vio"
#endif
#ifdef HAVE_COSYS_AIRSIM
                             ", cosys_airsim"
#endif
                             ")");
}

}  // namespace drone::slam
