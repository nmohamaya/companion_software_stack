// process3_slam_vio_nav/src/swvio_backend.cpp
// Sliding-Window Visual-Inertial Odometry backend — Phase 1 implementation.
//
// Phase 1 provides:
//   - IMU state propagation with error-state covariance
//   - Camera clone augmentation into the sliding window
//   - Oldest-clone marginalization
//   - Health state machine (covariance-based quality)
//
// Phase 2 (future) will add sliding-window Gauss-Newton optimization
// that constrains accumulated IMU drift via multi-view feature updates.

#include "slam/swvio_backend.h"

#include "slam/slam_math.h"
#include "util/diagnostic.h"
#include "util/ilogger.h"

#include <chrono>
#include <cmath>

namespace drone::slam {

// IMU sample validation thresholds
constexpr double   kMaxImuAccelMps2  = 200.0;  // ~20g
constexpr double   kMaxImuGyroRadSec = 35.0;   // ~2000 deg/s
constexpr double   kMaxDtSec         = 0.1;    // no IMU gap > 100ms
constexpr uint64_t kMaxTimestampNs   = 10ULL * 365 * 24 * 3600 * 1'000'000'000ULL;  // ~10 years

// ── Constructor ─────────────────────────────────────────────
SlidingWindowVIOBackend::SlidingWindowVIOBackend(const StereoCalibration& calib,
                                                 const ImuNoiseParams&    imu_params,
                                                 const SWVIOParams&       params)
    : params_(params)
    , calib_(calib)
    , imu_params_(imu_params)
    , extractor_(std::make_unique<SimulatedFeatureExtractor>())
    , matcher_(std::make_unique<SimulatedStereoMatcher>(calib)) {

    // Initialize IMU state at origin with identity orientation
    state_.imu.orientation = Eigen::Quaterniond::Identity();
    state_.imu.position    = Eigen::Vector3d::Zero();
    state_.imu.velocity    = Eigen::Vector3d::Zero();
    state_.imu.gyro_bias   = Eigen::Vector3d::Zero();
    state_.imu.accel_bias  = Eigen::Vector3d::Zero();
    state_.imu.timestamp   = 0.0;

    // Initialize covariance: 15x15 for IMU error state only (no clones yet)
    constexpr int kDim = SWVIOState::kImuStateDim;
    state_.covariance  = Eigen::MatrixXd::Zero(kDim, kDim);

    state_.covariance.block<3, 3>(SWVIOState::kIdxTheta,
                                  SWVIOState::kIdxTheta) = Eigen::Matrix3d::Identity() * 1e-3;
    state_.covariance.block<3, 3>(SWVIOState::kIdxPos,
                                  SWVIOState::kIdxPos)   = Eigen::Matrix3d::Identity() * 1e-2;
    state_.covariance.block<3, 3>(SWVIOState::kIdxVel,
                                  SWVIOState::kIdxVel)   = Eigen::Matrix3d::Identity() * 1e-2;
    state_.covariance.block<3, 3>(SWVIOState::kIdxBg,
                                  SWVIOState::kIdxBg)    = Eigen::Matrix3d::Identity() * 1e-4;
    state_.covariance.block<3, 3>(SWVIOState::kIdxBa,
                                  SWVIOState::kIdxBa)    = Eigen::Matrix3d::Identity() * 1e-4;

    // Precompute continuous-time noise covariance Q_c (constant across lifetime)
    const double sg2  = imu_params_.gyro_noise_density * imu_params_.gyro_noise_density;
    const double sa2  = imu_params_.accel_noise_density * imu_params_.accel_noise_density;
    const double sbg2 = imu_params_.gyro_random_walk * imu_params_.gyro_random_walk;
    const double sba2 = imu_params_.accel_random_walk * imu_params_.accel_random_walk;

    Q_c_                   = Eigen::Matrix<double, 12, 12>::Zero();
    Q_c_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sg2;
    Q_c_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sa2;
    Q_c_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * sbg2;
    Q_c_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * sba2;

    // Pre-allocate scratch covariance buffer — +1 because augmentation
    // temporarily grows the window to max_clones+1 before marginalization.
    const int max_dim = SWVIOState::kImuStateDim +
                        SWVIOState::kCloneStateDim * (params_.max_clones + 1);
    scratch_cov_ = Eigen::MatrixXd::Zero(max_dim, max_dim);
}

// ── IMU state propagation ───────────────────────────────────
// Propagates the nominal state and error-state covariance through
// a sequence of IMU samples using trapezoidal (midpoint) integration.
void SlidingWindowVIOBackend::propagate_imu(const std::vector<ImuSample>& imu_samples) {
    if (imu_samples.size() < 2) {
        return;
    }

    for (size_t i = 1; i < imu_samples.size(); ++i) {
        const double dt = imu_samples[i].timestamp - imu_samples[i - 1].timestamp;
        if (dt <= 0.0) {
            continue;
        }

        // Validate sample: reject NaN/Inf, out-of-range, or large time gaps
        const auto& accel = imu_samples[i].accel;
        const auto& gyro  = imu_samples[i].gyro;
        if (!std::isfinite(accel.x()) || !std::isfinite(accel.y()) || !std::isfinite(accel.z()) ||
            !std::isfinite(gyro.x()) || !std::isfinite(gyro.y()) || !std::isfinite(gyro.z()) ||
            accel.norm() > kMaxImuAccelMps2 || gyro.norm() > kMaxImuGyroRadSec || dt > kMaxDtSec) {
            DRONE_LOG_WARN("[SWVIO] IMU sample {} rejected: accel={:.1f} gyro={:.1f} dt={:.4f}", i,
                           accel.norm(), gyro.norm(), dt);
            continue;
        }

        // Bias-corrected IMU readings (midpoint for trapezoidal integration)
        const Eigen::Vector3d gyro_mid = 0.5 * (imu_samples[i - 1].gyro + imu_samples[i].gyro) -
                                         state_.imu.gyro_bias;
        const Eigen::Vector3d accel_mid = 0.5 * (imu_samples[i - 1].accel + imu_samples[i].accel) -
                                          state_.imu.accel_bias;

        // Current rotation matrix (world <- body)
        const Eigen::Matrix3d R = state_.imu.orientation.toRotationMatrix();

        // World-frame acceleration (rotate body accel to world + add gravity)
        const Eigen::Vector3d accel_world = R * accel_mid + params_.gravity;

        // ── Nominal state update ────────────────────────────
        state_.imu.position += state_.imu.velocity * dt + 0.5 * accel_world * dt * dt;
        state_.imu.velocity += accel_world * dt;
        state_.imu.orientation = (state_.imu.orientation * exp_map(gyro_mid * dt)).normalized();
        state_.imu.timestamp   = imu_samples[i].timestamp;

        // ── Error-state transition matrix F (15x15) ─────────
        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();

        // Cache R*skew(accel_mid) — used in both velocity and position blocks
        const Eigen::Matrix3d R_skew_a = R * skew(accel_mid);

        F.block<3, 3>(SWVIOState::kIdxTheta, SWVIOState::kIdxTheta) -= skew(gyro_mid) * dt;
        F.block<3, 3>(SWVIOState::kIdxTheta, SWVIOState::kIdxBg) = -Eigen::Matrix3d::Identity() *
                                                                   dt;
        F.block<3, 3>(SWVIOState::kIdxVel, SWVIOState::kIdxTheta) = -R_skew_a * dt;
        F.block<3, 3>(SWVIOState::kIdxVel, SWVIOState::kIdxBa)    = -R * dt;
        F.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxVel) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxTheta) = -0.5 * R_skew_a * dt * dt;
        F.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxBa)    = -0.5 * R * dt * dt;

        // ── Noise Jacobian G (15x12) ────────────────────────
        Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();

        G.block<3, 3>(SWVIOState::kIdxTheta, 0) = -Eigen::Matrix3d::Identity() * dt;
        G.block<3, 3>(SWVIOState::kIdxVel, 3)   = -R * dt;
        G.block<3, 3>(SWVIOState::kIdxPos, 3)   = -0.5 * R * dt * dt;
        G.block<3, 3>(SWVIOState::kIdxBg, 6)    = Eigen::Matrix3d::Identity() * dt;
        G.block<3, 3>(SWVIOState::kIdxBa, 9)    = Eigen::Matrix3d::Identity() * dt;

        // ── Covariance propagation (fixed-size for IMU block) ──
        const int n = state_.state_dim();

        // IMU-IMU block: P_ii = F * P_ii * F^T + G * Q_c * G^T
        const Eigen::Matrix<double, 15, 15> P_ii =
            F * state_.covariance.block<15, 15>(0, 0) * F.transpose() + G * Q_c_ * G.transpose();
        state_.covariance.block<15, 15>(0, 0) = P_ii;

        // IMU-clone cross-correlations: P_ij = F * P_ij for each clone j
        if (n > 15) {
            state_.covariance.block(0, 15, 15, n - 15) = F *
                                                         state_.covariance.block(0, 15, 15, n - 15);
            state_.covariance.block(15, 0, n - 15,
                                    15) = state_.covariance.block(0, 15, 15, n - 15).transpose();
        }
    }
}

// ── State augmentation ──────────────────────────────────────
// Clones the current IMU pose (orientation + position) into the
// sliding window and expands the covariance accordingly.
void SlidingWindowVIOBackend::augment_state(double timestamp) {
    CameraClone clone;
    clone.clone_id    = next_clone_id_++;
    clone.timestamp   = timestamp;
    clone.orientation = state_.imu.orientation;
    clone.position    = state_.imu.position;
    state_.clones.push_back(clone);

    // Augmentation Jacobian J (6x15): clone pose = IMU pose
    Eigen::Matrix<double, 6, 15> J          = Eigen::Matrix<double, 6, 15>::Zero();
    J.block<3, 3>(0, SWVIOState::kIdxTheta) = Eigen::Matrix3d::Identity();
    J.block<3, 3>(3, SWVIOState::kIdxPos)   = Eigen::Matrix3d::Identity();

    // Expand covariance from (old_dim) to (old_dim + 6) using pre-allocated scratch
    const int old_dim = state_.state_dim() - SWVIOState::kCloneStateDim;
    const int new_dim = state_.state_dim();

    scratch_cov_.block(0, 0, old_dim, old_dim) = state_.covariance.block(0, 0, old_dim, old_dim);

    // Cross-correlation: J * P_imu (first 15 cols of each row)
    const Eigen::MatrixXd JP                   = J * state_.covariance.block(0, 0, 15, old_dim);
    scratch_cov_.block(old_dim, 0, 6, old_dim) = JP;
    scratch_cov_.block(0, old_dim, old_dim, 6) = JP.transpose();

    // New clone's self-covariance: J * P_imu * J^T
    scratch_cov_.block<6, 6>(old_dim, old_dim) = J * state_.covariance.block<15, 15>(0, 0) *
                                                 J.transpose();

    state_.covariance = scratch_cov_.block(0, 0, new_dim, new_dim);
}

// ── Marginalize oldest clone ────────────────────────────────
// Removes the oldest camera clone when the window has more than
// max_clones entries (window briefly reaches max_clones+1 after
// augmentation, then shrinks back to max_clones here).
void SlidingWindowVIOBackend::marginalize_oldest_clone() {
    if (static_cast<int>(state_.clones.size()) <= params_.max_clones) {
        return;
    }

    // Remove the first clone (O(1) with deque)
    state_.clones.pop_front();

    // Remove rows/cols [15..20] from covariance (the oldest clone's 6 DOF)
    const int remove_start = SWVIOState::kImuStateDim;
    const int remove_end   = remove_start + SWVIOState::kCloneStateDim;
    const int old_dim      = static_cast<int>(state_.covariance.rows());
    const int new_dim      = old_dim - SWVIOState::kCloneStateDim;

    // Copy into pre-allocated scratch, skipping the removed block
    scratch_cov_.block(0, 0, remove_start,
                       remove_start) = state_.covariance.block(0, 0, remove_start, remove_start);

    const int after = old_dim - remove_end;
    if (after > 0) {
        scratch_cov_.block(remove_start, remove_start, after,
                           after) = state_.covariance.block(remove_end, remove_end, after, after);
        scratch_cov_.block(0, remove_start, remove_start,
                           after) = state_.covariance.block(0, remove_end, remove_start, after);
        scratch_cov_.block(remove_start, 0, after, remove_start) =
            state_.covariance.block(remove_end, 0, after, remove_start);
    }

    state_.covariance = scratch_cov_.block(0, 0, new_dim, new_dim);
}

// ── Enforce covariance symmetry ─────────────────────────────
void SlidingWindowVIOBackend::enforce_symmetry() {
    state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
}

// ── Health state machine ────────────────────────────────────
// Transitions between INITIALIZING -> NOMINAL / DEGRADED / LOST
// based on initialization frame count and position covariance trace.
void SlidingWindowVIOBackend::update_health() {
    VIOHealth new_health = health_;

    if (frames_processed_ < params_.init_frames) {
        new_health = VIOHealth::INITIALIZING;
    } else {
        const double pos_trace =
            state_.covariance.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxPos).trace();

        if (pos_trace <= params_.good_trace_max) {
            new_health = VIOHealth::NOMINAL;
        } else if (pos_trace <= params_.degraded_trace_max) {
            new_health = VIOHealth::DEGRADED;
        } else {
            new_health = VIOHealth::LOST;
        }
    }

    if (new_health != health_) {
        DRONE_LOG_INFO("[SWVIO] Health: {} -> {}", vio_health_name(health_),
                       vio_health_name(new_health));
        health_ = new_health;
    }
}

// ── Main pipeline entry point ───────────────────────────────
VIOResult<VIOOutput> SlidingWindowVIOBackend::process_frame(
    const drone::ipc::StereoFrame& frame, const std::vector<ImuSample>& imu_samples) {

    const auto                    pipeline_start = std::chrono::steady_clock::now();
    const uint64_t                seq            = frame.sequence_number;
    drone::util::FrameDiagnostics diag(seq);

    // Validate frame timestamp
    if (frame.timestamp_ns > kMaxTimestampNs) {
        return VIOResult<VIOOutput>::err(VIOError{VIOErrorCode::TrackingLost, "SWVIOBackend",
                                                  "timestamp_ns out of plausible range", seq});
    }

    VIOOutput output;
    output.frame_id = seq;

    // ── 1. IMU propagation ──────────────────────────────────
    if (!imu_samples.empty()) {
        propagate_imu(imu_samples);
        output.imu_samples_used = static_cast<int>(imu_samples.size());
    }

    // ── 2. Feature extraction ───────────────────────────────
    auto feat_result = extractor_->extract(frame, diag);
    if (feat_result.is_err()) {
        output.health = health_;
        return VIOResult<VIOOutput>::err(std::move(feat_result.error()));
    }

    auto& feat          = feat_result.value();
    output.num_features = static_cast<int>(feat.features.size());

    // ── 3. Stereo matching ──────────────────────────────────
    auto match_result = matcher_->match(frame, feat.features, diag);
    if (match_result.is_err()) {
        output.num_stereo_matches = 0;
        diag.add_warning("SWVIOBackend", "Stereo matching failed — proceeding without matches");
    } else {
        output.num_stereo_matches = static_cast<int>(match_result.value().matches.size());
    }

    // ── 4. State augmentation ───────────────────────────────
    const double frame_time = static_cast<double>(frame.timestamp_ns) * 1e-9;
    augment_state(frame_time);

    // ── 5. Marginalize if window exceeds max_clones ─────────
    marginalize_oldest_clone();

    // ── 6. Enforce covariance symmetry (once per frame) ─────
    enforce_symmetry();

    // ── 7. Health state machine ─────────────────────────────
    ++frames_processed_;
    update_health();
    output.health = health_;

    // ── 8. Build output pose from current IMU state ─────────
    output.pose.position    = state_.imu.position;
    output.pose.orientation = state_.imu.orientation;
    output.pose.timestamp   = frame_time;

    // Extract 6x6 pose covariance (orientation + position blocks)
    output.pose.covariance                   = Eigen::Matrix<double, 6, 6>::Zero();
    output.pose.covariance.block<3, 3>(0, 0) = state_.covariance.block<3, 3>(SWVIOState::kIdxTheta,
                                                                             SWVIOState::kIdxTheta);
    output.pose.covariance.block<3, 3>(3, 3) = state_.covariance.block<3, 3>(SWVIOState::kIdxPos,
                                                                             SWVIOState::kIdxPos);

    // Map health to pose quality
    switch (health_) {
        case VIOHealth::LOST: output.pose.quality = 0; break;
        case VIOHealth::INITIALIZING: [[fallthrough]];
        case VIOHealth::DEGRADED: output.pose.quality = 1; break;
        case VIOHealth::NOMINAL: output.pose.quality = 2; break;
    }

    output.position_trace =
        state_.covariance.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxPos).trace();

    // ── 9. Pipeline timing ──────────────────────────────────
    const auto pipeline_end = std::chrono::steady_clock::now();
    output.total_ms =
        std::chrono::duration<double, std::milli>(pipeline_end - pipeline_start).count();

    return VIOResult<VIOOutput>::ok(std::move(output));
}

VIOHealth SlidingWindowVIOBackend::health() const {
    return health_;
}

std::string SlidingWindowVIOBackend::name() const {
    return "SlidingWindowVIO";
}

}  // namespace drone::slam
