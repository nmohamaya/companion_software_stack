// process3_slam_vio_nav/src/swvio_backend.cpp
// Sliding-Window Visual-Inertial Odometry backend — Phase 1 implementation.
//
// Phase 1 provides:
//   - IMU state propagation with error-state covariance
//   - Camera clone augmentation into the sliding window
//   - Oldest-clone marginalization
//   - Health state machine (covariance-based quality)
//
// Phase 2 (future) will add MSCKF-style multi-view feature updates
// that constrain the accumulated IMU drift.

#include "slam/swvio_backend.h"

#include "slam/slam_math.h"
#include "util/diagnostic.h"
#include "util/ilogger.h"

#include <chrono>

namespace drone::slam {

// ── Constructor ─────────────────────────────────────────────
SlidingWindowVIOBackend::SlidingWindowVIOBackend(const StereoCalibration& calib,
                                                 const ImuNoiseParams&    imu_params,
                                                 const SWVIOParams& params, int init_frames,
                                                 double good_trace_max, double degraded_trace_max)
    : params_(params)
    , calib_(calib)
    , imu_params_(imu_params)
    , extractor_(std::make_unique<SimulatedFeatureExtractor>())
    , matcher_(std::make_unique<SimulatedStereoMatcher>(calib))
    , init_frames_(init_frames)
    , good_trace_max_(good_trace_max)
    , degraded_trace_max_(degraded_trace_max) {

    // Initialize IMU state at origin with identity orientation
    state_.imu.orientation = Eigen::Quaterniond::Identity();
    state_.imu.position    = Eigen::Vector3d::Zero();
    state_.imu.velocity    = Eigen::Vector3d::Zero();
    state_.imu.gyro_bias   = Eigen::Vector3d::Zero();
    state_.imu.accel_bias  = Eigen::Vector3d::Zero();
    state_.imu.timestamp   = 0.0;

    // Initialize covariance: 15x15 for IMU error state only (no clones yet).
    // Diagonal entries reflect initial uncertainty in each state component.
    constexpr int kDim = SWVIOState::kImuStateDim;
    state_.covariance  = Eigen::MatrixXd::Zero(kDim, kDim);

    // Orientation uncertainty (rad^2)
    state_.covariance.block<3, 3>(SWVIOState::kIdxTheta,
                                  SWVIOState::kIdxTheta) = Eigen::Matrix3d::Identity() * 1e-3;
    // Position uncertainty (m^2)
    state_.covariance.block<3, 3>(SWVIOState::kIdxPos,
                                  SWVIOState::kIdxPos) = Eigen::Matrix3d::Identity() * 1e-2;
    // Velocity uncertainty (m/s)^2
    state_.covariance.block<3, 3>(SWVIOState::kIdxVel,
                                  SWVIOState::kIdxVel) = Eigen::Matrix3d::Identity() * 1e-2;
    // Gyro bias uncertainty (rad/s)^2
    state_.covariance.block<3, 3>(SWVIOState::kIdxBg,
                                  SWVIOState::kIdxBg) = Eigen::Matrix3d::Identity() * 1e-4;
    // Accel bias uncertainty (m/s^2)^2
    state_.covariance.block<3, 3>(SWVIOState::kIdxBa,
                                  SWVIOState::kIdxBa) = Eigen::Matrix3d::Identity() * 1e-4;
}

// ── IMU state propagation ───────────────────────────────────
// For each consecutive pair of IMU samples, propagates the nominal
// state forward and updates the error-state covariance using the
// linearized continuous-time model discretized at the IMU rate.
void SlidingWindowVIOBackend::propagate_imu(const std::vector<ImuSample>& imu_samples) {
    if (imu_samples.size() < 2) {
        return;
    }

    for (size_t i = 1; i < imu_samples.size(); ++i) {
        const double dt = imu_samples[i].timestamp - imu_samples[i - 1].timestamp;
        if (dt <= 0.0) {
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
        // Position: p += v*dt + 0.5*a*dt^2
        state_.imu.position += state_.imu.velocity * dt + 0.5 * accel_world * dt * dt;

        // Velocity: v += a*dt
        state_.imu.velocity += accel_world * dt;

        // Orientation: q = q * exp(omega*dt)
        state_.imu.orientation = (state_.imu.orientation * exp_map(gyro_mid * dt)).normalized();

        // Update timestamp
        state_.imu.timestamp = imu_samples[i].timestamp;

        // ── Error-state transition matrix F (15x15) ─────────
        // Linearization of the error-state dynamics around the current
        // nominal state. Uses first-order approximation for dt terms.
        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();

        // F[theta, theta] = I - skew(omega)*dt (first-order approx of exp(-skew(w)*dt))
        F.block<3, 3>(SWVIOState::kIdxTheta, SWVIOState::kIdxTheta) -= skew(gyro_mid) * dt;

        // F[theta, bg] = -I*dt (gyro bias effect on orientation)
        F.block<3, 3>(SWVIOState::kIdxTheta, SWVIOState::kIdxBg) = -Eigen::Matrix3d::Identity() *
                                                                   dt;

        // F[vel, theta] = -R*skew(a)*dt (orientation error -> velocity error)
        F.block<3, 3>(SWVIOState::kIdxVel, SWVIOState::kIdxTheta) = -R * skew(accel_mid) * dt;

        // F[vel, ba] = -R*dt (accel bias effect on velocity)
        F.block<3, 3>(SWVIOState::kIdxVel, SWVIOState::kIdxBa) = -R * dt;

        // F[pos, vel] = I*dt
        F.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxVel) = Eigen::Matrix3d::Identity() * dt;

        // F[pos, theta] = -0.5*R*skew(a)*dt^2
        F.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxTheta) = -0.5 * R * skew(accel_mid) *
                                                                    dt * dt;

        // F[pos, ba] = -0.5*R*dt^2
        F.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxBa) = -0.5 * R * dt * dt;

        // ── Noise Jacobian G (15x12) ────────────────────────
        // Maps continuous-time noise [n_g(3), n_a(3), n_bg(3), n_ba(3)]
        // to the error state.
        Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();

        // Gyro noise -> orientation
        G.block<3, 3>(SWVIOState::kIdxTheta, 0) = -Eigen::Matrix3d::Identity() * dt;

        // Accel noise -> velocity
        G.block<3, 3>(SWVIOState::kIdxVel, 3) = -R * dt;

        // Accel noise -> position
        G.block<3, 3>(SWVIOState::kIdxPos, 3) = -0.5 * R * dt * dt;

        // Gyro bias random walk
        G.block<3, 3>(SWVIOState::kIdxBg, 6) = Eigen::Matrix3d::Identity() * dt;

        // Accel bias random walk
        G.block<3, 3>(SWVIOState::kIdxBa, 9) = Eigen::Matrix3d::Identity() * dt;

        // ── Continuous-time noise covariance Q_c (12x12) ────
        Eigen::Matrix<double, 12, 12> Q_c = Eigen::Matrix<double, 12, 12>::Zero();

        const double sg2  = imu_params_.gyro_noise_density * imu_params_.gyro_noise_density;
        const double sa2  = imu_params_.accel_noise_density * imu_params_.accel_noise_density;
        const double sbg2 = imu_params_.gyro_random_walk * imu_params_.gyro_random_walk;
        const double sba2 = imu_params_.accel_random_walk * imu_params_.accel_random_walk;

        Q_c.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sg2;
        Q_c.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sa2;
        Q_c.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * sbg2;
        Q_c.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * sba2;

        // ── Covariance propagation ──────────────────────────
        const int n = state_.state_dim();

        // IMU-IMU block: P_ii = F * P_ii * F^T + G * Q_c * G^T
        const Eigen::MatrixXd P_ii = F * state_.covariance.block<15, 15>(0, 0) * F.transpose() +
                                     G * Q_c * G.transpose();
        state_.covariance.block<15, 15>(0, 0) = P_ii;

        // IMU-clone cross-correlations: P_ij = F * P_ij for each clone j
        if (n > 15) {
            state_.covariance.block(0, 15, 15, n - 15) = F *
                                                         state_.covariance.block(0, 15, 15, n - 15);
            // Symmetric: P_ji = P_ij^T
            state_.covariance.block(15, 0, n - 15,
                                    15) = state_.covariance.block(0, 15, 15, n - 15).transpose();
        }

        // Enforce symmetry to prevent numerical drift
        state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
    }
}

// ── State augmentation ──────────────────────────────────────
// Clones the current IMU pose (orientation + position) into the
// sliding window and expands the covariance accordingly.
void SlidingWindowVIOBackend::augment_state(double timestamp) {
    // Create a new camera clone from the current IMU state
    CameraClone clone;
    clone.clone_id    = next_clone_id_++;
    clone.timestamp   = timestamp;
    clone.orientation = state_.imu.orientation;
    clone.position    = state_.imu.position;
    state_.clones.push_back(clone);

    // Build the augmentation Jacobian J (6x15):
    // Maps IMU error state to the new clone's error state.
    //   clone_theta = imu_theta  -> J[0:3, 0:3] = I
    //   clone_pos   = imu_pos    -> J[3:6, 3:6] = I
    Eigen::Matrix<double, 6, 15> J          = Eigen::Matrix<double, 6, 15>::Zero();
    J.block<3, 3>(0, SWVIOState::kIdxTheta) = Eigen::Matrix3d::Identity();
    J.block<3, 3>(3, SWVIOState::kIdxPos)   = Eigen::Matrix3d::Identity();

    // Expand covariance from (n x n) to (n+6 x n+6)
    const int old_dim = state_.state_dim() - SWVIOState::kCloneStateDim;  // before clone was added
    const int new_dim = state_.state_dim();

    Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(new_dim, new_dim);

    // Copy old covariance
    P_new.block(0, 0, old_dim, old_dim) = state_.covariance;

    // Cross-correlation: P * J^T (bottom-left and top-right)
    // J operates on the IMU block (first 15 rows/cols of P)
    const Eigen::MatrixXd JP            = J * state_.covariance.block(0, 0, 15, old_dim);
    P_new.block(old_dim, 0, 6, old_dim) = JP;
    P_new.block(0, old_dim, old_dim, 6) = JP.transpose();

    // New clone's self-covariance: J * P_imu * J^T
    P_new.block<6, 6>(old_dim, old_dim) = J * state_.covariance.block<15, 15>(0, 0) * J.transpose();

    state_.covariance = P_new;

    // Enforce symmetry
    state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
}

// ── Marginalize oldest clone ────────────────────────────────
// Removes the oldest camera clone from the state and shrinks
// the covariance matrix by removing the corresponding rows/cols.
void SlidingWindowVIOBackend::marginalize_oldest_clone() {
    if (static_cast<int>(state_.clones.size()) <= params_.max_clones) {
        return;
    }

    // Remove the first clone
    state_.clones.erase(state_.clones.begin());

    // Remove rows/cols [15..20] from covariance (the oldest clone's 6 DOF)
    const int remove_start = SWVIOState::kImuStateDim;
    const int remove_end   = remove_start + SWVIOState::kCloneStateDim;
    const int old_dim      = static_cast<int>(state_.covariance.rows());
    const int new_dim      = old_dim - SWVIOState::kCloneStateDim;

    Eigen::MatrixXd P_new = Eigen::MatrixXd::Zero(new_dim, new_dim);

    // Copy the IMU block (rows/cols 0..14)
    P_new.block(0, 0, remove_start, remove_start) = state_.covariance.block(0, 0, remove_start,
                                                                            remove_start);

    // Copy the remaining clones block (rows/cols after the removed one)
    const int after = old_dim - remove_end;
    if (after > 0) {
        // Bottom-right block
        P_new.block(remove_start, remove_start, after,
                    after) = state_.covariance.block(remove_end, remove_end, after, after);

        // Cross-correlations with IMU
        P_new.block(0, remove_start, remove_start,
                    after)        = state_.covariance.block(0, remove_end, remove_start, after);
        P_new.block(remove_start, 0, after,
                    remove_start) = state_.covariance.block(remove_end, 0, after, remove_start);
    }

    state_.covariance = P_new;
}

// ── Health state machine ────────────────────────────────────
// Transitions between INITIALIZING -> NOMINAL / DEGRADED / LOST
// based on frame count and position covariance trace.
void SlidingWindowVIOBackend::update_health(int /*num_features*/, int /*num_matches*/) {
    VIOHealth new_health = health_;

    if (frames_processed_ < init_frames_) {
        new_health = VIOHealth::INITIALIZING;
    } else {
        // Compute position covariance trace
        const double pos_trace =
            state_.covariance.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxPos).trace();

        if (pos_trace <= good_trace_max_) {
            new_health = VIOHealth::NOMINAL;
        } else if (pos_trace <= degraded_trace_max_) {
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

    // ── 6. Health state machine ─────────────────────────────
    ++frames_processed_;
    update_health(output.num_features, output.num_stereo_matches);
    output.health = health_;

    // ── 7. Build output pose from current IMU state ─────────
    output.pose.position    = state_.imu.position;
    output.pose.orientation = state_.imu.orientation;
    output.pose.timestamp =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

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

    // Position covariance trace for monitoring
    output.position_trace =
        state_.covariance.block<3, 3>(SWVIOState::kIdxPos, SWVIOState::kIdxPos).trace();

    // ── 8. Pipeline timing ──────────────────────────────────
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
