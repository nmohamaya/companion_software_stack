// process3_slam_vio_nav/include/slam/swvio_backend.h
// Sliding-Window Visual-Inertial Odometry (SWVIO) backend.
//
// Phase 1: IMU state propagation + state augmentation.
//   - Propagates IMU error state and covariance through IMU measurements
//   - Augments state with camera clones at each frame
//   - Marginalizes oldest clones when the window exceeds max_clones
//
// Phase 2 (future): MSCKF-style multi-view feature update.
//
// This backend implements IVIOBackend and is registered as "swvio"
// in the factory function create_vio_backend().
#pragma once

#include "slam/ifeature_extractor.h"
#include "slam/istereo_matcher.h"
#include "slam/ivio_interface.h"
#include "slam/swvio_types.h"
#include "slam/types.h"
#include "slam/vio_types.h"

#include <memory>
#include <string>

namespace drone::slam {

class SlidingWindowVIOBackend : public IVIOBackend {
public:
    /// @param calib      Stereo camera calibration parameters.
    /// @param imu_params IMU noise parameters (from datasheet/calibration).
    /// @param params     SWVIO-specific parameters (window size, thresholds, health).
    SlidingWindowVIOBackend(const StereoCalibration& calib, const ImuNoiseParams& imu_params,
                            const SWVIOParams& params);

    SlidingWindowVIOBackend(const SlidingWindowVIOBackend&)            = delete;
    SlidingWindowVIOBackend& operator=(const SlidingWindowVIOBackend&) = delete;

    [[nodiscard]] VIOResult<VIOOutput> process_frame(
        const drone::ipc::StereoFrame& frame, const std::vector<ImuSample>& imu_samples) override;

    [[nodiscard]] VIOHealth health() const override;

    [[nodiscard]] std::string name() const override;

    [[nodiscard]] int clone_count() const { return static_cast<int>(state_.clones.size()); }
    [[nodiscard]] int state_dim() const { return state_.state_dim(); }

private:
    // ── Core SWVIO operations ──────────────────────────────
    void propagate_imu(const std::vector<ImuSample>& imu_samples);
    void augment_state(double timestamp);
    void marginalize_oldest_clone();

    /// Update health based on position covariance trace and initialization frame count.
    void update_health();

    /// Enforce covariance symmetry (call once per frame, not per IMU step).
    void enforce_symmetry();

    // ── State ──────────────────────────────────────────────
    SWVIOState        state_;
    SWVIOParams       params_;
    StereoCalibration calib_;
    ImuNoiseParams    imu_params_;

    // ── Precomputed (constant across lifetime) ─────────────
    Eigen::Matrix<double, 12, 12> Q_c_;          // continuous-time noise covariance
    Eigen::MatrixXd               scratch_cov_;  // pre-allocated scratch for augment/marginalize

    // ── Sub-components ─────────────────────────────────────
    std::unique_ptr<IFeatureExtractor> extractor_;
    std::unique_ptr<IStereoMatcher>    matcher_;

    // ── Health state machine ───────────────────────────────
    VIOHealth health_           = VIOHealth::INITIALIZING;
    int       frames_processed_ = 0;
    uint64_t  next_clone_id_    = 0;
};

}  // namespace drone::slam
