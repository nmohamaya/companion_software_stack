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
    /// @param calib             Stereo camera calibration parameters.
    /// @param imu_params        IMU noise parameters (from datasheet/calibration).
    /// @param params            SWVIO-specific parameters (window size, thresholds).
    /// @param init_frames       Frames before transitioning out of INITIALIZING.
    /// @param good_trace_max    Position trace threshold for NOMINAL health.
    /// @param degraded_trace_max Position trace threshold for DEGRADED health.
    SlidingWindowVIOBackend(const StereoCalibration& calib, const ImuNoiseParams& imu_params,
                            const SWVIOParams& params, int init_frames, double good_trace_max,
                            double degraded_trace_max);

    VIOResult<VIOOutput> process_frame(const drone::ipc::StereoFrame& frame,
                                       const std::vector<ImuSample>&  imu_samples) override;

    [[nodiscard]] VIOHealth health() const override;

    [[nodiscard]] std::string name() const override;

private:
    // ── Core SWVIO operations ──────────────────────────────
    /// Propagate IMU state and covariance through a sequence of IMU samples.
    void propagate_imu(const std::vector<ImuSample>& imu_samples);

    /// Clone the current IMU pose into the sliding window.
    void augment_state(double timestamp);

    /// Remove the oldest camera clone when the window exceeds max_clones.
    void marginalize_oldest_clone();

    /// Update health state machine based on feature count and covariance.
    void update_health(int num_features, int num_matches);

    // ── State ──────────────────────────────────────────────
    SWVIOState        state_;
    SWVIOParams       params_;
    StereoCalibration calib_;
    ImuNoiseParams    imu_params_;

    // ── Sub-components ─────────────────────────────────────
    std::unique_ptr<IFeatureExtractor> extractor_;
    std::unique_ptr<IStereoMatcher>    matcher_;

    // ── Health state machine ───────────────────────────────
    VIOHealth health_             = VIOHealth::INITIALIZING;
    int       frames_processed_   = 0;
    int       init_frames_        = 10;
    double    good_trace_max_     = 0.1;
    double    degraded_trace_max_ = 1.0;

    uint64_t next_clone_id_ = 0;
};

}  // namespace drone::slam
