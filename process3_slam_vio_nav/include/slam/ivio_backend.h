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

#include "ipc/shm_types.h"
#include "slam/ifeature_extractor.h"
#include "slam/imu_preintegrator.h"
#include "slam/istereo_matcher.h"
#include "slam/types.h"
#include "slam/vio_types.h"
#include "util/diagnostic.h"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::slam {

// ─────────────────────────────────────────────────────────────
// Interface
// ─────────────────────────────────────────────────────────────

class IVIOBackend {
public:
    virtual ~IVIOBackend() = default;

    /// Process one stereo frame + buffered IMU data and produce a VIO output.
    ///
    /// @param frame        The stereo frame from the camera.
    /// @param imu_samples  IMU readings accumulated since the last call.
    /// @return Ok(VIOOutput) on success, Err(VIOError) on critical failure.
    virtual VIOResult<VIOOutput> process_frame(const drone::ipc::ShmStereoFrame& frame,
                                               const std::vector<ImuSample>&     imu_samples) = 0;

    /// Current pipeline health.
    [[nodiscard]] virtual VIOHealth health() const = 0;

    /// Human-readable name for logging.
    [[nodiscard]] virtual std::string name() const = 0;
};

// ─────────────────────────────────────────────────────────────
// Simulated VIO backend
//
// Wires together the simulated sub-components:
//   SimulatedFeatureExtractor → SimulatedStereoMatcher → ImuPreintegrator
//
// Pose generation:
//   - Uses the same circular trajectory as the legacy SimulatedVisualFrontend
//   - IMU pre-integration is computed and validated but the pose is
//     generated independently (no actual fusion yet — that's Phase 2B)
//   - This lets us validate the full pipeline plumbing and error handling
//     before adding the optimizer
//
// Health transitions:
//   INITIALIZING → NOMINAL after min_init_frames_ frames
//   NOMINAL ↔ DEGRADED based on feature count / match rate thresholds
//   Any → LOST if feature extraction or stereo matching fails hard
// ─────────────────────────────────────────────────────────────

class SimulatedVIOBackend final : public IVIOBackend {
public:
    explicit SimulatedVIOBackend(StereoCalibration calib = {}, ImuNoiseParams imu_params = {},
                                 uint64_t min_init_frames = 5)
        : calib_(calib)
        , preintegrator_(imu_params)
        , extractor_(std::make_unique<SimulatedFeatureExtractor>())
        , matcher_(std::make_unique<SimulatedStereoMatcher>(calib))
        , min_init_frames_(min_init_frames) {}

    VIOResult<VIOOutput> process_frame(const drone::ipc::ShmStereoFrame& frame,
                                       const std::vector<ImuSample>&     imu_samples) override {

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

private:
    // ── Simulated pose generation (same as legacy frontend) ──
    Pose generate_simulated_pose(uint64_t seq) {
        double t = static_cast<double>(seq) * 0.033;  // ~30Hz

        Pose p;
        p.timestamp   = t;
        p.position    = Eigen::Vector3d(5.0 * std::cos(t * 0.5) + noise_(rng_),
                                        5.0 * std::sin(t * 0.5) + noise_(rng_),
                                        2.0 + 0.1 * std::sin(t) + noise_(rng_));
        double yaw    = t * 0.5 + M_PI / 2.0;
        p.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        p.covariance  = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        p.quality     = 2;  // good
        return p;
    }

    // ── Health state machine ─────────────────────────────────
    void update_health(const VIOOutput& output, drone::util::FrameDiagnostics& diag) {
        VIOHealth new_health = health_;

        if (frame_count_ < min_init_frames_) {
            new_health = VIOHealth::INITIALIZING;
        } else if (output.num_features < kMinFeaturesNominal) {
            new_health = VIOHealth::DEGRADED;
            diag.add_warning("VIOBackend", "Degraded: only " + std::to_string(output.num_features) +
                                               " features (need " +
                                               std::to_string(kMinFeaturesNominal) + ")");
        } else if (output.num_stereo_matches < kMinMatchesNominal && output.num_features > 0) {
            new_health = VIOHealth::DEGRADED;
        } else {
            new_health = VIOHealth::NOMINAL;
        }

        transition_health(new_health, "");
    }

    void transition_health(VIOHealth new_health, const std::string& reason) {
        if (new_health != health_) {
            spdlog::info("[VIO] Health: {} → {}{}", vio_health_name(health_),
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

    // ── Thresholds ──────────────────────────────────────────
    static constexpr int kMinFeaturesNominal = 30;
    static constexpr int kMinMatchesNominal  = 15;
};

// ── Factory ────────────────────────────────────────────────

/// Create a VIO backend by name.
/// Supported: "simulated" (default).
/// Future: "msckf", "okvis", "vins"
inline std::unique_ptr<IVIOBackend> create_vio_backend(const std::string& backend     = "simulated",
                                                       const StereoCalibration& calib = {},
                                                       const ImuNoiseParams&    imu_params = {}) {

    if (backend == "simulated") {
        return std::make_unique<SimulatedVIOBackend>(calib, imu_params);
    }
    throw std::runtime_error("[VIOBackend] Unknown backend: " + backend +
                             " (available: simulated)");
}

}  // namespace drone::slam
