// process3_slam_vio_nav/include/slam/istereo_matcher.h
// Strategy interface for stereo feature matching.
//
// Given features detected in the left image of a stereo pair, the
// stereo matcher finds corresponding points in the right image and
// computes depth via triangulation (z = fx * baseline / disparity).
//
// All implementations return VIOResult<StereoMatchResult> with
// structured diagnostics on failure.
#pragma once

#include "ipc/shm_types.h"
#include "slam/vio_types.h"
#include "util/diagnostic.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <random>
#include <string>

namespace drone::slam {

// ─────────────────────────────────────────────────────────────
// Interface
// ─────────────────────────────────────────────────────────────

class IStereoMatcher {
public:
    virtual ~IStereoMatcher() = default;

    /// Match features across left/right images and compute depth.
    /// @param frame     The stereo frame (left_data + right_data).
    /// @param features  Features detected in the left image.
    /// @param diag      Frame diagnostics collector.
    /// @return Ok(result) on success, Err(VIOError) on failure.
    virtual VIOResult<StereoMatchResult> match(const drone::ipc::ShmStereoFrame& frame,
                                               const std::vector<Feature>&       features,
                                               drone::util::FrameDiagnostics&    diag) = 0;

    /// Human-readable name.
    virtual std::string name() const = 0;
};

// ─────────────────────────────────────────────────────────────
// Simulated stereo matcher
//
// Generates synthetic stereo correspondences by applying a
// deterministic disparity model:
//   disparity = baseline * fx / simulated_depth
// with additive Gaussian noise on the disparity.
//
// Simulates a realistic match rate (configurable) — not all
// features produce valid matches, modelling occlusion / texture.
// ─────────────────────────────────────────────────────────────

class SimulatedStereoMatcher final : public IStereoMatcher {
public:
    /// @param calib         Stereo calibration parameters.
    /// @param match_rate    Fraction of features that produce valid matches [0,1].
    /// @param min_depth     Minimum simulated depth (metres).
    /// @param max_depth     Maximum simulated depth (metres).
    explicit SimulatedStereoMatcher(StereoCalibration calib = {}, float match_rate = 0.85f,
                                    float min_depth = 1.0f, float max_depth = 30.0f)
        : calib_(calib), match_rate_(match_rate), min_depth_(min_depth), max_depth_(max_depth) {}

    VIOResult<StereoMatchResult> match(const drone::ipc::ShmStereoFrame& frame,
                                       const std::vector<Feature>&       features,
                                       drone::util::FrameDiagnostics&    diag) override {

        drone::util::ScopedDiagTimer timer(diag, "StereoMatcher");

        const uint64_t seq = frame.sequence_number;

        // ── Validate inputs ─────────────────────────────────
        if (features.empty()) {
            auto err = VIOError(VIOErrorCode::StereoMatchFailed, "StereoMatcher",
                                "No features provided for stereo matching", seq);
            diag.add_error("StereoMatcher", err.message);
            return VIOResult<StereoMatchResult>::err(std::move(err));
        }

        StereoMatchResult result;
        result.frame_id = seq;
        result.matches.reserve(features.size());

        // Deterministic RNG seeded from frame sequence
        std::mt19937                          rng(static_cast<uint32_t>(seq * 1301 + 7));
        std::uniform_real_distribution<float> u_match(0.0f, 1.0f);
        std::uniform_real_distribution<float> u_depth(min_depth_, max_depth_);
        std::normal_distribution<float>       noise_disp(0.0f, 0.3f);  // sub-pixel noise

        for (const auto& feat : features) {
            // Simulate imperfect matching: some features fail
            if (u_match(rng) > match_rate_) continue;

            float sim_depth = u_depth(rng);
            float disparity = static_cast<float>(calib_.fx * calib_.baseline) / sim_depth;
            disparity += noise_disp(rng);  // sub-pixel noise

            if (disparity < 0.5f) continue;  // too small — unreliable

            StereoMatch m;
            m.feature_id  = feat.id;
            m.left_pixel  = feat.pixel;
            m.right_pixel = Eigen::Vector2f(feat.pixel.x() - disparity, feat.pixel.y());
            m.disparity   = disparity;
            m.depth       = static_cast<float>(calib_.depth_from_disparity(disparity));
            m.confidence  = std::clamp(1.0f - (noise_disp(rng) / 2.0f), 0.3f, 1.0f);

            if (m.depth > 0 && m.depth < max_depth_ * 2.0f) {
                result.matches.push_back(m);
            }
        }

        // ── Compute aggregate stats ─────────────────────────
        int n_matches = static_cast<int>(result.matches.size());

        if (n_matches == 0) {
            auto err = VIOError(VIOErrorCode::StereoMatchFailed, "StereoMatcher",
                                "Zero valid stereo matches from " +
                                    std::to_string(features.size()) + " features",
                                seq);
            diag.add_error("StereoMatcher", err.message);
            return VIOResult<StereoMatchResult>::err(std::move(err));
        }

        // Mean & stddev of matched depths
        float sum = 0.0f;
        for (const auto& m : result.matches) sum += m.depth;
        result.mean_depth = sum / static_cast<float>(n_matches);

        float sq_sum = 0.0f;
        for (const auto& m : result.matches) {
            float d = m.depth - result.mean_depth;
            sq_sum += d * d;
        }
        result.depth_stddev = std::sqrt(sq_sum / static_cast<float>(n_matches));

        // ── Record diagnostics ──────────────────────────────
        diag.add_metric("StereoMatcher", "num_matches", n_matches);
        diag.add_metric("StereoMatcher", "match_rate",
                        static_cast<double>(n_matches) / static_cast<double>(features.size()));
        diag.add_metric("StereoMatcher", "mean_depth", result.mean_depth);
        diag.add_metric("StereoMatcher", "depth_stddev", result.depth_stddev);

        // Warn if match rate is unexpectedly low
        double actual_rate = static_cast<double>(n_matches) / static_cast<double>(features.size());
        if (actual_rate < 0.3) {
            diag.add_warning("StereoMatcher",
                             "Low match rate: " + std::to_string(actual_rate * 100.0) + "% (" +
                                 std::to_string(n_matches) + "/" + std::to_string(features.size()) +
                                 ")");
        }

        // Warn if depth variance is suspiciously low (flat scene / calibration issue)
        if (result.depth_stddev < 0.1f && n_matches > 10) {
            diag.add_warning("StereoMatcher",
                             "Very low depth variance (σ=" + std::to_string(result.depth_stddev) +
                                 "m) — possible calibration issue or flat scene");
        }

        return VIOResult<StereoMatchResult>::ok(std::move(result));
    }

    std::string name() const override { return "SimulatedStereoMatcher"; }

private:
    StereoCalibration calib_;
    float             match_rate_;
    float             min_depth_;
    float             max_depth_;
};

// ── Factory ────────────────────────────────────────────────

/// Create a stereo matcher by name.
/// Supported backends: "simulated" (default).
/// Future: "sgbm" (OpenCV Semi-Global Block Matching), "raft_stereo"
inline std::unique_ptr<IStereoMatcher> create_stereo_matcher(
    const std::string& backend = "simulated", const StereoCalibration& calib = {}) {

    if (backend == "simulated") {
        return std::make_unique<SimulatedStereoMatcher>(calib);
    }
    throw std::runtime_error("[StereoMatcher] Unknown backend: " + backend +
                             " (available: simulated)");
}

}  // namespace drone::slam
