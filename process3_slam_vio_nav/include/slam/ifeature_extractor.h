// process3_slam_vio_nav/include/slam/ifeature_extractor.h
// Strategy interface for visual feature extraction.
//
// Extracts 2D keypoints from a grayscale image.  In production this
// wraps ORB / FAST / SuperPoint / etc.  The SimulatedFeatureExtractor
// generates deterministic synthetic features for pipeline validation
// without requiring OpenCV at link time.
//
// All implementations return VIOResult<FeatureExtractionResult> so that
// failures carry structured diagnostics (error code, component, frame).
#pragma once

#include "ipc/shm_types.h"
#include "slam/vio_types.h"
#include "util/diagnostic.h"

#include <cmath>
#include <memory>
#include <random>
#include <string>

namespace drone::slam {

// ─────────────────────────────────────────────────────────────
// Interface
// ─────────────────────────────────────────────────────────────

class IFeatureExtractor {
public:
    virtual ~IFeatureExtractor() = default;

    /// Extract features from a grayscale image (left camera of stereo pair).
    /// @param frame   Stereo frame (uses left_data).
    /// @param diag    Frame diagnostics collector — caller-owned, optional info recorded.
    /// @return Ok(result) on success, Err(VIOError) on failure.
    virtual VIOResult<FeatureExtractionResult> extract(const drone::ipc::ShmStereoFrame& frame,
                                                       drone::util::FrameDiagnostics&    diag) = 0;

    /// Human-readable name (for logging / diagnostics).
    virtual std::string name() const = 0;
};

// ─────────────────────────────────────────────────────────────
// Simulated feature extractor
//
// Generates synthetic features at deterministic positions derived
// from the frame sequence number.  Feature count is configurable
// and includes small random perturbation for realism.
//
// This lets us validate the full VIO pipeline end-to-end in
// simulation without depending on real image content.
// ─────────────────────────────────────────────────────────────

class SimulatedFeatureExtractor final : public IFeatureExtractor {
public:
    /// @param num_features  Target number of features per frame (default 120).
    /// @param image_width   Image width for feature coordinate generation.
    /// @param image_height  Image height for feature coordinate generation.
    explicit SimulatedFeatureExtractor(int num_features = 120, int image_width = 640,
                                       int image_height = 480)
        : num_features_(num_features), image_width_(image_width), image_height_(image_height) {}

    VIOResult<FeatureExtractionResult> extract(const drone::ipc::ShmStereoFrame& frame,
                                               drone::util::FrameDiagnostics&    diag) override {

        drone::util::ScopedDiagTimer timer(diag, "FeatureExtractor");

        const uint64_t seq = frame.sequence_number;

        // ── Validate input ──────────────────────────────────
        if (frame.width == 0 || frame.height == 0) {
            auto err = VIOError(VIOErrorCode::FeatureExtractionFailed, "FeatureExtractor",
                                "Invalid frame dimensions: " + std::to_string(frame.width) + "x" +
                                    std::to_string(frame.height),
                                seq);
            diag.add_error("FeatureExtractor", err.message);
            return VIOResult<FeatureExtractionResult>::err(std::move(err));
        }

        FeatureExtractionResult result;
        result.frame_id = seq;
        result.features.reserve(static_cast<size_t>(num_features_));

        // Seed RNG from frame sequence for deterministic-per-frame behaviour
        // but different features every frame (simulates camera motion)
        std::mt19937                          rng(static_cast<uint32_t>(seq * 7919 + 42));
        std::uniform_real_distribution<float> u_x(margin_,
                                                  static_cast<float>(image_width_ - margin_));
        std::uniform_real_distribution<float> u_y(margin_,
                                                  static_cast<float>(image_height_ - margin_));
        std::uniform_real_distribution<float> u_response(20.0f, 100.0f);

        for (int i = 0; i < num_features_; ++i) {
            Feature f;
            f.id              = static_cast<uint32_t>(seq * 1000 + static_cast<uint64_t>(i));
            f.pixel           = Eigen::Vector2f(u_x(rng), u_y(rng));
            f.position_3d     = Eigen::Vector3d::Zero();  // not yet triangulated
            f.response        = u_response(rng);
            f.is_triangulated = false;
            result.features.push_back(f);
        }

        // ── Record diagnostics ──────────────────────────────
        int n = static_cast<int>(result.features.size());
        diag.add_metric("FeatureExtractor", "num_features", n);

        if (n < min_features_warn_) {
            diag.add_warning("FeatureExtractor", "Low feature count: " + std::to_string(n) +
                                                     " (threshold " +
                                                     std::to_string(min_features_warn_) + ")");
        }

        return VIOResult<FeatureExtractionResult>::ok(std::move(result));
    }

    std::string name() const override { return "SimulatedFeatureExtractor"; }

private:
    int   num_features_;
    int   image_width_;
    int   image_height_;
    float margin_            = 10.0f;  // keep features away from image border
    int   min_features_warn_ = 30;     // below this, emit a diagnostic warning
};

// ── Factory ────────────────────────────────────────────────

/// Create a feature extractor by name.
/// Supported backends: "simulated" (default).
/// Future: "orb", "fast", "superpoint"
inline std::unique_ptr<IFeatureExtractor> create_feature_extractor(
    const std::string& backend = "simulated", int num_features = 120, int image_width = 640,
    int image_height = 480) {

    if (backend == "simulated") {
        return std::make_unique<SimulatedFeatureExtractor>(num_features, image_width, image_height);
    }
    // Future backends go here
    throw std::runtime_error("[FeatureExtractor] Unknown backend: " + backend +
                             " (available: simulated)");
}

}  // namespace drone::slam
