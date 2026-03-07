// process3_slam_vio_nav/include/slam/vio_types.h
// VIO-specific types: errors, status, diagnostic structs, pre-integration
// output, and pipeline result types.
//
// Every VIO component returns Result<T, VIOError> so that failures
// propagate structured context (error code, component, frame, message)
// all the way to the caller.  This is the foundation for the
// "know exactly what went wrong" design philosophy.
#pragma once

#include "slam/types.h"
#include "util/diagnostic.h"
#include "util/result.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone::slam {

// ═══════════════════════════════════════════════════════════
// VIO Error types
// ═══════════════════════════════════════════════════════════

/// Error codes specific to the VIO pipeline.
/// Each maps to a distinct failure mode with a clear recovery strategy.
enum class VIOErrorCode : uint8_t {
    None = 0,

    // ── Feature extraction ────────────────────────────────
    InsufficientFeatures,     // too few features for reliable tracking
    FeatureExtractionFailed,  // extractor returned zero features

    // ── Stereo matching ───────────────────────────────────
    StereoMatchFailed,     // matcher produced no valid matches
    LowDisparityVariance,  // stereo baseline too short or scene too far

    // ── IMU ───────────────────────────────────────────────
    ImuDataGap,         // gap between consecutive IMU samples
    ImuSaturation,      // reading exceeds sensor measurement range
    ImuNotInitialized,  // pre-integrator used before first sample

    // ── Pipeline ──────────────────────────────────────────
    TrackingLost,           // visual tracking cannot continue
    DegenerateMotion,       // insufficient parallax to triangulate
    NumericalFailure,       // matrix decomposition or inversion failed
    InitializationPending,  // VIO not yet bootstrapped (need N frames)
    BackendNotReady,        // backend rejected frame (internal state)
};

inline const char* vio_error_name(VIOErrorCode c) {
    switch (c) {
        case VIOErrorCode::None: return "None";
        case VIOErrorCode::InsufficientFeatures: return "InsufficientFeatures";
        case VIOErrorCode::FeatureExtractionFailed: return "FeatureExtractionFailed";
        case VIOErrorCode::StereoMatchFailed: return "StereoMatchFailed";
        case VIOErrorCode::LowDisparityVariance: return "LowDisparityVariance";
        case VIOErrorCode::ImuDataGap: return "ImuDataGap";
        case VIOErrorCode::ImuSaturation: return "ImuSaturation";
        case VIOErrorCode::ImuNotInitialized: return "ImuNotInitialized";
        case VIOErrorCode::TrackingLost: return "TrackingLost";
        case VIOErrorCode::DegenerateMotion: return "DegenerateMotion";
        case VIOErrorCode::NumericalFailure: return "NumericalFailure";
        case VIOErrorCode::InitializationPending: return "InitializationPending";
        case VIOErrorCode::BackendNotReady: return "BackendNotReady";
        default: return "Unknown";
    }
}

/// Rich error type carrying context about *which* VIO stage failed,
/// *what* happened, and *when* (frame ID).
struct VIOError {
    VIOErrorCode code = VIOErrorCode::None;
    std::string  component;  // stage that produced the error
    std::string  message;    // human-readable explanation
    uint64_t     frame_id = 0;

    VIOError() = default;

    VIOError(VIOErrorCode c, std::string comp, std::string msg, uint64_t fid = 0)
        : code(c), component(std::move(comp)), message(std::move(msg)), frame_id(fid) {}

    /// Convenience: format as "[component] code: message (frame N)"
    [[nodiscard]] std::string to_string() const {
        return "[" + component + "] " + vio_error_name(code) + ": " + message + " (frame " +
               std::to_string(frame_id) + ")";
    }
};

/// Shorthand result type for VIO components.
template<typename T>
using VIOResult = drone::util::Result<T, VIOError>;

// ═══════════════════════════════════════════════════════════
// VIO Health / Status
// ═══════════════════════════════════════════════════════════

/// Coarse health status of the VIO pipeline.
enum class VIOHealth : uint8_t {
    INITIALIZING = 0,  // collecting bootstrap frames
    NOMINAL      = 1,  // all stages producing valid output
    DEGRADED     = 2,  // some warnings but still tracking
    LOST         = 3,  // tracking lost — need re-initialization
};

inline const char* vio_health_name(VIOHealth h) {
    switch (h) {
        case VIOHealth::INITIALIZING: return "INITIALIZING";
        case VIOHealth::NOMINAL: return "NOMINAL";
        case VIOHealth::DEGRADED: return "DEGRADED";
        case VIOHealth::LOST: return "LOST";
        default: return "???";
    }
}

// ═══════════════════════════════════════════════════════════
// Feature extraction result
// ═══════════════════════════════════════════════════════════

struct FeatureExtractionResult {
    std::vector<Feature> features;           // detected features
    uint64_t             frame_id      = 0;  // source frame sequence
    double               extraction_ms = 0;  // wall-clock time for extraction
};

// ═══════════════════════════════════════════════════════════
// Stereo match result
// ═══════════════════════════════════════════════════════════

/// A single stereo-matched feature with depth information.
struct StereoMatch {
    uint32_t        feature_id;   // references Feature::id
    Eigen::Vector2f left_pixel;   // left image coordinates
    Eigen::Vector2f right_pixel;  // right image coordinates
    float           disparity;    // left_x - right_x (pixels)
    float           depth;        // baseline * focal_length / disparity (metres)
    float           confidence;   // match quality [0, 1]
};

struct StereoMatchResult {
    std::vector<StereoMatch> matches;
    uint64_t                 frame_id     = 0;
    double                   match_ms     = 0;  // wall-clock time
    float                    mean_depth   = 0;  // average depth of valid matches
    float                    depth_stddev = 0;  // standard deviation of depths
};

// ═══════════════════════════════════════════════════════════
// IMU pre-integration result
// ═══════════════════════════════════════════════════════════

/// Pre-integrated IMU measurement between two keyframes.
/// Follows the Forster et al. (2017) formulation:
///   Δα = position change, Δβ = velocity change, Δγ = rotation change
struct PreintegratedMeasurement {
    double             dt = 0;          // total integration interval (sec)
    Eigen::Vector3d    delta_position;  // α — position change in body frame
    Eigen::Vector3d    delta_velocity;  // β — velocity change in body frame
    Eigen::Quaterniond delta_rotation;  // γ — rotation change

    /// 9×9 covariance: [δα(3), δβ(3), δφ(3)]
    Eigen::Matrix<double, 9, 9> covariance;

    /// Jacobian of pre-integrated measurement w.r.t. biases [b_g(3), b_a(3)]
    /// Used for first-order bias correction without re-integration.
    Eigen::Matrix<double, 9, 6> jacobian_bias;

    int  num_samples = 0;
    bool valid       = false;

    PreintegratedMeasurement()
        : delta_position(Eigen::Vector3d::Zero())
        , delta_velocity(Eigen::Vector3d::Zero())
        , delta_rotation(Eigen::Quaterniond::Identity())
        , covariance(Eigen::Matrix<double, 9, 9>::Zero())
        , jacobian_bias(Eigen::Matrix<double, 9, 6>::Zero()) {}
};

// ═══════════════════════════════════════════════════════════
// VIO output — the full result of one pipeline iteration
// ═══════════════════════════════════════════════════════════

struct VIOOutput {
    Pose      pose;  // estimated 6-DOF pose
    VIOHealth health = VIOHealth::INITIALIZING;

    // ── Per-frame diagnostics (for monitoring / debugging) ──
    int      num_features       = 0;  // features detected this frame
    int      num_stereo_matches = 0;  // successful stereo matches
    int      imu_samples_used   = 0;  // IMU samples pre-integrated
    double   feature_ms         = 0;  // feature extraction time
    double   stereo_ms          = 0;  // stereo matching time
    double   preint_ms          = 0;  // pre-integration time
    double   total_ms           = 0;  // full pipeline time
    uint64_t frame_id           = 0;
};

// ═══════════════════════════════════════════════════════════
// Stereo camera intrinsics (needed for depth from disparity)
// ═══════════════════════════════════════════════════════════

struct StereoCalibration {
    double fx       = 350.0;  // focal length x (pixels)
    double fy       = 350.0;  // focal length y (pixels)
    double cx       = 320.0;  // principal point x
    double cy       = 240.0;  // principal point y
    double baseline = 0.12;   // stereo baseline (metres)

    /// Depth from disparity: z = fx * baseline / disparity
    [[nodiscard]] double depth_from_disparity(double disparity) const {
        if (disparity < 1e-3) return -1.0;  // invalid / too small
        return fx * baseline / disparity;
    }
};

}  // namespace drone::slam
