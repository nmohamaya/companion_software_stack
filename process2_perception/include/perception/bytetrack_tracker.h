// process2_perception/include/perception/bytetrack_tracker.h
// ByteTrack multi-object tracker — two-stage association for occlusion recovery.
// Zhang et al., "ByteTrack: Multi-Object Tracking by Associating Every
// Detection Box", ECCV 2022.
// Implements Issue #163.
#pragma once

#include "perception/itracker.h"
#include "perception/kalman_tracker.h"

#include <cstdint>
#include <string>
#include <vector>

namespace drone::perception {

/// Configuration parameters for ByteTrackTracker.
struct ByteTrackParams {
    float    high_conf_threshold = 0.5f;
    float    low_conf_threshold  = 0.1f;
    double   max_iou_cost        = 0.7;
    uint32_t max_age             = 10;
    uint32_t min_hits            = 3;
};

/// ByteTrack tracker: two-stage Hungarian association on IoU cost.
///
/// Stage 1 matches ALL existing tracks against high-confidence detections.
/// Stage 2 matches UNMATCHED tracks from Stage 1 against low-confidence
/// detections, recovering tracks through brief occlusion without CNN features.
///
/// Reuses KalmanBoxTracker (8D Kalman) and HungarianSolver (Munkres) from
/// kalman_tracker.h — no duplication.
class ByteTrackTracker final : public ITracker {
public:
    using Params = ByteTrackParams;

    explicit ByteTrackTracker(Params params = {});

    [[nodiscard]] TrackedObjectList update(const Detection2DList& detections) override;
    [[nodiscard]] std::string       name() const override;
    void                            reset() override;

    /// IoU between two bounding boxes (public for testing).
    [[nodiscard]] static double compute_iou(const Detection2D& a, const Detection2D& b);

    /// IoU cost matrix: rows = tracks (by index), cols = detections.
    /// Each entry is 1.0 - IoU.
    [[nodiscard]] std::vector<std::vector<double>> compute_iou_cost_matrix(
        const std::vector<size_t>& track_indices, const std::vector<Detection2D>& detections) const;

private:
    Params                        params_;
    std::vector<KalmanBoxTracker> tracks_;
    uint32_t                      next_id_ = 1;
};

}  // namespace drone::perception
