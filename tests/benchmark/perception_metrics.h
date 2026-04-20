// tests/benchmark/perception_metrics.h
//
// Metrics library for grading the perception pipeline (Epic #523, Issue #570).
//
// Consumes per-frame ground-truth + prediction lists and emits detection and
// tracking metrics: TP/FP/FN, per-class precision/recall/F1, per-class AP,
// confusion matrix, MOTA, MOTP, ID switches, fragmentations.
//
// Scope:
//   - Pure C++17, no ML deps, CPU only.
//   - Image-space 2D bounding boxes (x, y, w, h in pixels).
//   - Greedy confidence-ordered matching per class.
//   - PASCAL VOC 11-point interpolation for AP.
//
// Out of scope for this TU: 3D metrics (handled in a follow-up once we settle
// on a 3D ground-truth format), grid metrics (E8.2/E8.3), latency metrics
// (E8.2 separate profiler).

#pragma once

#include <cstdint>
#include <map>
#include <vector>

namespace drone::benchmark {

// ────────────────────────────────────────────────────────────────────────────
// Primitives
// ────────────────────────────────────────────────────────────────────────────

struct BBox2D {
    float x{0.0F};  // top-left x (pixels)
    float y{0.0F};  // top-left y (pixels)
    float w{0.0F};  // width  (pixels)
    float h{0.0F};  // height (pixels)

    [[nodiscard]] float area() const noexcept;
};

// Intersection-over-union of two axis-aligned bboxes.
// Returns 0 for zero-area boxes or disjoint boxes.
[[nodiscard]] float iou(const BBox2D& a, const BBox2D& b) noexcept;

// ────────────────────────────────────────────────────────────────────────────
// Per-frame ground truth + predictions
// ────────────────────────────────────────────────────────────────────────────

struct GroundTruthDetection {
    uint32_t class_id{0};
    BBox2D   bbox{};
    uint32_t gt_track_id{0};  // identity across frames (0 = unidentified, not used for matching)
};

struct PredictedDetection {
    uint32_t class_id{0};
    BBox2D   bbox{};
    float    confidence{0.0F};
    uint32_t pred_track_id{0};  // tracker output ID (0 = detection-only, no track)
};

struct FrameData {
    uint64_t                          timestamp_ns{0};
    std::vector<GroundTruthDetection> ground_truth{};
    std::vector<PredictedDetection>   predictions{};
};

// ────────────────────────────────────────────────────────────────────────────
// Detection metrics (per-class and global)
// ────────────────────────────────────────────────────────────────────────────

struct ClassMetrics {
    uint32_t tp{0};
    uint32_t fp{0};
    uint32_t fn{0};

    [[nodiscard]] double precision() const noexcept;  // tp / (tp + fp), 0 if tp+fp==0
    [[nodiscard]] double recall() const noexcept;     // tp / (tp + fn), 0 if tp+fn==0
    [[nodiscard]] double f1() const noexcept;         // harmonic mean, 0 if either is 0
};

struct DetectionMetrics {
    // Per-class counters (keyed by class_id).
    std::map<uint32_t, ClassMetrics> per_class{};

    // Per-class AP at the IoU threshold used when these metrics were computed
    // (PASCAL VOC 11-point interpolation).
    std::map<uint32_t, double> per_class_ap{};

    // Confusion matrix: confusion_matrix[gt_class][pred_class] = count.
    // Sized [num_classes + 1] x [num_classes + 1]; index num_classes is the
    // "background" / unmatched slot for FNs (gt) or FPs (pred).
    std::vector<std::vector<uint32_t>> confusion_matrix{};

    uint32_t num_classes{0};

    // Convenience aggregates computed from per_class.
    [[nodiscard]] uint32_t total_tp() const noexcept;
    [[nodiscard]] uint32_t total_fp() const noexcept;
    [[nodiscard]] uint32_t total_fn() const noexcept;
    [[nodiscard]] double   micro_precision() const noexcept;
    [[nodiscard]] double   micro_recall() const noexcept;
    [[nodiscard]] double   mean_ap() const noexcept;  // mAP across classes present in per_class_ap
};

// Compute detection metrics over a sequence of frames at a single IoU threshold.
//
// Matching: within each frame, for each class, predictions are sorted by
// confidence descending and greedily matched to the highest-IoU unmatched GT
// of the same class whose IoU ≥ `iou_threshold`. Unmatched predictions count
// as FP; unmatched GT count as FN.
//
// Confusion matrix: for each unmatched prediction we still look for the
// highest-IoU GT of *any* class above `iou_threshold` to record class
// confusion (misclassification). A prediction that has no overlap with any
// GT is recorded against the background row.
//
// `num_classes` is the number of real classes (0..num_classes-1); the
// confusion matrix uses `num_classes` as the background index.
[[nodiscard]] DetectionMetrics compute_detection_metrics(const std::vector<FrameData>& frames,
                                                         float iou_threshold, uint32_t num_classes);

// Same as compute_detection_metrics but evaluated at multiple IoU thresholds.
// Returns a map keyed by threshold.
[[nodiscard]] std::map<float, DetectionMetrics> compute_detection_metrics_multi_iou(
    const std::vector<FrameData>& frames, const std::vector<float>& iou_thresholds,
    uint32_t num_classes);

// ────────────────────────────────────────────────────────────────────────────
// Tracking metrics (multi-frame, MOTA/MOTP)
// ────────────────────────────────────────────────────────────────────────────

struct TrackingMetrics {
    uint32_t total_gt{0};        // Σ |GT| across frames
    uint32_t total_tp{0};        // matched (pred, gt) pairs
    uint32_t total_fp{0};        // unmatched predictions
    uint32_t total_fn{0};        // unmatched GT
    uint32_t id_switches{0};     // GT.id → pred.track_id changed between frames
    uint32_t fragmentations{0};  // GT trajectory lost-then-regained transitions
    double   sum_iou{0.0};       // Σ IoU over matched pairs (MOTP numerator)
    uint32_t num_matches{0};     // # matched pairs (MOTP denominator)

    // MOTA = 1 - (fn + fp + id_switches) / total_gt
    [[nodiscard]] double mota() const noexcept;

    // MOTP = sum_iou / num_matches  (IoU-form MOTP; CLEAR-MOT's original uses
    // distance error, but IoU is the standard for 2D bbox trackers).
    [[nodiscard]] double motp() const noexcept;
};

// Compute tracking metrics across a sequence of frames.
// Requires predictions to carry stable `pred_track_id` values and GT to carry
// stable `gt_track_id` values; zero IDs are treated as untracked and will not
// contribute to ID-switch / fragmentation counts.
[[nodiscard]] TrackingMetrics compute_tracking_metrics(const std::vector<FrameData>& frames,
                                                       float                         iou_threshold);

// ────────────────────────────────────────────────────────────────────────────
// Single-class AP (11-point interpolation) — exposed for testing and for
// callers that want to compute AP on a pre-filtered set.
// ────────────────────────────────────────────────────────────────────────────

// Flattened prediction used by compute_ap.
struct ScoredPrediction {
    uint64_t frame_index{0};
    float    confidence{0.0F};
    BBox2D   bbox{};
};

// Flattened GT used by compute_ap.
struct ScoredGroundTruth {
    uint64_t frame_index{0};
    BBox2D   bbox{};
};

// PASCAL VOC 11-point Average Precision.
[[nodiscard]] double compute_ap(std::vector<ScoredPrediction>  preds,
                                std::vector<ScoredGroundTruth> gts, float iou_threshold);

}  // namespace drone::benchmark
