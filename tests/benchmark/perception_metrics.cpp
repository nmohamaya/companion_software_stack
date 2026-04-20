// tests/benchmark/perception_metrics.cpp

#include "perception_metrics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace drone::benchmark {

namespace {

constexpr float kMinArea = 1e-6F;

// Eleven recall checkpoints used by PASCAL VOC 2007 AP.
constexpr std::array<double, 11> kVocRecallPoints = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                                                     0.6, 0.7, 0.8, 0.9, 1.0};

// Background index sentinel for confusion matrix rows/columns.
[[nodiscard]] std::size_t background_index(uint32_t num_classes) noexcept {
    return static_cast<std::size_t>(num_classes);
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// BBox2D / IoU
// ────────────────────────────────────────────────────────────────────────────

float BBox2D::area() const noexcept {
    if (w <= 0.0F || h <= 0.0F) {
        return 0.0F;
    }
    return w * h;
}

float iou(const BBox2D& a, const BBox2D& b) noexcept {
    const float a_area = a.area();
    const float b_area = b.area();
    if (a_area < kMinArea || b_area < kMinArea) {
        return 0.0F;
    }

    const float ix1 = std::max(a.x, b.x);
    const float iy1 = std::max(a.y, b.y);
    const float ix2 = std::min(a.x + a.w, b.x + b.w);
    const float iy2 = std::min(a.y + a.h, b.y + b.h);

    const float iw = ix2 - ix1;
    const float ih = iy2 - iy1;
    if (iw <= 0.0F || ih <= 0.0F) {
        return 0.0F;
    }

    const float inter = iw * ih;
    const float uni   = a_area + b_area - inter;
    return uni > 0.0F ? inter / uni : 0.0F;
}

// ────────────────────────────────────────────────────────────────────────────
// ClassMetrics
// ────────────────────────────────────────────────────────────────────────────

double ClassMetrics::precision() const noexcept {
    const uint32_t denom = tp + fp;
    return denom == 0 ? 0.0 : static_cast<double>(tp) / static_cast<double>(denom);
}

double ClassMetrics::recall() const noexcept {
    const uint32_t denom = tp + fn;
    return denom == 0 ? 0.0 : static_cast<double>(tp) / static_cast<double>(denom);
}

double ClassMetrics::f1() const noexcept {
    const double p = precision();
    const double r = recall();
    if (p <= 0.0 || r <= 0.0) {
        return 0.0;
    }
    return (2.0 * p * r) / (p + r);
}

// ────────────────────────────────────────────────────────────────────────────
// DetectionMetrics aggregates
// ────────────────────────────────────────────────────────────────────────────

uint32_t DetectionMetrics::total_tp() const noexcept {
    uint32_t sum = 0;
    for (const auto& [_, m] : per_class) {
        sum += m.tp;
    }
    return sum;
}

uint32_t DetectionMetrics::total_fp() const noexcept {
    uint32_t sum = 0;
    for (const auto& [_, m] : per_class) {
        sum += m.fp;
    }
    return sum;
}

uint32_t DetectionMetrics::total_fn() const noexcept {
    uint32_t sum = 0;
    for (const auto& [_, m] : per_class) {
        sum += m.fn;
    }
    return sum;
}

double DetectionMetrics::micro_precision() const noexcept {
    const uint32_t tp    = total_tp();
    const uint32_t fp    = total_fp();
    const uint32_t denom = tp + fp;
    return denom == 0 ? 0.0 : static_cast<double>(tp) / static_cast<double>(denom);
}

double DetectionMetrics::micro_recall() const noexcept {
    const uint32_t tp    = total_tp();
    const uint32_t fn    = total_fn();
    const uint32_t denom = tp + fn;
    return denom == 0 ? 0.0 : static_cast<double>(tp) / static_cast<double>(denom);
}

double DetectionMetrics::mean_ap() const noexcept {
    if (per_class_ap.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (const auto& [_, ap] : per_class_ap) {
        sum += ap;
    }
    return sum / static_cast<double>(per_class_ap.size());
}

// ────────────────────────────────────────────────────────────────────────────
// Greedy per-frame matching
// ────────────────────────────────────────────────────────────────────────────

namespace {

struct MatchResult {
    // pred_to_gt[i] = index of matched GT within the same frame, or -1 if unmatched.
    // Only considers matches for preds of the same class above the IoU threshold.
    std::vector<int32_t> pred_to_gt;
    std::vector<int32_t> gt_to_pred;
    std::vector<float>   pred_match_iou;  // IoU of the matched pair, 0 if unmatched
};

// Greedy confidence-ordered, class-filtered matching.
MatchResult match_frame(const FrameData& frame, float iou_threshold) {
    const std::size_t nP = frame.predictions.size();
    const std::size_t nG = frame.ground_truth.size();

    MatchResult m;
    m.pred_to_gt.assign(nP, -1);
    m.gt_to_pred.assign(nG, -1);
    m.pred_match_iou.assign(nP, 0.0F);

    if (nP == 0 || nG == 0) {
        return m;
    }

    // Predictions in descending confidence order.
    std::vector<std::size_t> order(nP);
    std::iota(order.begin(), order.end(), 0U);
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return frame.predictions[a].confidence > frame.predictions[b].confidence;
    });

    for (const std::size_t p_idx : order) {
        const PredictedDetection& pred     = frame.predictions[p_idx];
        float                     best_iou = iou_threshold;  // must strictly exceed (or equal) this
        int32_t                   best_gt  = -1;
        for (std::size_t g_idx = 0; g_idx < nG; ++g_idx) {
            if (m.gt_to_pred[g_idx] != -1) {
                continue;  // GT already matched
            }
            const GroundTruthDetection& gt = frame.ground_truth[g_idx];
            if (gt.class_id != pred.class_id) {
                continue;
            }
            const float iou_val = iou(pred.bbox, gt.bbox);
            if (iou_val >= best_iou) {
                best_iou = iou_val;
                best_gt  = static_cast<int32_t>(g_idx);
            }
        }
        if (best_gt >= 0) {
            m.pred_to_gt[p_idx]                             = best_gt;
            m.gt_to_pred[static_cast<std::size_t>(best_gt)] = static_cast<int32_t>(p_idx);
            m.pred_match_iou[p_idx]                         = best_iou;
        }
    }

    return m;
}

// For unmatched predictions, find the highest-IoU GT of *any* class above
// the threshold — used for confusion matrix misclassification rows.
int32_t best_gt_any_class(const FrameData& frame, const PredictedDetection& pred,
                          float iou_threshold) {
    float   best_iou = iou_threshold;
    int32_t best_gt  = -1;
    for (std::size_t g_idx = 0; g_idx < frame.ground_truth.size(); ++g_idx) {
        const float iou_val = iou(pred.bbox, frame.ground_truth[g_idx].bbox);
        if (iou_val >= best_iou) {
            best_iou = iou_val;
            best_gt  = static_cast<int32_t>(g_idx);
        }
    }
    return best_gt;
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// AP — PASCAL VOC 11-point
// ────────────────────────────────────────────────────────────────────────────

double compute_ap(std::vector<ScoredPrediction> preds, std::vector<ScoredGroundTruth> gts,
                  float iou_threshold) {
    const std::size_t num_gt = gts.size();
    if (num_gt == 0) {
        return 0.0;  // undefined; convention: 0 AP when no positives exist
    }
    if (preds.empty()) {
        return 0.0;
    }

    // Sort predictions by confidence descending.
    std::sort(preds.begin(), preds.end(), [](const ScoredPrediction& a, const ScoredPrediction& b) {
        return a.confidence > b.confidence;
    });

    // Group GT by frame for faster lookup.
    std::unordered_map<uint64_t, std::vector<std::size_t>> gt_by_frame;
    for (std::size_t i = 0; i < gts.size(); ++i) {
        gt_by_frame[gts[i].frame_index].push_back(i);
    }

    std::vector<bool> gt_matched(num_gt, false);

    const std::size_t     nP = preds.size();
    std::vector<uint32_t> tp_cum(nP, 0);
    std::vector<uint32_t> fp_cum(nP, 0);

    uint32_t tp_running = 0;
    uint32_t fp_running = 0;

    for (std::size_t i = 0; i < nP; ++i) {
        const ScoredPrediction& pred     = preds[i];
        auto                    it       = gt_by_frame.find(pred.frame_index);
        int32_t                 best_gt  = -1;
        float                   best_iou = iou_threshold;
        if (it != gt_by_frame.end()) {
            for (const std::size_t g_idx : it->second) {
                if (gt_matched[g_idx]) {
                    continue;
                }
                const float iou_val = iou(pred.bbox, gts[g_idx].bbox);
                if (iou_val >= best_iou) {
                    best_iou = iou_val;
                    best_gt  = static_cast<int32_t>(g_idx);
                }
            }
        }
        if (best_gt >= 0) {
            gt_matched[static_cast<std::size_t>(best_gt)] = true;
            ++tp_running;
        } else {
            ++fp_running;
        }
        tp_cum[i] = tp_running;
        fp_cum[i] = fp_running;
    }

    // Precision/recall curves.
    std::vector<double> recall_curve(nP);
    std::vector<double> precision_curve(nP);
    for (std::size_t i = 0; i < nP; ++i) {
        const uint32_t denom = tp_cum[i] + fp_cum[i];
        precision_curve[i] =
            denom == 0 ? 0.0 : static_cast<double>(tp_cum[i]) / static_cast<double>(denom);
        recall_curve[i] = static_cast<double>(tp_cum[i]) / static_cast<double>(num_gt);
    }

    // 11-point interpolated precision: for each recall checkpoint r, take
    // max precision over all i with recall_curve[i] >= r.
    double ap_sum = 0.0;
    for (const double r : kVocRecallPoints) {
        double p_max = 0.0;
        for (std::size_t i = 0; i < nP; ++i) {
            if (recall_curve[i] >= r && precision_curve[i] > p_max) {
                p_max = precision_curve[i];
            }
        }
        ap_sum += p_max;
    }
    return ap_sum / static_cast<double>(kVocRecallPoints.size());
}

// ────────────────────────────────────────────────────────────────────────────
// compute_detection_metrics
// ────────────────────────────────────────────────────────────────────────────

DetectionMetrics compute_detection_metrics(const std::vector<FrameData>& frames,
                                           float iou_threshold, uint32_t num_classes) {
    DetectionMetrics out;
    out.num_classes = num_classes;

    // Confusion matrix sized [num_classes+1]x[num_classes+1]; last row/col is
    // the background slot for unmatched FPs/FNs.
    const std::size_t dim = static_cast<std::size_t>(num_classes) + 1;
    out.confusion_matrix.assign(dim, std::vector<uint32_t>(dim, 0U));

    // Per-class TP/FP/FN.
    // Also collect global per-class preds/gts for AP computation.
    std::map<uint32_t, std::vector<ScoredPrediction>>  per_class_preds;
    std::map<uint32_t, std::vector<ScoredGroundTruth>> per_class_gts;

    const std::size_t bg = background_index(num_classes);

    for (std::size_t frame_i = 0; frame_i < frames.size(); ++frame_i) {
        const FrameData&  frame = frames[frame_i];
        const MatchResult m     = match_frame(frame, iou_threshold);

        // Count TPs, build confusion rows for matched pairs.
        for (std::size_t p_idx = 0; p_idx < frame.predictions.size(); ++p_idx) {
            const PredictedDetection& pred = frame.predictions[p_idx];
            if (m.pred_to_gt[p_idx] >= 0) {
                out.per_class[pred.class_id].tp += 1;
                if (pred.class_id < num_classes) {
                    out.confusion_matrix[pred.class_id][pred.class_id] += 1;
                }
            } else {
                out.per_class[pred.class_id].fp += 1;
                const int32_t gt_any = best_gt_any_class(frame, pred, iou_threshold);
                if (gt_any >= 0) {
                    const uint32_t gt_cls =
                        frame.ground_truth[static_cast<std::size_t>(gt_any)].class_id;
                    const std::size_t gt_row =
                        gt_cls < num_classes ? static_cast<std::size_t>(gt_cls) : bg;
                    const std::size_t pr_col =
                        pred.class_id < num_classes ? static_cast<std::size_t>(pred.class_id) : bg;
                    out.confusion_matrix[gt_row][pr_col] += 1;
                } else {
                    const std::size_t pr_col =
                        pred.class_id < num_classes ? static_cast<std::size_t>(pred.class_id) : bg;
                    out.confusion_matrix[bg][pr_col] += 1;
                }
            }

            per_class_preds[pred.class_id].push_back(
                ScoredPrediction{static_cast<uint64_t>(frame_i), pred.confidence, pred.bbox});
        }

        // Count FNs (unmatched GT) and their confusion-matrix contribution.
        for (std::size_t g_idx = 0; g_idx < frame.ground_truth.size(); ++g_idx) {
            const GroundTruthDetection& gt = frame.ground_truth[g_idx];
            if (m.gt_to_pred[g_idx] < 0) {
                out.per_class[gt.class_id].fn += 1;
                const std::size_t gt_row =
                    gt.class_id < num_classes ? static_cast<std::size_t>(gt.class_id) : bg;
                out.confusion_matrix[gt_row][bg] += 1;
            }

            per_class_gts[gt.class_id].push_back(
                ScoredGroundTruth{static_cast<uint64_t>(frame_i), gt.bbox});
        }
    }

    // Per-class AP.
    for (const auto& [cls, preds] : per_class_preds) {
        auto                           gt_it = per_class_gts.find(cls);
        std::vector<ScoredGroundTruth> gts =
            gt_it == per_class_gts.end() ? std::vector<ScoredGroundTruth>{} : gt_it->second;
        out.per_class_ap[cls] = compute_ap(preds, std::move(gts), iou_threshold);
    }
    // Also record AP=0 for classes that have GT but no predictions (so mean_ap is fair).
    for (const auto& [cls, gts] : per_class_gts) {
        if (out.per_class_ap.find(cls) == out.per_class_ap.end()) {
            out.per_class_ap[cls] = 0.0;
        }
    }

    return out;
}

std::map<float, DetectionMetrics> compute_detection_metrics_multi_iou(
    const std::vector<FrameData>& frames, const std::vector<float>& iou_thresholds,
    uint32_t num_classes) {
    std::map<float, DetectionMetrics> out;
    for (const float thr : iou_thresholds) {
        out[thr] = compute_detection_metrics(frames, thr, num_classes);
    }
    return out;
}

// ────────────────────────────────────────────────────────────────────────────
// Tracking metrics — MOTA / MOTP / ID switches / fragmentations
// ────────────────────────────────────────────────────────────────────────────

double TrackingMetrics::mota() const noexcept {
    if (total_gt == 0) {
        return 0.0;
    }
    const double errors = static_cast<double>(total_fn + total_fp + id_switches);
    return 1.0 - errors / static_cast<double>(total_gt);
}

double TrackingMetrics::motp() const noexcept {
    return num_matches == 0 ? 0.0 : sum_iou / static_cast<double>(num_matches);
}

TrackingMetrics compute_tracking_metrics(const std::vector<FrameData>& frames,
                                         float                         iou_threshold) {
    TrackingMetrics tm;

    // Per-GT-track-id state across frames:
    //   last_pred_id: pred_track_id matched to this GT in the previous frame (0 = none).
    //   was_tracked : this GT has been matched at some earlier frame.
    //   last_tracked_frame : most recent frame index where this GT was matched.
    struct GtState {
        uint32_t last_pred_id{0};
        bool     was_tracked{false};
        int64_t  last_tracked_frame{-1};
    };
    std::unordered_map<uint32_t, GtState> gt_state;

    for (std::size_t frame_i = 0; frame_i < frames.size(); ++frame_i) {
        const FrameData& frame = frames[frame_i];
        tm.total_gt += static_cast<uint32_t>(frame.ground_truth.size());

        const MatchResult m = match_frame(frame, iou_threshold);

        // Track which GT IDs appear in this frame so we can detect "present but
        // unmatched" for fragmentation.
        std::unordered_map<uint32_t, bool /*matched*/> gt_present_this_frame;

        for (std::size_t g_idx = 0; g_idx < frame.ground_truth.size(); ++g_idx) {
            const GroundTruthDetection& gt        = frame.ground_truth[g_idx];
            const int32_t               match_idx = m.gt_to_pred[g_idx];
            const bool                  matched   = match_idx >= 0;
            gt_present_this_frame[gt.gt_track_id] = matched;

            if (matched) {
                tm.total_tp += 1;
                tm.sum_iou +=
                    static_cast<double>(m.pred_match_iou[static_cast<std::size_t>(match_idx)]);
                tm.num_matches += 1;

                if (gt.gt_track_id != 0) {
                    const PredictedDetection& pred =
                        frame.predictions[static_cast<std::size_t>(match_idx)];
                    GtState& st = gt_state[gt.gt_track_id];

                    // ID-switch: matched in prior frame with a different pred_track_id
                    // (only counted when both prior and current have non-zero pred IDs).
                    if (st.last_pred_id != 0 && pred.pred_track_id != 0 &&
                        st.last_pred_id != pred.pred_track_id) {
                        tm.id_switches += 1;
                    }

                    // Fragmentation: previously tracked, then a gap of one or more
                    // frames where this GT was present but unmatched (or absent),
                    // then matched again. We detect the *transition back to tracked*.
                    if (st.was_tracked && st.last_tracked_frame >= 0 &&
                        static_cast<int64_t>(frame_i) - st.last_tracked_frame > 1) {
                        tm.fragmentations += 1;
                    }

                    st.last_pred_id       = pred.pred_track_id;
                    st.was_tracked        = true;
                    st.last_tracked_frame = static_cast<int64_t>(frame_i);
                }
            } else {
                tm.total_fn += 1;
                if (gt.gt_track_id != 0) {
                    // GT present but unmatched — clear the "live" pred ID so a
                    // re-acquisition next frame shows as fragmentation, not ID switch.
                    GtState& st     = gt_state[gt.gt_track_id];
                    st.last_pred_id = 0;
                }
            }
        }

        // Unmatched predictions → FPs.
        for (std::size_t p_idx = 0; p_idx < frame.predictions.size(); ++p_idx) {
            if (m.pred_to_gt[p_idx] < 0) {
                tm.total_fp += 1;
            }
        }
    }

    return tm;
}

}  // namespace drone::benchmark
