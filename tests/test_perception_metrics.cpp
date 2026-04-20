// tests/test_perception_metrics.cpp
//
// Unit tests for the perception metrics framework (Issue #570 / Epic #523).

#include "benchmark/perception_metrics.h"

#include <chrono>
#include <random>

#include <gtest/gtest.h>

namespace db = drone::benchmark;

namespace {

constexpr float kTol = 1e-5F;

db::BBox2D bbox(float x, float y, float w, float h) {
    return db::BBox2D{x, y, w, h};
}

db::GroundTruthDetection gt(uint32_t class_id, db::BBox2D b, uint32_t track_id = 0) {
    return db::GroundTruthDetection{class_id, b, track_id};
}

db::PredictedDetection pred(uint32_t class_id, db::BBox2D b, float conf, uint32_t track_id = 0) {
    return db::PredictedDetection{class_id, b, conf, track_id};
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// IoU primitives
// ────────────────────────────────────────────────────────────────────────────

TEST(BBox2D, AreaAndIdentityIoU) {
    const db::BBox2D a = bbox(10, 10, 20, 30);
    EXPECT_FLOAT_EQ(a.area(), 600.0F);
    EXPECT_NEAR(db::iou(a, a), 1.0F, kTol);
}

TEST(BBox2D, DisjointBoxesIoUZero) {
    const db::BBox2D a = bbox(0, 0, 10, 10);
    const db::BBox2D b = bbox(20, 20, 10, 10);
    EXPECT_FLOAT_EQ(db::iou(a, b), 0.0F);
}

TEST(BBox2D, ZeroAreaReturnsZero) {
    const db::BBox2D a = bbox(0, 0, 10, 10);
    const db::BBox2D z = bbox(5, 5, 0, 0);
    EXPECT_FLOAT_EQ(db::iou(a, z), 0.0F);
}

TEST(BBox2D, HalfOverlap) {
    // Two 10x10 boxes, a=[0,10], b=[5,15]. Intersection area = 5*10=50.
    // Union = 100+100-50 = 150. IoU = 50/150 = 0.3333...
    const db::BBox2D a = bbox(0, 0, 10, 10);
    const db::BBox2D b = bbox(5, 0, 10, 10);
    EXPECT_NEAR(db::iou(a, b), 50.0F / 150.0F, kTol);
}

// ────────────────────────────────────────────────────────────────────────────
// ClassMetrics basic math
// ────────────────────────────────────────────────────────────────────────────

TEST(ClassMetrics, PrecisionRecallF1) {
    db::ClassMetrics m{};
    m.tp = 8;
    m.fp = 2;
    m.fn = 4;
    EXPECT_NEAR(m.precision(), 8.0 / 10.0, 1e-9);
    EXPECT_NEAR(m.recall(), 8.0 / 12.0, 1e-9);
    const double p = m.precision();
    const double r = m.recall();
    EXPECT_NEAR(m.f1(), 2.0 * p * r / (p + r), 1e-9);
}

TEST(ClassMetrics, ZeroDenominatorsDoNotNaN) {
    db::ClassMetrics m{};
    EXPECT_DOUBLE_EQ(m.precision(), 0.0);
    EXPECT_DOUBLE_EQ(m.recall(), 0.0);
    EXPECT_DOUBLE_EQ(m.f1(), 0.0);
}

// ────────────────────────────────────────────────────────────────────────────
// Detection metrics — corner cases (universal AC)
// ────────────────────────────────────────────────────────────────────────────

TEST(DetectionMetrics, NoDetectionsNoGroundTruth) {
    std::vector<db::FrameData> frames(3);
    const auto                 out = db::compute_detection_metrics(frames, 0.5F, /*num_classes=*/3);
    EXPECT_EQ(out.total_tp(), 0U);
    EXPECT_EQ(out.total_fp(), 0U);
    EXPECT_EQ(out.total_fn(), 0U);
    EXPECT_DOUBLE_EQ(out.micro_precision(), 0.0);
    EXPECT_DOUBLE_EQ(out.micro_recall(), 0.0);
    EXPECT_DOUBLE_EQ(out.mean_ap(), 0.0);
}

TEST(DetectionMetrics, DetectionsButNoGroundTruthAllFP) {
    db::FrameData f;
    f.predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F));
    f.predictions.push_back(pred(1, bbox(20, 20, 10, 10), 0.8F));
    const auto out = db::compute_detection_metrics({f}, 0.5F, /*num_classes=*/2);
    EXPECT_EQ(out.total_tp(), 0U);
    EXPECT_EQ(out.total_fp(), 2U);
    EXPECT_EQ(out.total_fn(), 0U);
    EXPECT_DOUBLE_EQ(out.micro_precision(), 0.0);
}

TEST(DetectionMetrics, GroundTruthButNoDetectionsAllFN) {
    db::FrameData f;
    f.ground_truth.push_back(gt(0, bbox(0, 0, 10, 10)));
    f.ground_truth.push_back(gt(1, bbox(20, 20, 10, 10)));
    const auto out = db::compute_detection_metrics({f}, 0.5F, /*num_classes=*/2);
    EXPECT_EQ(out.total_tp(), 0U);
    EXPECT_EQ(out.total_fp(), 0U);
    EXPECT_EQ(out.total_fn(), 2U);
    EXPECT_DOUBLE_EQ(out.micro_recall(), 0.0);
}

TEST(DetectionMetrics, PerfectOverlapAllTP) {
    db::FrameData f;
    f.ground_truth.push_back(gt(0, bbox(0, 0, 10, 10)));
    f.ground_truth.push_back(gt(0, bbox(20, 20, 10, 10)));
    f.predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F));
    f.predictions.push_back(pred(0, bbox(20, 20, 10, 10), 0.8F));
    const auto out = db::compute_detection_metrics({f}, 0.5F, /*num_classes=*/1);
    EXPECT_EQ(out.total_tp(), 2U);
    EXPECT_EQ(out.total_fp(), 0U);
    EXPECT_EQ(out.total_fn(), 0U);
    EXPECT_DOUBLE_EQ(out.micro_precision(), 1.0);
    EXPECT_DOUBLE_EQ(out.micro_recall(), 1.0);
    EXPECT_NEAR(out.per_class_ap.at(0), 1.0, 1e-9);
}

TEST(DetectionMetrics, ClassMismatchIsFPAndFN) {
    // GT says class 0, prediction says class 1 — even with perfect spatial overlap,
    // the prediction is FP for class 1 and GT is FN for class 0.
    db::FrameData f;
    f.ground_truth.push_back(gt(0, bbox(0, 0, 10, 10)));
    f.predictions.push_back(pred(1, bbox(0, 0, 10, 10), 0.9F));
    const auto out = db::compute_detection_metrics({f}, 0.5F, /*num_classes=*/2);
    EXPECT_EQ(out.per_class.at(0).tp, 0U);
    EXPECT_EQ(out.per_class.at(0).fn, 1U);
    EXPECT_EQ(out.per_class.at(1).tp, 0U);
    EXPECT_EQ(out.per_class.at(1).fp, 1U);
    // Confusion matrix should record [gt_cls=0][pred_cls=1] += 1
    EXPECT_EQ(out.confusion_matrix[0][1], 1U);
}

TEST(DetectionMetrics, BelowIoUThresholdIsFPAndFN) {
    db::FrameData f;
    f.ground_truth.push_back(gt(0, bbox(0, 0, 10, 10)));
    // 5x10 overlap → intersection 50, union 10*10 + 10*10 - 50 = 150 → IoU ≈ 0.333
    f.predictions.push_back(pred(0, bbox(5, 0, 10, 10), 0.9F));
    const auto at_05 = db::compute_detection_metrics({f}, 0.5F, /*num_classes=*/1);
    EXPECT_EQ(at_05.total_tp(), 0U);
    EXPECT_EQ(at_05.total_fp(), 1U);
    EXPECT_EQ(at_05.total_fn(), 1U);
    const auto at_03 = db::compute_detection_metrics({f}, 0.3F, /*num_classes=*/1);
    EXPECT_EQ(at_03.total_tp(), 1U);
    EXPECT_EQ(at_03.total_fp(), 0U);
    EXPECT_EQ(at_03.total_fn(), 0U);
}

TEST(DetectionMetrics, GreedyMatchesHighestConfidenceFirst) {
    // Two preds overlap the same GT. The higher-confidence one should take the TP,
    // the lower-confidence one should be an FP.
    db::FrameData f;
    f.ground_truth.push_back(gt(0, bbox(0, 0, 10, 10)));
    f.predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.6F));  // lower conf, perfect overlap
    f.predictions.push_back(pred(0, bbox(1, 1, 10, 10), 0.9F));  // higher conf, slight offset
    const auto out = db::compute_detection_metrics({f}, 0.5F, /*num_classes=*/1);
    EXPECT_EQ(out.total_tp(), 1U);
    EXPECT_EQ(out.total_fp(), 1U);
    EXPECT_EQ(out.total_fn(), 0U);
}

TEST(DetectionMetrics, ConfusionMatrixDimensions) {
    db::FrameData f;
    f.ground_truth.push_back(gt(0, bbox(0, 0, 10, 10)));
    const auto out = db::compute_detection_metrics({f}, 0.5F, /*num_classes=*/3);
    ASSERT_EQ(out.confusion_matrix.size(), 4U);  // 3 classes + background
    for (const auto& row : out.confusion_matrix) {
        ASSERT_EQ(row.size(), 4U);
    }
    // Unmatched GT of class 0 → [0][bg=3] += 1
    EXPECT_EQ(out.confusion_matrix[0][3], 1U);
}

TEST(DetectionMetrics, MultiIoUProducesMultipleResults) {
    db::FrameData f;
    f.ground_truth.push_back(gt(0, bbox(0, 0, 10, 10)));
    f.predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F));
    const auto out = db::compute_detection_metrics_multi_iou({f}, {0.5F, 0.75F, 0.9F}, 1);
    ASSERT_EQ(out.size(), 3U);
    for (const auto& [thr, m] : out) {
        EXPECT_EQ(m.total_tp(), 1U) << "threshold=" << thr;
    }
}

// ────────────────────────────────────────────────────────────────────────────
// AP
// ────────────────────────────────────────────────────────────────────────────

TEST(AP, EmptyPredictionsOrGT) {
    EXPECT_DOUBLE_EQ(db::compute_ap({}, {}, 0.5F), 0.0);
    std::vector<db::ScoredPrediction> preds = {{0, 0.9F, bbox(0, 0, 10, 10)}};
    EXPECT_DOUBLE_EQ(db::compute_ap(preds, {}, 0.5F), 0.0);
    std::vector<db::ScoredGroundTruth> gts = {{0, bbox(0, 0, 10, 10)}};
    EXPECT_DOUBLE_EQ(db::compute_ap({}, gts, 0.5F), 0.0);
}

TEST(AP, PerfectRecallPerfectPrecision) {
    std::vector<db::ScoredPrediction> preds = {
        {0, 0.9F, bbox(0, 0, 10, 10)},
        {1, 0.8F, bbox(100, 100, 10, 10)},
    };
    std::vector<db::ScoredGroundTruth> gts = {
        {0, bbox(0, 0, 10, 10)},
        {1, bbox(100, 100, 10, 10)},
    };
    EXPECT_NEAR(db::compute_ap(preds, gts, 0.5F), 1.0, 1e-9);
}

// ────────────────────────────────────────────────────────────────────────────
// Tracking metrics — MOTA / MOTP / ID switches / fragmentations
// ────────────────────────────────────────────────────────────────────────────

TEST(Tracking, EmptyFramesZeroMetrics) {
    const auto tm = db::compute_tracking_metrics({}, 0.5F);
    EXPECT_EQ(tm.total_gt, 0U);
    EXPECT_DOUBLE_EQ(tm.mota(), 0.0);
    EXPECT_DOUBLE_EQ(tm.motp(), 0.0);
}

TEST(Tracking, PerfectTrackMotaOneMotpOne) {
    std::vector<db::FrameData> frames(5);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        frames[i].ground_truth.push_back(gt(0, bbox(0, 0, 10, 10), /*gt_id=*/1));
        frames[i].predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F, /*pred_id=*/7));
    }
    const auto tm = db::compute_tracking_metrics(frames, 0.5F);
    EXPECT_EQ(tm.total_gt, 5U);
    EXPECT_EQ(tm.total_tp, 5U);
    EXPECT_EQ(tm.total_fp, 0U);
    EXPECT_EQ(tm.total_fn, 0U);
    EXPECT_EQ(tm.id_switches, 0U);
    EXPECT_EQ(tm.fragmentations, 0U);
    EXPECT_NEAR(tm.mota(), 1.0, 1e-9);
    EXPECT_NEAR(tm.motp(), 1.0, 1e-9);
}

TEST(Tracking, IdSwitchDetected) {
    std::vector<db::FrameData> frames(2);
    frames[0].ground_truth.push_back(gt(0, bbox(0, 0, 10, 10), 1));
    frames[0].predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F, /*pred_id=*/7));
    frames[1].ground_truth.push_back(gt(0, bbox(0, 0, 10, 10), 1));
    frames[1].predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F, /*pred_id=*/8));  // swap
    const auto tm = db::compute_tracking_metrics(frames, 0.5F);
    EXPECT_EQ(tm.id_switches, 1U);
    EXPECT_EQ(tm.total_tp, 2U);
    EXPECT_EQ(tm.total_fn, 0U);
    EXPECT_NEAR(tm.mota(), 1.0 - 1.0 / 2.0, 1e-9);  // one error in 2 GT
}

TEST(Tracking, FragmentationDetected) {
    // GT tracked at frame 0, missing match at frame 1, tracked again at frame 2.
    std::vector<db::FrameData> frames(3);
    frames[0].ground_truth.push_back(gt(0, bbox(0, 0, 10, 10), 1));
    frames[0].predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F, /*pred_id=*/7));
    // frame 1: GT still there, no prediction (tracker lost it)
    frames[1].ground_truth.push_back(gt(0, bbox(0, 0, 10, 10), 1));
    // frame 2: re-acquired
    frames[2].ground_truth.push_back(gt(0, bbox(0, 0, 10, 10), 1));
    frames[2].predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F, /*pred_id=*/7));
    const auto tm = db::compute_tracking_metrics(frames, 0.5F);
    EXPECT_EQ(tm.fragmentations, 1U);
    EXPECT_EQ(tm.id_switches, 0U);
    EXPECT_EQ(tm.total_fn, 1U);  // frame 1 GT was unmatched
}

TEST(Tracking, UnmatchedPredIsFP) {
    db::FrameData f;
    f.predictions.push_back(pred(0, bbox(0, 0, 10, 10), 0.9F, /*pred_id=*/7));
    const auto tm = db::compute_tracking_metrics({f}, 0.5F);
    EXPECT_EQ(tm.total_fp, 1U);
    EXPECT_EQ(tm.total_gt, 0U);
    // MOTA with total_gt=0 is defined as 0 here (not -inf).
    EXPECT_DOUBLE_EQ(tm.mota(), 0.0);
}

// ────────────────────────────────────────────────────────────────────────────
// Performance — 1000 detections × 1000 GT must complete in < 100 ms.
// ────────────────────────────────────────────────────────────────────────────

TEST(Performance, LargeFrameUnder100ms) {
    constexpr std::size_t N = 1000;
    db::FrameData         f;
    f.ground_truth.reserve(N);
    f.predictions.reserve(N);

    std::mt19937                          rng(42);
    std::uniform_real_distribution<float> pos(0.0F, 1920.0F);
    std::uniform_real_distribution<float> sz(8.0F, 64.0F);
    std::uniform_int_distribution<int>    cls(0, 9);
    std::uniform_real_distribution<float> conf(0.2F, 0.99F);

    // Build GT and pred side by side; half the preds are random (likely FP),
    // half are perturbed GTs (likely TP).
    for (std::size_t i = 0; i < N; ++i) {
        const float x = pos(rng);
        const float y = pos(rng);
        const float w = sz(rng);
        const float h = sz(rng);
        f.ground_truth.push_back(gt(static_cast<uint32_t>(cls(rng)), bbox(x, y, w, h),
                                    /*gt_id=*/static_cast<uint32_t>(i + 1)));
        if (i % 2 == 0) {
            f.predictions.push_back(pred(f.ground_truth.back().class_id,
                                         bbox(x + 1.0F, y + 1.0F, w, h), conf(rng),
                                         /*pred_id=*/static_cast<uint32_t>(i + 1)));
        } else {
            f.predictions.push_back(pred(static_cast<uint32_t>(cls(rng)),
                                         bbox(pos(rng), pos(rng), sz(rng), sz(rng)), conf(rng),
                                         /*pred_id=*/static_cast<uint32_t>(N + i + 1)));
        }
    }

    const auto t0  = std::chrono::steady_clock::now();
    const auto out = db::compute_detection_metrics({f}, 0.5F, /*num_classes=*/10);
    const auto t1  = std::chrono::steady_clock::now();
    const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    // Sanity: we built N GT and N predictions.
    EXPECT_EQ(out.total_tp() + out.total_fn(), N);
    EXPECT_EQ(out.total_tp() + out.total_fp(), N);
    EXPECT_LT(ms, 100) << "Metrics took " << ms << "ms (target <100ms)";
}
