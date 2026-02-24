// tests/test_kalman_tracker.cpp
// Unit tests for KalmanBoxTracker, HungarianSolver, MultiObjectTracker.
#include <gtest/gtest.h>
#include "perception/kalman_tracker.h"

using namespace drone::perception;

// ═══════════════════════════════════════════════════════════
// KalmanBoxTracker tests
// ═══════════════════════════════════════════════════════════

TEST(KalmanBoxTrackerTest, InitFromDetection) {
    Detection2D det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    EXPECT_EQ(tracker.track_id, 1u);
    EXPECT_EQ(tracker.class_id, ObjectClass::PERSON);
    EXPECT_FLOAT_EQ(tracker.confidence, 0.9f);
    EXPECT_EQ(tracker.hits, 1u);
    EXPECT_EQ(tracker.consecutive_misses, 0u);
    EXPECT_FALSE(tracker.is_confirmed());  // needs 3 hits
    EXPECT_FALSE(tracker.is_stale());
}

TEST(KalmanBoxTrackerTest, PredictedBbox) {
    Detection2D det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    auto pred = tracker.predicted_bbox();
    // Center should be close to (125, 240), size (50, 80)
    EXPECT_NEAR(pred.x + pred.w / 2.0f, 125.0f, 1.0f);
    EXPECT_NEAR(pred.y + pred.h / 2.0f, 240.0f, 1.0f);
    EXPECT_NEAR(pred.w, 50.0f, 1.0f);
    EXPECT_NEAR(pred.h, 80.0f, 1.0f);
}

TEST(KalmanBoxTrackerTest, PredictIncreasesAge) {
    Detection2D det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    EXPECT_EQ(tracker.age, 0u);
    tracker.predict();
    EXPECT_EQ(tracker.age, 1u);
    EXPECT_EQ(tracker.consecutive_misses, 1u);
}

TEST(KalmanBoxTrackerTest, UpdateResetsConsecutiveMisses) {
    Detection2D det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    tracker.predict();
    tracker.predict();
    EXPECT_EQ(tracker.consecutive_misses, 2u);

    Detection2D det2{105, 205, 50, 80, 0.85f, ObjectClass::PERSON, 0, 0};
    tracker.update(det2);
    EXPECT_EQ(tracker.consecutive_misses, 0u);
    EXPECT_EQ(tracker.hits, 2u);
}

TEST(KalmanBoxTrackerTest, ConfirmedAfter3Hits) {
    Detection2D det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);
    EXPECT_FALSE(tracker.is_confirmed());

    tracker.predict();
    tracker.update(det);
    EXPECT_FALSE(tracker.is_confirmed());  // hits=2

    tracker.predict();
    tracker.update(det);
    EXPECT_TRUE(tracker.is_confirmed());   // hits=3
}

TEST(KalmanBoxTrackerTest, StaleAfter10Misses) {
    Detection2D det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    for (int i = 0; i < 10; ++i) {
        tracker.predict();
        EXPECT_FALSE(tracker.is_stale());
    }
    tracker.predict();  // 11th miss
    EXPECT_TRUE(tracker.is_stale());
}

TEST(KalmanBoxTrackerTest, VelocityInitiallyZero) {
    Detection2D det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    auto vel = tracker.velocity();
    EXPECT_NEAR(vel.x(), 0.0f, 0.01f);
    EXPECT_NEAR(vel.y(), 0.0f, 0.01f);
}

// ═══════════════════════════════════════════════════════════
// HungarianSolver (greedy) tests
// ═══════════════════════════════════════════════════════════

TEST(HungarianSolverTest, EmptyCostMatrix) {
    std::vector<std::vector<double>> cost;
    auto result = HungarianSolver::solve(cost, 100.0);
    EXPECT_TRUE(result.assignment.empty());
    EXPECT_TRUE(result.unmatched_rows.empty());
    EXPECT_TRUE(result.unmatched_cols.empty());
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
}

TEST(HungarianSolverTest, SingleMatch) {
    std::vector<std::vector<double>> cost = {{5.0}};
    auto result = HungarianSolver::solve(cost, 100.0);
    ASSERT_EQ(result.assignment.size(), 1u);
    EXPECT_EQ(result.assignment[0], 0);
    EXPECT_DOUBLE_EQ(result.total_cost, 5.0);
}

TEST(HungarianSolverTest, PerfectAssignment) {
    // 3×3 diagonal-dominant cost → expect identity assignment
    std::vector<std::vector<double>> cost = {
        {1.0, 10.0, 10.0},
        {10.0, 2.0, 10.0},
        {10.0, 10.0, 3.0},
    };
    auto result = HungarianSolver::solve(cost, 100.0);
    EXPECT_EQ(result.assignment[0], 0);
    EXPECT_EQ(result.assignment[1], 1);
    EXPECT_EQ(result.assignment[2], 2);
    EXPECT_DOUBLE_EQ(result.total_cost, 6.0);
}

TEST(HungarianSolverTest, MaxCostRejectsExpensivePair) {
    std::vector<std::vector<double>> cost = {{50.0, 200.0}};
    auto result = HungarianSolver::solve(cost, 100.0);
    // Should match col 0 (cost 50 < 100), reject col 1
    EXPECT_EQ(result.assignment[0], 0);
    EXPECT_EQ(result.unmatched_cols.size(), 1u);
}

TEST(HungarianSolverTest, AllTooExpensive) {
    std::vector<std::vector<double>> cost = {{200.0}};
    auto result = HungarianSolver::solve(cost, 100.0);
    EXPECT_EQ(result.assignment[0], -1);
    EXPECT_EQ(result.unmatched_rows.size(), 1u);
}

TEST(HungarianSolverTest, MoreRowsThanCols) {
    std::vector<std::vector<double>> cost = {
        {1.0},
        {2.0},
    };
    auto result = HungarianSolver::solve(cost, 100.0);
    EXPECT_EQ(result.assignment[0], 0);  // greedy: row 0 gets col 0
    EXPECT_EQ(result.assignment[1], -1); // row 1 unmatched
    EXPECT_EQ(result.unmatched_rows.size(), 1u);
}

TEST(HungarianSolverTest, TotalCostInitializedToZero) {
    std::vector<std::vector<double>> cost;
    auto result = HungarianSolver::solve(cost, 100.0);
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
}

// ═══════════════════════════════════════════════════════════
// MultiObjectTracker tests
// ═══════════════════════════════════════════════════════════

TEST(MultiObjectTrackerTest, EmptyDetections) {
    MultiObjectTracker tracker;
    Detection2DList empty;
    auto result = tracker.update(empty);
    EXPECT_TRUE(result.objects.empty());
}

TEST(MultiObjectTrackerTest, SingleDetectionBecomesTrack) {
    MultiObjectTracker tracker;

    // Feed the same detection 3 times — tracker should confirm after 3 hits
    for (int i = 0; i < 3; ++i) {
        Detection2DList det_list;
        det_list.timestamp_ns = static_cast<uint64_t>(i) * 33000000;
        det_list.frame_sequence = static_cast<uint64_t>(i);
        det_list.detections.push_back(
            {100, 200, 50, 80, 0.9f, ObjectClass::PERSON,
             det_list.timestamp_ns, det_list.frame_sequence});

        auto result = tracker.update(det_list);
        if (i < 2) {
            // Not yet confirmed (needs >= 3 hits)
            EXPECT_TRUE(result.objects.empty()) << "Frame " << i;
        } else {
            // Should be confirmed now
            ASSERT_EQ(result.objects.size(), 1u) << "Frame " << i;
            EXPECT_EQ(result.objects[0].class_id, ObjectClass::PERSON);
        }
    }
}

TEST(MultiObjectTrackerTest, StaleTracksRemoved) {
    MultiObjectTracker tracker;

    // Create a track
    Detection2DList det_list;
    det_list.detections.push_back(
        {100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0});

    tracker.update(det_list);

    // Now send empty detections for 11+ frames — track should be pruned
    Detection2DList empty;
    for (int i = 0; i < 15; ++i) {
        tracker.update(empty);
    }

    auto result = tracker.update(empty);
    EXPECT_TRUE(result.objects.empty());
}
