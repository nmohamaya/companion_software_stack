// tests/test_kalman_tracker.cpp
// Unit tests for KalmanBoxTracker and HungarianSolver (Munkres).
#include "perception/kalman_tracker.h"

#include <gtest/gtest.h>

using namespace drone::perception;

// ═══════════════════════════════════════════════════════════
// KalmanBoxTracker tests
// ═══════════════════════════════════════════════════════════

TEST(KalmanBoxTrackerTest, InitFromDetection) {
    Detection2D      det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
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
    Detection2D      det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    auto pred = tracker.predicted_bbox();
    // Center should be close to (125, 240), size (50, 80)
    EXPECT_NEAR(pred.x + pred.w / 2.0f, 125.0f, 1.0f);
    EXPECT_NEAR(pred.y + pred.h / 2.0f, 240.0f, 1.0f);
    EXPECT_NEAR(pred.w, 50.0f, 1.0f);
    EXPECT_NEAR(pred.h, 80.0f, 1.0f);
}

TEST(KalmanBoxTrackerTest, PredictIncreasesAge) {
    Detection2D      det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    EXPECT_EQ(tracker.age, 0u);
    tracker.predict();
    EXPECT_EQ(tracker.age, 1u);
    EXPECT_EQ(tracker.consecutive_misses, 1u);
}

TEST(KalmanBoxTrackerTest, UpdateResetsConsecutiveMisses) {
    Detection2D      det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
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
    Detection2D      det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);
    EXPECT_FALSE(tracker.is_confirmed());

    tracker.predict();
    tracker.update(det);
    EXPECT_FALSE(tracker.is_confirmed());  // hits=2

    tracker.predict();
    tracker.update(det);
    EXPECT_TRUE(tracker.is_confirmed());  // hits=3
}

TEST(KalmanBoxTrackerTest, StaleAfter10Misses) {
    Detection2D      det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    for (int i = 0; i < 10; ++i) {
        tracker.predict();
        EXPECT_FALSE(tracker.is_stale());
    }
    tracker.predict();  // 11th miss
    EXPECT_TRUE(tracker.is_stale());
}

TEST(KalmanBoxTrackerTest, VelocityInitiallyZero) {
    Detection2D      det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);

    auto vel = tracker.velocity();
    EXPECT_NEAR(vel.x(), 0.0f, 0.01f);
    EXPECT_NEAR(vel.y(), 0.0f, 0.01f);
}

// ═══════════════════════════════════════════════════════════
// HungarianSolver (O(n³) Munkres) tests
// ═══════════════════════════════════════════════════════════

TEST(HungarianSolverTest, EmptyCostMatrix) {
    std::vector<std::vector<double>> cost;
    auto                             result = HungarianSolver::solve(cost, 100.0);
    EXPECT_TRUE(result.assignment.empty());
    EXPECT_TRUE(result.unmatched_rows.empty());
    EXPECT_TRUE(result.unmatched_cols.empty());
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
}

TEST(HungarianSolverTest, SingleMatch) {
    std::vector<std::vector<double>> cost   = {{5.0}};
    auto                             result = HungarianSolver::solve(cost, 100.0);
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
    std::vector<std::vector<double>> cost   = {{50.0, 200.0}};
    auto                             result = HungarianSolver::solve(cost, 100.0);
    // Should match col 0 (cost 50 < 100), reject col 1
    EXPECT_EQ(result.assignment[0], 0);
    EXPECT_EQ(result.unmatched_cols.size(), 1u);
}

TEST(HungarianSolverTest, AllTooExpensive) {
    std::vector<std::vector<double>> cost   = {{200.0}};
    auto                             result = HungarianSolver::solve(cost, 100.0);
    EXPECT_EQ(result.assignment[0], -1);
    EXPECT_EQ(result.unmatched_rows.size(), 1u);
}

TEST(HungarianSolverTest, MoreRowsThanCols) {
    std::vector<std::vector<double>> cost = {
        {1.0},
        {2.0},
    };
    auto result = HungarianSolver::solve(cost, 100.0);
    EXPECT_EQ(result.assignment[0], 0);   // optimal: row 0 gets col 0 (cost 1)
    EXPECT_EQ(result.assignment[1], -1);  // row 1 unmatched
    EXPECT_EQ(result.unmatched_rows.size(), 1u);
}

TEST(HungarianSolverTest, TotalCostInitializedToZero) {
    std::vector<std::vector<double>> cost;
    auto                             result = HungarianSolver::solve(cost, 100.0);
    EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
}

// ═══════════════════════════════════════════════════════════
// Hungarian optimality tests (Munkres vs greedy)
// ═══════════════════════════════════════════════════════════

TEST(HungarianSolverTest, MunkresBeatsGreedy) {
    // A greedy row-scan would assign row 0→col 0 (cost 1), then row 1→col 1 (cost 100).
    // Optimal Hungarian: row 0→col 1 (cost 2), row 1→col 0 (cost 3), total 5 vs 101.
    std::vector<std::vector<double>> cost = {
        {1.0, 2.0},
        {3.0, 100.0},
    };
    auto result = HungarianSolver::solve(cost, 200.0);
    ASSERT_EQ(result.assignment.size(), 2u);
    EXPECT_EQ(result.assignment[0], 1);  // row 0 → col 1 (cost 2)
    EXPECT_EQ(result.assignment[1], 0);  // row 1 → col 0 (cost 3)
    EXPECT_DOUBLE_EQ(result.total_cost, 5.0);
}

TEST(HungarianSolverTest, MunkresLarger3x3) {
    // Classic assignment problem — optimal is NOT the row-greedy scan
    std::vector<std::vector<double>> cost = {
        {10, 5, 13},
        {3, 7, 8},
        {6, 9, 4},
    };
    auto result = HungarianSolver::solve(cost, 100.0);
    // Optimal: (0→1, 1→0, 2→2) → 5+3+4 = 12
    ASSERT_EQ(result.assignment.size(), 3u);
    EXPECT_DOUBLE_EQ(result.total_cost, 12.0);
    EXPECT_EQ(result.assignment[0], 1);
    EXPECT_EQ(result.assignment[1], 0);
    EXPECT_EQ(result.assignment[2], 2);
}

TEST(HungarianSolverTest, MoreColsThanRows) {
    // 2 rows × 3 cols
    std::vector<std::vector<double>> cost = {
        {5.0, 1.0, 8.0},
        {7.0, 9.0, 2.0},
    };
    auto result = HungarianSolver::solve(cost, 100.0);
    ASSERT_EQ(result.assignment.size(), 2u);
    EXPECT_EQ(result.assignment[0], 1);  // cost 1
    EXPECT_EQ(result.assignment[1], 2);  // cost 2
    EXPECT_DOUBLE_EQ(result.total_cost, 3.0);
    EXPECT_EQ(result.unmatched_cols.size(), 1u);  // col 0 unmatched
}

// ═══════════════════════════════════════════════════════════
// Motion model tests (Epic #519)
// ═══════════════════════════════════════════════════════════

TEST(KalmanBoxTrackerTest, MotionModelFromString) {
    EXPECT_EQ(motion_model_from_string("constant_velocity"), MotionModel::CONSTANT_VELOCITY);
    EXPECT_EQ(motion_model_from_string("constant_acceleration"),
              MotionModel::CONSTANT_ACCELERATION);
    EXPECT_EQ(motion_model_from_string("unknown_model"), MotionModel::CONSTANT_VELOCITY);
    EXPECT_EQ(motion_model_from_string(""), MotionModel::CONSTANT_VELOCITY);
}

TEST(KalmanBoxTrackerTest, ConstantAccelerationHigherVelocityNoise) {
    Detection2D det{100, 200, 50, 80, 0.9f, ObjectClass::DRONE, 0, 0};

    KalmanBoxTracker cv_tracker(det, 1, MotionModel::CONSTANT_VELOCITY);
    KalmanBoxTracker ca_tracker(det, 2, MotionModel::CONSTANT_ACCELERATION);

    // CA model uses 10x higher velocity process noise (0.1 vs 0.01).
    EXPECT_FLOAT_EQ(cv_tracker.process_noise_velocity(), 0.01f);
    EXPECT_FLOAT_EQ(ca_tracker.process_noise_velocity(), 0.1f);
    EXPECT_GT(ca_tracker.process_noise_velocity(), cv_tracker.process_noise_velocity());
}

TEST(KalmanBoxTrackerTest, DefaultMotionModelIsConstantVelocity) {
    Detection2D      det{100, 200, 50, 80, 0.9f, ObjectClass::PERSON, 0, 0};
    KalmanBoxTracker tracker(det, 1);
    // Default model is CONSTANT_VELOCITY — verify via process noise.
    EXPECT_FLOAT_EQ(tracker.process_noise_velocity(), 0.01f);
}
