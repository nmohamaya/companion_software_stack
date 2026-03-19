// process2_perception/include/perception/kalman_tracker.h
// Kalman filter-based 2D bounding box tracker (SORT-style).
// Implements ITracker interface (Phase 1B, Issue #113).
#pragma once
#include "perception/itracker.h"
#include "perception/types.h"

#include <vector>

#include <Eigen/Dense>

namespace drone::perception {

class KalmanBoxTracker {
public:
    static constexpr int STATE_DIM = 8;  // [x, y, w, h, vx, vy, vw, vh]
    static constexpr int MEAS_DIM  = 4;  // [x, y, w, h]

    using StateVec = Eigen::Matrix<float, STATE_DIM, 1>;
    using StateMat = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;
    using MeasVec  = Eigen::Matrix<float, MEAS_DIM, 1>;
    using MeasMat  = Eigen::Matrix<float, MEAS_DIM, STATE_DIM>;

    KalmanBoxTracker(const Detection2D& initial_det, uint32_t id);
    void                          predict(float dt = 1.0f / 30.0f);
    void                          update(const Detection2D& det);
    [[nodiscard]] Detection2D     predicted_bbox() const;
    [[nodiscard]] Eigen::Vector2f velocity() const;

    uint32_t    track_id;
    ObjectClass class_id           = ObjectClass::UNKNOWN;
    float       confidence         = 0;
    uint32_t    age                = 0;
    uint32_t    hits               = 0;
    uint32_t    consecutive_misses = 0;

    [[nodiscard]] bool is_confirmed() const { return hits >= 3; }
    [[nodiscard]] bool is_stale() const { return consecutive_misses > 10; }

private:
    StateVec x_ = StateVec::Zero();
    StateMat F_ = StateMat::Zero(), P_ = StateMat::Zero(), Q_ = StateMat::Zero();
    MeasMat  H_                                 = MeasMat::Zero();
    Eigen::Matrix<float, MEAS_DIM, MEAS_DIM> R_ = Eigen::Matrix<float, MEAS_DIM, MEAS_DIM>::Zero();
};

/// Hungarian assignment solver for data association.
class HungarianSolver {
public:
    struct Result {
        std::vector<int> assignment;  // assignment[row] = col (-1 if unmatched)
        double           total_cost = 0.0;
        std::vector<int> unmatched_rows;
        std::vector<int> unmatched_cols;
    };

    static Result solve(const std::vector<std::vector<double>>& cost, double max_cost = 100.0);
};

/// Multi-object tracker using SORT algorithm (Kalman + Hungarian).
/// Implements ITracker for factory-based construction.
class SortTracker : public ITracker {
public:
    [[nodiscard]] TrackedObjectList update(const Detection2DList& detections) override;
    [[nodiscard]] std::string       name() const override;
    void                            reset() override;

private:
    std::vector<KalmanBoxTracker> tracks_;
    uint32_t                      next_id_ = 1;

    std::vector<std::vector<double>> compute_cost_matrix(
        const std::vector<Detection2D>& detections) const;
};

/// Backward-compatible alias.
using MultiObjectTracker = SortTracker;

}  // namespace drone::perception
