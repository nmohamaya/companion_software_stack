// process2_perception/include/perception/kalman_tracker.h
// Kalman filter-based 2D bounding box tracker (SORT-style).
#pragma once
#include "perception/types.h"
#include <Eigen/Dense>
#include <vector>

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
    void predict(float dt = 1.0f / 30.0f);
    void update(const Detection2D& det);
    Detection2D predicted_bbox() const;
    Eigen::Vector2f velocity() const;

    uint32_t    track_id;
    ObjectClass class_id = ObjectClass::UNKNOWN;
    float       confidence = 0;
    uint32_t    age = 0;
    uint32_t    hits = 0;
    uint32_t    consecutive_misses = 0;

    bool is_confirmed() const { return hits >= 3; }
    bool is_stale()     const { return consecutive_misses > 10; }

private:
    StateVec x_;
    StateMat F_, P_, Q_;
    MeasMat  H_;
    Eigen::Matrix<float, MEAS_DIM, MEAS_DIM> R_;
};

/// Hungarian assignment solver for data association.
class HungarianSolver {
public:
    struct Result {
        std::vector<int> assignment;  // assignment[row] = col (-1 if unmatched)
        double total_cost = 0.0;
        std::vector<int> unmatched_rows;
        std::vector<int> unmatched_cols;
    };

    static Result solve(const std::vector<std::vector<double>>& cost,
                        double max_cost = 100.0);
};

/// Multi-object tracker using SORT algorithm.
class MultiObjectTracker {
public:
    TrackedObjectList update(const Detection2DList& detections);

private:
    std::vector<KalmanBoxTracker> tracks_;
    uint32_t next_id_ = 1;

    std::vector<std::vector<double>> compute_cost_matrix(
        const std::vector<Detection2D>& detections) const;
};

} // namespace drone::perception
