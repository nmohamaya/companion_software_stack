// process2_perception/src/kalman_tracker.cpp
// KalmanBoxTracker + HungarianSolver + MultiObjectTracker implementation.
#include "perception/kalman_tracker.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// KalmanBoxTracker
// ═══════════════════════════════════════════════════════════
KalmanBoxTracker::KalmanBoxTracker(const Detection2D& det, uint32_t id)
    : track_id(id), class_id(det.class_id), confidence(det.confidence),
      age(0), hits(1), consecutive_misses(0)
{
    // State: [x, y, w, h, vx, vy, vw, vh]
    x_ = StateVec::Zero();
    x_(0) = det.x + det.w / 2.0f;  // center x
    x_(1) = det.y + det.h / 2.0f;  // center y
    x_(2) = det.w;
    x_(3) = det.h;

    // State transition matrix (constant velocity model)
    F_ = StateMat::Identity();

    // Measurement matrix
    H_ = MeasMat::Zero();
    H_(0, 0) = 1; H_(1, 1) = 1; H_(2, 2) = 1; H_(3, 3) = 1;

    // State covariance
    P_ = StateMat::Identity() * 10.0f;
    P_(4, 4) = 1000.0f; P_(5, 5) = 1000.0f;
    P_(6, 6) = 1000.0f; P_(7, 7) = 1000.0f;

    // Process noise
    Q_ = StateMat::Identity() * 1.0f;
    Q_(4, 4) = 0.01f; Q_(5, 5) = 0.01f;
    Q_(6, 6) = 0.0001f; Q_(7, 7) = 0.0001f;

    // Measurement noise
    R_ = Eigen::Matrix<float, MEAS_DIM, MEAS_DIM>::Identity() * 1.0f;
}

void KalmanBoxTracker::predict(float dt) {
    F_ = StateMat::Identity();
    F_(0, 4) = dt; F_(1, 5) = dt;
    F_(2, 6) = dt; F_(3, 7) = dt;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
    ++age;
    ++consecutive_misses;
}

void KalmanBoxTracker::update(const Detection2D& det) {
    MeasVec z;
    z << det.x + det.w / 2.0f, det.y + det.h / 2.0f, det.w, det.h;

    MeasVec y = z - H_ * x_;
    auto S = H_ * P_ * H_.transpose() + R_;
    auto K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (StateMat::Identity() - K * H_) * P_;

    ++hits;
    consecutive_misses = 0;
    class_id = det.class_id;
    confidence = det.confidence;
}

Detection2D KalmanBoxTracker::predicted_bbox() const {
    Detection2D d;
    d.w = x_(2);
    d.h = x_(3);
    d.x = x_(0) - d.w / 2.0f;
    d.y = x_(1) - d.h / 2.0f;
    d.confidence = confidence;
    d.class_id = class_id;
    return d;
}

Eigen::Vector2f KalmanBoxTracker::velocity() const {
    return {x_(4), x_(5)};
}

// ═══════════════════════════════════════════════════════════
// HungarianSolver (greedy nearest neighbor for simplicity)
// ═══════════════════════════════════════════════════════════
HungarianSolver::Result HungarianSolver::solve(
    const std::vector<std::vector<double>>& cost, double max_cost)
{
    Result result;
    int rows = static_cast<int>(cost.size());
    int cols = rows > 0 ? static_cast<int>(cost[0].size()) : 0;

    result.assignment.resize(rows, -1);
    std::vector<bool> col_used(cols, false);

    // Greedy nearest-neighbor matching
    for (int r = 0; r < rows; ++r) {
        double best = max_cost;
        int best_c = -1;
        for (int c = 0; c < cols; ++c) {
            if (!col_used[c] && cost[r][c] < best) {
                best = cost[r][c];
                best_c = c;
            }
        }
        if (best_c >= 0) {
            result.assignment[r] = best_c;
            col_used[best_c] = true;
            result.total_cost += best;
        }
    }

    for (int r = 0; r < rows; ++r) {
        if (result.assignment[r] < 0)
            result.unmatched_rows.push_back(r);
    }
    for (int c = 0; c < cols; ++c) {
        if (!col_used[c])
            result.unmatched_cols.push_back(c);
    }
    return result;
}

// ═══════════════════════════════════════════════════════════
// MultiObjectTracker (SORT algorithm)
// ═══════════════════════════════════════════════════════════
std::vector<std::vector<double>> MultiObjectTracker::compute_cost_matrix(
    const std::vector<Detection2D>& detections) const
{
    std::vector<std::vector<double>> cost(tracks_.size(),
        std::vector<double>(detections.size(), 0.0));

    for (size_t t = 0; t < tracks_.size(); ++t) {
        auto pred = tracks_[t].predicted_bbox();
        auto pred_center = pred.center();
        for (size_t d = 0; d < detections.size(); ++d) {
            auto det_center = detections[d].center();
            double dist = (pred_center - det_center).norm();
            cost[t][d] = dist;
        }
    }
    return cost;
}

TrackedObjectList MultiObjectTracker::update(const Detection2DList& det_list) {
    // Step 1: Predict all existing tracks
    for (auto& track : tracks_) {
        track.predict();
    }

    // Step 2: Associate detections to tracks
    auto cost = compute_cost_matrix(det_list.detections);
    auto result = HungarianSolver::solve(cost, 100.0);

    // Step 3: Update matched tracks
    for (size_t t = 0; t < tracks_.size(); ++t) {
        int d = result.assignment[t];
        if (d >= 0) {
            tracks_[t].update(det_list.detections[d]);
        }
    }

    // Step 4: Create new tracks for unmatched detections
    // When there are no existing tracks, all detections are unmatched
    if (tracks_.empty()) {
        for (size_t d = 0; d < det_list.detections.size(); ++d) {
            tracks_.emplace_back(det_list.detections[d], next_id_++);
        }
    } else {
        for (int d : result.unmatched_cols) {
            tracks_.emplace_back(det_list.detections[d], next_id_++);
        }
    }

    // Step 5: Remove stale tracks
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [](const KalmanBoxTracker& t) { return t.is_stale(); }),
        tracks_.end());

    // Step 6: Build output
    TrackedObjectList output;
    output.timestamp_ns = det_list.timestamp_ns;
    output.frame_sequence = det_list.frame_sequence;

    for (const auto& track : tracks_) {
        if (!track.is_confirmed()) continue;

        TrackedObject obj;
        obj.track_id   = track.track_id;
        obj.class_id   = track.class_id;
        obj.confidence = track.confidence;
        auto pred = track.predicted_bbox();
        obj.position_2d = pred.center();
        obj.velocity_2d = track.velocity();
        obj.age    = track.age;
        obj.hits   = track.hits;
        obj.misses = track.consecutive_misses;
        obj.timestamp_ns = det_list.timestamp_ns;
        obj.state  = TrackedObject::State::CONFIRMED;
        output.objects.push_back(obj);
    }
    return output;
}

} // namespace drone::perception
