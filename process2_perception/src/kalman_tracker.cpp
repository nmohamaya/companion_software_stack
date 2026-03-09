// process2_perception/src/kalman_tracker.cpp
// KalmanBoxTracker + HungarianSolver (Munkres) + SortTracker implementation.
// Phase 1B (Issue #113): proper O(n³) Hungarian, ITracker interface.
#include "perception/kalman_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// KalmanBoxTracker
// ═══════════════════════════════════════════════════════════
KalmanBoxTracker::KalmanBoxTracker(const Detection2D& det, uint32_t id)
    : track_id(id)
    , class_id(det.class_id)
    , confidence(det.confidence)
    , age(0)
    , hits(1)
    , consecutive_misses(0) {
    // State: [x, y, w, h, vx, vy, vw, vh]
    x_    = StateVec::Zero();
    x_(0) = det.x + det.w / 2.0f;  // center x
    x_(1) = det.y + det.h / 2.0f;  // center y
    x_(2) = det.w;
    x_(3) = det.h;

    // State transition matrix (constant velocity model)
    F_ = StateMat::Identity();

    // Measurement matrix
    H_       = MeasMat::Zero();
    H_(0, 0) = 1;
    H_(1, 1) = 1;
    H_(2, 2) = 1;
    H_(3, 3) = 1;

    // State covariance
    P_       = StateMat::Identity() * 10.0f;
    P_(4, 4) = 1000.0f;
    P_(5, 5) = 1000.0f;
    P_(6, 6) = 1000.0f;
    P_(7, 7) = 1000.0f;

    // Process noise
    Q_       = StateMat::Identity() * 1.0f;
    Q_(4, 4) = 0.01f;
    Q_(5, 5) = 0.01f;
    Q_(6, 6) = 0.0001f;
    Q_(7, 7) = 0.0001f;

    // Measurement noise
    R_ = Eigen::Matrix<float, MEAS_DIM, MEAS_DIM>::Identity() * 1.0f;
}

void KalmanBoxTracker::predict(float dt) {
    F_       = StateMat::Identity();
    F_(0, 4) = dt;
    F_(1, 5) = dt;
    F_(2, 6) = dt;
    F_(3, 7) = dt;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
    ++age;
    ++consecutive_misses;
}

void KalmanBoxTracker::update(const Detection2D& det) {
    MeasVec z;
    z << det.x + det.w / 2.0f, det.y + det.h / 2.0f, det.w, det.h;

    MeasVec y = z - H_ * x_;
    auto    S = H_ * P_ * H_.transpose() + R_;
    auto    K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (StateMat::Identity() - K * H_) * P_;

    ++hits;
    consecutive_misses = 0;
    class_id           = det.class_id;
    confidence         = det.confidence;
}

Detection2D KalmanBoxTracker::predicted_bbox() const {
    Detection2D d;
    d.w          = x_(2);
    d.h          = x_(3);
    d.x          = x_(0) - d.w / 2.0f;
    d.y          = x_(1) - d.h / 2.0f;
    d.confidence = confidence;
    d.class_id   = class_id;
    return d;
}

Eigen::Vector2f KalmanBoxTracker::velocity() const {
    return {x_(4), x_(5)};
}

// ═══════════════════════════════════════════════════════════
// HungarianSolver — O(n³) Kuhn-Munkres algorithm
// Replaces the previous greedy nearest-neighbor (Phase 1B).
// ═══════════════════════════════════════════════════════════
HungarianSolver::Result HungarianSolver::solve(const std::vector<std::vector<double>>& cost,
                                               double                                  max_cost) {
    Result result;
    int    rows = static_cast<int>(cost.size());
    int    cols = rows > 0 ? static_cast<int>(cost[0].size()) : 0;

    // Validate that all rows have the same number of columns
    for (int r = 0; r < rows; ++r) {
        if (static_cast<int>(cost[r].size()) != cols) {
            throw std::invalid_argument(
                "HungarianSolver::solve: ragged cost matrix — row 0 has " + std::to_string(cols) +
                " cols but row " + std::to_string(r) + " has " + std::to_string(cost[r].size()));
        }
    }

    if (rows == 0 || cols == 0) {
        result.assignment.resize(rows, -1);
        for (int r = 0; r < rows; ++r) result.unmatched_rows.push_back(r);
        for (int c = 0; c < cols; ++c) result.unmatched_cols.push_back(c);
        return result;
    }

    // Pad to square matrix (n×n) with max_cost fill for dummy entries
    int n = std::max(rows, cols);

    // u[i], v[j]: dual variables (potentials) for rows and cols (1-indexed)
    std::vector<double> u(n + 1, 0.0);
    std::vector<double> v(n + 1, 0.0);
    // p[j]: row assigned to column j (1-indexed, 0 = unassigned)
    std::vector<int> p(n + 1, 0);
    // way[j]: predecessor column on the augmenting path
    std::vector<int> way(n + 1, 0);

    // Process each row
    for (int i = 1; i <= n; ++i) {
        // Start augmenting path from virtual column 0
        p[0]                   = i;
        int                 j0 = 0;  // current column in path (starts at virtual)
        std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
        std::vector<bool>   used(n + 1, false);

        do {
            used[j0]     = true;
            int    i0    = p[j0];
            double delta = std::numeric_limits<double>::infinity();
            int    j1    = 0;

            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                // Cost for (i0-1, j-1) — use max_cost for padded entries
                double c   = (i0 <= rows && j <= cols) ? cost[i0 - 1][j - 1] : max_cost;
                double cur = c - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j]  = j0;
                }
                if (minv[j] < delta) {
                    delta = cur < minv[j] ? cur : minv[j];
                    j1    = j;
                }
            }

            // Update potentials
            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] != 0);

        // Unwind augmenting path
        do {
            int j1 = way[j0];
            p[j0]  = p[j1];
            j0     = j1;
        } while (j0 != 0);
    }

    // Extract assignment: p[j] = row assigned to column j (1-indexed)
    // Convert to: assignment[row] = col
    result.assignment.resize(rows, -1);
    std::vector<bool> col_used(cols, false);

    for (int j = 1; j <= cols; ++j) {
        if (p[j] >= 1 && p[j] <= rows) {
            int r = p[j] - 1;
            int c = j - 1;
            if (cost[r][c] < max_cost) {
                result.assignment[r] = c;
                col_used[c]          = true;
                result.total_cost += cost[r][c];
            }
        }
    }

    for (int r = 0; r < rows; ++r) {
        if (result.assignment[r] < 0) result.unmatched_rows.push_back(r);
    }
    for (int c = 0; c < cols; ++c) {
        if (!col_used[c]) result.unmatched_cols.push_back(c);
    }
    return result;
}

// ═══════════════════════════════════════════════════════════
// SortTracker (SORT algorithm) — implements ITracker
// ═══════════════════════════════════════════════════════════
std::string SortTracker::name() const {
    return "sort";
}

void SortTracker::reset() {
    tracks_.clear();
    next_id_ = 1;
}

std::vector<std::vector<double>> SortTracker::compute_cost_matrix(
    const std::vector<Detection2D>& detections) const {
    std::vector<std::vector<double>> cost(tracks_.size(),
                                          std::vector<double>(detections.size(), 0.0));

    for (size_t t = 0; t < tracks_.size(); ++t) {
        auto pred        = tracks_[t].predicted_bbox();
        auto pred_center = pred.center();
        for (size_t d = 0; d < detections.size(); ++d) {
            auto   det_center = detections[d].center();
            double dist       = (pred_center - det_center).norm();
            cost[t][d]        = dist;
        }
    }
    return cost;
}

TrackedObjectList SortTracker::update(const Detection2DList& det_list) {
    // Step 1: Predict all existing tracks
    for (auto& track : tracks_) {
        track.predict();
    }

    // Step 2: Associate detections to tracks
    auto cost   = compute_cost_matrix(det_list.detections);
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
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [](const KalmanBoxTracker& t) { return t.is_stale(); }),
                  tracks_.end());

    // Step 6: Build output
    TrackedObjectList output;
    output.timestamp_ns   = det_list.timestamp_ns;
    output.frame_sequence = det_list.frame_sequence;

    for (const auto& track : tracks_) {
        if (!track.is_confirmed()) continue;

        TrackedObject obj;
        obj.track_id     = track.track_id;
        obj.class_id     = track.class_id;
        obj.confidence   = track.confidence;
        auto pred        = track.predicted_bbox();
        obj.position_2d  = pred.center();
        obj.velocity_2d  = track.velocity();
        obj.bbox_w       = pred.w;
        obj.bbox_h       = pred.h;
        obj.age          = track.age;
        obj.hits         = track.hits;
        obj.misses       = track.consecutive_misses;
        obj.timestamp_ns = det_list.timestamp_ns;
        obj.state        = TrackedObject::State::CONFIRMED;
        output.objects.push_back(obj);
    }
    return output;
}

// ═══════════════════════════════════════════════════════════
// Tracker factory  (ITracker)
// ═══════════════════════════════════════════════════════════
std::unique_ptr<ITracker> create_tracker(const std::string&                    backend,
                                         [[maybe_unused]] const drone::Config* cfg) {
    if (backend == "sort") {
        return std::make_unique<SortTracker>();
    }
    throw std::invalid_argument("Unknown tracker backend: " + backend);
}

}  // namespace drone::perception
