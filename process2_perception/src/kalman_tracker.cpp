// process2_perception/src/kalman_tracker.cpp
// KalmanBoxTracker + HungarianSolver (Munkres) implementation + tracker factory.
// Phase 1B (Issue #113). SORT removed in Issue #205 — ByteTrack supersedes it.
#include "perception/kalman_tracker.h"

#include "perception/bytetrack_tracker.h"
#include "util/config.h"
#include "util/config_keys.h"
#include "util/per_class_config.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace drone::perception {

// ═══════════════════════════════════════════════════════════
// KalmanBoxTracker
// ═══════════════════════════════════════════════════════════
KalmanBoxTracker::KalmanBoxTracker(const Detection2D& det, uint32_t id, MotionModel model)
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

    // Process noise — tuned by motion model (Epic #519).
    // CONSTANT_ACCELERATION uses higher velocity noise to accommodate
    // rapid speed changes (manoeuvring targets like drones/animals).
    const float vel_noise = (model == MotionModel::CONSTANT_ACCELERATION) ? 0.1f : 0.01f;
    Q_                    = StateMat::Identity() * 1.0f;
    Q_(4, 4)              = vel_noise;
    Q_(5, 5)              = vel_noise;
    Q_(6, 6)              = 0.0001f;
    Q_(7, 7)              = 0.0001f;

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
// Tracker factory  (ITracker)
// ═══════════════════════════════════════════════════════════
using TrackerResult = drone::util::Result<std::unique_ptr<ITracker>>;

TrackerResult create_tracker(const std::string&                    backend,
                             [[maybe_unused]] const drone::Config* cfg) {
    auto make_bytetrack = [&]() -> std::unique_ptr<ITracker> {
        ByteTrackTracker::Params params;
        if (cfg) {
            namespace tk               = drone::cfg_key::perception::tracker;
            params.high_conf_threshold = cfg->get<float>(tk::HIGH_CONF_THRESHOLD, 0.5f);
            params.low_conf_threshold  = cfg->get<float>(tk::LOW_CONF_THRESHOLD, 0.1f);
            params.max_iou_cost        = cfg->get<double>(tk::MAX_IOU_COST, 0.7);
            params.max_age             = cfg->get<uint32_t>(tk::MAX_AGE, 10);
            params.min_hits            = cfg->get<uint32_t>(tk::MIN_HITS, 3);
            // Per-class motion models (Epic #519).
            auto model_strings = drone::util::load_per_class<std::string>(
                *cfg, tk::PER_CLASS_MOTION_MODEL, "constant_velocity");
            for (uint8_t i = 0; i < drone::util::kPerClassCount; ++i) {
                params.motion_models[i] = motion_model_from_string(model_strings[i]);
            }
        }
        return std::make_unique<ByteTrackTracker>(params);
    };

    if (backend == "bytetrack") {
        return TrackerResult::ok(make_bytetrack());
    }

    // Unknown backend — warn and fall back to ByteTrack using the shared factory path.
    spdlog::warn("[tracker_factory] Unknown tracker backend '{}' — falling back to bytetrack",
                 backend);
    return TrackerResult::ok(make_bytetrack());
}

}  // namespace drone::perception
