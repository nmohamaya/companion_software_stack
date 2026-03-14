// process2_perception/src/bytetrack_tracker.cpp
// ByteTrack two-stage association implementation.
// Issue #163.
#include "perception/bytetrack_tracker.h"

#include <algorithm>
#include <cmath>

namespace drone::perception {

ByteTrackTracker::ByteTrackTracker(Params params) : params_(params) {}

std::string ByteTrackTracker::name() const {
    return "bytetrack";
}

void ByteTrackTracker::reset() {
    tracks_.clear();
    next_id_ = 1;
}

// ═══════════════════════════════════════════════════════════
// IoU computation
// ═══════════════════════════════════════════════════════════

double ByteTrackTracker::compute_iou(const Detection2D& a, const Detection2D& b) {
    // Bounding boxes are (x, y, w, h) where (x,y) is the top-left corner.
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.w, b.x + b.w);
    float y2 = std::min(a.y + a.h, b.y + b.h);

    float inter_w = std::max(0.0f, x2 - x1);
    float inter_h = std::max(0.0f, y2 - y1);
    float inter   = inter_w * inter_h;

    float area_a = a.w * a.h;
    float area_b = b.w * b.h;
    float uni    = area_a + area_b - inter;

    if (uni <= 0.0f) return 0.0;
    return static_cast<double>(inter / uni);
}

// ═══════════════════════════════════════════════════════════
// IoU cost matrix
// ═══════════════════════════════════════════════════════════

std::vector<std::vector<double>> ByteTrackTracker::compute_iou_cost_matrix(
    const std::vector<size_t>& track_indices, const std::vector<Detection2D>& detections) const {
    std::vector<std::vector<double>> cost(track_indices.size(),
                                          std::vector<double>(detections.size(), 1.0));

    for (size_t r = 0; r < track_indices.size(); ++r) {
        Detection2D pred = tracks_[track_indices[r]].predicted_bbox();
        for (size_t c = 0; c < detections.size(); ++c) {
            cost[r][c] = 1.0 - compute_iou(pred, detections[c]);
        }
    }
    return cost;
}

// ═══════════════════════════════════════════════════════════
// Two-stage update
// ═══════════════════════════════════════════════════════════

TrackedObjectList ByteTrackTracker::update(const Detection2DList& det_list) {
    // ── Step 1: Predict all existing tracks ─────────────────
    for (auto& track : tracks_) {
        track.predict();
    }

    // ── Step 2: Split detections by confidence ──────────────
    std::vector<Detection2D> high_dets;
    std::vector<Detection2D> low_dets;

    for (const auto& det : det_list.detections) {
        if (det.confidence >= params_.high_conf_threshold) {
            high_dets.push_back(det);
        } else if (det.confidence >= params_.low_conf_threshold) {
            low_dets.push_back(det);
        }
        // Below low_conf_threshold → discarded
    }

    // Build full track index list
    std::vector<size_t> all_track_indices(tracks_.size());
    for (size_t i = 0; i < tracks_.size(); ++i) {
        all_track_indices[i] = i;
    }

    // Track which tracks got matched (by index into tracks_)
    std::vector<bool> track_matched(tracks_.size(), false);

    // ── Stage 1: ALL tracks vs high-conf detections ─────────
    std::vector<bool> high_det_matched(high_dets.size(), false);

    if (!tracks_.empty() && !high_dets.empty()) {
        auto cost   = compute_iou_cost_matrix(all_track_indices, high_dets);
        auto result = HungarianSolver::solve(cost, params_.max_iou_cost);

        for (size_t r = 0; r < tracks_.size(); ++r) {
            int d = result.assignment[r];
            if (d >= 0) {
                tracks_[r].update(high_dets[d]);
                track_matched[r]    = true;
                high_det_matched[d] = true;
            }
        }
    }

    // ── Stage 2: UNMATCHED tracks vs low-conf detections ────
    if (!low_dets.empty()) {
        std::vector<size_t> unmatched_track_indices;
        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (!track_matched[i]) {
                unmatched_track_indices.push_back(i);
            }
        }

        if (!unmatched_track_indices.empty()) {
            auto cost   = compute_iou_cost_matrix(unmatched_track_indices, low_dets);
            auto result = HungarianSolver::solve(cost, params_.max_iou_cost);

            for (size_t r = 0; r < unmatched_track_indices.size(); ++r) {
                int d = result.assignment[r];
                if (d >= 0) {
                    size_t ti = unmatched_track_indices[r];
                    tracks_[ti].update(low_dets[d]);
                    track_matched[ti] = true;
                }
            }
        }
    }

    // ── Step 3: Create new tracks from unmatched HIGH-conf ──
    // Low-conf detections never create new tracks.
    for (size_t d = 0; d < high_dets.size(); ++d) {
        if (!high_det_matched[d]) {
            tracks_.emplace_back(high_dets[d], next_id_++);
        }
    }

    // ── Step 4: Prune stale tracks ──────────────────────────
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const KalmanBoxTracker& t) {
                                     return t.consecutive_misses > params_.max_age;
                                 }),
                  tracks_.end());

    // ── Step 5: Build output (confirmed tracks only) ────────
    TrackedObjectList output;
    output.timestamp_ns   = det_list.timestamp_ns;
    output.frame_sequence = det_list.frame_sequence;

    for (const auto& track : tracks_) {
        if (track.hits < params_.min_hits) continue;

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

}  // namespace drone::perception
