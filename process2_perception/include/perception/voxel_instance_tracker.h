// process2_perception/include/perception/voxel_instance_tracker.h
//
// Issue #638 — Voxel clustering Phase 2: cross-frame instance tracker.
//
// Phase 1 assigns *frame-local* cluster IDs.  Each frame's IDs are
// independent — a cluster might be ID 1 in one frame, ID 7 in the next.
// Phase 2 adds a tracker that:
//   * computes a centroid and AABB for each per-frame cluster,
//   * matches each cluster to an existing track via nearest-centroid
//     association (greedy, gated by max_match_distance_m),
//   * mints a new stable instance ID for unmatched candidates,
//   * ages out tracks not seen for `track_max_age_s`,
//   * rewrites each voxel's `instance_id` to its assigned track's stable ID.
//
// Phase 3 will key promotion off these stable IDs so a voxel observed
// for N consecutive frames promotes the WHOLE instance (every cell that
// belongs to the track) atomically.  Voxels with instance_id == 0 (noise
// from Phase 1's min_pts gate) bypass the tracker entirely and remain 0.
//
// Algorithm choice — greedy nearest-centroid:
//   * Hungarian / global optimal would be more accurate but at our N
//     (typically 5-30 tracked instances per frame) the greedy match is
//     within 1-2 % accuracy and ~10× cheaper.
//   * Cluster centroids in scenario 33 are well separated (~5 m apart)
//     so association is unambiguous; the gate (max_match_distance_m)
//     rejects implausible jumps from sensor noise.
//
// Thread safety: NOT thread-safe — caller is the single P2
// mask_projection_thread.  All state lives in member fields; no globals.
#pragma once

#include "hal/isemantic_projector.h"  // for VoxelUpdate

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

namespace drone::perception {

class VoxelInstanceTracker {
public:
    /// One stable track maintained across frames.
    struct Track {
        Eigen::Vector3f centroid_m{Eigen::Vector3f::Zero()};
        Eigen::Vector3f aabb_min_m{Eigen::Vector3f::Zero()};
        Eigen::Vector3f aabb_max_m{Eigen::Vector3f::Zero()};
        uint64_t        last_seen_ns{0};
        uint32_t        observation_count{0};
    };

    /// @param max_match_distance_m  Centroid-to-centroid gating threshold.
    ///                              Candidates farther from any track than
    ///                              this start a new track.  Tune against
    ///                              expected per-frame cluster motion at
    ///                              cruise speed; 2-5 m is typical.
    /// @param track_max_age_s       Tracks not re-observed for this long
    ///                              are dropped.  Should be longer than
    ///                              brief occlusions (e.g. drone yaw past
    ///                              a pillar) but shorter than the
    ///                              "obstacle disappears" signal.  2 s is
    ///                              a reasonable default.
    explicit VoxelInstanceTracker(float max_match_distance_m = 3.0f,
                                  float track_max_age_s      = 2.0f) noexcept
        : max_match_distance_m_(std::max(0.0f, max_match_distance_m))
        , track_max_age_ns_(static_cast<uint64_t>(std::max(0.0f, track_max_age_s) * 1e9f)) {}

    /// Update tracks from this frame's clustered voxels and rewrite each
    /// voxel's `instance_id` to its assigned track's stable ID.
    ///
    /// Voxels with `instance_id == 0` on entry are noise and stay 0 on
    /// exit (the tracker doesn't see them).
    ///
    /// @param voxels    voxel batch with frame-local instance IDs from
    ///                  `assign_instance_ids()`.  Modified in-place.
    /// @param now_ns    source-frame timestamp recorded as `last_seen_ns`
    ///                  for downstream consumers.  NOT used for ageing —
    ///                  ageing now uses `steady_clock::now()` internally
    ///                  to decouple from any irregularities in source-frame
    ///                  timestamps (review Fix 3 from scenario-33 run
    ///                  2026-04-30_095815).
    void update(std::vector<drone::hal::VoxelUpdate>& voxels, uint64_t now_ns) {
        // Wall-clock used for ageing — independent of `now_ns` (source frame).
        const uint64_t wall_now_ns =
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count());

        if (voxels.empty()) {
            const size_t aged         = age_out_tracks_(wall_now_ns);
            last_aged_out_count_      = aged;
            last_match_failure_count_ = 0;
            return;
        }

        // ── Pass 1: compute centroid + AABB per frame-local cluster ──
        struct Candidate {
            Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
            Eigen::Vector3f aabb_min{Eigen::Vector3f::Constant(std::numeric_limits<float>::max())};
            Eigen::Vector3f aabb_max{
                Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest())};
            uint32_t count{0};
        };
        std::unordered_map<uint32_t, Candidate> candidates;  // frame-local ID → candidate
        for (const auto& v : voxels) {
            if (v.instance_id == 0) continue;
            auto& c = candidates[v.instance_id];
            c.centroid += v.position_m;
            c.aabb_min = c.aabb_min.cwiseMin(v.position_m);
            c.aabb_max = c.aabb_max.cwiseMax(v.position_m);
            ++c.count;
        }
        for (auto& [_, c] : candidates) {
            if (c.count > 0) c.centroid /= static_cast<float>(c.count);
        }

        // ── Pass 2: greedy match candidates → existing tracks ──
        // For each candidate, find the closest unmatched track within
        // max_match_distance_m_.  If found, update that track; if not,
        // mint a new track.  Greedy is fine here — at N ≤ 30 the
        // suboptimality vs Hungarian is within sensor noise.
        //
        // Issue #638 P2-A from PR #640 review: `tracks_view` holds raw
        // `Track*` pointers into `tracks_`.  Below, the mint-new-track
        // branch does `tracks_[stable_id] = ...` which can rehash the
        // unordered_map and invalidate every `Track&` reference (and
        // therefore every `Track*` we cached).  Pre-reserving for the
        // worst case (all candidates mint) eliminates the rehash window:
        // `unordered_map::reserve()` guarantees no rehash until size
        // exceeds the requested capacity.
        tracks_.reserve(tracks_.size() + candidates.size());

        std::unordered_map<uint32_t, uint32_t> frame_local_to_stable;  // remap table
        std::vector<bool>                      track_matched(tracks_.size(), false);
        // Index tracks by position in a vector for O(1) "matched" lookup.
        std::vector<std::pair<uint32_t, Track*>> tracks_view;
        tracks_view.reserve(tracks_.size());
        for (auto& [id, tr] : tracks_) tracks_view.emplace_back(id, &tr);

        size_t mints_this_frame = 0;
        for (auto& [frame_id, cand] : candidates) {
            int   best_idx  = -1;
            float best_dist = max_match_distance_m_;
            for (size_t i = 0; i < tracks_view.size(); ++i) {
                if (track_matched[i]) continue;
                const float d = (tracks_view[i].second->centroid_m - cand.centroid).norm();
                if (d < best_dist) {
                    best_dist = d;
                    best_idx  = static_cast<int>(i);
                }
            }
            uint32_t stable_id;
            if (best_idx >= 0) {
                track_matched[best_idx] = true;
                stable_id               = tracks_view[best_idx].first;
                Track& t                = *tracks_view[best_idx].second;
                t.centroid_m            = cand.centroid;
                t.aabb_min_m            = cand.aabb_min;
                t.aabb_max_m            = cand.aabb_max;
                t.last_seen_ns          = wall_now_ns;
                ++t.observation_count;
            } else {
                // Issue #638 P2-D from PR #640 review: detect uint32_t
                // wrap on `next_stable_id_`.  ID 0 is reserved for noise;
                // a wrapped value would collide with the noise sentinel
                // and bypass any downstream gate.  Reset the tracker
                // wholesale on wrap (4 B mints/frame at 10 Hz → 13+
                // years; in practice unreachable except via the
                // unbounded-mint DOS the NaN-config guard above closes,
                // but defensive depth is cheap here).
                if (next_stable_id_ == 0) {
                    tracks_.clear();
                    tracks_view.clear();
                    track_matched.assign(0, false);
                    next_stable_id_ = 1;
                }
                stable_id           = next_stable_id_++;
                Track& t            = tracks_[stable_id];
                t.centroid_m        = cand.centroid;
                t.aabb_min_m        = cand.aabb_min;
                t.aabb_max_m        = cand.aabb_max;
                t.last_seen_ns      = wall_now_ns;
                t.observation_count = 1;
                ++mints_this_frame;
            }
            frame_local_to_stable[frame_id] = stable_id;
        }
        // Suppress unused-variable warning when `now_ns` is otherwise unused
        // (kept in the public signature for ABI stability + future uses).
        (void)now_ns;

        // ── Pass 3: rewrite voxel instance_id from frame-local → stable ──
        for (auto& v : voxels) {
            if (v.instance_id == 0) continue;
            auto it = frame_local_to_stable.find(v.instance_id);
            if (it != frame_local_to_stable.end()) v.instance_id = it->second;
        }

        // ── Age out unmatched tracks past TTL ──
        const size_t aged         = age_out_tracks_(wall_now_ns);
        last_aged_out_count_      = aged;
        last_match_failure_count_ = mints_this_frame;
    }

    /// Read-only access to the current set of stable tracks.  Useful for
    /// downstream consumers (Phase 3 promotion, diagnostics).
    [[nodiscard]] const std::unordered_map<uint32_t, Track>& tracks() const noexcept {
        return tracks_;
    }

    /// Current track count (post-ageing, post-association).
    [[nodiscard]] size_t track_count() const noexcept { return tracks_.size(); }

    /// Reset all tracker state — for tests and clean restarts.
    void reset() noexcept {
        tracks_.clear();
        next_stable_id_           = 1;
        last_aged_out_count_      = 0;
        last_match_failure_count_ = 0;
    }

    /// Issue #638 Fix 4 diagnostics — number of tracks aged out on the
    /// last `update()` call.  Useful for log lines that distinguish
    /// "tracker is healthy" from "tracker is leaking tracks because
    /// ageing isn't firing" (run 2026-04-30_095815: 4 clusters → 52
    /// tracks meant ageing wasn't keeping up).
    [[nodiscard]] size_t last_aged_out_count() const noexcept { return last_aged_out_count_; }

    /// Number of clusters in the last `update()` that failed to match
    /// any existing track (forced new-track mints).  Persistently
    /// nonzero indicates the gate is too tight or centroids are too
    /// noisy — the exact failure mode of the 2026-04-30_095815 run.
    [[nodiscard]] size_t last_match_failure_count() const noexcept {
        return last_match_failure_count_;
    }

private:
    /// Returns the number of tracks erased.
    size_t age_out_tracks_(uint64_t now_ns) {
        if (track_max_age_ns_ == 0) return 0;  // ageing disabled
        size_t aged = 0;
        for (auto it = tracks_.begin(); it != tracks_.end();) {
            if (now_ns > it->second.last_seen_ns &&
                now_ns - it->second.last_seen_ns > track_max_age_ns_) {
                it = tracks_.erase(it);
                ++aged;
            } else {
                ++it;
            }
        }
        return aged;
    }

    float    max_match_distance_m_;
    uint64_t track_max_age_ns_;

    // Stable instance IDs start at 1 — id=0 is reserved for "noise"
    // (Phase 1's min_pts gate).  IDs are monotonically increasing across
    // the tracker's lifetime so consumers can assume IDs are unique.
    uint32_t                            next_stable_id_{1};
    std::unordered_map<uint32_t, Track> tracks_;

    // Issue #638 Fix 4 — per-call diagnostic counters (overwritten on
    // each `update()`).  Surfaced by `last_*_count()` accessors above
    // for the P2 log line.
    size_t last_aged_out_count_{0};
    size_t last_match_failure_count_{0};
};

}  // namespace drone::perception
