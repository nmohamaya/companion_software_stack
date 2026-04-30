// process2_perception/include/perception/voxel_obstacle_snapshot.h
//
// Issue #645 — Voxel-derived obstacles surfaced to ObstacleAvoider3D.
//
// Hand-off type from PATH A's mask_projection_thread (producer) to the
// fusion_thread (consumer).  The fusion thread already publishes the
// `DetectedObjectList` that the local avoider subscribes to; this snapshot
// lets fusion append GEOMETRIC_OBSTACLE entries synthesised from the
// VoxelInstanceTracker's stable cross-frame tracks so the avoider can repel
// from non-COCO obstacles (cubes, pillars, walls) that YOLO never detects.
//
// Coordinate frame: world (NED-derived from SLAM pose), same frame the
// camera+radar fusion publishes.  No transform is needed downstream.
//
// Thread safety: this struct is the payload of a `drone::TripleBuffer`,
// which gives us a lock-free latest-value handoff.  Mutex-protected
// observability is forbidden on flight-critical threads (CLAUDE.md), so we
// stay lock-free — the producer (mask_projection_thread, ~10 Hz) writes
// after each `tracker->update()`, and the consumer (fusion_thread, 30 Hz)
// reads the latest snapshot before each publish.
#pragma once

#include "ipc/ipc_types.h"

#include <algorithm>
#include <cstdint>

namespace drone::perception {

/// Maximum number of voxel-derived obstacles surfaced per snapshot.
/// Matches typical scenario complexity (≤30 tracked instances) with margin.
/// Independent of `MAX_DETECTED_OBJECTS=64`; the consumer caps to whichever
/// space remains in the published `DetectedObjectList`.
inline constexpr uint32_t kMaxVoxelObstacleEntries = 64;

/// One voxel-derived obstacle — a flat copy of the relevant fields from
/// `VoxelInstanceTracker::Track`.  Trivially copyable so the whole snapshot
/// can live inside a `TripleBuffer<VoxelObstacleSnapshot>`.
struct VoxelObstacleEntry {
    uint32_t stable_id{0};
    float    centroid_x{0.0f};  // world (m)
    float    centroid_y{0.0f};
    float    centroid_z{0.0f};
    float    aabb_min_x{0.0f};
    float    aabb_min_y{0.0f};
    float    aabb_min_z{0.0f};
    float    aabb_max_x{0.0f};
    float    aabb_max_y{0.0f};
    float    aabb_max_z{0.0f};
    uint64_t last_seen_ns{0};       // wall-clock (steady_clock ns)
    uint32_t observation_count{0};  // total Phase 2 re-associations
};

struct VoxelObstacleSnapshot {
    uint64_t           timestamp_ns{0};  // source-frame capture time
    uint32_t           num_entries{0};
    VoxelObstacleEntry entries[kMaxVoxelObstacleEntries]{};
};

// Trivially copyable so it can sit in TripleBuffer<>.
static_assert(sizeof(VoxelObstacleSnapshot) > 0, "snapshot must have non-zero size");

/// Track-id high-bit prefix used to mark synthetic GEOMETRIC_OBSTACLE entries
/// derived from voxel-cluster tracks (vs. fusion's camera/radar tracks).
/// Consumers can OR-test the high two bits to identify the source.
inline constexpr uint32_t kVoxelTrackIdPrefix = 0xC0000000u;

/// Diagnostic counters returned by `append_voxel_obstacles_to_list`.
/// Useful for log lines and unit tests.
struct VoxelObstacleAppendStats {
    uint32_t appended{0};
    uint32_t skipped_obs{0};
    uint32_t skipped_age{0};
    uint32_t skipped_full{0};  // capacity exhausted on output list
};

/// Append voxel-derived obstacles from `snap` to `out` as
/// `GEOMETRIC_OBSTACLE` `DetectedObject` entries, gated on
/// `min_observations` and `max_age_ns` (relative to `now_ns`).  Only entries
/// up to `MAX_DETECTED_OBJECTS` survive — the rest count toward
/// `skipped_full`.
///
/// Stateless and pure: easy to unit-test without spinning up the full
/// fusion thread.  Used internally by `fusion_thread` and by
/// `tests/test_voxel_obstacle_surface.cpp`.
inline VoxelObstacleAppendStats append_voxel_obstacles_to_list(const VoxelObstacleSnapshot&    snap,
                                                               drone::ipc::DetectedObjectList& out,
                                                               uint64_t now_ns,
                                                               uint32_t min_observations,
                                                               uint64_t max_age_ns) {
    VoxelObstacleAppendStats stats{};
    for (uint32_t i = 0; i < snap.num_entries; ++i) {
        if (out.num_objects >= drone::ipc::MAX_DETECTED_OBJECTS) {
            // Remaining entries can't fit — count them toward skipped_full
            // so callers can detect saturation.
            stats.skipped_full += (snap.num_entries - i);
            break;
        }
        const auto& e = snap.entries[i];
        if (e.observation_count < min_observations) {
            ++stats.skipped_obs;
            continue;
        }
        if (now_ns > e.last_seen_ns && now_ns - e.last_seen_ns > max_age_ns) {
            ++stats.skipped_age;
            continue;
        }

        auto& dst    = out.objects[out.num_objects];
        dst.track_id = kVoxelTrackIdPrefix | e.stable_id;
        dst.class_id = drone::ipc::ObjectClass::GEOMETRIC_OBSTACLE;
        // Confidence ramp: ≥ min_obs → 0.6, ≥ 10 → 1.0 plateau.
        const float obs_norm   = std::clamp(static_cast<float>(e.observation_count) / 10.0f, 0.0f,
                                            1.0f);
        dst.confidence         = 0.6f + 0.4f * obs_norm;
        dst.position_x         = e.centroid_x;
        dst.position_y         = e.centroid_y;
        dst.position_z         = e.centroid_z;
        dst.velocity_x         = 0.0f;  // voxel tracker doesn't estimate velocity
        dst.velocity_y         = 0.0f;
        dst.velocity_z         = 0.0f;
        dst.heading            = 0.0f;
        dst.bbox_x             = 0.0f;
        dst.bbox_y             = 0.0f;
        dst.bbox_w             = 0.0f;
        dst.bbox_h             = 0.0f;
        dst.has_camera         = false;
        dst.has_radar          = false;
        const float dx         = e.aabb_max_x - e.aabb_min_x;
        const float dy         = e.aabb_max_y - e.aabb_min_y;
        const float dz         = e.aabb_max_z - e.aabb_min_z;
        dst.estimated_radius_m = std::max(0.0f, 0.5f * std::max(dx, dy));
        dst.estimated_height_m = std::max(0.0f, dz);
        dst.radar_update_count = 0;
        dst.depth_confidence   = 1.0f;
        ++out.num_objects;
        ++stats.appended;
    }
    return stats;
}

}  // namespace drone::perception
