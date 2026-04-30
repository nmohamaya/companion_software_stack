// tests/test_voxel_obstacle_surface.cpp
//
// Issue #645 — Unit tests for the voxel-derived obstacle surface that
// `fusion_thread` uses to append GEOMETRIC_OBSTACLE entries to its
// published DetectedObjectList.  The conversion logic lives in
// `append_voxel_obstacles_to_list()` in voxel_obstacle_snapshot.h so it can
// be exercised here without spinning up the full P2 thread graph.

#include "ipc/ipc_types.h"
#include "perception/voxel_obstacle_snapshot.h"

#include <gtest/gtest.h>

using drone::perception::append_voxel_obstacles_to_list;
using drone::perception::kVoxelTrackIdPrefix;
using drone::perception::VoxelObstacleEntry;
using drone::perception::VoxelObstacleSnapshot;

namespace {

VoxelObstacleEntry make_entry(uint32_t id, float cx, float cy, float cz, float halfx, float halfy,
                              float halfz, uint64_t last_seen_ns, uint32_t obs) {
    VoxelObstacleEntry e{};
    e.stable_id         = id;
    e.centroid_x        = cx;
    e.centroid_y        = cy;
    e.centroid_z        = cz;
    e.aabb_min_x        = cx - halfx;
    e.aabb_min_y        = cy - halfy;
    e.aabb_min_z        = cz - halfz;
    e.aabb_max_x        = cx + halfx;
    e.aabb_max_y        = cy + halfy;
    e.aabb_max_z        = cz + halfz;
    e.last_seen_ns      = last_seen_ns;
    e.observation_count = obs;
    return e;
}

}  // namespace

// Two qualifying entries → both appear, prefix applied, fields populated.
TEST(VoxelObstacleSurfaceTest, AppendsQualifyingEntries) {
    VoxelObstacleSnapshot snap{};
    snap.timestamp_ns = 1'000'000'000ULL;
    snap.entries[0]   = make_entry(/*id=*/7, /*cx=*/8.0f, /*cy=*/12.0f, /*cz=*/5.0f,
                                 /*halfx=*/0.5f, /*halfy=*/0.5f, /*halfz=*/2.5f,
                                 /*last_seen_ns=*/900'000'000ULL, /*obs=*/5);
    snap.entries[1]   = make_entry(/*id=*/9, /*cx=*/22.0f, /*cy=*/14.0f, /*cz=*/5.0f,
                                 /*halfx=*/0.6f, /*halfy=*/0.4f, /*halfz=*/2.0f,
                                 /*last_seen_ns=*/950'000'000ULL, /*obs=*/12);
    snap.num_entries  = 2;

    drone::ipc::DetectedObjectList out{};
    out.num_objects = 0;

    auto stats = append_voxel_obstacles_to_list(snap, out, /*now_ns=*/1'000'000'000ULL,
                                                /*min_observations=*/3,
                                                /*max_age_ns=*/500'000'000ULL);
    EXPECT_EQ(stats.appended, 2u);
    EXPECT_EQ(stats.skipped_obs, 0u);
    EXPECT_EQ(stats.skipped_age, 0u);
    EXPECT_EQ(stats.skipped_full, 0u);
    ASSERT_EQ(out.num_objects, 2u);

    // Entry 0
    EXPECT_EQ(out.objects[0].track_id, kVoxelTrackIdPrefix | 7u);
    EXPECT_EQ(out.objects[0].class_id, drone::ipc::ObjectClass::GEOMETRIC_OBSTACLE);
    EXPECT_FLOAT_EQ(out.objects[0].position_x, 8.0f);
    EXPECT_FLOAT_EQ(out.objects[0].position_y, 12.0f);
    EXPECT_FLOAT_EQ(out.objects[0].position_z, 5.0f);
    EXPECT_FLOAT_EQ(out.objects[0].velocity_x, 0.0f);
    EXPECT_FLOAT_EQ(out.objects[0].estimated_radius_m, 0.5f);
    EXPECT_FLOAT_EQ(out.objects[0].estimated_height_m, 5.0f);
    EXPECT_FALSE(out.objects[0].has_camera);
    EXPECT_FALSE(out.objects[0].has_radar);
    EXPECT_FLOAT_EQ(out.objects[0].depth_confidence, 1.0f);
    EXPECT_GT(out.objects[0].confidence, 0.6f);
    EXPECT_LT(out.objects[0].confidence, 1.0f + 1e-4f);

    // Entry 1 — obs=12 → confidence at the 1.0 plateau (clamped).
    EXPECT_NEAR(out.objects[1].confidence, 1.0f, 1e-4f);
}

// Confidence ramp + plateau.
TEST(VoxelObstacleSurfaceTest, ConfidenceRampsLinearlyThenPlateaus) {
    VoxelObstacleSnapshot snap{};
    snap.entries[0]  = make_entry(1, 0, 0, 0, 1, 1, 1, 0, /*obs=*/3);
    snap.entries[1]  = make_entry(2, 0, 0, 0, 1, 1, 1, 0, /*obs=*/5);
    snap.entries[2]  = make_entry(3, 0, 0, 0, 1, 1, 1, 0, /*obs=*/10);
    snap.entries[3]  = make_entry(4, 0, 0, 0, 1, 1, 1, 0, /*obs=*/100);
    snap.num_entries = 4;

    drone::ipc::DetectedObjectList out{};
    auto stats = append_voxel_obstacles_to_list(snap, out, /*now_ns=*/0, /*min_observations=*/1,
                                                /*max_age_ns=*/UINT64_MAX);
    ASSERT_EQ(stats.appended, 4u);
    // 0.6 + 0.4 * (3/10) = 0.72
    EXPECT_NEAR(out.objects[0].confidence, 0.72f, 1e-4f);
    // 0.6 + 0.4 * (5/10) = 0.80
    EXPECT_NEAR(out.objects[1].confidence, 0.80f, 1e-4f);
    // 0.6 + 0.4 * 1.0 = 1.0 plateau
    EXPECT_NEAR(out.objects[2].confidence, 1.0f, 1e-4f);
    EXPECT_NEAR(out.objects[3].confidence, 1.0f, 1e-4f);
}

// Below min_observations → skipped.
TEST(VoxelObstacleSurfaceTest, SkipsEntriesBelowMinObservations) {
    VoxelObstacleSnapshot snap{};
    snap.entries[0]  = make_entry(1, 0, 0, 0, 1, 1, 1, 0, /*obs=*/2);
    snap.entries[1]  = make_entry(2, 0, 0, 0, 1, 1, 1, 0, /*obs=*/4);
    snap.num_entries = 2;

    drone::ipc::DetectedObjectList out{};
    auto stats = append_voxel_obstacles_to_list(snap, out, 0, /*min_observations=*/3, UINT64_MAX);
    EXPECT_EQ(stats.appended, 1u);
    EXPECT_EQ(stats.skipped_obs, 1u);
    ASSERT_EQ(out.num_objects, 1u);
    EXPECT_EQ(out.objects[0].track_id, kVoxelTrackIdPrefix | 2u);
}

// Stale entries (older than max_age_ns) → skipped.
TEST(VoxelObstacleSurfaceTest, SkipsStaleEntries) {
    VoxelObstacleSnapshot snap{};
    // last_seen 800ms ago, with max_age = 500ms → skipped
    snap.entries[0] = make_entry(1, 0, 0, 0, 1, 1, 1, /*last_seen_ns=*/200'000'000ULL, /*obs=*/10);
    // last_seen 100ms ago → kept
    snap.entries[1]  = make_entry(2, 0, 0, 0, 1, 1, 1, /*last_seen_ns=*/900'000'000ULL, /*obs=*/10);
    snap.num_entries = 2;

    drone::ipc::DetectedObjectList out{};
    auto stats = append_voxel_obstacles_to_list(snap, out, /*now_ns=*/1'000'000'000ULL,
                                                /*min_observations=*/3,
                                                /*max_age_ns=*/500'000'000ULL);
    EXPECT_EQ(stats.appended, 1u);
    EXPECT_EQ(stats.skipped_age, 1u);
    EXPECT_EQ(out.objects[0].track_id, kVoxelTrackIdPrefix | 2u);
}

// Future-dated entries (last_seen_ns > now_ns) — skipped_age must NOT fire
// from the unsigned subtraction underflow path.
TEST(VoxelObstacleSurfaceTest, FutureTimestampsAreNotMisclassifiedAsStale) {
    VoxelObstacleSnapshot snap{};
    // last_seen 100ms in the FUTURE (clock skew)
    snap.entries[0]  = make_entry(1, 0, 0, 0, 1, 1, 1, /*last_seen_ns=*/1'100'000'000ULL,
                                  /*obs=*/10);
    snap.num_entries = 1;

    drone::ipc::DetectedObjectList out{};
    auto stats = append_voxel_obstacles_to_list(snap, out, /*now_ns=*/1'000'000'000ULL,
                                                /*min_observations=*/3,
                                                /*max_age_ns=*/500'000'000ULL);
    EXPECT_EQ(stats.appended, 1u);
    EXPECT_EQ(stats.skipped_age, 0u);
}

// Existing fusion entries are preserved when appending.
TEST(VoxelObstacleSurfaceTest, PreservesExistingDetectedObjects) {
    drone::ipc::DetectedObjectList out{};
    out.num_objects         = 1;
    out.objects[0].track_id = 42;
    out.objects[0].class_id = drone::ipc::ObjectClass::PERSON;

    VoxelObstacleSnapshot snap{};
    snap.entries[0]  = make_entry(7, 1, 1, 1, 1, 1, 1, 0, /*obs=*/10);
    snap.num_entries = 1;

    auto stats = append_voxel_obstacles_to_list(snap, out, 0, /*min_observations=*/3, UINT64_MAX);
    EXPECT_EQ(stats.appended, 1u);
    ASSERT_EQ(out.num_objects, 2u);
    EXPECT_EQ(out.objects[0].track_id, 42u);
    EXPECT_EQ(out.objects[0].class_id, drone::ipc::ObjectClass::PERSON);
    EXPECT_EQ(out.objects[1].track_id, kVoxelTrackIdPrefix | 7u);
    EXPECT_EQ(out.objects[1].class_id, drone::ipc::ObjectClass::GEOMETRIC_OBSTACLE);
}

// Capacity exhaustion is reported in skipped_full and remaining entries
// don't overrun the array.
TEST(VoxelObstacleSurfaceTest, RespectsMaxDetectedObjectsCap) {
    drone::ipc::DetectedObjectList out{};
    out.num_objects = drone::ipc::MAX_DETECTED_OBJECTS - 2;  // only 2 slots free

    VoxelObstacleSnapshot snap{};
    for (uint32_t i = 0; i < 5; ++i) {
        snap.entries[i] = make_entry(i + 1, 0, 0, 0, 1, 1, 1, 0, 10);
    }
    snap.num_entries = 5;

    auto stats = append_voxel_obstacles_to_list(snap, out, 0, /*min_observations=*/3, UINT64_MAX);
    EXPECT_EQ(stats.appended, 2u);
    EXPECT_EQ(stats.skipped_full, 3u);
    EXPECT_EQ(out.num_objects, drone::ipc::MAX_DETECTED_OBJECTS);
}

// Empty snapshot → no work, no errors.
TEST(VoxelObstacleSurfaceTest, EmptySnapshotIsNoOp) {
    drone::ipc::DetectedObjectList out{};
    out.num_objects = 3;
    VoxelObstacleSnapshot snap{};
    snap.num_entries = 0;
    auto stats = append_voxel_obstacles_to_list(snap, out, 0, /*min_observations=*/3, UINT64_MAX);
    EXPECT_EQ(stats.appended, 0u);
    EXPECT_EQ(out.num_objects, 3u);
}

// validate() passes on synthesised entries.
TEST(VoxelObstacleSurfaceTest, SynthesisedEntriesPassValidate) {
    VoxelObstacleSnapshot snap{};
    snap.entries[0]  = make_entry(1, 8, 12, 5, 0.5f, 0.5f, 2.5f, 0, 5);
    snap.num_entries = 1;

    drone::ipc::DetectedObjectList out{};
    append_voxel_obstacles_to_list(snap, out, 0, /*min_observations=*/3, UINT64_MAX);
    EXPECT_TRUE(out.validate());
}
