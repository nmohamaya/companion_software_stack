// tests/test_voxel_instance_tracker.cpp
// Unit tests for the cross-frame voxel instance tracker (Issue #638 Phase 2).
//
// Tests the truth table that downstream Phase 3 will rely on:
//   - empty input is a no-op (no tracks created)
//   - noise voxels (instance_id=0) bypass the tracker
//   - one stationary cluster gets a stable ID across frames
//   - one moving cluster keeps the same ID as long as motion < gate
//   - distinct clusters get distinct stable IDs
//   - tracks not re-observed for >max_age decay out
//   - new clusters that appear later mint new (non-reused) IDs
//   - reset() restores fresh state

#include "perception/voxel_instance_tracker.h"

#include <gtest/gtest.h>

using drone::hal::VoxelUpdate;
using drone::perception::VoxelInstanceTracker;

namespace {

VoxelUpdate make_voxel(float x, float y, float z, uint32_t frame_id) {
    VoxelUpdate v;
    v.position_m  = Eigen::Vector3f(x, y, z);
    v.confidence  = 0.9f;
    v.occupancy   = 1.0f;
    v.instance_id = frame_id;
    return v;
}

// Add `count` voxels around (cx, cy, cz) with the given frame-local id.
void add_cluster(std::vector<VoxelUpdate>& voxels, float cx, float cy, float cz, uint32_t frame_id,
                 int count = 5) {
    for (int i = 0; i < count; ++i) {
        voxels.push_back(make_voxel(cx + 0.05f * i, cy, cz, frame_id));
    }
}

}  // namespace

TEST(VoxelInstanceTracker, EmptyInputIsNoop) {
    VoxelInstanceTracker     tracker;
    std::vector<VoxelUpdate> voxels;
    tracker.update(voxels, 0);
    EXPECT_EQ(tracker.track_count(), 0u);
}

TEST(VoxelInstanceTracker, NoiseVoxelsBypassTracker) {
    // All voxels carry instance_id == 0 — the tracker must ignore them.
    VoxelInstanceTracker     tracker;
    std::vector<VoxelUpdate> voxels;
    for (int i = 0; i < 10; ++i) voxels.push_back(make_voxel(0.1f * i, 0.0f, 0.0f, 0));
    tracker.update(voxels, 0);

    EXPECT_EQ(tracker.track_count(), 0u) << "noise must not create tracks";
    for (const auto& v : voxels) EXPECT_EQ(v.instance_id, 0u) << "noise must stay id=0 on exit";
}

TEST(VoxelInstanceTracker, StationaryClusterGetsStableIdAcrossFrames) {
    VoxelInstanceTracker tracker;

    // Frame 0: cluster at (5, 5, 5) with frame-local ID 1.
    {
        std::vector<VoxelUpdate> voxels;
        add_cluster(voxels, 5.0f, 5.0f, 5.0f, /*frame_id=*/1);
        tracker.update(voxels, /*now_ns=*/100'000'000ull);
        EXPECT_EQ(tracker.track_count(), 1u);
        EXPECT_NE(voxels[0].instance_id, 0u);
    }
    // tracks() returns an unordered_map (no random/back access); the
    // single track's stable ID equals what the voxels were rewritten to,
    // which is what later frames must match.
    ASSERT_EQ(tracker.tracks().size(), 1u);
    const uint32_t first_stable_id = tracker.tracks().begin()->first;

    // Frame 1: same cluster, different frame-local ID (e.g. 7) — the
    // tracker must recognise it and assign the same stable ID.
    std::vector<VoxelUpdate> voxels;
    add_cluster(voxels, 5.0f, 5.0f, 5.0f, /*frame_id=*/7);
    tracker.update(voxels, /*now_ns=*/200'000'000ull);

    EXPECT_EQ(tracker.track_count(), 1u);
    EXPECT_EQ(voxels[0].instance_id, first_stable_id)
        << "stationary cluster must keep its stable ID across frames";
}

TEST(VoxelInstanceTracker, MovingClusterWithinGateKeepsSameId) {
    // Cluster moves 0.5 m per frame — well within max_match_distance=3 m.
    VoxelInstanceTracker tracker(/*max_match_distance_m=*/3.0f);

    std::vector<VoxelUpdate> v0;
    add_cluster(v0, 0.0f, 0.0f, 5.0f, /*frame_id=*/1);
    tracker.update(v0, 100'000'000ull);
    const uint32_t initial_id = v0[0].instance_id;

    std::vector<VoxelUpdate> v1;
    add_cluster(v1, 0.5f, 0.0f, 5.0f, /*frame_id=*/1);
    tracker.update(v1, 200'000'000ull);
    EXPECT_EQ(v1[0].instance_id, initial_id);

    std::vector<VoxelUpdate> v2;
    add_cluster(v2, 1.0f, 0.0f, 5.0f, /*frame_id=*/1);
    tracker.update(v2, 300'000'000ull);
    EXPECT_EQ(v2[0].instance_id, initial_id);
    EXPECT_EQ(tracker.track_count(), 1u);
}

TEST(VoxelInstanceTracker, JumpBeyondGateMintsNewId) {
    // Cluster "teleports" 10 m between frames — outside gate of 3 m.
    // Should mint a new track (and old track ages out via TTL).
    VoxelInstanceTracker tracker(/*max_match_distance_m=*/3.0f, /*track_max_age_s=*/5.0f);

    std::vector<VoxelUpdate> v0;
    add_cluster(v0, 0.0f, 0.0f, 5.0f, 1);
    tracker.update(v0, 100'000'000ull);
    const uint32_t old_id = v0[0].instance_id;

    std::vector<VoxelUpdate> v1;
    add_cluster(v1, 10.0f, 0.0f, 5.0f, 1);
    tracker.update(v1, 200'000'000ull);
    EXPECT_NE(v1[0].instance_id, old_id) << "10 m jump must not match a 3 m-gated track";

    // Two distinct tracks now exist (old one hasn't aged out yet).
    EXPECT_EQ(tracker.track_count(), 2u);
}

TEST(VoxelInstanceTracker, TwoConcurrentClustersGetDistinctStableIds) {
    VoxelInstanceTracker tracker;

    std::vector<VoxelUpdate> voxels;
    add_cluster(voxels, 0.0f, 0.0f, 5.0f, /*frame_id=*/1);
    add_cluster(voxels, 30.0f, 0.0f, 5.0f, /*frame_id=*/2);
    tracker.update(voxels, 100'000'000ull);

    EXPECT_EQ(tracker.track_count(), 2u);
    EXPECT_NE(voxels[0].instance_id, voxels[5].instance_id);
}

TEST(VoxelInstanceTracker, TracksAgeOutAfterTtl) {
    VoxelInstanceTracker tracker(/*max_match=*/3.0f, /*track_max_age_s=*/0.1f);

    std::vector<VoxelUpdate> v0;
    add_cluster(v0, 5.0f, 5.0f, 5.0f, 1);
    tracker.update(v0, 0);
    EXPECT_EQ(tracker.track_count(), 1u);

    // Tick well past max_age with no observations — track must age out.
    std::vector<VoxelUpdate> empty;
    tracker.update(empty, 500'000'000ull);  // 0.5 s
    EXPECT_EQ(tracker.track_count(), 0u);
}

TEST(VoxelInstanceTracker, NewClustersMintFreshIds) {
    // Track 1 ages out, then a NEW cluster appears — tracker must NOT
    // reuse the old ID (consumers downstream rely on monotonic IDs).
    VoxelInstanceTracker tracker(/*max_match=*/3.0f, /*track_max_age_s=*/0.1f);

    std::vector<VoxelUpdate> v0;
    add_cluster(v0, 5.0f, 5.0f, 5.0f, 1);
    tracker.update(v0, 0);
    const uint32_t first_id = v0[0].instance_id;

    // Age out.
    std::vector<VoxelUpdate> empty;
    tracker.update(empty, 500'000'000ull);
    ASSERT_EQ(tracker.track_count(), 0u);

    // New cluster appears — must mint a brand-new ID.
    std::vector<VoxelUpdate> v1;
    add_cluster(v1, 100.0f, 100.0f, 5.0f, 1);
    tracker.update(v1, 600'000'000ull);
    EXPECT_NE(v1[0].instance_id, first_id) << "new tracks must not reuse aged-out IDs";
}

TEST(VoxelInstanceTracker, ResetClearsAllState) {
    VoxelInstanceTracker     tracker;
    std::vector<VoxelUpdate> v;
    add_cluster(v, 5.0f, 5.0f, 5.0f, 1);
    tracker.update(v, 100'000'000ull);
    EXPECT_EQ(tracker.track_count(), 1u);

    tracker.reset();
    EXPECT_EQ(tracker.track_count(), 0u);

    // After reset, IDs restart from 1.
    std::vector<VoxelUpdate> v2;
    add_cluster(v2, 5.0f, 5.0f, 5.0f, 1);
    tracker.update(v2, 200'000'000ull);
    EXPECT_EQ(v2[0].instance_id, 1u);
}
