// tests/test_grid_instance_promotion.cpp
// Unit tests for OccupancyGrid3D's instance-aware voxel promotion gate
// (Issue #638 Phase 3).
//
// Truth table:
//   - gate disabled (default 0) → every voxel writes to grid as before
//   - gate enabled, instance_id == 0 (noise) → ALL voxels rejected
//   - gate enabled, instance_id != 0, observations < threshold → rejected
//   - gate enabled, instance observations >= threshold → voxels write
//   - distinct instances tracked independently
//   - clear_instance_state() resets the counter table
//   - VoxelInsertStats.instance_skipped reflects gate rejections

#include "planner/occupancy_grid_3d.h"

#include <gtest/gtest.h>

using drone::ipc::ObjectClass;
using drone::ipc::SemanticVoxel;
using drone::planner::OccupancyGrid3D;

namespace {

SemanticVoxel make_voxel(float x, float y, float z, uint32_t instance_id) {
    SemanticVoxel v{};
    v.position_x     = x;
    v.position_y     = y;
    v.position_z     = z;
    v.confidence     = 0.9f;
    v.occupancy      = 1.0f;
    v.semantic_label = ObjectClass::GEOMETRIC_OBSTACLE;
    v.timestamp_ns   = 0;
    v.instance_id    = instance_id;
    return v;
}

OccupancyGrid3D make_grid(int promotion_observations) {
    return OccupancyGrid3D(/*resolution=*/1.0f, /*extent=*/50.0f, /*inflation=*/1.0f,
                           /*cell_ttl_s=*/10.0f, /*min_confidence=*/0.3f,
                           /*promotion_hits=*/0, /*radar_promotion_hits=*/3,
                           /*min_promotion_depth_confidence=*/0.3f,
                           /*max_static_cells=*/0, /*prediction_enabled=*/false,
                           /*prediction_dt_s=*/0.0f, /*require_radar_for_promotion=*/false,
                           /*voxel_promotion_hits=*/3, /*static_cell_ttl_s=*/0.0f,
                           /*voxel_instance_promotion_observations=*/promotion_observations);
}

}  // namespace

TEST(GridInstancePromotion, GateDisabledByDefaultWritesAllVoxels) {
    // Legacy behaviour — every voxel reaches the grid regardless of
    // instance_id. Backwards compatibility for scenarios that haven't
    // opted into instance promotion.
    OccupancyGrid3D grid      = make_grid(/*promotion_observations=*/0);
    SemanticVoxel   v_noise   = make_voxel(2.0f, 2.0f, 2.0f, /*instance_id=*/0);
    SemanticVoxel   v_tracked = make_voxel(5.0f, 5.0f, 5.0f, /*instance_id=*/42);

    auto stats_a = grid.insert_voxels(&v_noise, 1, 100.0f, 0.3f);
    auto stats_b = grid.insert_voxels(&v_tracked, 1, 100.0f, 0.3f);

    EXPECT_GT(stats_a.inserted, 0u);
    EXPECT_GT(stats_b.inserted, 0u);
    EXPECT_EQ(stats_a.instance_skipped, 0u);
    EXPECT_EQ(stats_b.instance_skipped, 0u);
}

TEST(GridInstancePromotion, NoiseVoxelsRejectedWhenGateEnabled) {
    OccupancyGrid3D grid    = make_grid(/*promotion_observations=*/2);
    SemanticVoxel   v_noise = make_voxel(2.0f, 2.0f, 2.0f, /*instance_id=*/0);

    auto stats = grid.insert_voxels(&v_noise, 1, 100.0f, 0.3f);
    EXPECT_EQ(stats.inserted, 0u);
    EXPECT_GT(stats.instance_skipped, 0u);
    EXPECT_FALSE(grid.is_occupied(grid.world_to_grid(2.0f, 2.0f, 2.0f)));
}

TEST(GridInstancePromotion, TrackedInstanceBelowThresholdIsRejected) {
    // threshold = 3 frames; only one batch yet → all voxels still skipped.
    OccupancyGrid3D grid = make_grid(/*promotion_observations=*/3);
    SemanticVoxel   v    = make_voxel(5.0f, 5.0f, 5.0f, /*instance_id=*/7);

    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f);
    EXPECT_EQ(stats.inserted, 0u);
    EXPECT_GT(stats.instance_skipped, 0u);
    EXPECT_EQ(grid.tracked_instance_count(), 1u);
    EXPECT_EQ(grid.promoted_instance_count(), 0u);
}

TEST(GridInstancePromotion, InstanceCrossesThresholdAndStartsWriting) {
    // threshold = 3.  Frames 1+2 reject, frame 3 admits all instance-7
    // voxels into the grid.
    OccupancyGrid3D grid = make_grid(/*promotion_observations=*/3);
    SemanticVoxel   v    = make_voxel(5.0f, 5.0f, 5.0f, /*instance_id=*/7);

    grid.insert_voxels(&v, 1, 100.0f, 0.3f);  // obs=1
    EXPECT_FALSE(grid.is_occupied(grid.world_to_grid(5.0f, 5.0f, 5.0f)));

    grid.insert_voxels(&v, 1, 100.0f, 0.3f);  // obs=2
    EXPECT_FALSE(grid.is_occupied(grid.world_to_grid(5.0f, 5.0f, 5.0f)));

    auto stats = grid.insert_voxels(&v, 1, 100.0f, 0.3f);  // obs=3 → promoted
    EXPECT_GT(stats.inserted, 0u);
    EXPECT_EQ(stats.instance_skipped, 0u) << "above threshold, no skips";
    EXPECT_TRUE(grid.is_occupied(grid.world_to_grid(5.0f, 5.0f, 5.0f)));
    EXPECT_EQ(grid.promoted_instance_count(), 1u);
}

TEST(GridInstancePromotion, DistinctInstancesTrackedIndependently) {
    // Two instances; A reaches threshold, B doesn't.  A's voxels write,
    // B's don't.
    OccupancyGrid3D grid = make_grid(/*promotion_observations=*/2);
    SemanticVoxel   a    = make_voxel(5.0f, 5.0f, 5.0f, /*instance_id=*/1);
    SemanticVoxel   b    = make_voxel(20.0f, 20.0f, 5.0f, /*instance_id=*/2);

    // Frame 1: both observed (obs=1 each, both below threshold).
    SemanticVoxel batch1[2] = {a, b};
    grid.insert_voxels(batch1, 2, 100.0f, 0.3f);
    EXPECT_FALSE(grid.is_occupied(grid.world_to_grid(5.0f, 5.0f, 5.0f)));
    EXPECT_FALSE(grid.is_occupied(grid.world_to_grid(20.0f, 20.0f, 5.0f)));

    // Frame 2: only A observed (A obs=2 → promoted, B stays at 1).
    grid.insert_voxels(&a, 1, 100.0f, 0.3f);
    EXPECT_TRUE(grid.is_occupied(grid.world_to_grid(5.0f, 5.0f, 5.0f)));
    EXPECT_FALSE(grid.is_occupied(grid.world_to_grid(20.0f, 20.0f, 5.0f)));
    EXPECT_EQ(grid.promoted_instance_count(), 1u);
    EXPECT_EQ(grid.tracked_instance_count(), 2u);
}

TEST(GridInstancePromotion, BatchCountedAsOneFrameNotPerVoxel) {
    // Twenty voxels in one batch all carrying instance_id=5 must count
    // as ONE observation, not twenty.  Otherwise a single batch with N
    // voxels would always promote on the first call regardless of
    // threshold — defeating the gate's purpose.
    OccupancyGrid3D            grid = make_grid(/*promotion_observations=*/3);
    std::vector<SemanticVoxel> batch;
    for (int i = 0; i < 20; ++i) {
        batch.push_back(make_voxel(5.0f + 0.05f * i, 5.0f, 5.0f, /*instance_id=*/5));
    }
    grid.insert_voxels(batch.data(), batch.size(), 100.0f, 0.3f);
    EXPECT_FALSE(grid.is_occupied(grid.world_to_grid(5.0f, 5.0f, 5.0f)))
        << "20 voxels in one batch is one observation, not twenty";
    EXPECT_EQ(grid.promoted_instance_count(), 0u);
}

TEST(GridInstancePromotion, ClearInstanceStateResetsCounters) {
    OccupancyGrid3D grid = make_grid(/*promotion_observations=*/3);
    SemanticVoxel   v    = make_voxel(5.0f, 5.0f, 5.0f, /*instance_id=*/9);

    grid.insert_voxels(&v, 1, 100.0f, 0.3f);
    grid.insert_voxels(&v, 1, 100.0f, 0.3f);
    EXPECT_EQ(grid.tracked_instance_count(), 1u);

    grid.clear_instance_state();
    EXPECT_EQ(grid.tracked_instance_count(), 0u);

    // Post-clear: instance 9 starts over from observation count 0.
    grid.insert_voxels(&v, 1, 100.0f, 0.3f);
    EXPECT_FALSE(grid.is_occupied(grid.world_to_grid(5.0f, 5.0f, 5.0f)))
        << "post-clear, instance must accumulate observations again";
}
