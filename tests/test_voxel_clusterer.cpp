// tests/test_voxel_clusterer.cpp
// Unit tests for the voxel clusterer (Issue #638 Phase 1).
//
// Tests cover the truth table that downstream Phase 3 will rely on:
//   - empty input is a no-op
//   - clustering disabled (eps==0 or min_pts==0) leaves every voxel id=0
//   - one tight cluster gets one non-zero ID
//   - sparse noise (every voxel >2*eps from every other) stays id=0
//   - two distant clusters get distinct IDs
//   - cluster spanning multiple eps-cells unions transitively
//   - small components below min_pts get id=0
//   - scratch buffer can be reused across calls without state leakage

#include "perception/voxel_clusterer.h"

#include <limits>
#include <unordered_set>

#include <gtest/gtest.h>

using drone::hal::VoxelUpdate;
using drone::perception::assign_instance_ids;
using drone::perception::count_clusters;
using drone::perception::VoxelClusterScratch;

namespace {

VoxelUpdate make_voxel(float x, float y, float z) {
    VoxelUpdate v;
    v.position_m = Eigen::Vector3f(x, y, z);
    v.confidence = 0.9f;
    v.occupancy  = 1.0f;
    return v;
}

// Returns the set of distinct non-zero instance IDs present in voxels.
std::unordered_set<uint32_t> distinct_ids(const std::vector<VoxelUpdate>& voxels) {
    std::unordered_set<uint32_t> ids;
    for (const auto& v : voxels) {
        if (v.instance_id != 0) ids.insert(v.instance_id);
    }
    return ids;
}

}  // namespace

TEST(VoxelClusterer, EmptyInputIsNoop) {
    std::vector<VoxelUpdate> voxels;
    assign_instance_ids(voxels, 1.0f, 3);
    EXPECT_TRUE(voxels.empty());
    EXPECT_EQ(count_clusters(voxels), 0u);
}

TEST(VoxelClusterer, DisabledByZeroEps) {
    // 10 tightly-packed voxels — would form one cluster at any positive eps.
    // eps_m == 0 disables clustering, every voxel must keep id=0.
    //
    // P1-B from PR #639 review: pre-seed instance_id with a marker so the
    // assertion proves the disable branch ACTIVELY wrote 0, not that the
    // default-constructed value (also 0) leaked through unchanged.
    std::vector<VoxelUpdate> voxels;
    for (int i = 0; i < 10; ++i) {
        voxels.push_back(make_voxel(0.1f * i, 0.0f, 0.0f));
        voxels.back().instance_id = 0xDEADBEEFu;  // marker — must be overwritten
    }

    assign_instance_ids(voxels, /*eps_m=*/0.0f, /*min_pts=*/3);
    for (const auto& v : voxels) {
        EXPECT_EQ(v.instance_id, 0u) << "disable branch must overwrite the marker with 0";
    }
    EXPECT_EQ(count_clusters(voxels), 0u);
}

TEST(VoxelClusterer, DisabledByZeroMinPts) {
    // P1-B: same false-green guard as above.
    std::vector<VoxelUpdate> voxels;
    for (int i = 0; i < 10; ++i) {
        voxels.push_back(make_voxel(0.1f * i, 0.0f, 0.0f));
        voxels.back().instance_id = 0xDEADBEEFu;  // marker — must be overwritten
    }

    assign_instance_ids(voxels, /*eps_m=*/1.0f, /*min_pts=*/0);
    for (const auto& v : voxels) {
        EXPECT_EQ(v.instance_id, 0u) << "disable branch must overwrite the marker with 0";
    }
}

TEST(VoxelClusterer, DisabledByNaNEps) {
    // P2-D from PR #639 review: a tampered config could pass NaN through
    // `cfg.get<float>(... eps_m)`.  `NaN <= 0.0f` is false, so the old
    // disable guard let NaN slip through, then `1/NaN = NaN`, then
    // `static_cast<int>(std::floor(NaN))` is UB.  The fix uses
    // `!std::isfinite(eps_m)` to short-circuit.
    std::vector<VoxelUpdate> voxels;
    for (int i = 0; i < 5; ++i) {
        voxels.push_back(make_voxel(0.1f * i, 0.0f, 0.0f));
        voxels.back().instance_id = 0xDEADBEEFu;
    }
    assign_instance_ids(voxels, std::numeric_limits<float>::quiet_NaN(), 3);
    for (const auto& v : voxels) EXPECT_EQ(v.instance_id, 0u);
}

TEST(VoxelClusterer, NaNPositionIsHandledSafely) {
    // P1-A from PR #639 review: a buggy depth backend could emit a NaN
    // position; without the explicit `std::isfinite` guard, the floor cast
    // is UB.  Test must run cleanly under UBSan.
    std::vector<VoxelUpdate> voxels;
    voxels.push_back(make_voxel(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f));
    voxels.push_back(make_voxel(0.0f, std::numeric_limits<float>::infinity(), 0.0f));
    voxels.push_back(make_voxel(1.0f, 1.0f, 1.0f));
    voxels.push_back(make_voxel(1.1f, 1.0f, 1.0f));
    voxels.push_back(make_voxel(1.2f, 1.0f, 1.0f));

    assign_instance_ids(voxels, 1.0f, 3);
    EXPECT_EQ(voxels[0].instance_id, 0u) << "NaN position must be flagged as noise";
    EXPECT_EQ(voxels[1].instance_id, 0u) << "Inf position must be flagged as noise";
    // The remaining 3 finite voxels form one cluster.
    EXPECT_EQ(voxels[2].instance_id, voxels[3].instance_id);
    EXPECT_EQ(voxels[3].instance_id, voxels[4].instance_id);
    EXPECT_NE(voxels[2].instance_id, 0u);
}

TEST(VoxelClusterer, OneTightClusterGetsOneId) {
    // 8 voxels packed into a 1m cube, eps=1m → all in one cluster.
    std::vector<VoxelUpdate> voxels;
    for (int i = 0; i < 8; ++i) voxels.push_back(make_voxel(0.1f * i, 0.0f, 0.0f));

    assign_instance_ids(voxels, /*eps_m=*/1.0f, /*min_pts=*/3);
    auto ids = distinct_ids(voxels);
    EXPECT_EQ(ids.size(), 1u);
    EXPECT_EQ(*ids.begin(), 1u);  // first non-zero ID
    for (const auto& v : voxels) EXPECT_EQ(v.instance_id, 1u);
}

TEST(VoxelClusterer, SparseNoiseStaysUnclustered) {
    // 6 voxels each ≥10m from every other — no neighbourhoods touch.
    std::vector<VoxelUpdate> voxels = {
        make_voxel(0.0f, 0.0f, 0.0f),   make_voxel(20.0f, 0.0f, 0.0f),
        make_voxel(0.0f, 20.0f, 0.0f),  make_voxel(0.0f, 0.0f, 20.0f),
        make_voxel(20.0f, 20.0f, 0.0f), make_voxel(-20.0f, -20.0f, -20.0f),
    };

    assign_instance_ids(voxels, /*eps_m=*/1.0f, /*min_pts=*/3);
    for (const auto& v : voxels) EXPECT_EQ(v.instance_id, 0u);
    EXPECT_EQ(count_clusters(voxels), 0u);
}

TEST(VoxelClusterer, TwoDistantClustersGetDistinctIds) {
    // Cluster A around origin, cluster B around (50, 0, 0).  Distance
    // between clusters is >>eps; within each cluster voxels are <eps apart.
    std::vector<VoxelUpdate> voxels;
    for (int i = 0; i < 5; ++i) voxels.push_back(make_voxel(0.1f * i, 0.0f, 0.0f));
    for (int i = 0; i < 5; ++i) voxels.push_back(make_voxel(50.0f + 0.1f * i, 0.0f, 0.0f));

    assign_instance_ids(voxels, /*eps_m=*/1.0f, /*min_pts=*/3);
    auto ids = distinct_ids(voxels);
    EXPECT_EQ(ids.size(), 2u);

    // First five voxels share an ID; last five share a different ID.
    for (int i = 1; i < 5; ++i) EXPECT_EQ(voxels[0].instance_id, voxels[i].instance_id);
    for (int i = 6; i < 10; ++i) EXPECT_EQ(voxels[5].instance_id, voxels[i].instance_id);
    EXPECT_NE(voxels[0].instance_id, voxels[5].instance_id);
}

TEST(VoxelClusterer, ChainedClusterUnionsTransitively) {
    // Voxels at x = 0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0 with eps=0.6.
    // Each pair (i, i+1) is within eps, so the whole chain becomes one
    // cluster even though endpoints are 3 m apart.
    std::vector<VoxelUpdate> voxels;
    for (float x = 0.0f; x <= 3.0f; x += 0.5f) voxels.push_back(make_voxel(x, 0.0f, 0.0f));

    assign_instance_ids(voxels, /*eps_m=*/0.6f, /*min_pts=*/3);
    auto ids = distinct_ids(voxels);
    EXPECT_EQ(ids.size(), 1u) << "transitive union should merge the chain into one component";
    for (const auto& v : voxels) EXPECT_NE(v.instance_id, 0u);
}

TEST(VoxelClusterer, SmallComponentBelowMinPtsBecomesNoise) {
    // Cluster A: 5 voxels (>= min_pts) → real cluster
    // Cluster B: 2 voxels (< min_pts) → noise (id=0)
    std::vector<VoxelUpdate> voxels;
    for (int i = 0; i < 5; ++i) voxels.push_back(make_voxel(0.1f * i, 0.0f, 0.0f));
    voxels.push_back(make_voxel(50.0f, 0.0f, 0.0f));
    voxels.push_back(make_voxel(50.1f, 0.0f, 0.0f));

    assign_instance_ids(voxels, /*eps_m=*/1.0f, /*min_pts=*/3);

    // First 5 voxels share a non-zero ID.
    EXPECT_NE(voxels[0].instance_id, 0u);
    for (int i = 1; i < 5; ++i) EXPECT_EQ(voxels[0].instance_id, voxels[i].instance_id);
    // Last 2 voxels are below min_pts → noise.
    EXPECT_EQ(voxels[5].instance_id, 0u);
    EXPECT_EQ(voxels[6].instance_id, 0u);
}

TEST(VoxelClusterer, ScratchReuseIsClean) {
    // A scratch buffer reused across calls must produce the same result
    // as a fresh buffer.  Catches state-leakage bugs (forgotten clear()).
    VoxelClusterScratch scratch;

    std::vector<VoxelUpdate> a;
    for (int i = 0; i < 5; ++i) a.push_back(make_voxel(0.1f * i, 0.0f, 0.0f));
    assign_instance_ids(a, 1.0f, 3, &scratch);
    for (const auto& v : a) EXPECT_EQ(v.instance_id, 1u);

    // Now run the same input again with the same scratch.  Without proper
    // clear(), the parent vector / cell_to_voxel map would carry stale
    // entries from the first call and could mis-cluster.
    std::vector<VoxelUpdate> b;
    for (int i = 0; i < 5; ++i) b.push_back(make_voxel(100.0f + 0.1f * i, 0.0f, 0.0f));
    assign_instance_ids(b, 1.0f, 3, &scratch);
    auto ids_b = distinct_ids(b);
    EXPECT_EQ(ids_b.size(), 1u);
    EXPECT_EQ(*ids_b.begin(), 1u) << "second call must restart IDs from 1, not continue from prior";
}
