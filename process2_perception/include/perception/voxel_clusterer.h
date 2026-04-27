// process2_perception/include/perception/voxel_clusterer.h
//
// Issue #638 — Voxel clustering Phase 1: assign per-frame cluster IDs to a
// batch of `hal::VoxelUpdate`s based on 3D spatial proximity.
//
// Algorithm: Union-Find on a uniform grid hash.  Each voxel is binned into
// a `eps_m`-sided cell; voxels in adjacent cells (Chebyshev distance ≤ 1)
// are linked.  After all unions, connected components are walked; clusters
// with ≥ `min_pts` members get a unique non-zero ID, smaller components
// get id=0 (treated as noise downstream).
//
// Why Union-Find on a grid (not DBSCAN proper):
//   * Deterministic and order-independent — DBSCAN's cluster IDs depend on
//     iteration order; Union-Find produces the same partitioning regardless.
//   * O(N) for the bin pass + O(N · α(N)) for the unions — much cheaper than
//     DBSCAN's O(N²) worst case at our typical N (~5 k voxels/frame).
//   * No kd-tree or distance-metric setup; all comparisons are integer cell
//     index lookups.
//   * Per-frame-only — Phase 2 will add cross-frame instance tracking on top
//     of these stable per-frame IDs.
//
// NOT thread-safe — caller is the single P2 mask_projection_thread, runs
// once per voxel batch.  Self-contained: no class state, no allocations
// outside the local scratch maps (which can be passed in via Scratch to
// amortise across frames if profiling shows it matters).
//
// Behaviour when cluster list is empty (eps_m <= 0 or min_pts <= 0):
// every voxel keeps instance_id = 0 (clustering effectively disabled).
#pragma once

#include "hal/isemantic_projector.h"  // for VoxelUpdate

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>

namespace drone::perception {

/// Optional scratch buffers — pass the same Scratch instance across frames
/// to amortise the inner unordered_map allocations.  All members are reset
/// at the start of each `assign_instance_ids()` call.
struct VoxelClusterScratch {
    /// Cell index → first voxel index that occupies it.  Used to find
    /// neighbours during the union pass.
    std::unordered_map<uint64_t, int> cell_to_voxel;
    /// Union-Find parent table sized to voxel count.
    std::vector<int> parent;
    /// Per-component voxel count, indexed by representative root.
    std::unordered_map<int, int> root_size;
    /// Representative root → final emitted instance_id (0 = noise).
    std::unordered_map<int, uint32_t> root_to_id;
};

namespace detail {

/// Pack three int16 cell indices into one 64-bit key.  Cell indices come
/// from `floor(world_coord / eps_m)`; ±32k cells covers a 32-km cube at
/// any practical eps_m so int16 is plenty.
[[nodiscard]] inline uint64_t pack_cell_key(int x, int y, int z) noexcept {
    return (static_cast<uint64_t>(static_cast<uint32_t>(x) & 0xFFFFu) << 32) |
           (static_cast<uint64_t>(static_cast<uint32_t>(y) & 0xFFFFu) << 16) |
           static_cast<uint64_t>(static_cast<uint32_t>(z) & 0xFFFFu);
}

/// Path-compressed Union-Find find().
[[nodiscard]] inline int uf_find(std::vector<int>& parent, int x) noexcept {
    while (parent[x] != x) {
        parent[x] = parent[parent[x]];  // halving
        x         = parent[x];
    }
    return x;
}

/// Union-by-rank-free union — caller has already done find() on both args.
inline void uf_union(std::vector<int>& parent, int rx, int ry) noexcept {
    if (rx != ry) parent[rx] = ry;
}

}  // namespace detail

/// Assign per-frame cluster IDs to `voxels` based on 3D spatial proximity.
///
/// Each voxel's `instance_id` is set in-place.  Voxels in clusters of
/// fewer than `min_pts` members get id=0 (noise — Phase 3's
/// `OccupancyGrid3D::insert_voxels()` will skip promotion for these).
/// Cluster IDs start at 1; the highest emitted ID equals the number of
/// non-noise clusters.
///
/// @param voxels    voxel batch from `MaskDepthProjector::project()`
/// @param eps_m     spatial bin size (m) — voxels in adjacent eps-cells
///                  link into one cluster.  A reasonable default is the
///                  grid resolution of OccupancyGrid3D (e.g. 0.5–1.0 m).
/// @param min_pts   minimum voxel count for a connected component to be
///                  considered a real cluster.  Smaller components get
///                  id=0.  3–5 is typical for noise rejection.
/// @param scratch   optional pre-allocated scratch buffers; pass the same
///                  instance across frames to avoid hashmap reallocation.
inline void assign_instance_ids(std::vector<drone::hal::VoxelUpdate>& voxels, float eps_m,
                                int min_pts, VoxelClusterScratch* scratch = nullptr) {
    // Disabled (or pathological config) → leave every voxel as noise.
    // Also disabled when eps_m is non-finite (NaN/Inf) — guards against
    // tampered config + downstream UB in the floor cast (review P2-D
    // from PR #639).
    if (voxels.empty() || !std::isfinite(eps_m) || eps_m <= 0.0f || min_pts <= 0) {
        for (auto& v : voxels) v.instance_id = 0;
        return;
    }

    // Use either caller-provided scratch or a stack-local one.
    VoxelClusterScratch  local;
    VoxelClusterScratch& s = scratch != nullptr ? *scratch : local;
    s.cell_to_voxel.clear();
    s.parent.assign(voxels.size(), 0);
    for (int i = 0; i < static_cast<int>(voxels.size()); ++i) s.parent[i] = i;
    s.root_size.clear();
    s.root_to_id.clear();

    const float inv_eps = 1.0f / eps_m;
    // Issue #638 P1-A — guard against int overflow in the floor() cast.
    // Cell indices are packed into 16-bit slots in pack_cell_key; values
    // outside ±32k cells alias silently.  Clamp positions so the int cast
    // never overflows even with adversarial input.  Also covers the
    // small-eps corner case: at eps=0.001m and world ±32m, indices reach
    // ±32k — at the boundary.  This caps the meaningful flight envelope
    // at ±32k * eps_m metres (≈ 16 km at the typical 0.5 m eps).
    constexpr int kMaxCellIndex = 32767;

    // Pass 1 — bin every voxel and union it with the *first* voxel in
    // each of the 27 adjacent cells (Chebyshev ≤ 1 — face/edge/corner).
    // Linking only to the first occupant of each neighbour cell is enough
    // for transitive closure: every voxel later landing in a neighbour
    // cell unions with that same first occupant, joining the component.
    for (int i = 0; i < static_cast<int>(voxels.size()); ++i) {
        auto& v = voxels[i];
        // Issue #638 P1-A — non-finite position (NaN/Inf from a buggy
        // depth backend) cannot be safely binned.  static_cast<int>
        // (std::floor(NaN)) is undefined behaviour per C++17.  Mark as
        // noise and skip; the upstream sender stays unaware so future
        // diagnostic improvements can surface the count.
        if (!std::isfinite(v.position_m.x()) || !std::isfinite(v.position_m.y()) ||
            !std::isfinite(v.position_m.z())) {
            v.instance_id = 0;
            continue;
        }
        const float fx = v.position_m.x() * inv_eps;
        const float fy = v.position_m.y() * inv_eps;
        const float fz = v.position_m.z() * inv_eps;
        // Clamp before cast — preserves int domain safety even at small
        // eps values where world coordinates can produce out-of-range
        // cell indices (review P2-A).
        const int cx = std::clamp(static_cast<int>(std::floor(fx)), -kMaxCellIndex, kMaxCellIndex);
        const int cy = std::clamp(static_cast<int>(std::floor(fy)), -kMaxCellIndex, kMaxCellIndex);
        const int cz = std::clamp(static_cast<int>(std::floor(fz)), -kMaxCellIndex, kMaxCellIndex);

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    auto it =
                        s.cell_to_voxel.find(detail::pack_cell_key(cx + dx, cy + dy, cz + dz));
                    if (it == s.cell_to_voxel.end()) continue;
                    const int rx = detail::uf_find(s.parent, i);
                    const int ry = detail::uf_find(s.parent, it->second);
                    detail::uf_union(s.parent, rx, ry);
                }
            }
        }
        // Record this voxel as the first occupant of its own cell (only
        // if no earlier voxel claimed it — earlier voxels stay
        // authoritative so this voxel was already unioned to them).
        s.cell_to_voxel.emplace(detail::pack_cell_key(cx, cy, cz), i);
    }

    // Pass 2 — count component sizes.
    for (int i = 0; i < static_cast<int>(voxels.size()); ++i) {
        const int r = detail::uf_find(s.parent, i);
        ++s.root_size[r];
    }

    // Pass 3 — assign IDs to qualifying components, write back to voxels.
    uint32_t next_id = 1;
    for (int i = 0; i < static_cast<int>(voxels.size()); ++i) {
        const int r = detail::uf_find(s.parent, i);
        if (s.root_size[r] < min_pts) {
            voxels[i].instance_id = 0;
            continue;
        }
        auto [it, inserted] = s.root_to_id.try_emplace(r, next_id);
        if (inserted) ++next_id;
        voxels[i].instance_id = it->second;
    }
}

/// Convenience accessor — returns the number of distinct non-noise
/// instance IDs present in `voxels`.  Useful for diagnostic logs.
///
/// Counts distinct IDs (not max ID) so the result remains correct after
/// Phase 2's tracker replaces compact frame-local IDs with non-compact
/// stable instance IDs (review P2-H from PR #639).
[[nodiscard]] inline uint32_t count_clusters(const std::vector<drone::hal::VoxelUpdate>& voxels) {
    std::unordered_set<uint32_t> ids;
    for (const auto& v : voxels) {
        if (v.instance_id != 0) ids.insert(v.instance_id);
    }
    return static_cast<uint32_t>(ids.size());
}

}  // namespace drone::perception
