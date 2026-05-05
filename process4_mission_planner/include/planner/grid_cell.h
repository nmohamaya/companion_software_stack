// process4_mission_planner/include/planner/grid_cell.h
//
// Tiny standalone header for the GridCell index type + its hash, extracted
// from occupancy_grid_3d.h to break a circular include between OccupancyGrid3D
// and RadarFovGate (issue #698 Fix #1, Phase 2 of plan-scenario-33-pass.md):
// the grid wants to call into the gate, the gate wants to track per-cell
// residency.  Both depend on this header; nothing here depends on either.

#pragma once

#include <cstddef>
#include <cstdint>

namespace drone::planner {

struct GridCell {
    int x = 0, y = 0, z = 0;

    bool operator==(const GridCell& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
    bool operator!=(const GridCell& o) const noexcept { return !(*this == o); }
};

/// Hash for GridCell (for use in unordered containers).
struct GridCellHash {
    size_t operator()(const GridCell& c) const noexcept {
        // FNV-1a inspired mixing — same constants as the original definition
        // in occupancy_grid_3d.h pre-extraction.
        size_t h = 2166136261u;
        h ^= static_cast<size_t>(c.x);
        h *= 16777619u;
        h ^= static_cast<size_t>(c.y);
        h *= 16777619u;
        h ^= static_cast<size_t>(c.z);
        h *= 16777619u;
        return h;
    }
};

}  // namespace drone::planner
