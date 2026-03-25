// process4_mission_planner/include/planner/dstar_lite_planner.h
// D* Lite Incremental Path Planner — searches backward from goal and
// maintains its priority queue across frames, only re-expanding nodes
// whose edge costs changed due to obstacle map updates.
//
// Algorithm: Koenig & Likhachev, 2002.
// Implements Issue #158.
#pragma once

#include "planner/grid_planner_base.h"
#include "planner/occupancy_grid_3d.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

namespace drone::planner {

class DStarLitePlanner final : public GridPlannerBase {
public:
    explicit DStarLitePlanner(const GridPlannerConfig& config = {}) : GridPlannerBase(config) {}

    std::string name() const override { return "DStarLitePlanner"; }

private:
    // 8-connected horizontal neighbours for 2D search at flight altitude.
    // Includes 4 cardinal (±x, ±y) + 4 diagonal (±x,±y) directions, all dz=0.
    // Diagonal moves allow the path to go NE/NW/SE/SW directly instead of
    // producing staircase patterns that confuse the path-follower.
    static constexpr int kNumNeighbors          = 8;
    static constexpr int kHorizNeighbours[8][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},  {0, -1, 0},  // cardinal
        {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0}  // diagonal
    };
    static constexpr float kHorizCosts[8] = {
        1.0f,   1.0f,   1.0f,   1.0f,   // cardinal
        1.414f, 1.414f, 1.414f, 1.414f  // diagonal
    };

protected:
    bool do_search(const GridCell& start, const GridCell& goal,
                   std::vector<std::array<float, 3>>& out_world_path) override {
        // ── Check for reinitialisation triggers ──────────────
        bool need_init = !initialized_ || goal != last_goal_;

        // Start moved substantially → accumulate km correction
        if (initialized_ && start != last_start_) {
            km_ += heuristic(last_start_, start);
            last_start_ = start;
        }

        // Queue too large → reinitialise to avoid memory bloat
        if (initialized_ && U_.size() > 100000) {
            spdlog::info("[D*Lite] Queue size {} > 100k — reinitialising", U_.size());
            need_init = true;
        }

        // km_ too large → key recycling wastes iterations (drone moved far)
        if (initialized_ && km_ > 10.0f) {
            spdlog::info("[D*Lite] km_={:.1f} > 10 — reinitialising to avoid key churn", km_);
            need_init = true;
        }

        if (need_init) {
            initialize(start, goal);
        } else {
            // Incremental update: process changed cells
            auto changes = grid_.drain_changes();
            if (!changes.empty()) {
                // Large change sets cause cascading invalidations that are more
                // expensive than a fresh search.  Reinitialise above threshold.
                if (changes.size() > 500) {
                    spdlog::debug("[D*Lite] {} changes > threshold — reinitialising",
                                  changes.size());
                    initialize(start, goal);
                } else {
                    for (const auto& [cell, is_occupied] : changes) {
                        // Update all neighbours of the changed cell
                        for (int n = 0; n < kNumNeighbors; ++n) {
                            GridCell nb{cell.x + kHorizNeighbours[n][0],
                                        cell.y + kHorizNeighbours[n][1],
                                        cell.z + kHorizNeighbours[n][2]};
                            if (grid_.in_bounds(nb)) {
                                update_vertex(nb);
                            }
                        }
                        // Also update the changed cell itself
                        update_vertex(cell);
                    }
                }
            }
        }

        // ── Run compute_shortest_path ────────────────────────
        auto search_t0 = std::chrono::steady_clock::now();
        bool ok        = compute_shortest_path();
        auto search_ms =
            std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - search_t0)
                .count();

        if (!ok || g(last_start_) >= kInf) {
            spdlog::info("[D*Lite] No path: start=({},{},{}) goal=({},{},{}) g(start)={:.0f} "
                         "queue={} occupied={} search={:.0f}ms",
                         last_start_.x, last_start_.y, last_start_.z, last_goal_.x, last_goal_.y,
                         last_goal_.z, g(last_start_), U_.size(), grid_.occupied_count(),
                         search_ms);
            return false;
        }

        // ── Extract path ─────────────────────────────────────
        bool path_ok = extract_path(last_start_, last_goal_, out_world_path);
        if (path_ok && out_world_path.size() >= 2) {
            auto& first = out_world_path[1];  // first step (index 0 is start)
            auto& last  = out_world_path.back();
            spdlog::info("[D*Lite] Path OK: {} pts, search={:.0f}ms, "
                         "first=({:.0f},{:.0f},{:.0f}) last=({:.0f},{:.0f},{:.0f}) "
                         "g(start)={:.0f} occupied={}",
                         out_world_path.size(), search_ms, first[0], first[1], first[2], last[0],
                         last[1], last[2], g(last_start_), grid_.occupied_count());
        } else {
            spdlog::info("[D*Lite] Path extraction FAILED: search={:.0f}ms g(start)={:.1f} "
                         "occupied={}",
                         search_ms, g(last_start_), grid_.occupied_count());
        }
        return path_ok;
    }

private:
    static constexpr float kInf = 1e9f;

    // ── D* Lite key pair ─────────────────────────────────────
    struct Key {
        float k1 = 0.0f, k2 = 0.0f;

        bool operator<(const Key& o) const { return k1 < o.k1 || (k1 == o.k1 && k2 < o.k2); }
        bool operator>(const Key& o) const { return o < *this; }
    };

    // ── Priority queue entry ─────────────────────────────────
    struct QueueEntry {
        Key      key;
        GridCell cell;

        bool operator<(const QueueEntry& o) const {
            if (key < o.key) return true;
            if (o.key < key) return false;
            // Tie-break on cell to make the set order deterministic
            if (cell.x != o.cell.x) return cell.x < o.cell.x;
            if (cell.y != o.cell.y) return cell.y < o.cell.y;
            return cell.z < o.cell.z;
        }
    };

    // ── Heuristic (Euclidean) ────────────────────────────────
    static float heuristic(const GridCell& a, const GridCell& b) {
        float dx = static_cast<float>(a.x - b.x);
        float dy = static_cast<float>(a.y - b.y);
        float dz = static_cast<float>(a.z - b.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // ── Access g and rhs values ──────────────────────────────
    float g(const GridCell& s) const {
        auto it = g_.find(s);
        return (it != g_.end()) ? it->second : kInf;
    }

    float rhs(const GridCell& s) const {
        auto it = rhs_.find(s);
        return (it != rhs_.end()) ? it->second : kInf;
    }

    // ── Edge cost ────────────────────────────────────────────
    float cost(const GridCell& a, const GridCell& b) const {
        // Start and goal cells are always passable:
        // - Start: the drone is physically there; old TTL cells may linger from
        //   before the drone arrived (self-exclusion only prevents NEW insertions).
        // - Goal: snap may have placed it in a cell that later became occupied;
        //   the drone still needs to navigate toward it.
        if ((a != last_start_ && a != last_goal_ && grid_.is_occupied(a)) ||
            (b != last_start_ && b != last_goal_ && grid_.is_occupied(b)))
            return kInf;
        if (!grid_.in_bounds(a) || !grid_.in_bounds(b)) return kInf;
        // Z-band: prune cells outside the flight altitude band to prevent
        // 3D search from wasting iterations exploring irrelevant altitudes.
        if (z_band_cells_ > 0) {
            if (b.z < z_min_ || b.z > z_max_) return kInf;
        }
        int dx = std::abs(a.x - b.x);
        int dy = std::abs(a.y - b.y);
        int dz = std::abs(a.z - b.z);
        if (dx > 1 || dy > 1 || dz > 1) return kInf;
        int diag = dx + dy + dz;
        if (diag == 1) return 1.0f;
        if (diag == 2) return 1.414f;
        if (diag == 3) return 1.732f;
        return kInf;  // same cell or non-adjacent
    }

    // ── Calculate key ────────────────────────────────────────
    Key calculate_key(const GridCell& s) const {
        float g_val   = g(s);
        float rhs_val = rhs(s);
        float min_val = std::min(g_val, rhs_val);
        return {min_val + heuristic(last_start_, s) + km_, min_val};
    }

    // ── Initialize ───────────────────────────────────────────
    void initialize(const GridCell& start, const GridCell& goal_in) {
        g_.clear();
        rhs_.clear();
        U_.clear();
        queue_index_.clear();
        km_          = 0.0f;
        last_start_  = start;
        initialized_ = true;

        // 2D mode (horizontal-only neighbours): snap goal Z to start Z so the
        // search stays at a single altitude level.  The velocity command handles Z.
        GridCell goal = goal_in;
        if (goal.z != start.z) {
            spdlog::debug("[D*Lite] 2D mode: snapping goal Z {} → {}", goal.z, start.z);
            goal.z = start.z;
        }
        last_goal_ = goal;

        // Compute Z-band limits from start/goal altitudes
        z_band_cells_ = config_.z_band_cells;
        if (z_band_cells_ > 0) {
            z_min_ = std::min(start.z, goal.z) - z_band_cells_;
            z_max_ = std::max(start.z, goal.z) + z_band_cells_;
        }

        // Drain any pending changes since we're doing a full init
        grid_.drain_changes();

        rhs_[goal] = 0.0f;
        queue_insert(goal, calculate_key(goal));

        spdlog::info("[D*Lite] Init: start=({},{},{}) goal=({},{},{}) neighbors={}", start.x,
                     start.y, start.z, goal.x, goal.y, goal.z, kNumNeighbors);
    }

    // ── Update vertex ────────────────────────────────────────
    void update_vertex(const GridCell& u) {
        if (u == last_goal_) return;  // goal's rhs is always 0

        // Compute rhs as min over successors
        float min_rhs = kInf;
        for (int n = 0; n < kNumNeighbors; ++n) {
            GridCell s_prime{u.x + kHorizNeighbours[n][0], u.y + kHorizNeighbours[n][1],
                             u.z + kHorizNeighbours[n][2]};
            if (grid_.in_bounds(s_prime)) {
                float c = cost(u, s_prime);
                if (c < kInf) {
                    float val = c + g(s_prime);
                    if (val < min_rhs) min_rhs = val;
                }
            }
        }
        rhs_[u] = min_rhs;

        // Remove from queue if present
        remove_from_queue(u);

        // Re-insert if locally inconsistent
        if (g(u) != rhs(u)) {
            queue_insert(u, calculate_key(u));
        }
    }

    // ── Queue helpers (O(log N) via reverse index) ─────────────
    void queue_insert(const GridCell& cell, const Key& key) {
        auto [sit, inserted] = U_.insert({key, cell});
        if (inserted) {
            queue_index_[cell] = sit;
        }
    }

    void remove_from_queue(const GridCell& u) {
        auto it = queue_index_.find(u);
        if (it != queue_index_.end()) {
            U_.erase(it->second);
            queue_index_.erase(it);
        }
    }

    // ── Compute shortest path ────────────────────────────────
    bool compute_shortest_path() {
        const bool use_timeout = (config_.max_search_time_ms > 0.0f);
        const auto deadline    = use_timeout ? std::chrono::steady_clock::now() +
                                                std::chrono::microseconds(static_cast<int64_t>(
                                                    config_.max_search_time_ms * 1000.0f))
                                             : std::chrono::steady_clock::time_point{};

        int iterations = 0;
        while (!U_.empty()) {
            auto top_it    = U_.begin();
            Key  top_key   = top_it->key;
            Key  start_key = calculate_key(last_start_);

            // Termination: top key >= start key AND start is locally consistent
            if (!(top_key < start_key) && rhs(last_start_) == g(last_start_)) {
                break;
            }

            ++iterations;
            if (iterations > config_.max_iterations) {
                spdlog::info("[D*Lite] Hit iteration limit {}", config_.max_iterations);
                break;
            }

            // Check wall-clock timeout every 64 iterations
            if (use_timeout && (iterations & 63) == 0) {
                if (std::chrono::steady_clock::now() >= deadline) {
                    spdlog::info("[D*Lite] Timeout after {} iterations ({:.1f}ms limit)",
                                 iterations, config_.max_search_time_ms);
                    break;
                }
            }

            GridCell u     = top_it->cell;
            Key      k_old = top_it->key;
            Key      k_new = calculate_key(u);
            U_.erase(top_it);
            queue_index_.erase(u);

            if (k_old < k_new) {
                // Key has changed — re-insert with updated key
                queue_insert(u, k_new);
            } else if (g(u) > rhs(u)) {
                // Over-consistent → make consistent
                g_[u] = rhs(u);
                // Update predecessors
                for (int n = 0; n < kNumNeighbors; ++n) {
                    GridCell s{u.x + kHorizNeighbours[n][0], u.y + kHorizNeighbours[n][1],
                               u.z + kHorizNeighbours[n][2]};
                    if (grid_.in_bounds(s)) {
                        update_vertex(s);
                    }
                }
            } else {
                // Under-consistent → reset and update
                g_[u] = kInf;
                update_vertex(u);
                for (int n = 0; n < kNumNeighbors; ++n) {
                    GridCell s{u.x + kHorizNeighbours[n][0], u.y + kHorizNeighbours[n][1],
                               u.z + kHorizNeighbours[n][2]};
                    if (grid_.in_bounds(s)) {
                        update_vertex(s);
                    }
                }
            }
        }

        spdlog::debug("[D*Lite] search: {} iters, queue={}, g(start)={:.0f}", iterations, U_.size(),
                      g(last_start_));
        return g(last_start_) < kInf;
    }

    // ── Extract path from start to goal ──────────────────────
    bool extract_path(const GridCell& start, const GridCell& goal,
                      std::vector<std::array<float, 3>>& out) const {
        out.clear();
        out.push_back(grid_.grid_to_world(start));

        GridCell  current   = start;
        int       steps     = 0;
        const int max_steps = config_.max_iterations;

        while (current != goal && steps < max_steps) {
            ++steps;
            float    best_cost = kInf;
            GridCell best_next = current;

            for (int n = 0; n < kNumNeighbors; ++n) {
                GridCell nb{current.x + kHorizNeighbours[n][0], current.y + kHorizNeighbours[n][1],
                            current.z + kHorizNeighbours[n][2]};
                if (!grid_.in_bounds(nb)) continue;
                float c = cost(current, nb);
                if (c >= kInf) continue;
                float total = c + g(nb);
                if (total < best_cost) {
                    best_cost = total;
                    best_next = nb;
                }
            }

            if (best_next == current) {
                // No progress — path broken
                spdlog::debug("[D*Lite] Path extraction stuck at ({},{},{})", current.x, current.y,
                              current.z);
                return false;
            }

            current = best_next;
            out.push_back(grid_.grid_to_world(current));
        }

        if (current != goal) {
            spdlog::debug("[D*Lite] Path extraction hit step limit");
            return false;
        }

        spdlog::debug("[D*Lite] Path found: {} waypoints", out.size());
        return out.size() >= 2;
    }

    // ── D* Lite state ────────────────────────────────────────
    std::unordered_map<GridCell, float, GridCellHash>                          g_;
    std::unordered_map<GridCell, float, GridCellHash>                          rhs_;
    std::set<QueueEntry>                                                       U_;
    std::unordered_map<GridCell, std::set<QueueEntry>::iterator, GridCellHash> queue_index_;
    GridCell                                                                   last_start_{};
    GridCell                                                                   last_goal_{};
    float                                                                      km_          = 0.0f;
    bool                                                                       initialized_ = false;
    int                                                                        z_band_cells_ = 0;
    int                                                                        z_min_        = -50;
    int                                                                        z_max_        = 50;
};

}  // namespace drone::planner
