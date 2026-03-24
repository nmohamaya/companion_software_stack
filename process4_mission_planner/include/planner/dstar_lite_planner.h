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
                        for (int n = 0; n < 26; ++n) {
                            GridCell nb{cell.x + kNeighbours[n][0], cell.y + kNeighbours[n][1],
                                        cell.z + kNeighbours[n][2]};
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
        bool ok = compute_shortest_path();

        if (!ok || g(last_start_) >= kInf) {
            spdlog::debug("[D*Lite] No path found — direct line fallback");
            return false;
        }

        // ── Extract path ─────────────────────────────────────
        return extract_path(last_start_, last_goal_, out_world_path);
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
        if (grid_.is_occupied(a) || grid_.is_occupied(b)) return kInf;
        if (!grid_.in_bounds(a) || !grid_.in_bounds(b)) return kInf;
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
    void initialize(const GridCell& start, const GridCell& goal) {
        g_.clear();
        rhs_.clear();
        U_.clear();
        queue_index_.clear();
        km_          = 0.0f;
        last_start_  = start;
        last_goal_   = goal;
        initialized_ = true;

        // Drain any pending changes since we're doing a full init
        grid_.drain_changes();

        rhs_[goal] = 0.0f;
        queue_insert(goal, calculate_key(goal));

        spdlog::debug("[D*Lite] Initialized: start=({},{},{}) goal=({},{},{})", start.x, start.y,
                      start.z, goal.x, goal.y, goal.z);
    }

    // ── Update vertex ────────────────────────────────────────
    void update_vertex(const GridCell& u) {
        if (u == last_goal_) return;  // goal's rhs is always 0

        // Compute rhs as min over successors
        float min_rhs = kInf;
        for (int n = 0; n < 26; ++n) {
            GridCell s_prime{u.x + kNeighbours[n][0], u.y + kNeighbours[n][1],
                             u.z + kNeighbours[n][2]};
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
                spdlog::debug("[D*Lite] Hit iteration limit {}", config_.max_iterations);
                break;
            }

            // Check wall-clock timeout every 64 iterations
            if (use_timeout && (iterations & 63) == 0) {
                if (std::chrono::steady_clock::now() >= deadline) {
                    spdlog::debug("[D*Lite] Timeout after {} iterations ({:.1f}ms limit)",
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
                for (int n = 0; n < 26; ++n) {
                    GridCell s{u.x + kNeighbours[n][0], u.y + kNeighbours[n][1],
                               u.z + kNeighbours[n][2]};
                    if (grid_.in_bounds(s)) {
                        update_vertex(s);
                    }
                }
            } else {
                // Under-consistent → reset and update
                g_[u] = kInf;
                update_vertex(u);
                for (int n = 0; n < 26; ++n) {
                    GridCell s{u.x + kNeighbours[n][0], u.y + kNeighbours[n][1],
                               u.z + kNeighbours[n][2]};
                    if (grid_.in_bounds(s)) {
                        update_vertex(s);
                    }
                }
            }
        }

        spdlog::debug("[D*Lite] compute_shortest_path: {} iterations, queue size {}", iterations,
                      U_.size());
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

            for (int n = 0; n < 26; ++n) {
                GridCell nb{current.x + kNeighbours[n][0], current.y + kNeighbours[n][1],
                            current.z + kNeighbours[n][2]};
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
};

}  // namespace drone::planner
