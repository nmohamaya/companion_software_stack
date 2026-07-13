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
#include "util/ilogger.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace drone::planner {

class DStarLitePlanner final : public GridPlannerBase {
public:
    explicit DStarLitePlanner(const GridPlannerConfig& config = {}) : GridPlannerBase(config) {}

    std::string name() const override { return "DStarLitePlanner"; }

    // Issue #764 — telemetry: how often the greedy g-descent stalled and the
    // A* fallback ran / recovered a path. A persistently non-zero `recovered`
    // count means D*Lite's incremental g-field is leaving holes for this
    // workload (the fallback is masking it correctly, but worth noting).
    [[nodiscard]] uint64_t astar_fallback_count() const noexcept { return astar_fallback_count_; }
    [[nodiscard]] uint64_t astar_fallback_recovered() const noexcept {
        return astar_fallback_recovered_;
    }

    // Issue #821 — planner responsiveness instrumentation (Phase 1: OBSERVE
    // ONLY, no behaviour change). Accessed from the planner thread only, so
    // plain counters (same pattern as the #764 astar counters above).
    /// Ticks that returned no-path → hover.
    [[nodiscard]] uint64_t no_path_count() const noexcept { return no_path_count_; }
    /// ...of those, how many were cases where the shadow-A* probe DID find a path — i.e.
    /// D*Lite's incremental g(start)=inf was a false negative. High ⇒ an
    /// A*-fallback-on-no-path fix would eliminate that many hovers.
    [[nodiscard]] uint64_t no_path_astar_recoverable() const noexcept {
        return no_path_astar_recoverable_;
    }
    /// D*Lite from-scratch re-inits + causes (goal-flip ⇒ snap-goal churn). NB the
    /// cause counters are diagnostic tallies, not a strict partition of reinit_count():
    /// the very first (cold-start) init has no cause bucket, and coincident triggers on
    /// one tick bump more than one cause — so Σ(causes) may differ from reinit_count().
    [[nodiscard]] uint64_t reinit_count() const noexcept { return reinit_count_; }
    [[nodiscard]] uint64_t reinit_goal_flip() const noexcept { return reinit_goal_flip_; }
    [[nodiscard]] uint64_t reinit_km() const noexcept { return reinit_km_; }
    [[nodiscard]] uint64_t reinit_queue() const noexcept { return reinit_queue_; }
    [[nodiscard]] uint64_t reinit_changes() const noexcept { return reinit_changes_; }
    /// Peak compute_shortest_path() wall-time (µs) — the ~983 ms re-init spikes.
    [[nodiscard]] uint64_t max_search_us() const noexcept { return max_search_us_; }

    // Test seam (Issue #764): run the A* fallback extraction directly so the
    // guaranteed-correct fallback can be unit-tested without first reproducing
    // D*Lite's greedy-stall state. Sets the start/goal passability anchors that
    // cost() keys off. DO NOT call from production — production reaches A* only
    // via do_search()'s greedy-stall fallback path.
    bool extract_astar_for_test(const GridCell& start, const GridCell& goal,
                                std::vector<std::array<float, 3>>& out) {
        last_start_ = start;
        last_goal_  = goal;
        return extract_path_astar(start, goal, out);
    }

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
        // Issue #821 — attribute WHY we re-init. A from-scratch search is the
        // ~983 ms cost that collapses the 10 Hz loop; goal-flip is driven by
        // the snap-goal churn (grid_planner_base snap invalidation as static
        // cell count drifts), so counting it separately tells us whether
        // snap-goal hysteresis would be the high-value fix. Diagnostic only.
        if (initialized_ && goal != last_goal_) ++reinit_goal_flip_;

        // Start moved substantially → accumulate km correction
        if (initialized_ && start != last_start_) {
            km_ += heuristic(last_start_, start);
            last_start_ = start;
        }

        // Queue too large → reinitialise to avoid memory bloat
        if (initialized_ && U_.size() > 100000) {
            DRONE_LOG_INFO("[D*Lite] Queue size {} > 100k — reinitialising", U_.size());
            need_init = true;
            ++reinit_queue_;  // #821 diagnostic
        }

        // km_ too large → key recycling wastes iterations (drone moved far)
        if (initialized_ && km_ > 10.0f) {
            DRONE_LOG_INFO("[D*Lite] km_={:.1f} > 10 — reinitialising to avoid key churn", km_);
            need_init = true;
            ++reinit_km_;  // #821 diagnostic
        }

        if (need_init) {
            initialize(start, goal);
            ++reinit_count_;  // #821 diagnostic — total from-scratch searches
        } else {
            // Incremental update: process changed cells
            auto changes = grid_.drain_changes();
            if (!changes.empty()) {
                // Large change sets cause cascading invalidations that are more
                // expensive than a fresh search.  Reinitialise above threshold.
                if (changes.size() > 500) {
                    DRONE_LOG_DEBUG("[D*Lite] {} changes > threshold — reinitialising",
                                    changes.size());
                    initialize(start, goal);
                    ++reinit_count_;    // #821 diagnostic — total from-scratch searches
                    ++reinit_changes_;  // #821 diagnostic — cause: large change set
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
        // #821 diagnostic — track the peak search cost. The re-init spikes
        // (~983 ms observed) are what collapse the loop from 10 Hz to ~1 Hz.
        const auto search_us = static_cast<uint64_t>(search_ms * 1000.0f);
        if (search_us > max_search_us_) max_search_us_ = search_us;

        if (!ok || g(last_start_) >= kInf) {
            // Issue #821 shadow-A* probe (DIAGNOSTIC ONLY — result discarded,
            // flight behaviour byte-identical to before). WHY: D*Lite's
            // incremental g-field can report g(start)=inf even when a path
            // exists (stale state after a re-init). A complete A* over the same
            // grid/cost is authoritative for path *existence*. Running it here
            // and discarding the result measures the false-negative rate — i.e.
            // exactly how many of these hovers an A*-fallback-on-no-path fix
            // would remove — WITHOUT changing behaviour yet. HOW: extract_path_
            // astar is const + uses its own local search structures (verified),
            // so it cannot perturb D*Lite's g_/U_/km_ state or out_world_path.
            ++no_path_count_;
            int probe_pts = 0;
            if (config_.diagnostics_enabled) {
                std::vector<std::array<float, 3>> probe;  // throwaway — never used to steer
                if (extract_path_astar(last_start_, last_goal_, probe) && probe.size() >= 2) {
                    ++no_path_astar_recoverable_;
                    probe_pts = static_cast<int>(probe.size());
                }
            }
            // Provenance: static vs dynamic split of the sealing cells (is the
            // block a real inflated-obstacle edge or residual promoted ghosts?).
            // NB grid_.occupied_count() is the DYNAMIC layer only and static_count()
            // the STATIC layer — they are disjoint, so total occupied = sum. (PR #822
            // mislabelled `occupied=`/`dynamic=`: it printed the dynamic count under
            // `occupied=` and `dynamic − static` under `dynamic=`. Issue #824 fix.)
            const auto dynamic_cells = grid_.occupied_count();
            const auto static_cells  = grid_.static_count();
            DRONE_LOG_INFO(
                "[D*Lite] No path: start=({},{},{}) goal=({},{},{}) g(start)={:.0f} queue={} "
                "dynamic={} static={} total_occupied={} shadow_astar_pts={} search={:.0f}ms",
                last_start_.x, last_start_.y, last_start_.z, last_goal_.x, last_goal_.y,
                last_goal_.z, g(last_start_), U_.size(), dynamic_cells, static_cells,
                dynamic_cells + static_cells, probe_pts, search_ms);
            return false;
        }

        // ── Extract path ─────────────────────────────────────
        bool path_ok = extract_path(last_start_, last_goal_, out_world_path);
        if (path_ok && out_world_path.size() >= 2) {
            auto& first = out_world_path[1];  // first step (index 0 is start)
            auto& last  = out_world_path.back();
            DRONE_LOG_INFO("[D*Lite] Path OK: {} pts, search={:.0f}ms, "
                           "first=({:.0f},{:.0f},{:.0f}) last=({:.0f},{:.0f},{:.0f}) "
                           "g(start)={:.0f} occupied={}",
                           out_world_path.size(), search_ms, first[0], first[1], first[2], last[0],
                           last[1], last[2], g(last_start_), grid_.occupied_count());
            return true;
        }

        // Issue #764 — the greedy g-descent in extract_path() stalled even
        // though compute_shortest_path reported a finite g(start) (a path
        // exists).  D*Lite's g-field is lazy: cells off the realized descent
        // keep g=inf, so the greedy walk can hit a "g-hole" and report "no
        // progress" on an otherwise-traversable grid.  Fall back to a
        // guaranteed-correct A* over the SAME grid/cost() — it honours
        // occupancy/inflation/corner-cutting identically (never bypasses an
        // obstacle).  Reached only on the stall path, so the common case keeps
        // D*Lite's fast incremental walk.  See DESIGN_RATIONALE DR-051.
        ++astar_fallback_count_;
        if (extract_path_astar(last_start_, last_goal_, out_world_path) &&
            out_world_path.size() >= 2) {
            ++astar_fallback_recovered_;
            DRONE_LOG_INFO("[D*Lite] greedy stall → A* fallback recovered {} pts "
                           "(search={:.0f}ms g(start)={:.1f} occupied={})",
                           out_world_path.size(), search_ms, g(last_start_),
                           grid_.occupied_count());
            return true;
        }

        DRONE_LOG_INFO("[D*Lite] Path extraction FAILED (greedy + A* fallback): "
                       "search={:.0f}ms g(start)={:.1f} occupied={}",
                       search_ms, g(last_start_), grid_.occupied_count());
        return false;
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

    // ── Corner-cutting guard (Issue #258) ─────────────────────
    // For a diagonal move from `a` to `b`, verify that all cardinal-step
    // intermediaries are passable.  For a 2D diagonal (dx,dy != 0) the
    // two intermediaries are (a.x+sdx, a.y, a.z) and (a.x, a.y+sdy, a.z).
    // For a 3D corner move all three face-adjacent cells must be free.
    // This prevents the planner from routing the drone through L-shaped
    // obstacle gaps where it would physically clip the obstacle.
    [[nodiscard]] bool diagonal_passable(const GridCell& a, const GridCell& b) const {
        int sdx = b.x - a.x;  // -1, 0, or +1
        int sdy = b.y - a.y;
        int sdz = b.z - a.z;

        // Check each axis pair that forms a diagonal component.
        // For each non-zero axis, the face-adjacent cell along that axis
        // must be unoccupied (unless it is start/goal, which are always passable).
        auto cell_blocked = [&](const GridCell& c) -> bool {
            if (c == last_start_ || c == last_goal_) return false;
            return grid_.is_occupied(c);
        };

        if (sdx != 0 && sdy != 0) {
            // XY diagonal component: check both cardinal steps
            if (cell_blocked({a.x + sdx, a.y, a.z}) || cell_blocked({a.x, a.y + sdy, a.z}))
                return false;
        }
        if (sdx != 0 && sdz != 0) {
            // XZ diagonal component
            if (cell_blocked({a.x + sdx, a.y, a.z}) || cell_blocked({a.x, a.y, a.z + sdz}))
                return false;
        }
        if (sdy != 0 && sdz != 0) {
            // YZ diagonal component
            if (cell_blocked({a.x, a.y + sdy, a.z}) || cell_blocked({a.x, a.y, a.z + sdz}))
                return false;
        }
        return true;
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
        if (diag == 2) {
            // Corner-cutting guard (Issue #258): a diagonal move must not
            // squeeze between two occupied cardinal neighbours.  Check that
            // BOTH intermediate cells along the two non-zero axes are free.
            if (!diagonal_passable(a, b)) return kInf;
            return 1.414f;
        }
        if (diag == 3) {
            // 3D corner move: all three face-adjacent intermediaries must be free.
            if (!diagonal_passable(a, b)) return kInf;
            return 1.732f;
        }
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
            DRONE_LOG_DEBUG("[D*Lite] 2D mode: snapping goal Z {} → {}", goal.z, start.z);
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

        DRONE_LOG_INFO("[D*Lite] Init: start=({},{},{}) goal=({},{},{}) neighbors={}", start.x,
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
                DRONE_LOG_INFO("[D*Lite] Hit iteration limit {}", config_.max_iterations);
                break;
            }

            // Check wall-clock timeout every 64 iterations
            if (use_timeout && (iterations & 63) == 0) {
                if (std::chrono::steady_clock::now() >= deadline) {
                    DRONE_LOG_INFO("[D*Lite] Timeout after {} iterations ({:.1f}ms limit)",
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

        DRONE_LOG_DEBUG("[D*Lite] search: {} iters, queue={}, g(start)={:.0f}", iterations,
                        U_.size(), g(last_start_));
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
                DRONE_LOG_DEBUG("[D*Lite] Path extraction stuck at ({},{},{})", current.x,
                                current.y, current.z);
                return false;
            }

            current = best_next;
            out.push_back(grid_.grid_to_world(current));
        }

        if (current != goal) {
            DRONE_LOG_DEBUG("[D*Lite] Path extraction hit step limit");
            return false;
        }

        DRONE_LOG_DEBUG("[D*Lite] Path found: {} waypoints", out.size());
        return out.size() >= 2;
    }

    // ── A* fallback extraction (Issue #764) ───────────────────
    // Guaranteed-correct path search over the SAME grid and cost() as D*Lite,
    // used only when the greedy g-descent in extract_path() stalls on a lazy
    // g-field hole. Reuses cost() (occupancy/inflation/corner-cutting/z-band).
    // Heuristic: octile distance with the SAME unit costs cost() uses (cardinal
    // 1.0, diagonal 1.414) — admissible AND consistent for this 8-connected grid,
    // so the closed set is provably safe (Copilot PR #788: plain Euclidean
    // slightly overestimates the 1.414 diagonal and is not strictly admissible).
    // Finds a path if one exists *within* max_iterations / max_search_time_ms;
    // never bypasses an obstacle.
    bool extract_path_astar(const GridCell& start, const GridCell& goal,
                            std::vector<std::array<float, 3>>& out) const {
        out.clear();

        // Trivial path: already at the goal (Copilot PR #788 — the size>=2
        // enforcement below would otherwise report a 1-point path as failure and
        // trigger a spurious hover). Emit a degenerate 2-point "hold" path.
        if (start == goal) {
            const auto w = grid_.grid_to_world(start);
            out.push_back(w);
            out.push_back(w);
            return true;
        }

        // Octile heuristic consistent with cost() (cardinal 1.0, diagonal 1.414).
        auto octile = [](const GridCell& a, const GridCell& b) {
            const int dx   = std::abs(a.x - b.x);
            const int dy   = std::abs(a.y - b.y);
            const int dmin = std::min(dx, dy);
            const int dmax = std::max(dx, dy);
            return 1.414f * static_cast<float>(dmin) + 1.0f * static_cast<float>(dmax - dmin);
        };

        struct ANode {
            float    f = 0.0f;
            GridCell cell{};
            bool     operator<(const ANode& o) const { return f > o.f; }  // min-heap
        };
        std::priority_queue<ANode>                           open;
        std::unordered_map<GridCell, float, GridCellHash>    gscore;
        std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
        std::unordered_set<GridCell, GridCellHash>           closed;

        gscore[start] = 0.0f;
        open.push({octile(start, goal), start});

        const int  max_steps   = config_.max_iterations;
        const bool use_timeout = (config_.max_search_time_ms > 0.0f);
        const auto deadline    = use_timeout ? std::chrono::steady_clock::now() +
                                                std::chrono::microseconds(static_cast<int64_t>(
                                                    config_.max_search_time_ms * 1000.0f))
                                             : std::chrono::steady_clock::time_point{};
        int        expansions  = 0;

        while (!open.empty()) {
            const GridCell cur = open.top().cell;
            open.pop();
            if (cur == goal) {
                std::vector<GridCell> rev{goal};
                bool                  reached_start = false;
                for (GridCell c = goal; c != start;) {
                    auto it = came_from.find(c);
                    if (it == came_from.end()) break;  // broken chain — incomplete
                    c = it->second;
                    rev.push_back(c);
                    if (c == start) {
                        reached_start = true;
                        break;
                    }
                }
                // Copilot PR #788: a broken came_from chain must NOT yield a
                // "valid" path that doesn't start at `start` — that would feed the
                // controller a path beginning mid-grid. Treat as no-path.
                if (!reached_start) {
                    DRONE_LOG_DEBUG("[D*Lite] A* reconstruction incomplete — no valid path");
                    out.clear();
                    return false;
                }
                out.reserve(rev.size());
                for (auto rit = rev.rbegin(); rit != rev.rend(); ++rit) {
                    out.push_back(grid_.grid_to_world(*rit));
                }
                return out.size() >= 2;
            }
            if (!closed.insert(cur).second) continue;  // already expanded (consistent heuristic)
            if (++expansions > max_steps) {
                DRONE_LOG_DEBUG("[D*Lite] A* fallback hit step limit {}", max_steps);
                return false;
            }
            if (use_timeout && (expansions & 63) == 0 &&
                std::chrono::steady_clock::now() >= deadline) {
                DRONE_LOG_DEBUG("[D*Lite] A* fallback timeout");
                return false;
            }
            const float cur_g = gscore[cur];
            for (int n = 0; n < kNumNeighbors; ++n) {
                GridCell nb{cur.x + kHorizNeighbours[n][0], cur.y + kHorizNeighbours[n][1],
                            cur.z + kHorizNeighbours[n][2]};
                if (closed.count(nb)) continue;
                const float c = cost(cur, nb);
                if (c >= kInf) continue;
                const float tentative = cur_g + c;
                auto        it        = gscore.find(nb);
                if (it == gscore.end() || tentative < it->second) {
                    gscore[nb]    = tentative;
                    came_from[nb] = cur;
                    open.push({tentative + octile(nb, goal), nb});
                }
            }
        }
        return false;  // genuinely no path
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

    // Issue #764 — A* fallback telemetry (single-threaded planning loop).
    uint64_t astar_fallback_count_     = 0;
    uint64_t astar_fallback_recovered_ = 0;

    // Issue #821 — planner responsiveness instrumentation (Phase 1).
    // WHY: the return-leg hover was diagnosed four ways from aggregate logs
    // (no cell positions, no "why did the search fail"). These counters make
    // the two candidate root causes MEASURABLE before we commit a fix —
    // (a) no_path_astar_recoverable_ = how often g(start)=inf is a false
    //     negative a complete A* would rescue, and
    // (b) reinit_goal_flip_ = the snap-goal churn that forces expensive
    //     from-scratch searches (max_search_us_ captures the resulting spikes).
    // HOW: updated on the planner thread inside do_search() (single-writer, so
    // plain uint64_t like the #764 astar counters); read by the periodic
    // [PlannerDiag] emit in P4 main.cpp.
    uint64_t no_path_count_             = 0;
    uint64_t no_path_astar_recoverable_ = 0;
    uint64_t reinit_count_              = 0;
    uint64_t reinit_goal_flip_          = 0;
    uint64_t reinit_km_                 = 0;
    uint64_t reinit_queue_              = 0;
    uint64_t reinit_changes_            = 0;
    uint64_t max_search_us_             = 0;
};

}  // namespace drone::planner
