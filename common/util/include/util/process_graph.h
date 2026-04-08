// common/util/include/util/process_graph.h
// Phase 4 (#92) — Dual-edge process dependency graph.
//
// The graph uses two separate edge types because launch ordering and
// restart cascading are not the same relationship:
//
//   launch_after[]    = "this process needs data from X, so X must be
//                        alive before launch."
//   restart_cascade[] = "if this process dies, also stop+restart Y
//                        because Y may hold stale state."
//
// ProcessGraph provides:
//   - launch_order()       → topological sort of launch_after edges
//   - cascade_targets(p)   → transitive closure of restart_cascade from p
//   - validate()           → cycle detection + reference validity check
//   - populate_defaults()  → wire up the standard 6-process edge table
//
// Data is stored in adjacency lists using string keys. The graph is built
// once at startup from config, so allocation efficiency is not critical.
#pragma once

#include "util/ilogger.h"

#include <algorithm>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone::util {

class ProcessGraph {
public:
    /// Register a process name. Must be called before adding edges.
    void add_process(const std::string& name) { nodes_.insert(name); }

    /// Add a launch dependency: @p child must launch after @p parent.
    /// Both names must have been registered via add_process().
    void add_launch_dep(const std::string& child, const std::string& parent) {
        launch_deps_[child].push_back(parent);
    }

    /// Add a restart cascade edge: when @p source dies, @p target must
    /// also be stopped and restarted (after source is back).
    void add_cascade(const std::string& source, const std::string& target) {
        cascade_deps_[source].push_back(target);
    }

    /// Topological sort of the launch_after graph.
    /// Returns processes in launch order (dependencies first).
    /// Returns empty vector if the graph contains a cycle.
    [[nodiscard]] std::vector<std::string> launch_order() const {
        // Build in-degree map
        std::unordered_map<std::string, int>                      in_degree;
        std::unordered_map<std::string, std::vector<std::string>> forward;

        for (const auto& node : nodes_) {
            in_degree[node] = 0;
        }

        for (const auto& [child, parents] : launch_deps_) {
            for (const auto& parent : parents) {
                if (nodes_.count(parent) == 0 || nodes_.count(child) == 0) continue;
                forward[parent].push_back(child);
                in_degree[child]++;
            }
        }

        // Kahn's algorithm
        std::queue<std::string> ready;
        for (const auto& [node, deg] : in_degree) {
            if (deg == 0) {
                ready.push(node);
            }
        }

        // Deterministic ordering: sort the initial queue
        // (Use a sorted container for reproducibility)
        std::vector<std::string> sorted_ready;
        while (!ready.empty()) {
            sorted_ready.push_back(ready.front());
            ready.pop();
        }
        std::sort(sorted_ready.begin(), sorted_ready.end());
        for (const auto& s : sorted_ready) {
            ready.push(s);
        }

        std::vector<std::string> order;
        while (!ready.empty()) {
            auto current = ready.front();
            ready.pop();
            order.push_back(current);

            if (forward.count(current) > 0) {
                // Sort children for deterministic ordering
                auto children = forward[current];
                std::sort(children.begin(), children.end());
                for (const auto& child : children) {
                    in_degree[child]--;
                    if (in_degree[child] == 0) {
                        ready.push(child);
                    }
                }
            }
        }

        // If not all nodes are in order, there's a cycle
        if (order.size() != nodes_.size()) {
            DRONE_LOG_ERROR("[ProcessGraph] Cycle detected in launch dependencies — "
                            "got {} of {} nodes",
                            order.size(), nodes_.size());
            return {};
        }

        return order;
    }

    /// Get all transitive cascade targets when @p process dies.
    /// Returns the set of processes that must be stopped and restarted.
    /// Does NOT include @p process itself in the result.
    [[nodiscard]] std::vector<std::string> cascade_targets(const std::string& process) const {
        std::unordered_set<std::string> visited;
        std::vector<std::string>        result;

        // BFS through cascade edges
        std::queue<std::string> queue;
        if (cascade_deps_.count(process) > 0) {
            for (const auto& target : cascade_deps_.at(process)) {
                if (nodes_.count(target) > 0 && visited.insert(target).second) {
                    queue.push(target);
                }
            }
        }

        while (!queue.empty()) {
            auto current = queue.front();
            queue.pop();
            result.push_back(current);

            if (cascade_deps_.count(current) > 0) {
                for (const auto& target : cascade_deps_.at(current)) {
                    if (nodes_.count(target) > 0 && visited.insert(target).second) {
                        queue.push(target);
                    }
                }
            }
        }

        // Sort for deterministic output
        std::sort(result.begin(), result.end());
        return result;
    }

    /// Validate the graph: check for cycles and dangling references.
    /// Returns true if the graph is valid.
    [[nodiscard]] bool validate() const {
        // Check all referenced nodes exist
        for (const auto& [child, parents] : launch_deps_) {
            if (nodes_.count(child) == 0) {
                DRONE_LOG_ERROR("[ProcessGraph] launch_after references unknown process: {}",
                                child);
                return false;
            }
            for (const auto& parent : parents) {
                if (nodes_.count(parent) == 0) {
                    DRONE_LOG_ERROR("[ProcessGraph] launch_after references unknown dependency: {}",
                                    parent);
                    return false;
                }
            }
        }

        for (const auto& [source, targets] : cascade_deps_) {
            if (nodes_.count(source) == 0) {
                DRONE_LOG_ERROR("[ProcessGraph] restart_cascade references unknown source: {}",
                                source);
                return false;
            }
            for (const auto& target : targets) {
                if (nodes_.count(target) == 0) {
                    DRONE_LOG_ERROR("[ProcessGraph] restart_cascade references unknown target: {}",
                                    target);
                    return false;
                }
            }
        }

        // Check for launch_after cycles via topological sort
        auto order = launch_order();
        if (order.empty() && !nodes_.empty()) {
            return false;  // Cycle detected
        }

        // Check for cascade cycles (same algorithm on cascade edges)
        // Cascade cycles aren't inherently invalid (they'd just cause
        // complete restart of the connected component), but they likely
        // indicate a config error. We detect but warn rather than fail.
        for (const auto& node : nodes_) {
            auto targets = cascade_targets(node);
            for (const auto& t : targets) {
                auto reverse_targets = cascade_targets(t);
                if (std::find(reverse_targets.begin(), reverse_targets.end(), node) !=
                    reverse_targets.end()) {
                    DRONE_LOG_WARN("[ProcessGraph] Cascade cycle detected: {} ↔ {}", node, t);
                }
            }
        }

        return true;
    }

    /// Populate the default edge table from ADR-004 §2.3.
    void populate_defaults() {
        add_process("video_capture");
        add_process("perception");
        add_process("slam_vio_nav");
        add_process("comms");
        add_process("mission_planner");
        add_process("payload_manager");

        // Launch dependencies (launch_after)
        add_launch_dep("perception", "video_capture");
        add_launch_dep("slam_vio_nav", "perception");
        add_launch_dep("mission_planner", "comms");
        add_launch_dep("mission_planner", "slam_vio_nav");
        add_launch_dep("payload_manager", "comms");

        // Restart cascade edges
        add_cascade("slam_vio_nav", "perception");
        add_cascade("comms", "mission_planner");
        add_cascade("comms", "payload_manager");
    }

    /// Get all registered process names (sorted for deterministic output).
    [[nodiscard]] std::vector<std::string> processes() const {
        std::vector<std::string> result(nodes_.begin(), nodes_.end());
        std::sort(result.begin(), result.end());
        return result;
    }

    /// Get number of registered processes.
    [[nodiscard]] size_t size() const { return nodes_.size(); }

    /// Check if a process is registered.
    [[nodiscard]] bool has_process(const std::string& name) const { return nodes_.count(name) > 0; }

    /// Get launch dependencies for a process (direct parents only).
    [[nodiscard]] std::vector<std::string> launch_deps(const std::string& process) const {
        if (launch_deps_.count(process) > 0) {
            auto deps = launch_deps_.at(process);
            std::sort(deps.begin(), deps.end());
            return deps;
        }
        return {};
    }

private:
    std::unordered_set<std::string>                           nodes_;
    std::unordered_map<std::string, std::vector<std::string>> launch_deps_;   // child → parents
    std::unordered_map<std::string, std::vector<std::string>> cascade_deps_;  // source → targets
};

}  // namespace drone::util
