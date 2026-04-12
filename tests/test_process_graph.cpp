// tests/test_process_graph.cpp
// Unit tests for ProcessGraph dual-edge dependency graph (Phase 4, #92).
//
// Tests topological sort (launch_order), cascade_targets traversal,
// cycle detection, edge isolation, and the default edge table.

#include "util/process_graph.h"
#include "util/restart_policy.h"

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using drone::util::ProcessGraph;

// ═══════════════════════════════════════════════════════════
// Empty graph
// ═══════════════════════════════════════════════════════════

TEST(ProcessGraph, EmptyGraphLaunchOrderIsEmpty) {
    ProcessGraph graph;
    auto         order = graph.launch_order();
    EXPECT_TRUE(order.empty());
}

TEST(ProcessGraph, EmptyGraphValidates) {
    ProcessGraph graph;
    EXPECT_TRUE(graph.validate());
}

TEST(ProcessGraph, EmptyGraphCascadeTargetsEmpty) {
    ProcessGraph graph;
    auto         targets = graph.cascade_targets("nonexistent");
    EXPECT_TRUE(targets.empty());
}

// ═══════════════════════════════════════════════════════════
// Single process
// ═══════════════════════════════════════════════════════════

TEST(ProcessGraph, SingleProcessLaunchOrder) {
    ProcessGraph graph;
    graph.add_process("alpha");

    auto order = graph.launch_order();
    ASSERT_EQ(order.size(), 1u);
    EXPECT_EQ(order[0], "alpha");
}

TEST(ProcessGraph, SingleProcessNoCascade) {
    ProcessGraph graph;
    graph.add_process("alpha");
    auto targets = graph.cascade_targets("alpha");
    EXPECT_TRUE(targets.empty());
}

// ═══════════════════════════════════════════════════════════
// Topological sort — launch_after edges
// ═══════════════════════════════════════════════════════════

TEST(ProcessGraph, LinearChainLaunchOrder) {
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_process("C");
    graph.add_launch_dep("B", "A");  // B after A
    graph.add_launch_dep("C", "B");  // C after B

    auto order = graph.launch_order();
    ASSERT_EQ(order.size(), 3u);

    // A must come before B, B before C
    auto pos_a = std::find(order.begin(), order.end(), "A") - order.begin();
    auto pos_b = std::find(order.begin(), order.end(), "B") - order.begin();
    auto pos_c = std::find(order.begin(), order.end(), "C") - order.begin();
    EXPECT_LT(pos_a, pos_b);
    EXPECT_LT(pos_b, pos_c);
}

TEST(ProcessGraph, DiamondDependencyLaunchOrder) {
    ProcessGraph graph;
    graph.add_process("root");
    graph.add_process("left");
    graph.add_process("right");
    graph.add_process("sink");
    graph.add_launch_dep("left", "root");
    graph.add_launch_dep("right", "root");
    graph.add_launch_dep("sink", "left");
    graph.add_launch_dep("sink", "right");

    auto order = graph.launch_order();
    ASSERT_EQ(order.size(), 4u);

    auto pos_root  = std::find(order.begin(), order.end(), "root") - order.begin();
    auto pos_left  = std::find(order.begin(), order.end(), "left") - order.begin();
    auto pos_right = std::find(order.begin(), order.end(), "right") - order.begin();
    auto pos_sink  = std::find(order.begin(), order.end(), "sink") - order.begin();

    EXPECT_LT(pos_root, pos_left);
    EXPECT_LT(pos_root, pos_right);
    EXPECT_LT(pos_left, pos_sink);
    EXPECT_LT(pos_right, pos_sink);
}

TEST(ProcessGraph, IndependentProcessesAllAppearInOrder) {
    ProcessGraph graph;
    graph.add_process("x");
    graph.add_process("y");
    graph.add_process("z");

    auto order = graph.launch_order();
    ASSERT_EQ(order.size(), 3u);
    // All three should appear (order among them is deterministic due to sorting)
    EXPECT_NE(std::find(order.begin(), order.end(), "x"), order.end());
    EXPECT_NE(std::find(order.begin(), order.end(), "y"), order.end());
    EXPECT_NE(std::find(order.begin(), order.end(), "z"), order.end());
}

// ═══════════════════════════════════════════════════════════
// Cascade targets — restart_cascade edges
// ═══════════════════════════════════════════════════════════

TEST(ProcessGraph, DirectCascadeTargets) {
    ProcessGraph graph;
    graph.add_process("comms");
    graph.add_process("mission_planner");
    graph.add_process("payload_manager");
    graph.add_cascade("comms", "mission_planner");
    graph.add_cascade("comms", "payload_manager");

    auto targets = graph.cascade_targets("comms");
    ASSERT_EQ(targets.size(), 2u);
    EXPECT_NE(std::find(targets.begin(), targets.end(), "mission_planner"), targets.end());
    EXPECT_NE(std::find(targets.begin(), targets.end(), "payload_manager"), targets.end());
}

TEST(ProcessGraph, TransitiveCascade) {
    // A → B → C (cascade edges)
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_process("C");
    graph.add_cascade("A", "B");
    graph.add_cascade("B", "C");

    auto targets = graph.cascade_targets("A");
    ASSERT_EQ(targets.size(), 2u);
    EXPECT_NE(std::find(targets.begin(), targets.end(), "B"), targets.end());
    EXPECT_NE(std::find(targets.begin(), targets.end(), "C"), targets.end());
}

TEST(ProcessGraph, DeepTransitiveCascade) {
    // A → B → C → D (cascade edges, 3 levels)
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_process("C");
    graph.add_process("D");
    graph.add_cascade("A", "B");
    graph.add_cascade("B", "C");
    graph.add_cascade("C", "D");

    auto targets = graph.cascade_targets("A");
    ASSERT_EQ(targets.size(), 3u);
    EXPECT_NE(std::find(targets.begin(), targets.end(), "B"), targets.end());
    EXPECT_NE(std::find(targets.begin(), targets.end(), "C"), targets.end());
    EXPECT_NE(std::find(targets.begin(), targets.end(), "D"), targets.end());
}

TEST(ProcessGraph, CascadeDoesNotIncludeSelf) {
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_cascade("A", "B");

    auto targets = graph.cascade_targets("A");
    EXPECT_EQ(std::find(targets.begin(), targets.end(), "A"), targets.end());
}

TEST(ProcessGraph, NoCascadeReturnsEmpty) {
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");

    auto targets = graph.cascade_targets("A");
    EXPECT_TRUE(targets.empty());
}

// ═══════════════════════════════════════════════════════════
// Edge type separation — cascade vs launch
// ═══════════════════════════════════════════════════════════

TEST(ProcessGraph, CascadeDoesNotAffectLaunchOrder) {
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_cascade("A", "B");  // cascade only, not launch_after

    auto order = graph.launch_order();
    ASSERT_EQ(order.size(), 2u);
    // Both should appear, but B doesn't depend on A for launch
    // (they're independent for launch ordering)
}

TEST(ProcessGraph, LaunchDepDoesNotAffectCascade) {
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_launch_dep("B", "A");  // launch only, not cascade

    auto targets = graph.cascade_targets("A");
    EXPECT_TRUE(targets.empty());  // No cascade edges
}

TEST(ProcessGraph, PerceptionCrashDoesNotCascadeToVideoCapture) {
    // Reproduce the specific scenario from Issue #92:
    // perception crashes → video_capture should NOT be in cascade targets
    ProcessGraph graph;
    graph.populate_defaults();

    auto targets = graph.cascade_targets("perception");
    EXPECT_TRUE(targets.empty());
    // Explicitly check video_capture is NOT in targets
    EXPECT_EQ(std::find(targets.begin(), targets.end(), "video_capture"), targets.end());
}

TEST(ProcessGraph, CommsCrashCascadesToMissionAndPayload) {
    // comms crashes → mission_planner + payload_manager should cascade
    ProcessGraph graph;
    graph.populate_defaults();

    auto targets = graph.cascade_targets("comms");
    ASSERT_EQ(targets.size(), 2u);
    EXPECT_NE(std::find(targets.begin(), targets.end(), "mission_planner"), targets.end());
    EXPECT_NE(std::find(targets.begin(), targets.end(), "payload_manager"), targets.end());

    // Should NOT include slam_vio_nav, perception, video_capture
    EXPECT_EQ(std::find(targets.begin(), targets.end(), "slam_vio_nav"), targets.end());
    EXPECT_EQ(std::find(targets.begin(), targets.end(), "perception"), targets.end());
    EXPECT_EQ(std::find(targets.begin(), targets.end(), "video_capture"), targets.end());
}

TEST(ProcessGraph, SlamCrashCascadesToPerceptionOnly) {
    ProcessGraph graph;
    graph.populate_defaults();

    auto targets = graph.cascade_targets("slam_vio_nav");
    ASSERT_EQ(targets.size(), 1u);
    EXPECT_EQ(targets[0], "perception");
}

// ═══════════════════════════════════════════════════════════
// Cycle detection
// ═══════════════════════════════════════════════════════════

TEST(ProcessGraph, LaunchCycleReturnsEmptyOrder) {
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_launch_dep("A", "B");
    graph.add_launch_dep("B", "A");

    auto order = graph.launch_order();
    EXPECT_TRUE(order.empty());  // Cycle detected
}

TEST(ProcessGraph, LaunchCycleFailsValidation) {
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_launch_dep("A", "B");
    graph.add_launch_dep("B", "A");

    EXPECT_FALSE(graph.validate());
}

TEST(ProcessGraph, DanglingReferenceFailsValidation) {
    ProcessGraph graph;
    graph.add_process("A");
    // Add launch dep referencing non-existent process
    graph.add_launch_dep("A", "nonexistent");

    EXPECT_FALSE(graph.validate());
}

// ═══════════════════════════════════════════════════════════
// Default edge table (ADR-004 §2.3)
// ═══════════════════════════════════════════════════════════

TEST(ProcessGraph, DefaultEdgeTableLaunchOrder) {
    ProcessGraph graph;
    graph.populate_defaults();

    auto order = graph.launch_order();
    ASSERT_EQ(order.size(), 6u);

    // video_capture and comms have no dependencies — should be first
    auto pos_vc   = std::find(order.begin(), order.end(), "video_capture") - order.begin();
    auto pos_comm = std::find(order.begin(), order.end(), "comms") - order.begin();
    auto pos_perc = std::find(order.begin(), order.end(), "perception") - order.begin();
    auto pos_slam = std::find(order.begin(), order.end(), "slam_vio_nav") - order.begin();
    auto pos_mp   = std::find(order.begin(), order.end(), "mission_planner") - order.begin();
    auto pos_pm   = std::find(order.begin(), order.end(), "payload_manager") - order.begin();

    // perception after video_capture
    EXPECT_LT(pos_vc, pos_perc);
    // slam_vio_nav after perception
    EXPECT_LT(pos_perc, pos_slam);
    // mission_planner after comms AND slam_vio_nav
    EXPECT_LT(pos_comm, pos_mp);
    EXPECT_LT(pos_slam, pos_mp);
    // payload_manager after comms
    EXPECT_LT(pos_comm, pos_pm);
}

TEST(ProcessGraph, DefaultEdgeTableValidates) {
    ProcessGraph graph;
    graph.populate_defaults();
    EXPECT_TRUE(graph.validate());
}

TEST(ProcessGraph, DefaultEdgeTableHasAllProcesses) {
    ProcessGraph graph;
    graph.populate_defaults();

    EXPECT_EQ(graph.size(), 6u);
    EXPECT_TRUE(graph.has_process("video_capture"));
    EXPECT_TRUE(graph.has_process("perception"));
    EXPECT_TRUE(graph.has_process("slam_vio_nav"));
    EXPECT_TRUE(graph.has_process("comms"));
    EXPECT_TRUE(graph.has_process("mission_planner"));
    EXPECT_TRUE(graph.has_process("payload_manager"));
}

// ═══════════════════════════════════════════════════════════
// Utility methods
// ═══════════════════════════════════════════════════════════

TEST(ProcessGraph, ProcessesReturnsSortedList) {
    ProcessGraph graph;
    graph.add_process("z_proc");
    graph.add_process("a_proc");
    graph.add_process("m_proc");

    auto procs = graph.processes();
    ASSERT_EQ(procs.size(), 3u);
    EXPECT_EQ(procs[0], "a_proc");
    EXPECT_EQ(procs[1], "m_proc");
    EXPECT_EQ(procs[2], "z_proc");
}

TEST(ProcessGraph, LaunchDepsReturnsDirectParents) {
    ProcessGraph graph;
    graph.add_process("A");
    graph.add_process("B");
    graph.add_process("C");
    graph.add_launch_dep("C", "A");
    graph.add_launch_dep("C", "B");

    auto deps = graph.launch_deps("C");
    ASSERT_EQ(deps.size(), 2u);
    EXPECT_NE(std::find(deps.begin(), deps.end(), "A"), deps.end());
    EXPECT_NE(std::find(deps.begin(), deps.end(), "B"), deps.end());
}

TEST(ProcessGraph, HasProcessReturnsFalseForUnknown) {
    ProcessGraph graph;
    graph.add_process("known");
    EXPECT_TRUE(graph.has_process("known"));
    EXPECT_FALSE(graph.has_process("unknown"));
}
