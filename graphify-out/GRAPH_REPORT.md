# Graph Report - .  (2026-04-08)

## Corpus Check
- Large corpus: 879 files · ~577,729 words. Semantic extraction will be expensive (many Claude tokens). Consider running on a subfolder, or use --no-semantic to run AST-only.

## Summary
- 2173 nodes · 3345 edges · 62 communities detected
- Extraction: 71% EXTRACTED · 29% INFERRED · 0% AMBIGUOUS · INFERRED: 956 edges (avg confidence: 0.52)
- Token cost: 0 input · 0 output

## God Nodes (most connected - your core abstractions)
1. `Console` - 81 edges
2. `Git` - 76 edges
3. `PipelineStateData` - 72 edges
4. `GitHub` - 64 edges
5. `Claude` - 45 edges
6. `PipelineState` - 45 edges
7. `TestConsole` - 40 edges
8. `Notifier` - 37 edges
9. `TestTransitionFunction` - 36 edges
10. `BuildSystem` - 35 edges

## Surprising Connections (you probably didn't know these)
- `main()` --calls--> `print_usage()`  [INFERRED]
  process2_perception/src/main.cpp → tools/fault_injector/main.cpp
- `main()` --calls--> `send_fc_command()`  [INFERRED]
  process2_perception/src/main.cpp → process4_mission_planner/src/main.cpp
- `Tests for orchestrator.config — role lookup, tier mapping, boundaries.` --uses--> `ModelTier`  [INFERRED]
  tests/test_orchestrator/test_config.py → scripts/orchestrator/config.py
- `Test the role configuration registry.` --uses--> `ModelTier`  [INFERRED]
  tests/test_orchestrator/test_config.py → scripts/orchestrator/config.py
- `Test role categorization by tier.` --uses--> `ModelTier`  [INFERRED]
  tests/test_orchestrator/test_config.py → scripts/orchestrator/config.py

## Hyperedges (group relationships)
- **7-Process Autonomous Drone Software Stack** — cmakelists_process1, cmakelists_process2, cmakelists_process3, cmakelists_process4, cmakelists_process5, cmakelists_process6, cmakelists_process7, claude_concept_seven_processes [EXTRACTED 1.00]
- **P1 IPC Output Channels** — claude_ipc_channel_drone_mission_cam, claude_ipc_channel_drone_stereo_cam, cmakelists_process1 [EXTRACTED 1.00]
- **P4 Mission Planner Input Channels** — claude_ipc_channel_detected_objects, claude_ipc_channel_slam_pose, claude_ipc_channel_fc_state, claude_ipc_channel_gcs_commands, claude_ipc_channel_payload_status, claude_ipc_channel_system_health, cmakelists_process4 [EXTRACTED 1.00]
- **P2 Perception Algorithm Implementations** — ack_bytetrack, ack_sort, ack_hungarian, e2e_talk_p2_perception [EXTRACTED 1.00]
- **Three-Layer Watchdog Components** — readme_concept_thread_heartbeat, readme_concept_thread_watchdog, readme_concept_process_manager, readme_concept_systemd_units, claude_concept_three_layer_watchdog [EXTRACTED 1.00]
- **Remote Pipeline Monitoring Modules** — agent_report_notifications_py, agent_report_tmux_py, agent_report_pipeline_monitor_py, agent_report_checkpoints_py, agent_report_deploy_issue_py, agent_report_cli_py [EXTRACTED 1.00]
- **CMake Required Dependencies** — cmakelists_dep_threads, cmakelists_dep_spdlog, cmakelists_dep_eigen3, cmakelists_dep_nlohmann_json, cmakelists_dep_zenohc [EXTRACTED 1.00]
- **CMake Optional Dependencies** — cmakelists_dep_mavsdk, cmakelists_dep_opencv, cmakelists_dep_gazebo, cmakelists_dep_systemd [EXTRACTED 1.00]
- **P1 publishes video frames to P2 (mission) and P3 (stereo) via IPC** — video_capture_design_doc, perception_design_doc, slam_vio_nav_design_doc, ipc_design_shm_types [EXTRACTED 1.00]
- **Three-Layer Watchdog: Thread + Process + OS layers** — hardening_thread_heartbeat_registry, hardening_thread_watchdog, system_monitor_processmanager, hardening_systemd_units, system_monitor_three_layer_watchdog [EXTRACTED 1.00]
- **VIO Pipeline: Feature Extraction -> Stereo Matching -> IMU Preintegration -> Pose** — slam_vio_ifeatureextractor, slam_vio_istereomatcher, slam_vio_imupreintegrator, slam_vio_iviobackend, slam_vio_vio_health [EXTRACTED 1.00]
- **P2 Perception Pipeline: Inference -> Tracker -> Fusion** — perception_inference_thread, perception_tracker_thread, perception_fusion_thread, perception_idetector, perception_bytetrack [EXTRACTED 1.00]
- **HAL Common Interfaces: ICamera, IFCLink, IGCSLink, IGimbal, IIMUSource, IRadar** — hal_design_icamera, hal_design_ifclink, hal_design_igcslink, hal_design_igimbal, hal_design_iimusource, hal_design_iradar [EXTRACTED 1.00]
- **Mission-critical IPC channels: trajectory_cmd, fc_commands, slam_pose, detected_objects** — ipc_design_topic_mapping, mission_planner_design_doc, comms_design_p5, slam_vio_nav_design_doc, perception_design_doc [EXTRACTED 1.00]

## Communities

### Community 0 - "Build System & Orchestrator"
Cohesion: 0.02
Nodes (176): BuildError, BuildSystem, Build system subprocess wrapper — cmake, ctest, clang-format, lcov.  Wraps build, Parse ctest output for key metrics., Check clang-format compliance for given files. Returns issue count., Check if clang-format-18 is available., Get line coverage percentage from lcov. Returns None if unavailable., Extract expected test count from tests/TESTS.md.          The Total row format i (+168 more)

### Community 1 - "Mission Planner & Tracking"
Cohesion: 0.02
Nodes (61): escalate(), is_process_dead(), planner(), cmd_battery(), cmd_fc_link(), cmd_gcs_command(), cmd_mission_upload(), cmd_sequence() (+53 more)

### Community 2 - "Perception Pipeline"
Cohesion: 0.02
Nodes (38): get_role(), Single source of truth for agent roles, models, and pipeline configuration.  Rep, Look up a role by name. Raises KeyError if not found., Resolve the project root directory (parent of scripts/)., Configuration for a single agent role., resolve_project_dir(), RoleConfig, load_model() (+30 more)

### Community 3 - "Pipeline State Machine"
Cohesion: 0.02
Nodes (51): InvalidTransitionError, is_checkpoint(), is_terminal(), PipelineState, Pipeline FSM — 12-state guided workflow for deploy-issue --pipeline.  Replaces t, Raised when an action is not valid for the current state., Compute the next state given current state and action.      Returns the next Pip, Get the list of valid actions for a given state. (+43 more)

### Community 4 - "Comms Design & IPC"
Cohesion: 0.02
Nodes (106): GCS Correlation ID Assignment, Comms Fault Injection (Battery/FC Link Loss Override), FC Receive Thread (10 Hz), FC Transmit Thread (20 Hz), GCS Receive Thread (2 Hz), GCS Transmit Thread (2 Hz), IFCLink Interface, IGCSLink Interface (+98 more)

### Community 5 - "Test Infrastructure"
Cohesion: 0.03
Nodes (51): project_dir(), Shared fixtures for orchestrator tests., A TestConsole with no pre-loaded inputs., A temporary project directory with minimal structure for testing., test_io(), IOProtocol, Console I/O abstraction for testable interactive output.  Provides IOProtocol in, Test double that captures output and provides scripted input.      Usage in test (+43 more)

### Community 6 - "Notifications & Monitoring"
Cohesion: 0.03
Nodes (36): from_env(), Notifier, NotifyConfig, NotifyEvent, Push notification support via ntfy.sh for pipeline checkpoints.  Sends mobile-fr, Send push notifications via ntfy.sh.      Gracefully degrades — notification fai, Check if this event type should trigger a notification., Send notification when pipeline reaches a checkpoint.          Returns True if n (+28 more)

### Community 7 - "Architecture Decisions (ADR)"
Cohesion: 0.03
Nodes (91): ADR-001 Decision: Select Zenoh over iceoryx2, ADR-001: IPC Framework Selection — iceoryx2 vs Zenoh, ADR-001 Option A: iceoryx2, ADR-001 Option B: Zenoh, ADR-001 Rationale: Network Transparency Non-Negotiable, ADR-001 Rationale: Zero-Copy SHM for Video Frames, ADR-002 MessageBusVariant (std::variant dispatch), ADR-002: Modular IPC Backend Architecture (+83 more)

### Community 8 - "Debug & Issue Tracking"
Cohesion: 0.03
Nodes (88): Debug Logging for Issue #234 D* Lite Drift, D* Lite Search Diagnostics Logging, Mission State Tick Planner vs Avoider Velocity Log, DIAG Logging in tick_navigate, Debug Logging for Epic #237 Smooth Flight Path Tuning, Dormant Pool Pollution Fix, Gazebo Run Results for Epic #237 (3 Runs), Phase A Survey-Phase Obstacle Mapping (Epic #237) (+80 more)

### Community 9 - "Acknowledgments & Libraries"
Cohesion: 0.03
Nodes (84): ByteTrack Algorithm (Zhang et al., ECCV 2022), D* Lite Algorithm (Koenig & Likhachev, AAAI 2002), Hungarian Algorithm (Kuhn 1955, Munkres 1957), IMU Pre-integration (Forster et al., 2017), Library: Eigen3 (MPL 2.0), Library: Gazebo (Apache 2.0, optional), Library: MAVSDK (BSD 3-Clause, optional), Library: nlohmann/json (MIT) (+76 more)

### Community 10 - "Agent Boundary Checking"
Cohesion: 0.04
Nodes (45): check_boundaries(), _matches_any(), Agent boundary enforcement command.  Replaces check-agent-boundaries.sh (180 lin, Return list of files that violate role boundaries.      A file is allowed if it, Check if filepath matches any of the patterns.      Supports:       - Directory, Check agent boundaries for changed files on current branch., run(), Branch and worktree cleanup command.  Replaces cleanup-branches.sh (119 lines). (+37 more)

### Community 11 - "Pipeline Monitor"
Cohesion: 0.04
Nodes (39): Pipeline monitoring commands — list, attach, status.  Provides the `python -m or, Kill a pipeline tmux session.      Note: no per-user authorization check — assum, List all active pipeline tmux sessions., Attach to a running pipeline tmux session., Show status of a pipeline or all pipelines., run_attach(), run_kill(), run_list() (+31 more)

### Community 12 - "IPC Publishers & Subscribers"
Cohesion: 0.05
Nodes (19): LatencyTracker(), next_power_of_two(), to_us(), TEST(), unique_topic(), TEST(), wait_for(), poll_receive() (+11 more)

### Community 13 - "Agent Routing & Dispatch"
Cohesion: 0.03
Nodes (24): branch_name_for_issue(), branch_prefix_for_issue(), Label-to-role triage and diff-based review agent routing.  Replaces:   - deploy-, Generate a branch name prefix from issue title and type., Generate a full branch name for an issue., Result of review agent routing decision., Determine which review and test agents to launch based on PR diff.      Always i, Result of issue label triage. (+16 more)

### Community 14 - "Cross-cutting Concepts"
Cohesion: 0.04
Nodes (64): Cross-Domain Agent Handoff Protocol, 13-Agent Concurrent Development Pipeline, Amdahl's Law Applied to Verification Pipeline, Atomic and Memory Ordering (std::atomic), Factory Pattern, Git Worktree Isolation for Concurrent Agents, Graceful Degradation (Optional Dependencies), Hardware Abstraction Layer Interfaces (+56 more)

### Community 15 - "SLAM/VIO Navigation"
Cohesion: 0.05
Nodes (19): exp_map(), integrate(), skew(), transition_health(), update_health(), handle_cascade_stops(), reap_children(), split_args() (+11 more)

### Community 16 - "Boundary Tests"
Cohesion: 0.04
Nodes (15): Tests for orchestrator.commands.boundaries — agent boundary enforcement., Test boundary checks for feature-integration role., Test the pattern matching helper., Test boundary checks for feature-infra-core role., Test boundary checks for feature-infra-platform role., Test with multiple files, some allowed, some not., Test boundary checks for feature-perception role., Test boundary checks for feature-nav role. (+7 more)

### Community 17 - "Sensor Fusion Engine"
Cohesion: 0.08
Nodes (26): cam_intr_from(), make_radar_det(), make_test_calib(), make_test_tracked(), set_matching_radar(), TEST(), test_estimate_depth(), adopt_camera() (+18 more)

### Community 18 - "Config Management"
Cohesion: 0.07
Nodes (18): ModelTier, Agent model tier — determines cost and capability., ConfigTest, Tests for orchestrator.config — role lookup, tier mapping, boundaries., Test label routing configuration., Test pipeline version string., Test project directory resolution., Test the role configuration registry. (+10 more)

### Community 19 - "D* Lite Path Planner"
Cohesion: 0.08
Nodes (18): calculate_key(), compute_shortest_path(), cost(), diagonal_passable(), g(), heuristic(), initialize(), queue_insert() (+10 more)

### Community 20 - "ByteTrack Object Tracker"
Cohesion: 0.1
Nodes (6): compute_iou(), compute_iou_cost_matrix(), update(), make_det(), make_det_list(), TEST()

### Community 21 - "CMake Build Targets"
Cohesion: 0.19
Nodes (25): Common HAL Library (drone_hal), Common IPC Library (drone_ipc), Common Recorder Library (drone_recorder), Common Util Library (drone_util), Eigen3, Gazebo Transport (gz-transport13/gz-msgs10), Google Test (GTest), MAVSDK (optional) (+17 more)

### Community 22 - "Config Validation"
Cohesion: 0.11
Nodes (6): drone(), err(), error(), ok(), Result(), ConfigValidatorTest

### Community 23 - "Cross-Agent Context"
Cohesion: 0.11
Nodes (12): CrossAgentContext, format_context(), gather_context(), Cross-agent context gathering — shared state injected into agent prompts.  Repla, Gathered context from shared state files., Gather cross-agent context from shared state files.      Reads each file if it e, Format cross-agent context for injection into an agent prompt.      Returns a st, Tests for orchestrator.context — cross-agent context gathering. (+4 more)

### Community 24 - "CLI Command Parser"
Cohesion: 0.1
Nodes (5): build_parser(), _cmd_version(), main(), CLI entry point — argparse subcommands for all orchestrator commands.  Usage: py, Build the top-level argument parser with all subcommands.

### Community 25 - "Agent Changelog"
Cohesion: 0.18
Nodes (15): Agent Changelog, PR #359: Multi-developer GitHub Collaboration (Issue #358), PR #374: Remote Pipeline Monitoring (Issue #371), PR #375: --json-logs flag (Issue #299), checkpoints.py — Pipeline Checkpoint Functions, cli.py — Pipeline Subcommand Group, deploy_issue.py — --tmux/--notify Flags, AGENT_REPORT: Issue #371 Remote Pipeline Monitoring (+7 more)

### Community 26 - "Legacy FC/GCS Links"
Cohesion: 0.14
Nodes (0): 

### Community 27 - "GCS Client Tool"
Cohesion: 0.24
Nodes (11): build_zenoh_config(), decode_wire_header(), format_bytes(), main(), on_sample(), print_stats(), Human-readable byte count., Print accumulated statistics. (+3 more)

### Community 28 - "Gimbal Controller"
Cohesion: 0.18
Nodes (0): 

### Community 29 - "Multi-Agent Pipeline ADR"
Cohesion: 0.24
Nodes (10): ADR-005: Parallel AI Agent Workflow via git worktree, ADR-005 Rationale: Worktrees Prevent Branch Collision and Build Interference, ADR-010 Agent Roster (13 Agents: 6 Feature, 4 Review, 2 Test, 1 Ops), ADR-010 Anti-Hallucination Measures (validate-session.sh), ADR-010 Boundary Enforcement (Pre-commit, CI, Tool Restrictions), ADR-010 Cross-Agent Context System (4 Layers), ADR-010: Multi-Agent Pipeline Architecture, ADR-010 Pipeline Mode 12-State Machine with 5 Human Checkpoints (+2 more)

### Community 30 - "Community 30"
Cohesion: 0.29
Nodes (8): ADR-007 Error Handling Strategy, Error Handling Design Document, ErrorCode Enum, Error Handling Rationale: No exceptions on flight path, nodiscard enforced, Result<T,E> Monadic Error Type, VIOResult<T> Domain Error Alias, VoidResult Specialisation, VIOError Structure and Error Codes

### Community 31 - "Community 31"
Cohesion: 0.33
Nodes (6): CI Pipeline Setup Guide, CI Format Gate (clang-format-18), CI Sanitizer Matrix (ASan/TSan/UBSan), deploy/run_ci_local.sh (Local CI Script), Rationale: CI fail-fast:false Allows All Sanitizer Legs to Report, Rationale: Format Gate Runs First to Avoid Wasting Build Minutes

### Community 32 - "Community 32"
Cohesion: 0.7
Nodes (4): make_origin_pose(), make_pose(), make_single_object(), TEST()

### Community 33 - "Community 33"
Cohesion: 0.5
Nodes (2): square_100(), TEST()

### Community 34 - "Community 34"
Cohesion: 0.5
Nodes (0): 

### Community 35 - "Community 35"
Cohesion: 0.5
Nodes (3): Log session command — captures session transcript.  Replaces log-session.sh (31, Run a command and log its output to tasks/sessions/.      Returns the command's, run()

### Community 36 - "Community 36"
Cohesion: 1.0
Nodes (0): 

### Community 37 - "Community 37"
Cohesion: 1.0
Nodes (0): 

### Community 38 - "Community 38"
Cohesion: 1.0
Nodes (1): CLI command implementations — one module per command.

### Community 39 - "Community 39"
Cohesion: 1.0
Nodes (2): Lesson: Check Method Signatures Before Writing Tests, Lesson: Check Struct Field Names Before Writing Tests

### Community 40 - "Community 40"
Cohesion: 1.0
Nodes (0): 

### Community 41 - "Community 41"
Cohesion: 1.0
Nodes (1): All captured output as a single string.

### Community 42 - "Community 42"
Cohesion: 1.0
Nodes (1): Build config from environment variables.          NTFY_TOPIC:    topic name (req

### Community 43 - "Community 43"
Cohesion: 1.0
Nodes (1): README: Timing Diagram — Publish Rates

### Community 44 - "Community 44"
Cohesion: 1.0
Nodes (1): Library: Google Test (BSD 3-Clause)

### Community 45 - "Community 45"
Cohesion: 1.0
Nodes (1): Library: PX4-Autopilot (BSD 3-Clause, dev/test)

### Community 46 - "Community 46"
Cohesion: 1.0
Nodes (1): Lesson: Fully Qualify Namespaces in Tests

### Community 47 - "Community 47"
Cohesion: 1.0
Nodes (1): Lesson: Explicitly Include All Standard Library Headers

### Community 48 - "Community 48"
Cohesion: 1.0
Nodes (1): Lesson: Follow tasks/todo.md Workflow From Session Start

### Community 49 - "Community 49"
Cohesion: 1.0
Nodes (1): Active Work Tracker

### Community 50 - "Community 50"
Cohesion: 1.0
Nodes (1): SPSC Ring Buffer (Intra-Process)

### Community 51 - "Community 51"
Cohesion: 1.0
Nodes (1): Service Channels (Request-Reply)

### Community 52 - "Community 52"
Cohesion: 1.0
Nodes (1): IPC Topic Mapping (SHM Segment to Zenoh Key)

### Community 53 - "Community 53"
Cohesion: 1.0
Nodes (1): P7 Supervised Mode (fork+exec parent)

### Community 54 - "Community 54"
Cohesion: 1.0
Nodes (1): safe_name_copy Utility (Replaces strncpy)

### Community 55 - "Community 55"
Cohesion: 1.0
Nodes (1): CorrelationContext (64-bit Correlation IDs)

### Community 56 - "Community 56"
Cohesion: 1.0
Nodes (1): Mission Camera Thread (30 Hz)

### Community 57 - "Community 57"
Cohesion: 1.0
Nodes (1): Stereo Camera Thread (30 Hz)

### Community 58 - "Community 58"
Cohesion: 1.0
Nodes (1): ImuRingBuffer (Mutex Bounded Ring)

### Community 59 - "Community 59"
Cohesion: 1.0
Nodes (1): P3 Thread Rate Clamping (Issue #220)

### Community 60 - "Community 60"
Cohesion: 1.0
Nodes (1): Current State Metrics (Roadmap)

### Community 61 - "Community 61"
Cohesion: 1.0
Nodes (1): GCS Client Requirements

## Knowledge Gaps
- **409 isolated node(s):** `PoseDoubleBuffer`, `Decode a 24-byte wire header from raw bytes.      Returns a dict with header fie`, `Human-readable byte count.`, `Print accumulated statistics.`, `Callback for every Zenoh sample received.` (+404 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Community 36`** (2 nodes): `CMakeCXXCompilerId.cpp`, `main()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 37`** (2 nodes): `test_crasher.cpp`, `main()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 38`** (2 nodes): `__init__.py`, `CLI command implementations — one module per command.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 39`** (2 nodes): `Lesson: Check Method Signatures Before Writing Tests`, `Lesson: Check Struct Field Names Before Writing Tests`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 40`** (1 nodes): `compiler_depend.ts`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 41`** (1 nodes): `All captured output as a single string.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 42`** (1 nodes): `Build config from environment variables.          NTFY_TOPIC:    topic name (req`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 43`** (1 nodes): `README: Timing Diagram — Publish Rates`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 44`** (1 nodes): `Library: Google Test (BSD 3-Clause)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 45`** (1 nodes): `Library: PX4-Autopilot (BSD 3-Clause, dev/test)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 46`** (1 nodes): `Lesson: Fully Qualify Namespaces in Tests`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 47`** (1 nodes): `Lesson: Explicitly Include All Standard Library Headers`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 48`** (1 nodes): `Lesson: Follow tasks/todo.md Workflow From Session Start`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 49`** (1 nodes): `Active Work Tracker`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 50`** (1 nodes): `SPSC Ring Buffer (Intra-Process)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 51`** (1 nodes): `Service Channels (Request-Reply)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 52`** (1 nodes): `IPC Topic Mapping (SHM Segment to Zenoh Key)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 53`** (1 nodes): `P7 Supervised Mode (fork+exec parent)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 54`** (1 nodes): `safe_name_copy Utility (Replaces strncpy)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 55`** (1 nodes): `CorrelationContext (64-bit Correlation IDs)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 56`** (1 nodes): `Mission Camera Thread (30 Hz)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 57`** (1 nodes): `Stereo Camera Thread (30 Hz)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 58`** (1 nodes): `ImuRingBuffer (Mutex Bounded Ring)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 59`** (1 nodes): `P3 Thread Rate Clamping (Issue #220)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 60`** (1 nodes): `Current State Metrics (Roadmap)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 61`** (1 nodes): `GCS Client Requirements`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `PipelineState` connect `Pipeline State Machine` to `Build System & Orchestrator`, `Test Infrastructure`?**
  _High betweenness centrality (0.161) - this node is a cross-community bridge._
- **Why does `Git` connect `Agent Boundary Checking` to `Build System & Orchestrator`, `Test Infrastructure`?**
  _High betweenness centrality (0.058) - this node is a cross-community bridge._
- **Why does `Console` connect `Build System & Orchestrator` to `CLI Command Parser`, `Agent Boundary Checking`, `Pipeline Monitor`, `Test Infrastructure`?**
  _High betweenness centrality (0.057) - this node is a cross-community bridge._
- **Are the 68 inferred relationships involving `Console` (e.g. with `TestTestConsole` and `TestConsoleInstantiation`) actually correct?**
  _`Console` has 68 INFERRED edges - model-reasoned connections that need verification._
- **Are the 44 inferred relationships involving `Git` (e.g. with `CheckStatus` and `CheckResult`) actually correct?**
  _`Git` has 44 INFERRED edges - model-reasoned connections that need verification._
- **Are the 66 inferred relationships involving `PipelineStateData` (e.g. with `TestPipelineStateDataDefaults` and `TestStateAccessors`) actually correct?**
  _`PipelineStateData` has 66 INFERRED edges - model-reasoned connections that need verification._
- **Are the 47 inferred relationships involving `GitHub` (e.g. with `RoutingDecision` and `ReviewAssessment`) actually correct?**
  _`GitHub` has 47 INFERRED edges - model-reasoned connections that need verification._