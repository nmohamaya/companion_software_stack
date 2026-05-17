# Documentation

Routing by goal — pick the row that matches what you're trying to do. `docs/` is organised by [Diátaxis](https://diataxis.fr/) quadrant: `tutorials/` (learn), `how-to/` (do), `reference/` (look up), `explanation/` (understand) — plus the project-specific `adr/`, `architecture/`, `design/`, and `tracking/`.

| I want to… | Start here |
|---|---|
| **Install + run the stack locally** | [`how-to/INSTALL.md`](how-to/INSTALL.md) · [`tutorials/GETTING_STARTED.md`](tutorials/GETTING_STARTED.md) · [`tutorials/DEV_MACHINE_SETUP.md`](tutorials/DEV_MACHINE_SETUP.md) |
| **Run scenarios in Gazebo SITL** | [`architecture/SIMULATION_ARCHITECTURE.md`](architecture/SIMULATION_ARCHITECTURE.md) (Tier 1/2) · `config/scenarios/*.json` (canonical list) |
| **Run scenarios in Cosys-AirSim (Tier 3)** | [`how-to/COSYS_SETUP.md`](how-to/COSYS_SETUP.md) · [`architecture/COSYS_SIMULATION_ARCHITECTURE.md`](architecture/COSYS_SIMULATION_ARCHITECTURE.md) · [ADR-011](adr/ADR-011-cosys-airsim-photorealistic-simulation.md) |
| **Understand the per-process design** | [`design/`](design/) — one doc per process (`video_capture`, `perception`, `slam_vio_nav`, `mission_planner`, `comms`, `payload_manager`, `system_monitor`) |
| **Understand the IPC wire format** | [`design/ipc_design.md`](design/ipc_design.md) · [`design/API.md`](design/API.md) · [`architecture/ipc-key-expressions.md`](architecture/ipc-key-expressions.md) · `common/ipc/include/ipc/ipc_types.h` (authoritative source) |
| **Understand the HAL** | [`design/hal_design.md`](design/hal_design.md) · [ADR-006](adr/ADR-006-hal-hardware-abstraction-strategy.md) · `common/hal/include/hal/` (authoritative source) |
| **Understand SLAM/VIO** | [`design/slam_vio_nav_design.md`](design/slam_vio_nav_design.md) · [ADR-014](adr/ADR-014-stereo-inertial-vio-algorithm-selection.md) — the SWVIO living implementation guide `design/swvio_implementation.md` is currently a local-only draft on the other dev machine and lands with the first SwvioBackend PR |
| **Understand perception (PATH A)** | [`design/perception_design.md`](design/perception_design.md) · [ADR-013](adr/ADR-013-stereo-radar-redundancy-vs-fusion.md) — the in-flight `design/perception_architecture.md` and `design/perception_v2_detailed_design.md` are local-only drafts pending commit from the other dev machine |
| **Understand safety / watchdog / fault recovery** | [`design/hardening-design.md`](design/hardening-design.md) · [`architecture/process-health-monitoring.md`](architecture/process-health-monitoring.md) · [ADR-004](adr/ADR-004-process-thread-watchdog-architecture.md) |
| **Look up a config key** | `config/default.json` (defaults) · `common/util/include/util/config_keys.h` (declarations) · [`reference/config_reference.md`](reference/config_reference.md) (reference) · [`how-to/CONFIG_GUIDE.md`](how-to/CONFIG_GUIDE.md) (how-to) |
| **Develop a feature / fix a bug** | [`how-to/DEVELOPMENT_WORKFLOW.md`](how-to/DEVELOPMENT_WORKFLOW.md) — per-PR loop |
| **Land a large epic (10+ PRs)** | [`how-to/INTEGRATION_ROLLUP_WORKFLOW.md`](how-to/INTEGRATION_ROLLUP_WORKFLOW.md) · [ADR-015](adr/ADR-015-integration-branch-rollup-strategy.md) |
| **Use the multi-agent pipeline** | [`explanation/MULTI_AGENT_GUIDE.md`](explanation/MULTI_AGENT_GUIDE.md) · [`explanation/AGENT_HANDOFF.md`](explanation/AGENT_HANDOFF.md) · [ADR-010](adr/ADR-010-multi-agent-pipeline-architecture.md) · `.claude/agents/` (roster) |
| **Use the remote pipeline / cloud runners** | [`how-to/REMOTE_PIPELINE_GUIDE.md`](how-to/REMOTE_PIPELINE_GUIDE.md) |
| **Understand C++ patterns used here** | [`reference/CPP_PATTERNS_GUIDE.md`](reference/CPP_PATTERNS_GUIDE.md) — `Result<T,E>`, RAII, concurrency tiering · [ADR-007](adr/ADR-007-error-handling.md) |
| **Understand CI / sanitizers** | [`how-to/CI_SETUP.md`](how-to/CI_SETUP.md) — gate ordering, sanitizer discipline, `GTEST_SKIP` patterns · [`tracking/CI_ISSUES.md`](tracking/CI_ISSUES.md) |
| **Avoid bash/git footguns** | [`explanation/SAFETY_CRITICAL_SHELL_DISCIPLINE.md`](explanation/SAFETY_CRITICAL_SHELL_DISCIPLINE.md) |
| **Understand the AI/ML training opt-out** | [`explanation/ai-training-opt-out.md`](explanation/ai-training-opt-out.md) · [`../NOTICE`](../NOTICE) |
| **See what's in flight / shipped** | [`tracking/ROADMAP.md`](tracking/ROADMAP.md) · [`tracking/PROGRESS.md`](tracking/PROGRESS.md) — `git log` is the deepest source |
| **Understand a past bug** | [`tracking/BUG_FIXES.md`](tracking/BUG_FIXES.md) |
| **Read a design rationale (DR-NNN)** | [`tracking/DESIGN_RATIONALE.md`](tracking/DESIGN_RATIONALE.md) — judgement calls on review comments |
| **Read an architecture decision (ADR-NNN)** | [`adr/README.md`](adr/README.md) — load-bearing decisions, immutable once accepted |
| **See production-readiness gaps** | [`tracking/PRODUCTION_READINESS.md`](tracking/PRODUCTION_READINESS.md) |
| **See the test inventory** | [`../tests/TESTS.md`](../tests/TESTS.md) — single source of truth for test counts and per-suite coverage |

---

## Directory map

```
docs/
├── README.md                      # This file — goal-routing index
├── tutorials/                     # Diátaxis: learning-oriented (start-from-zero walkthroughs)
│   ├── GETTING_STARTED.md
│   └── DEV_MACHINE_SETUP.md
├── how-to/                        # Diátaxis: task-oriented (do X)
│   ├── INSTALL.md
│   ├── CONFIG_GUIDE.md
│   ├── COSYS_SETUP.md
│   ├── REMOTE_PIPELINE_GUIDE.md
│   ├── CI_SETUP.md
│   ├── DEVELOPMENT_WORKFLOW.md
│   └── INTEGRATION_ROLLUP_WORKFLOW.md   # 8-phase rollup checklist (paired with ADR-015)
├── reference/                     # Diátaxis: information-oriented (look up specific facts)
│   ├── config_reference.md
│   └── CPP_PATTERNS_GUIDE.md
├── explanation/                   # Diátaxis: understanding-oriented (why / discussion)
│   ├── SAFETY_CRITICAL_SHELL_DISCIPLINE.md
│   ├── MULTI_AGENT_GUIDE.md
│   ├── AGENT_HANDOFF.md
│   └── ai-training-opt-out.md           # AI/ML training reservation of rights
├── adr/                           # Architecture Decision Records (immutable)
│   ├── README.md                  # ADR index with status badges
│   └── ADR-NNN-*.md               # One file per ADR
├── architecture/                  # Cross-cutting architecture notes
│   ├── SIMULATION_ARCHITECTURE.md       # Tier 1 / Tier 2 (pure-sim + Gazebo)
│   ├── COSYS_SIMULATION_ARCHITECTURE.md # Tier 3 (Cosys-AirSim / UE5)
│   ├── MAKE_OR_BUY.md
│   ├── STRATEGIC_PLAN.md
│   ├── PRODUCTION_READINESS.md
│   ├── network-transport.md
│   ├── ipc-key-expressions.md
│   ├── observability.md
│   └── process-health-monitoring.md
├── design/                        # Per-process / per-component design docs
│   ├── API.md                     # IPC message reference
│   ├── hal_design.md              # HAL interface reference
│   ├── ipc_design.md              # IPC architecture
│   ├── error_handling_design.md   # Result<T,E> pattern
│   ├── hardening-design.md        # Three-layer watchdog
│   ├── hardware_strategy_plan.md  # Hardware target strategy (local-only, gitignored)
│   ├── video_capture_design.md    # P1
│   ├── perception_design.md       # P2 (v1)
│   ├── perception_architecture.md # P2 (v2 — current; local-only, gitignored)
│   ├── perception_v2_detailed_design.md  # P2 (v2 — implementation walkthrough; local-only, gitignored)
│   ├── slam_vio_nav_design.md     # P3
│   ├── swvio_implementation.md    # P3 — SWVIO living doc (local-only, gitignored)
│   ├── mission_planner_design.md  # P4
│   ├── comms_design.md            # P5
│   ├── payload_manager_design.md  # P6
│   ├── system_monitor_design.md   # P7
│   └── modularity_guide.md        # Epic #284 abstractions (local-only, gitignored)
└── tracking/                      # Append-only progress / bugs / decisions
    ├── ROADMAP.md
    ├── PROGRESS.md
    ├── BUG_FIXES.md
    ├── CI_ISSUES.md
    ├── DESIGN_RATIONALE.md
    ├── IMPROVEMENTS.md
    ├── PRODUCTION_READINESS.md
    └── INTEGRATION_MERGE_SCENARIO_SWEEP-*.md
```

> The legacy `docs/guides/` folder was split into the four Diátaxis quadrants above in #745 / Phase D. The folder still exists on disk locally for `work_instructions.md` (gitignored) but contains no tracked files.

## Single sources of truth

Quantitative project facts have one canonical location each. See [`CLAUDE.md` § Single Sources of Truth](../CLAUDE.md#single-sources-of-truth-ssot) for the full registry. Docs in this directory must defer to those sources rather than restating values — when you see "see `tests/TESTS.md` for count" in a doc, that's the SSOT pattern in action.

## Why Diátaxis?

[Diátaxis](https://diataxis.fr/) is a documentation framework that separates four kinds of content readers actually want:

- **Tutorials** answer "I'm new, walk me through it."
- **How-to guides** answer "I need to do X."
- **Reference** answers "what's the exact signature/value/option for Y?"
- **Explanation** answers "why does the system work this way / what's the model behind it?"

Mixing the four in one folder pushes readers to grep instead of navigate. Splitting them is the single biggest navigability improvement we can make at this size of docset.
