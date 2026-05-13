# Architecture Decision Records (ADRs)

This directory captures the load-bearing architectural decisions made on the project. Each ADR records the context, the decision, alternatives considered, and the consequences. ADRs are **immutable once accepted** — superseding decisions get a new ADR that references the older one.

## When to write an ADR

Write an ADR when a decision:
- Closes off realistic alternatives (e.g. picking Zenoh as the sole IPC backend foreclosed shared-memory)
- Has consequences beyond the PR introducing it (downstream code, future hardware, build infra)
- A future contributor reading the code months later would ask "why does it work this way?"

Do **not** write an ADR for:
- A single-file refactor with no cross-cutting impact
- A judgement call on a review comment — those live in [`docs/tracking/DESIGN_RATIONALE.md`](../tracking/DESIGN_RATIONALE.md) as `DR-NNN` entries
- A bug fix — that lives in [`docs/tracking/BUG_FIXES.md`](../tracking/BUG_FIXES.md)

## Index

| ADR | Title | Status | Domain | Date |
|-----|-------|--------|--------|------|
| [ADR-001](ADR-001-ipc-framework-selection.md) | IPC Framework Selection — Zenoh | ✅ Accepted | IPC | 2026-02-28 |
| [ADR-002](ADR-002-modular-ipc-backend-architecture.md) | Modular IPC Backend Architecture | ✅ Accepted | IPC | 2026-03-01 |
| [ADR-003](ADR-003-cpp17-language-standard.md) | C++17 Language Standard | ✅ Accepted | Build | 2026-03-02 |
| [ADR-004](ADR-004-process-thread-watchdog-architecture.md) | Process + Thread Watchdog Architecture | ✅ Accepted | Safety | 2026-03-04 |
| [ADR-005](ADR-005-parallel-agent-git-worktree.md) | Parallel Agent Git Worktree Model | ✅ Accepted | Workflow | 2026-03-12 |
| [ADR-006](ADR-006-hal-hardware-abstraction-strategy.md) | HAL Hardware Abstraction Strategy | ✅ Accepted | HAL | 2026-03-13 |
| [ADR-007](ADR-007-error-handling.md) | Error Handling — `Result<T,E>` | ✅ Accepted | Core | 2026-03-13 |
| [ADR-008](ADR-008-p4-header-only-class-extraction.md) | P4 Header-Only Class Extraction | ✅ Accepted | Mission planner | 2026-03-14 |
| [ADR-009](ADR-009-tier1-tier2-simulation-architecture.md) | Tier 1 / Tier 2 Simulation Architecture | ✅ Accepted | Simulation | 2026-03-14 |
| [ADR-010](ADR-010-multi-agent-pipeline-architecture.md) | Multi-Agent Pipeline Architecture | ✅ Accepted | Workflow | 2026-04-06 |
| [ADR-011](ADR-011-cosys-airsim-photorealistic-simulation.md) | Cosys-AirSim Photorealistic Simulation (Tier 3) | ✅ Accepted | Simulation | 2026-04-14 |
| [ADR-012](ADR-012-detector-licensing-yolo-vs-rtdetr.md) | Detector Licensing — YOLOv8 vs RT-DETR | ✅ Accepted | Perception | 2026-04-20 |
| [ADR-013](ADR-013-stereo-radar-redundancy-vs-fusion.md) | Stereo + Radar Defence-in-Depth vs Unified Fusion | ✅ Accepted | Perception | 2026-05-03 |
| [ADR-014](ADR-014-stereo-inertial-vio-algorithm-selection.md) | Stereo-Inertial VIO Algorithm Selection (SWVIO) | ✅ Accepted | SLAM/VIO | 2026-05-13 |
| [ADR-015](ADR-015-integration-branch-rollup-strategy.md) | Integration-Branch Rollup Strategy | ✅ Accepted | Workflow | 2026-05-13 |

Status legend: ✅ Accepted · 🟡 Proposed · ⚠️ Superseded · ❌ Rejected

## Format

Most ADRs follow the template below; new ADRs should adopt it (older ADRs use slight variations — adapt the section names where useful but keep the overall shape):

1. **Header metadata** — Status, Date, Author, Deciders, Supersedes, Related
2. **Context** — what problem forced the decision, what's at stake
3. **Decision** — the chosen approach (one paragraph)
4. **Consequences** — positive, negative, neutral
5. **Alternatives considered** — what we rejected and why
6. **Status** — accepted/proposed/superseded + date
7. **References** — linked issues, PRs, related ADRs, design docs

ADRs do not duplicate the design documents in [`../design/`](../design/) — they record *why* the system is the way it is. The design docs describe *what* the system is.
