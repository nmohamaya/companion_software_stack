# ADR-010: Multi-Agent Pipeline Architecture

**Status:** Accepted  
**Date:** 2026-04-06  
**Issue:** #357

## Context

The project uses 15+ git worktrees for concurrent AI agent work (ADR-005), but has no formal agent role definitions, coordination layer, boundary enforcement, or session management. As the team scales, **Amdahl's Law** tells us the bottleneck isn't code production — it's verification. In safety-critical drone software, you can't relax quality gates to increase throughput.

**The Amdahl's Law insight:** If S is the serial fraction of the pipeline, maximum speedup with N agents is `1 / (S + (1-S)/N)`. By parallelizing verification (4 review agents + 2 test agents running simultaneously) and keeping the serial fraction to just the tech lead's merge decision, we minimize S and maximize throughput without sacrificing quality.

## Decision

Implement a **13-agent pipeline** with formal role definitions, orchestration scripts, CI integration, and shared state coordination.

### Agent Roster

#### Production Agents (write code)

| # | Role | Model | Scope |
|---|------|-------|-------|
| 1 | `tech-lead` | Opus | Orchestration, routing, merge decisions |
| 2 | `feature-perception` | Opus | P1/P2, camera/detector HAL |
| 3 | `feature-nav` | Opus | P3/P4, planner/avoider HAL |
| 4 | `feature-integration` | Opus | P5/P6/P7, IPC, HAL |
| 5 | `feature-infra-core` | Opus | common/, CMake, config (Epic #284 + #300 A-B) |
| 6 | `feature-infra-platform` | Opus | deploy/, CI, boards/ (Epic #300 C-E) |

#### Verification Agents (review in parallel)

| # | Role | Model | Access | Focus |
|---|------|-------|--------|-------|
| 7 | `review-memory-safety` | Opus | Read-only | RAII, ownership, lifetimes |
| 8 | `review-concurrency` | Opus | Read-only | Races, atomics, deadlocks |
| 9 | `review-fault-recovery` | Opus | Read-only | Watchdog, degradation, restart |
| 10 | `review-security` | Opus | Read-only | Input validation, auth, TLS |
| 11 | `test-unit` | Sonnet | tests/ only | GTest, coverage delta |
| 12 | `test-scenario` | Sonnet | tests/ only | Gazebo SITL, integration |

#### Operations Agent

| # | Role | Model | Access | Focus |
|---|------|-------|--------|-------|
| 13 | `ops-github` | Haiku | gh CLI only | Issue triage, milestones, board |

### Pipeline Flow

```
Issue Created → ops-github triages/labels
     ↓
Tech Lead routes → Feature Agent (in worktree)
     ↓
Feature Agent creates PR
     ↓
Tech Lead routes PR → Parallel Review:
  ├─ review-memory-safety  (always)
  ├─ review-security       (always)
  ├─ review-concurrency    (if atomics/mutexes/threads)
  ├─ review-fault-recovery (if P4/P5/P7/watchdog)
  ├─ test-unit             (always)
  └─ test-scenario         (if IPC/HAL/Gazebo)
     ↓
All reviews pass → Tech Lead merges (serial decision)
```

### Review Routing

Not all reviewers on every PR — routing is based on diff content:

| Change touches... | Agents triggered |
|-------------------|-----------------|
| Any file | Memory Safety, Unit Test, Security |
| `std::atomic`, `mutex`, `thread`, `lock_guard` | + Concurrency |
| P4/P5/P7, watchdog, fault handling | + Fault Recovery |
| IPC topics, HAL backends, Gazebo configs | + Scenario Test |

### Infrastructure Split Rationale

Two infra agents are needed because Epic #284 (Modularity, 13 issues) requires common/util abstractions and interfaces, while Epic #300 (Multi-Customer Platform, 30 issues) requires deploy/CI/certification work. Different skills, different file scopes.

### Anti-Hallucination Measures

All agents include rules to prevent hallucination:
- Verify existence before citing functions, files, or APIs
- Never claim tests pass without running them
- Mark uncertain claims with `[UNVERIFIED]`
- `validate-session.sh` performs post-session verification (build, test count, symbol existence, include existence, diff sanity)

### Boundary Enforcement

Three layers:
1. **Pre-commit hook** — Fast feedback on scope violations (warning)
2. **CI job** — `agent-boundaries` workflow checks branch-name→scope mapping (warning)
3. **Agent tool restrictions** — Review agents are read-only, test agents write to tests/ only

## Consequences

### Positive

- **Parallel verification** reduces the serial fraction of the pipeline per Amdahl's Law
- **Formal role definitions** prevent scope creep and file conflicts between agents
- **Anti-hallucination checks** catch fabricated claims before they reach code review
- **Boundary enforcement** at 3 layers (pre-commit, CI, tool restrictions) provides defense in depth
- **Ops agent** handles GitHub housekeeping cheaply (Haiku model)
- **Review routing** avoids wasting compute on irrelevant reviews
- **Session management** provides reproducibility and metrics for pipeline optimization

### Negative

- **13 agent definitions** to maintain — role files may drift from actual capabilities
- **Orchestration complexity** — 12 scripts add operational surface area
- **Boundary enforcement is advisory** — agents can work outside their scope when necessary (by design)
- **Auto-review workflow disabled initially** — requires manual enabling after validation

### Risks

- **Stale domain knowledge** — `.claude/shared-context/domain-knowledge.md` must be updated as the codebase evolves
- **Review routing heuristics** may miss edge cases (e.g., concurrency bug in a file that doesn't contain "mutex")
- **Dashboard metrics** depend on consistent commit message formatting

## Alternatives Considered

1. **CLAUDE.md-only approach** — Define roles in a single file without formal agent definitions or scripts. Simpler but no boundary enforcement, no routing, no metrics.

2. **Fewer agents (5-6)** — Combine review specialties. Lower overhead but misses the Amdahl's Law insight: parallel specialized reviews are the key win.

3. **No ops agent** — Handle GitHub housekeeping manually. Works at small scale but becomes a bottleneck as issue count grows.

## References

- ADR-005: Parallel Agent Git Worktree Strategy
- `temp_claude/` reference implementation
- `.claude/agents/` — All 13 role definitions
- `scripts/` — Orchestration scripts
- `docs/guides/AGENT_HANDOFF.md` — Cross-domain handoff protocol
