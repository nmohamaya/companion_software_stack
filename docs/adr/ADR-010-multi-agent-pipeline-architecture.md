# ADR-010: Multi-Agent Pipeline Architecture

**Status:** Accepted
**Date:** 2026-04-06 (updated 2026-04-07)
**Issue:** #357, #360

## Context

The project uses 15+ git worktrees for concurrent AI agent work (ADR-005), but has no formal agent role definitions, coordination layer, boundary enforcement, or session management. As the team scales, **Amdahl's Law** tells us the bottleneck isn't code production — it's verification. In safety-critical drone software, you can't relax quality gates to increase throughput.

**The Amdahl's Law insight:** If S is the serial fraction of the pipeline, maximum speedup with N agents is `1 / (S + (1-S)/N)`. By parallelizing verification (4 review agents + 2 test agents running simultaneously) and keeping the serial fraction to just the human's checkpoint decisions, we minimize S and maximize throughput without sacrificing quality.

## Decision

Implement a **13-agent pipeline** with formal role definitions, orchestration scripts, CI integration, shared state coordination, and a guided pipeline mode with human checkpoints.

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

#### Verification Agents (review + test in parallel)

| # | Role | Model | Access | Focus |
|---|------|-------|--------|-------|
| 7 | `review-memory-safety` | Opus | Read-only | RAII, ownership, lifetimes |
| 8 | `review-concurrency` | Opus | Read-only | Races, atomics, deadlocks |
| 9 | `review-fault-recovery` | Opus | Read-only | Watchdog, degradation, restart |
| 10 | `review-security` | Opus | Read-only | Input validation, auth, TLS |
| 11 | `test-unit` | Sonnet | tests/ only | Build, run tests, verify count vs baseline |
| 12 | `test-scenario` | Sonnet | tests/ only | Gazebo SITL scenario integration |

#### Operations Agent

| # | Role | Model | Access | Focus |
|---|------|-------|--------|-------|
| 13 | `ops-github` | Haiku | gh CLI only | Auto-triage unlabeled issues, milestones, stale cleanup |

### Launch Modes

`deploy-issue.sh` supports four modes for deploying agents:

| Mode | Flag | Behavior |
|------|------|----------|
| Interactive | (default) | Agent gets issue context, human converses in real time |
| Headless | `--headless` | Non-interactive print mode, no approval prompts |
| Auto | `--auto` | Agent works autonomously, writes AGENT_REPORT.md, human reviews afterward |
| Pipeline | `--pipeline` | Full guided flow: 12-state machine with 5 human checkpoints |

### Pipeline Mode (12-State Machine)

The `--pipeline` flag chains all scripts into a single guided flow:

```
AGENT_WORK → CP1 → VALIDATE → CP2 → PR_CREATE → CP3 → REVIEW → CP4
                                                         ↕
                                                  FIX_AND_REVALIDATE
                                                         ↓
                                                  CP5 → CLEANUP / ABORT
```

#### 5 Human Checkpoints

| # | Checkpoint | User sees | Options |
|---|-----------|-----------|---------|
| 1 | Changes review | AGENT_REPORT.md + diff | accept / changes / reject |
| 2 | Hallucination report | validate-session.sh output | commit / back / abort |
| 3 | PR preview | Generated title + body | create / edit / back / abort |
| 4 | Review & test findings | P1-P4 from review agents + test results | accept / fix / back / reject |
| 5 | Final summary | Commit log + validation status | done / re-review / back / abort |

Every checkpoint supports **back-navigation** to the previous checkpoint. The `[f]ix` option at CP4 feeds findings to the feature agent, then **re-runs all review + test agents** to verify fixes (loop: CP4 → FIX → REVIEW → CP4).

### Pipeline Flow (End-to-End)

```
Issue fetch + label routing
  ↓ (ops-github auto-triages if no labels)
Worktree + branch creation
  ↓
Feature agent works autonomously (writes AGENT_REPORT.md)
  ↓
[CP1] Human reviews changes
  ↓
validate-session.sh (build, tests, hallucination detection)
  ↓
[CP2] Human reviews validation
  ↓
git push + PR created
  ↓
[CP3] Human previews PR
  ↓
deploy-review.sh launches in parallel:
  ├─ review-memory-safety  (always)
  ├─ review-security       (always)
  ├─ review-concurrency    (if atomics/mutexes/threads in diff)
  ├─ review-fault-recovery (if P4/P5/P7/watchdog in diff)
  ├─ test-unit             (always — build, run tests, verify count)
  └─ test-scenario         (if IPC/HAL/Gazebo configs in diff)
  ↓
Consolidated review + test results posted as PR comment
  ↓
[CP4] Human reviews findings
  ↓ (if fix needed: feature agent fixes → re-runs all agents → back to CP4)
[CP5] Final push
  ↓
CLEANUP: verify issue↔PR link, update shared state, offer branch cleanup
```

### Cross-Agent Context System

Agents start cold — each invocation has no memory of prior sessions. To maintain alignment, four layers of context are injected into every agent prompt at startup:

| Layer | File | What it answers | Updated by |
|-------|------|----------------|------------|
| Active work | `tasks/active-work.md` | "What are other agents doing right now?" | deploy-issue.sh (start + cleanup) |
| Recent changelog | `tasks/agent-changelog.md` | "What was just completed?" | Pipeline CLEANUP |
| Project status | `.claude/shared-context/project-status.md` | "Where does the project stand?" | Human (end of session) |
| Domain knowledge | `.claude/shared-context/domain-knowledge.md` | "What pitfalls should I avoid?" | Any agent (tech-lead reviews) |

Both `deploy-issue.sh` and `start-agent.sh` inject this context via `gather_cross_agent_context()`. Auto/pipeline modes inject directly (they call `claude` directly); headless/interactive modes defer to `start-agent.sh` to avoid double injection.

#### CLEANUP State Updates

At pipeline completion, the CLEANUP state:
1. Marks `active-work.md` entry as completed (awk block-scoped replacement)
2. Appends session summary to `agent-changelog.md`
3. Reminds about `domain-knowledge.md` if AGENT_REPORT.md mentions pitfalls
4. Verifies issue↔PR link (auto-fixes `Closes #N` if missing)

#### Required Documentation Updates

Every agent (auto + pipeline) is instructed to update before completing:
- `tests/TESTS.md` — test count and entries
- `docs/tracking/PROGRESS.md` — improvement entry
- `docs/tracking/ROADMAP.md` — mark issue done
- `docs/tracking/BUG_FIXES.md` — bug fix entry

### Review Routing

Not all reviewers on every PR — routing is based on diff content:

| Change touches... | Agents triggered |
|-------------------|-----------------|
| Any file | memory-safety + security + test-unit |
| `std::atomic`, `mutex`, `thread`, `lock_guard` | + concurrency |
| P4/P5/P7, watchdog, fault handling | + fault-recovery |
| IPC topics, HAL backends, Gazebo configs | + test-scenario |

### Ops-GitHub Integration

The `ops-github` agent (Haiku) is invoked automatically when `deploy-issue.sh` encounters an issue with no recognized domain labels. It reads the issue title and body, applies appropriate `domain:*` and `type:*` labels via `gh issue edit`, then routing proceeds normally. It can also be launched manually for triage, stale cleanup, and milestone tracking.

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

### Infrastructure Split Rationale

Two infra agents are needed because Epic #284 (Modularity, 13 issues) requires common/util abstractions and interfaces, while Epic #300 (Multi-Customer Platform, 30 issues) requires deploy/CI/certification work. Different skills, different file scopes.

## Consequences

### Positive

- **Parallel verification** reduces the serial fraction of the pipeline per Amdahl's Law
- **5 human checkpoints** keep the human in control without requiring them to run each script manually
- **Cross-agent context injection** ensures every agent starts aligned on project state, active work, and known pitfalls
- **Test agents actually run tests** (build, execute, verify count) — not just review the diff
- **Fix-and-reverify loop** (CP4 → FIX → REVIEW → CP4) ensures fixes don't introduce new issues
- **Ops-github auto-triage** eliminates the "add labels first" friction for unlabeled issues
- **Formal role definitions** prevent scope creep and file conflicts between agents
- **Review routing** avoids wasting compute on irrelevant reviews
- **Issue↔PR enforcement** at CLEANUP prevents orphaned issues

### Negative

- **13 agent definitions** to maintain — role files may drift from actual capabilities
- **Orchestration complexity** — 12+ scripts add operational surface area
- **Boundary enforcement is advisory** — agents can work outside their scope when necessary (by design)
- **Shared context files can go stale** — `project-status.md` requires human updates each session
- **Token costs** — full pipeline (1 feature + 2-6 review/test agents) costs ~$3-10 per issue

### Risks

- **Stale project status** — `project-status.md` must be updated by the human each session; stale data misleads agents
- **Review routing heuristics** may miss edge cases (e.g., concurrency bug in a file that doesn't contain "mutex")
- **Dashboard metrics** depend on consistent commit message formatting
- **Double context injection** if new launch paths are added without understanding the deploy-issue.sh/start-agent.sh split

## Alternatives Considered

1. **CLAUDE.md-only approach** — Define roles in a single file without formal agent definitions or scripts. Simpler but no boundary enforcement, no routing, no metrics.

2. **Fewer agents (5-6)** — Combine review specialties. Lower overhead but misses the Amdahl's Law insight: parallel specialized reviews are the key win.

3. **No ops agent** — Handle GitHub housekeeping manually. Works at small scale but becomes a bottleneck as issue count grows.

4. **No pipeline mode** — Run each script manually (deploy-issue, validate, deploy-review, etc.). Works but error-prone and slow. Pipeline mode chains them with checkpoints.

5. **Fully autonomous (no checkpoints)** — Remove human checkpoints entirely. Faster but unacceptable for safety-critical code. The 5-checkpoint design gives speed with control.

## References

- ADR-005: Parallel Agent Git Worktree Strategy
- `.claude/agents/` — All 13 role definitions
- `.claude/shared-context/` — Project status and domain knowledge
- `scripts/` — Orchestration scripts
- `docs/guides/MULTI_AGENT_GUIDE.md` — Full usage guide with setup, walkthrough, and examples
- `docs/guides/AGENT_HANDOFF.md` — Cross-domain handoff protocol
