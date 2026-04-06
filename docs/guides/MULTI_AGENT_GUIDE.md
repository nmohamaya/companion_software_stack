# Multi-Agent Pipeline Guide

This guide explains how the 13-agent pipeline works, how to use it, and how agents coordinate with each other and GitHub.

## How It Works

The pipeline applies **Amdahl's Law** to software verification: feature agents produce code in parallel, but more importantly, 4 review agents + 2 test agents verify every PR in parallel. The only serial step is the tech lead's merge decision — minimizing the serial fraction that limits speedup.

```
                          ┌──────────────────────────────────────────────────────┐
                          │                    GitHub                            │
                          │  Issues ──► Labels ──► PRs ──► Reviews ──► Merge    │
                          └──────┬──────────────────────────────┬───────────────┘
                                 │                              │
                    ┌────────────▼────────────┐    ┌────────────▼────────────┐
                    │   deploy-issue.sh 123    │    │   deploy-review.sh 456  │
                    │   (routes by labels)     │    │   (routes by diff)      │
                    └────────────┬────────────┘    └────────────┬────────────┘
                                 │                              │
                    ┌────────────▼────────────┐    ┌────────────▼────────────┐
                    │      start-agent.sh      │    │      start-agent.sh     │
                    │   (model + role setup)    │    │   (parallel launches)   │
                    └────────────┬────────────┘    └────────────┬────────────┘
                                 │                              │
              ┌──────────────────▼──────────────────────────────▼──────────┐
              │                    Agent Execution                         │
              │                                                           │
              │  ┌─────────────────────────────────────────────────────┐  │
              │  │              FEATURE AGENTS (produce)               │  │
              │  │  perception │ nav │ integration │ infra-core │ plat │  │
              │  └──────────────────────┬──────────────────────────────┘  │
              │                         │ PR created                      │
              │  ┌──────────────────────▼──────────────────────────────┐  │
              │  │           REVIEW AGENTS (verify in parallel)        │  │
              │  │  memory-safety │ concurrency │ fault │ security     │  │
              │  └──────────────────────┬──────────────────────────────┘  │
              │  ┌──────────────────────▼──────────────────────────────┐  │
              │  │            TEST AGENTS (verify in parallel)         │  │
              │  │            unit-test  │  scenario-test              │  │
              │  └──────────────────────┬──────────────────────────────┘  │
              │                         │                                 │
              └─────────────────────────┼─────────────────────────────────┘
                                        │
                          ┌─────────────▼─────────────┐
                          │        TECH LEAD           │
                          │  (serial merge decision)   │
                          │  validate-session.sh       │
                          └─────────────┬─────────────┘
                                        │
                                  Merge to main
```

## Agent Interaction Flow

```mermaid
flowchart TD
    Issue[GitHub Issue with labels] -->|deploy-issue.sh| Route{Route by labels}
    Route -->|perception| FP[feature-perception]
    Route -->|nav-planning| FN[feature-nav]
    Route -->|comms/integration| FI[feature-integration]
    Route -->|infra/common| FIC[feature-infra-core]
    Route -->|deploy/ci| FIP[feature-infra-platform]
    Route -->|safety-audit| RMS[review-memory-safety]
    Route -->|security-audit| RS[review-security]
    Route -->|test-coverage| TU[test-unit]
    Route -->|cross-domain| TL[tech-lead]

    FP -->|creates PR| PR[Pull Request]
    FN -->|creates PR| PR
    FI -->|creates PR| PR
    FIC -->|creates PR| PR
    FIP -->|creates PR| PR

    PR -->|deploy-review.sh| Review{Route by diff content}
    Review -->|always| RMS2[review-memory-safety]
    Review -->|always| RS2[review-security]
    Review -->|always| TU2[test-unit]
    Review -->|atomics/mutex| RC[review-concurrency]
    Review -->|watchdog/P4/P5/P7| RF[review-fault-recovery]
    Review -->|IPC/HAL/Gazebo| TS[test-scenario]

    RMS2 -->|comments| PR
    RS2 -->|comments| PR
    TU2 -->|comments| PR
    RC -->|comments| PR
    RF -->|comments| PR
    TS -->|comments| PR

    PR -->|all reviews pass| TL2[tech-lead]
    TL2 -->|validate-session.sh| Merge[Merge to main]
```

## Integration Branch Flow (Multi-Issue Epics)

When a feature spans multiple issues across domains, use an **integration branch** to keep main demo-ready:

```mermaid
flowchart TD
    Main[main branch] -->|create| IB[integration/epic-XXX]
    IB -->|"deploy-issue.sh 100 --base integration/epic-XXX"| W1[feature/issue-100-...]
    IB -->|"deploy-issue.sh 101 --base integration/epic-XXX"| W2[feature/issue-101-...]
    IB -->|"deploy-issue.sh 102 --base integration/epic-XXX"| W3[feature/issue-102-...]

    W1 -->|PR targets integration branch| IB
    W2 -->|PR targets integration branch| IB
    W3 -->|PR targets integration branch| IB

    IB -->|"all sub-issues merged + CI green"| Main
```

## The 13-Agent Roster

| # | Role | Model | Type | Scope |
|---|------|-------|------|-------|
| 1 | `tech-lead` | Opus | Orchestrator | Routing, merge decisions, coordination |
| 2 | `feature-perception` | Opus | Feature | P1/P2, camera/detector HAL |
| 3 | `feature-nav` | Opus | Feature | P3/P4, planner/avoider HAL |
| 4 | `feature-integration` | Opus | Feature | P5/P6/P7, IPC, HAL backends |
| 5 | `feature-infra-core` | Opus | Feature | common/, CMake, config |
| 6 | `feature-infra-platform` | Opus | Feature | deploy/, CI, boards/, certification |
| 7 | `review-memory-safety` | Opus | Review (read-only) | RAII, ownership, lifetimes |
| 8 | `review-concurrency` | Opus | Review (read-only) | Races, atomics, deadlocks |
| 9 | `review-fault-recovery` | Opus | Review (read-only) | Watchdog, degradation |
| 10 | `review-security` | Opus | Review (read-only) | Input validation, auth, TLS |
| 11 | `test-unit` | Sonnet | Test | GTest, coverage delta (tests/ only) |
| 12 | `test-scenario` | Sonnet | Test | Gazebo SITL, integration (tests/ only) |
| 13 | `ops-github` | Haiku | Operations | Issue triage, milestones, board |

### Agent Boundaries

Each agent has a defined scope of files it may edit. Review agents are **read-only** — they can only read code and post comments. The `check-agent-boundaries.sh` script enforces these boundaries (advisory in CI).

## Quick Start

### Prerequisites

- [Claude Code CLI](https://docs.anthropic.com/en/docs/claude-code) installed and authenticated
- GitHub CLI (`gh`) authenticated: `gh auth login`
- `jq` installed: `sudo apt install jq`
- Repository cloned with the agent definitions in `.claude/agents/`

### 1. Deploy an Agent for a GitHub Issue

The simplest way to use the pipeline — point it at an issue and it handles the rest:

```bash
# Interactive (default) — you approve changes and converse in real time
bash scripts/deploy-issue.sh 123

# Auto mode — agent works autonomously, you review a report afterward
bash scripts/deploy-issue.sh 123 --auto

# Preview routing without executing
bash scripts/deploy-issue.sh 123 --dry-run

# For epic sub-issues, branch from the integration branch
bash scripts/deploy-issue.sh 123 --base integration/epic-XXX
```

#### Launch Modes

| Mode | Flag | Permissions | You do... |
|------|------|-------------|-----------|
| **Interactive** | *(default)* | You approve each change | Converse with the agent in real time |
| **Auto** | `--auto` | File edits auto-approved | Review `AGENT_REPORT.md` after, then accept/change/reject |
| **Headless** | `--headless` | None (for CI sandboxes) | Pre-configure permissions externally |

**What happens (all modes):**
1. Fetches the issue from GitHub (title, body, labels)
2. Routes to an agent based on labels (priority-based: specific labels beat broad ones)
3. Creates a git worktree + branch
4. Updates `tasks/active-work.md`
5. Launches the agent with the issue context as its prompt

**Auto mode additionally:**
6. Agent commits locally but does NOT push or create a PR
7. Agent writes `AGENT_REPORT.md` with a structured change report
8. You review the report and choose: **accept**, **request changes**, or **reject**

### 2. Launch a Review on a PR

```bash
# Auto-route reviewers based on the PR's diff content
bash scripts/deploy-review.sh 456

# Preview which reviewers would be triggered
bash scripts/deploy-review.sh 456 --dry-run

# Force all reviewers
bash scripts/deploy-review.sh 456 --all
```

Review agents launch in parallel and post a consolidated comment on the PR when done. Monitor progress in a separate terminal:

```bash
# Watch log sizes grow (see which agents are still working)
watch -n 5 'wc -l tasks/sessions/*-review-pr456-*.log'

# Tail a specific reviewer's output in real time
tail -f tasks/sessions/*-review-pr456-review-memory-safety.log
```

### 3. Manual Mode — Step-by-Step Walkthrough

This is the most hands-on workflow. You control every step, running each script individually. Best for learning the pipeline or when you want full visibility.

> **Note on paths:** When working inside a worktree (`.claude/worktrees/issue-N/`),
> scripts live in the main repo, not the worktree. Set `REPO_ROOT` to the main repo path:
> ```bash
> export REPO_ROOT=/path/to/companion_software_stack
> ```
> All script commands below use `$REPO_ROOT/scripts/...` to work from any directory.

#### Step 1: Deploy the feature agent interactively

```bash
# Preview routing first
bash scripts/deploy-issue.sh 315 --base integration/epic-357 --dry-run

# Launch — you'll approve changes and converse with the agent
bash scripts/deploy-issue.sh 315 --base integration/epic-357
```

The agent opens an interactive session. You guide the work, approve edits, and the agent commits when you're satisfied.

#### Step 2: Validate the agent's work (hallucination detection)

```bash
bash scripts/validate-session.sh
```

Review the output — it checks build, test count, test execution, `#include` paths, and PR body references. Fix any issues before proceeding.

#### Step 3: Push and create the PR

```bash
cd .claude/worktrees/issue-315
git push -u origin <branch-name>
gh pr create --base integration/epic-357
```

#### Step 4: Deploy review agents on the PR

```bash
# Back in the main repo directory
bash scripts/deploy-review.sh <pr-number>
```

This launches review agents in parallel (memory-safety, security, + conditional concurrency/fault-recovery based on diff content). Monitor them:

```bash
# In a separate terminal — watch progress
watch -n 5 'wc -l tasks/sessions/*-review-pr<number>-*.log'

# Peek at a specific reviewer
tail -f tasks/sessions/*-review-pr<number>-review-memory-safety.log
```

When complete, a consolidated review is posted as a PR comment.

#### Step 5: Review the findings

The consolidated review is posted as a PR comment with findings grouped by reviewer and severity:

| Severity | Meaning | Action |
|----------|---------|--------|
| **P1** | Critical / blocks merge | Fix immediately |
| **P2** | Memory, perf, should fix | Fix before merge |
| **P3** | Code quality | Fix or document why not — follow-up OK |
| **P4** | Tests, docs, nice-to-have | Fix if time permits |

Read the PR comment on GitHub or fetch it locally:

```bash
# View the review comment
gh pr view <pr-number> --comments
```

#### Step 6: Feed review findings to the feature agent

Open an interactive session in the worktree with the **same feature agent** that did the original work — it has the most context for fixing issues.

```bash
cd .claude/worktrees/issue-<NUMBER>
bash "$REPO_ROOT/scripts/start-agent.sh" <agent-role>
```

Then paste a structured prompt listing each finding with severity, location, and the fix. Be specific — the more detail you give, the better the fixes:

```
The review agents found these issues on PR #<NUMBER>. Fix all P1 and P2 findings:

P1 (must fix):
1. <finding> — <specific fix instruction with file:line if available>
2. <finding> — <specific fix instruction>

P2 (should fix):
1. <finding> — <specific fix instruction>
2. <finding> — <specific fix instruction>

P3 (follow-up OK — file as separate issues if not fixing now):
1. <finding>
```

**Tips for writing the fix prompt:**
- Copy the fix suggestions directly from the review comment — reviewers give specific code snippets
- Include file paths and line numbers when the review provides them
- Group by severity so the agent prioritizes correctly
- Tell the agent explicitly which severities to fix now vs defer

The agent applies the fixes. You approve each change interactively.

**Example** (from a real review of IPC version fields):

```
The review agents found these issues on PR #361. Fix all P1 and P2 findings:

P1 (must fix):
1. validate() never checks version field — add `if (version != CURRENT_VERSION)
   return false;` as first line of every validate() method in ipc_types.h
2. FaultOverrides has no validate() method — add one that checks version and
   sentinel-range validity
3. IpcWaypoint aggregate inits in 5+ files NOT updated — silent coordinate
   corruption in process4_mission_planner/src/main.cpp,
   tests/test_fault_response_executor.cpp, tests/test_mission_state_tick.cpp,
   tests/test_mission_fsm.cpp, tests/test_collision_recovery.cpp
4. wire_deserialize accepts version-mismatched payloads — check
   out.version == T::CURRENT_VERSION after deserialization

P2 (should fix):
1. Add default member initializers (= 0, = {}, = false) to ALL fields in all
   IPC structs
2. Add _pad0 to FaultOverrides for consistency
3. Add missing static_assert(sizeof(...)) for SystemHealth, ThreadHealth,
   ProcessHealthEntry, ThreadHealthEntry, FaultOverrides, RadarDetection
4. Add version mismatch rejection tests for each struct
5. Add std::isfinite() checks for GCSCommand::validate() param1/param2/param3
```

#### Step 7: Re-validate and push fixes

```bash
bash "$REPO_ROOT/scripts/validate-session.sh"
git push
```

Optionally re-run reviews on the updated PR to verify all findings are addressed:

```bash
bash "$REPO_ROOT/scripts/deploy-review.sh" <pr-number>
```

#### Step 8: Merge in GitHub UI

Review the PR one final time in GitHub, then merge. After merge:

```bash
# Cleanup worktree and branches
bash scripts/cleanup-branches.sh
```

### 4. Orchestrated Session

For more control, use the session orchestrator which adds pre/post validation:

```bash
# Full session with pre-flight, agent, validation, and changelog
bash scripts/run-session.sh feature-nav "Implement A* path planner" --issue 789
```

### 5. Launch an Agent Directly

For ad-hoc work or interactive sessions:

```bash
# List all available roles
bash scripts/start-agent.sh --list

# Launch with a task
bash scripts/start-agent.sh feature-perception "Add YOLOv8 detector backend"

# Interactive session (no task — opens Claude CLI)
bash scripts/start-agent.sh feature-nav

# Dry-run: print resolved config without launching
bash scripts/start-agent.sh feature-integration "test" --dry-run
```

### 6. Validate a Session

After an agent completes work, verify nothing was hallucinated:

```bash
# Validate current branch
bash scripts/validate-session.sh

# Validate a specific branch
bash scripts/validate-session.sh --branch feature/issue-123-foo
```

Checks: build succeeds, test count matches baseline, all tests pass, `#include` paths exist, PR body references real files.

### 7. After the Agent Finishes — Post-Agent Checklist

Once the agent completes its work (interactive or auto mode), changes are **local commits in the worktree** — nothing is pushed. Follow these steps:

```bash
# 1. Go to the worktree
cd .claude/worktrees/issue-<NUMBER>

# 2. Review what the agent did
git log --oneline <base-branch>..HEAD        # commits
git diff --stat <base-branch>..HEAD          # files changed

# 3. Validate (hallucination detection — checks build, tests, includes)
bash "$REPO_ROOT/scripts/validate-session.sh"

# 4. Push and create PR
git push -u origin <branch-name>
gh pr create --base <base-branch>            # main or integration/epic-XXX

# 5. Deploy review agents on the PR
bash "$REPO_ROOT/scripts/deploy-review.sh" <pr-number>

# 6. Address review comments, then merge the PR
```

**If using an integration branch** (multi-issue epic), there are additional steps after all sub-issues are merged:

```bash
# 7. Create final PR from integration branch to main
gh pr create --base main --head integration/epic-XXX \
  --title "feat(#XXX): Epic description"

# 8. After merge, cleanup
git branch -d integration/epic-XXX
git push origin --delete integration/epic-XXX
bash "$REPO_ROOT/scripts/cleanup-branches.sh"
```

**Quick reference for `<base-branch>`:**
- Single issue targeting main: `main`
- Sub-issue targeting an epic: `integration/epic-XXX`

### 8. View Dashboards and Stats

```bash
# Team-wide dashboard
bash scripts/agent-dashboard.sh

# Per-agent stats
bash scripts/agent-dashboard.sh feature-perception

# Git-based agent stats (commits, lines, cost)
bash scripts/agent-stats.sh
```

### 9. Cleanup

```bash
# Remove stale branches and worktrees
bash scripts/cleanup-branches.sh

# List all active worktrees
git worktree list
```

## Pipeline Mode (`--pipeline`)

The pipeline mode chains all scripts into a single guided flow with **5 human checkpoints**. Everything between checkpoints is automated.

```bash
bash scripts/deploy-issue.sh 123 --pipeline
bash scripts/deploy-issue.sh 123 --pipeline --base integration/epic-300
```

### Flow

```
Issue fetch + label routing (automated)
  → Worktree + branch creation (automated)
  → Agent works autonomously (automated)
  → AGENT_REPORT.md written
  │
  ▼
┌─────────────────────────────────────────────────────┐
│ CP1: Changes Review                                  │
│   You see: AGENT_REPORT.md + git diff --stat         │
│   Options: [a]ccept / [c]hanges / [r]eject           │
└─────────────────────┬───────────────────────────────┘
                      ▼
  validate-session.sh runs (automated: build, tests, hallucination detection)
                      │
┌─────────────────────▼───────────────────────────────┐
│ CP2: Hallucination Report                            │
│   You see: PASS/WARN/FAIL for build, test count,     │
│            test execution, #include paths, PR sanity  │
│   Options: [c]ommit / [b]ack to CP1 / [a]bort       │
└─────────────────────┬───────────────────────────────┘
                      ▼
  git push + PR title/body generated (automated)
                      │
┌─────────────────────▼───────────────────────────────┐
│ CP3: PR Preview                                      │
│   You see: PR title + body                           │
│   Options: [c]reate / [e]dit / [b]ack / [a]bort     │
└─────────────────────┬───────────────────────────────┘
                      ▼
  deploy-review.sh runs (automated: 2-4 review agents in parallel)
                      │
┌─────────────────────▼───────────────────────────────┐
│ CP4: Safety Review Findings                          │
│   You see: P1-P4 findings from memory-safety,        │
│            security, concurrency, fault-recovery      │
│   Options: [a]ccept / [f]ix / [b]ack / [r]eject     │
└─────────────────────┬───────────────────────────────┘
                      ▼
  Feature agent fixes P1/P2 findings (automated, if [f]ix chosen)
  Re-validation runs (automated)
                      │
┌─────────────────────▼───────────────────────────────┐
│ CP5: Final Summary                                   │
│   You see: commit log + validation status            │
│   Options: [d]one / [r]e-review / [b]ack / [a]bort  │
└─────────────────────┬───────────────────────────────┘
                      ▼
  Cleanup offered (automated)
  → Merge in GitHub UI (manual)
```

### Back-Navigation

Every checkpoint supports going **back** to the previous checkpoint. The pipeline is a state machine, not a linear sequence:

- **CP2 → CP1**: Re-review changes, request more modifications
- **CP3 → CP2**: Re-commit with different content
- **CP4 → CP3**: Edit PR before re-reviewing
- **CP5 → CP4**: Fix more items or re-review

### Example Walkthrough

```bash
$ bash scripts/deploy-issue.sh 315 --pipeline --base integration/epic-300

═══════════════════════════════════════════════════════
  Phase 1/5 — Agent Working (feature-infra-core)
═══════════════════════════════════════════════════════
  ... agent works for ~5 minutes ...

═══════════════════════════════════════════════════════
  CHECKPOINT 1/5 — Changes Review
═══════════════════════════════════════════════════════
  # Change Report — Issue #315
  ## Summary
  Added version fields to all 21 IPC structs...
  ...
  [a] accept  [c] changes  [r] reject
  Your choice: a

  Running Validation (validate-session.sh)...
  [1/5] Build verification       PASS
  [2/5] Test count verification   PASS  (1309 tests)
  [3/5] Test execution            PASS
  [4/5] Include verification      PASS
  [5/5] PR sanity                 PASS

═══════════════════════════════════════════════════════
  CHECKPOINT 2/5 — Commit Approval
═══════════════════════════════════════════════════════
  Validation passed.
  [c] commit  [b] back  [a] abort
  Your choice: c

═══════════════════════════════════════════════════════
  CHECKPOINT 3/5 — PR Preview
═══════════════════════════════════════════════════════
  Title: feat(#315): Add version fields to all IPC structs
  Body: ...
  [c] create  [e] edit  [b] back  [a] abort
  Your choice: c
  PR created: https://github.com/.../pull/361

  Deploying review agents... (4 agents in parallel)
  DONE  review-memory-safety
  DONE  review-security
  DONE  review-concurrency
  DONE  review-fault-recovery

═══════════════════════════════════════════════════════
  CHECKPOINT 4/5 — Safety Review Findings
═══════════════════════════════════════════════════════
  P1: validate() never checks version field (2 findings)
  P2: Missing default member initializers (1 finding)
  P3: Positional aggregate init fragile (1 finding)
  [a] accept  [f] fix  [b] back  [r] reject
  Your choice: f

  Launching feature agent to fix findings...
  ... agent fixes P1/P2 ...
  Re-validating... PASS

═══════════════════════════════════════════════════════
  CHECKPOINT 5/5 — Final Summary
═══════════════════════════════════════════════════════
  c885d53 feat(#315): Add version fields to all IPC structs
  a1b2c3d fix(#315): address review findings
  Validation passed.
  [d] done  [r] re-review  [b] back  [a] abort
  Your choice: d

  Pipeline Complete
  PR: https://github.com/.../pull/361
  Next step: Review and merge the PR in GitHub UI.
```

---

## Label Routing Reference

Issues are routed to agents based on GitHub labels. When multiple domain labels exist, **priority wins** (not label order):

| Priority | Labels | Agent |
|----------|--------|-------|
| 3 (highest) | `perception`, `domain:perception` | feature-perception |
| 3 | `nav-planning`, `domain:nav` | feature-nav |
| 3 | `comms`, `domain:comms` | feature-integration |
| 3 | `ipc` | feature-infra-core |
| 2 | `integration`, `domain:integration` | feature-integration |
| 2 | `common`, `infra`, `infrastructure`, `modularity`, `domain:infra-core` | feature-infra-core |
| 1 (lowest) | `platform`, `deploy`, `ci`, `bsp`, `domain:infra-platform` | feature-infra-platform |
| Direct | `safety-audit` | review-memory-safety |
| Direct | `security-audit` | review-security |
| Direct | `test-coverage` | test-unit |
| Direct | `cross-domain` | tech-lead |

## Review Routing Reference

Reviews are routed based on **diff content**, not labels:

| Trigger in diff | Reviewers |
|-----------------|-----------|
| Any file | memory-safety + security + unit-test |
| `std::atomic`, `mutex`, `thread`, `lock_guard` | + concurrency |
| P4/P5/P7 paths, watchdog, fault handling | + fault-recovery |
| IPC, HAL, Gazebo configs | + scenario-test |

## Shared State & Cross-Agent Context

Agents coordinate through these shared files:

| File | Purpose | Who writes | Who reads |
|------|---------|-----------|-----------|
| `tasks/active-work.md` | Live work tracker — what's in-progress | deploy-issue.sh (start + cleanup) | All agents at session start |
| `tasks/agent-changelog.md` | Completed work log — what was recently done | Pipeline CLEANUP, run-session.sh | All agents at session start |
| `.claude/shared-context/domain-knowledge.md` | Non-obvious pitfalls discovered during work | Any agent (tech-lead reviews) | All agents at session start |
| `docs/guides/AGENT_HANDOFF.md` | Cross-domain handoff protocol | tech-lead | Agents during handoff |
| `tests/TESTS.md` | Test inventory and baseline count | Feature agents | All agents (verify test count) |
| `docs/tracking/PROGRESS.md` | Improvement history | Feature agents | Agents needing project context |
| `docs/tracking/ROADMAP.md` | Planned work and completion status | Feature agents | Agents checking what's done |
| `docs/tracking/BUG_FIXES.md` | Bug fix log with root causes | Feature agents | Agents investigating regressions |

### How Cross-Agent Context Works

Every agent session automatically receives context about other agents' work. Both `deploy-issue.sh` and `start-agent.sh` inject this context into the agent's prompt at startup:

1. **Active work awareness** — The agent sees entries from `tasks/active-work.md` showing which issues are in-progress, on which branches, by which agents. This prevents two agents from unknowingly modifying the same files.

2. **Recent completion log** — The agent sees the last ~5 entries from `tasks/agent-changelog.md` showing what was recently completed. This prevents duplicate work and gives awareness of recent codebase changes.

3. **Domain knowledge pitfalls** — The agent receives the full contents of `.claude/shared-context/domain-knowledge.md`, which contains non-obvious pitfalls discovered during previous sessions (e.g., "Zenoh sessions leak if not closed in test teardown", "radar returns NaN at ranges < 0.5m").

### Lifecycle of Shared State

```
Session Start                    Session End (Pipeline CLEANUP)
─────────────                    ─────────────────────────────
                                 
deploy-issue.sh                  CLEANUP state:
  │                                │
  ├─ Write active-work.md         ├─ Mark active-work.md → completed
  │  (status: in-progress)        │
  │                                ├─ Append to agent-changelog.md
  ├─ Read active-work.md          │  (issue, branch, role, findings)
  │  (inject into prompt)          │
  │                                ├─ Check AGENT_REPORT.md for pitfalls
  ├─ Read agent-changelog.md      │  → remind about domain-knowledge.md
  │  (inject into prompt)          │
  │                                └─ Verify issue↔PR linking
  └─ Read domain-knowledge.md
     (inject into prompt)
```

### Required Documentation Updates

Every agent (in both `--auto` and `--pipeline` mode) is instructed to update these docs before completing work:

- **`tests/TESTS.md`** — If tests were added/modified, update the count and add entries
- **`docs/tracking/PROGRESS.md`** — Add an improvement entry (number, title, date, files, rationale)
- **`docs/tracking/ROADMAP.md`** — Mark the issue as done if it appears in the roadmap
- **`docs/tracking/BUG_FIXES.md`** — Add an entry if the work is a bug fix

These updates ensure that the next agent session starts with accurate project state, not stale data.

## Workflow Examples

### Example 1: Single Issue

```bash
# 1. Create issue on GitHub with labels: "perception", "enhancement"
# 2. Deploy
bash scripts/deploy-issue.sh 400

# Agent creates worktree, implements feature, creates PR
# 3. Review
bash scripts/deploy-review.sh <pr-number>

# 4. Validate and merge
bash scripts/validate-session.sh --branch feature/issue-400-...
# Tech lead merges if all checks pass
```

### Example 2: Multi-Issue Epic with Integration Branch

```bash
# 1. Create integration branch
git checkout -b integration/epic-500 main
git push -u origin integration/epic-500

# 2. Deploy sub-issues against the integration branch
bash scripts/deploy-issue.sh 501 --base integration/epic-500
bash scripts/deploy-issue.sh 502 --base integration/epic-500
bash scripts/deploy-issue.sh 503 --base integration/epic-500

# Each agent creates a PR targeting integration/epic-500 (not main)

# 3. After all sub-PRs merge to integration branch:
# Create a single PR from integration/epic-500 → main
gh pr create --base main --head integration/epic-500 \
  --title "feat(#500): Epic description" \
  --body "Merges epic work from integration branch"

# 4. Delete integration branch after merge
git branch -d integration/epic-500
git push origin --delete integration/epic-500
```

### Example 3: Interactive Debugging Session

```bash
# Launch an agent interactively for a specific domain
bash scripts/start-agent.sh feature-integration

# You can now have a conversation with the agent about P5/P6/P7 code
```

### Example 4: Auto Mode — Full Walkthrough

This example walks through the complete `--auto` workflow for issue #315 (add version fields to IPC structs), targeting an integration branch.

#### Step 1: Create the integration branch (one-time per epic)

```bash
git branch integration/epic-357 main
git push -u origin integration/epic-357
```

#### Step 2: Ensure the issue has labels

```bash
$ gh issue view 315 --json labels --jq '.labels[].name'
ipc
platform
```

Good — `ipc` (priority 3) will route to `feature-infra-core`.

#### Step 3: Preview routing

```bash
$ bash scripts/deploy-issue.sh 315 --base integration/epic-357 --dry-run

Fetching issue #315...
  Title:  feat(#300-C1): Add version fields to all IPC structs
  Labels: ipc
platform

Routing decision
  Issue:  #315 — feat(#300-C1): Add version fields to all IPC structs
  Role:   feature-infra-core
  Base:   integration/epic-357
  Branch: feature/issue-315-feat300-c1-add-version-fields-to-all-ipc

DRY RUN — would perform:
  1. git worktree add .claude/worktrees/issue-315 -b feature/issue-315-... integration/epic-357
  2. Update tasks/active-work.md
  3. Launch: start-agent.sh feature-infra-core <issue prompt>
```

#### Step 4: Launch in auto mode

```bash
$ bash scripts/deploy-issue.sh 315 --base integration/epic-357 --auto

Fetching issue #315...
  Title:  feat(#300-C1): Add version fields to all IPC structs
  Labels: ipc
platform

Routing decision
  Issue:  #315 — feat(#300-C1): Add version fields to all IPC structs
  Role:   feature-infra-core
  Base:   integration/epic-357
  Branch: feature/issue-315-feat300-c1-add-version-fields-to-all-ipc

Creating worktree...
  DONE  Worktree: .claude/worktrees/issue-315
  DONE  Updated tasks/active-work.md

Auto mode — agent working autonomously...
  Report: .claude/worktrees/issue-315/AGENT_REPORT.md

# ... agent works for several minutes ...
```

#### Step 5: Review the report

When the agent finishes, the report is displayed automatically:

```
═══════════════════════════════════════════════════════
  Agent work complete — review phase
═══════════════════════════════════════════════════════

# Change Report — Issue #315

## Summary
Added `uint16_t version` field to all 12 IPC structs in ipc_types.h with
default value 1. Updated serialization, added version validation on
deserialization, and added 24 new unit tests.

## Files Changed
| File                          | Change   | Reason                          |
|-------------------------------|----------|---------------------------------|
| common/ipc/include/ipc_types.h | Modified | Added version field to structs |
| common/ipc/src/message_bus.cpp  | Modified | Version validation on receive  |
| tests/test_ipc_versioning.cpp   | Created  | 24 new tests for versioning    |

## Tests Added/Modified
- test_ipc_versioning.cpp: version field defaults, wire format, mismatch handling

## Decisions Made
- Used uint16_t (not uint8_t) to allow >255 versions over project lifetime
- Version mismatch logs a warning but doesn't reject — forward compatibility

## Risks / Review Attention
- Wire format changed — existing recorded data won't deserialize without migration
- static_assert(is_trivially_copyable) still passes for all structs

## Build & Test Status
Build: PASS (zero warnings)
Tests: 1259 → 1283 (24 added, 0 failures)

═══════════════════════════════════════════════════════

  [a]ccept  — approve changes, ready for PR
  [c]hanges — request changes (opens interactive session)
  [r]eject  — discard all changes

  Your verdict: _
```

#### Step 6a: Accept

```
  Your verdict: a

  Accepted. Changes are committed in: .claude/worktrees/issue-315
  Next steps:
    cd .claude/worktrees/issue-315
    git push -u origin feature/issue-315-feat300-c1-add-version-fields-to-all-ipc
    gh pr create --base integration/epic-357
```

#### Step 6b: Request changes

```
  Your verdict: c

  Opening interactive session — tell the agent what to change.
  The agent has all its previous context.

> Use uint8_t instead of uint16_t for the version field, and add a
> migration helper for old recorded data.
```

The agent picks up its previous context (`--continue`) and applies your feedback. When done, you review again.

#### Step 6c: Reject

```
  Your verdict: r

  This will reset all changes. Are you sure? [y/N] y
  Changes discarded.
```

All changes in the worktree are reverted.

#### Step 7: Validate, push, and deploy reviewers

After accepting (step 6a), validate and create the PR:

```bash
cd .claude/worktrees/issue-315

# Validate — check for hallucinations
bash "$REPO_ROOT/scripts/validate-session.sh"

# Push and create PR
git push -u origin feature/issue-315-feat300-c1-add-version-fields-to-all-ipc
gh pr create --base integration/epic-357

# Deploy review agents — they run in parallel
bash "$REPO_ROOT/scripts/deploy-review.sh" <pr-number>
```

Monitor review progress in a separate terminal:

```bash
watch -n 5 'wc -l tasks/sessions/*-review-pr<number>-*.log'
```

#### Step 8: Review findings and feed back to the feature agent

The review agents post a consolidated comment on the PR. Read it:

```bash
gh pr view <pr-number> --comments
```

Each finding has a severity (P1-P4). For P1/P2 findings, open an interactive session
with the **same feature agent** that did the original work — it has the best context:

```bash
cd .claude/worktrees/issue-315
bash "$REPO_ROOT/scripts/start-agent.sh" feature-infra-core
```

Paste a structured prompt with each finding, severity, and specific fix instructions
(see [Manual Mode Step 6](#step-6-feed-review-findings-to-the-feature-agent) for the
full prompt template and a real-world example).

The agent applies fixes. You approve each change. Then re-validate and push:

```bash
bash "$REPO_ROOT/scripts/validate-session.sh"
git push
```

Optionally re-run reviewers to verify fixes:

```bash
bash "$REPO_ROOT/scripts/deploy-review.sh" <pr-number>
```

#### Step 9: Merge in GitHub UI

Once all P1/P2 findings are addressed, merge the PR in GitHub.

## Limitations

1. **No real-time agent-to-agent communication.** Agents coordinate through files (`active-work.md`, branches, PRs) — not direct messaging. If Agent A needs Agent B's output, Agent A must wait for Agent B's PR to merge.

2. **Review agents are advisory only.** Review agents post comments but cannot block merges programmatically. The tech lead (human or orchestrator) must read and act on review comments.

3. **No automatic conflict resolution.** When two agents modify overlapping files (even via separate worktrees), merge conflicts must be resolved manually. The integration branch pattern mitigates this but doesn't eliminate it.

4. **Label routing requires upfront labeling.** `deploy-issue.sh` routes based on GitHub labels. If an issue has no recognized labels, the script fails. Labels must be applied before deployment.

5. **Single-machine worktree limit.** All worktrees exist on one machine. The pipeline doesn't distribute agents across multiple machines (though the Claude Agent SDK could support this).

6. **Claude CLI dependency.** Agents require the Claude Code CLI installed and authenticated. Each agent invocation consumes API tokens — there's no built-in cost cap or rate limiting.

7. **Limited persistent agent memory across sessions.** Each agent invocation starts fresh. Cross-agent context (active work, recent changelog, domain knowledge) is injected into every prompt automatically, but agents cannot recall the full history of previous sessions or decisions. The shared-state files provide continuity but not deep context.

8. **CI integration is advisory.** The `agent-checks.yml` workflow warns about boundary violations and PR size — it doesn't block merges. The `auto-review.yml` workflow is disabled by default.

9. **Hallucination detection is heuristic.** `validate-session.sh` checks for common hallucination patterns (missing includes, phantom files in PR body, test count regression) but cannot catch all cases. Human review remains essential.

10. **No rollback automation.** If an agent's merged PR introduces a regression, there's no automated revert. Standard git revert workflows apply.

## Future Improvements

1. **Distributed agent execution.** Run agents on multiple machines or cloud instances, coordinated through GitHub as the source of truth. Each machine pulls issues, creates worktrees, and pushes PRs independently.

2. **Cost tracking and budgets.** Track API token usage per agent/session/issue. Set per-issue cost budgets with automatic termination. Dashboard shows cost-per-test, cost-per-line, and cost-per-fix.

3. **Automatic PR chaining.** When Agent A's PR merges, automatically trigger dependent agents (Agent B) based on handoff issues. Currently requires manual `deploy-issue.sh` invocation.

4. **Persistent agent context.** Give agents access to a vector store of previous sessions, decisions, and domain knowledge so they can learn from past work without re-deriving context.

5. **Merge queue integration.** Integrate with GitHub's merge queue to automatically sequence merges, run CI, and handle rebase conflicts.

6. **Real-time conflict detection.** Monitor active worktrees and warn when two agents are modifying overlapping files before they create PRs.

7. **Agent performance scoring.** Track metrics per agent role: review comment hit rate, test coverage delta, build break rate, hallucination rate. Use this to tune agent prompts and routing.

8. **Multi-repo support.** Extend the pipeline to work across multiple repositories (e.g., a shared library repo + an application repo).

9. **Blocking CI gates.** Promote `agent-checks.yml` from advisory to blocking once the pipeline is validated. Enable `auto-review.yml` to automatically trigger review agents on every PR.

10. **Natural language issue routing.** Use the issue title and body (not just labels) to route to the correct agent, eliminating the labeling requirement.
