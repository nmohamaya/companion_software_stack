<!-- SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary -->
<!-- Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md. -->

---
name: deploy-wave
description: Deploy multiple related issues as a wave — plan all upfront, classify dependencies, execute independent issues in parallel against an integration branch, review the combined diff, merge to main
argument-hint: "<issue-numbers...> --epic <N> [--base <branch>]"
---

# /deploy-wave — Multi-Issue Wave Deployment

Deploy a wave of related issues from an epic. Plans all issues upfront as a batch, classifies dependencies between issues, executes independent issues in parallel using worktrees, runs a single combined review on the full wave diff, and creates a final merge PR to main.

**This is the orchestrator for multi-issue waves.** It replaces running `/deploy-issue` N times by adding wave-level planning, dependency analysis, parallel execution, and combined review — catching cross-issue conflicts and shared-interface problems before they compound.

## Session Continuity

If this skill was already loaded earlier in the conversation (e.g., from a prior invocation or context compaction), the Skill tool call may fail with "Unknown skill." In that case, **immediately acknowledge to the user**: "The deploy-wave skill is already active in this session — I'll continue with your arguments." Then proceed to execute the pipeline with the provided arguments. Never leave the user with silence.

## Arguments

Parse `$ARGUMENTS` to extract:
- **issue_numbers** (required): Space-separated issue numbers (e.g., `288 289 290`)
- **--epic \<N\>** (required): Parent epic number for branch naming and tracking
- **--base \<branch\>** (optional): Target branch for the final merge PR (default: `main`)

If no arguments provided, ask the user for the issue numbers and epic.

## Pipeline Overview

```
┌─────────────────────────────────────────────┐
│ PHASE 1: SETUP                              │
│ Fetch all issues → Create integration branch │
└──────────────────────┬──────────────────────┘
                       ▼
┌─────────────────────────────────────────────┐
│ PHASE 2: WAVE PLAN (key value-add)          │
│ Tech-lead plans ALL issues → User approves  │
│ Dependency graph → Execution groups         │
└──────────────────────┬──────────────────────┘
                       ▼
┌─────────────────────────────────────────────┐
│ PHASE 3: PER-GROUP EXECUTION (loop)         │
│ For each execution group:                   │
│   Independent → Parallel agents (worktrees) │
│   Dependent   → Sequential agent            │
│   Merge sequentially → Validate → Batch CP  │
└──────────────────────┬──────────────────────┘
                       ▼
┌─────────────────────────────────────────────┐
│ PHASE 4: WAVE REVIEW                        │
│ Two-pass review on combined integration     │
│ branch diff vs main                         │
└──────────────────────┬──────────────────────┘
                       ▼
┌─────────────────────────────────────────────┐
│ PHASE 5: MERGE & CLEANUP                    │
│ Final merge PR: integration → main          │
│ Close issues, update tracking               │
└─────────────────────────────────────────────┘
```

---

## PHASE 1: Setup [SEQUENTIAL]

**Step 1.1 — Fetch all issues**

```bash
for N in <issue_numbers>; do
  gh issue view $N --json title,body,labels,state,milestone
done
```

Display a summary table:

```
═══ Wave Issues ═══
  #288 — Per-process CMake enable options       [infra, modularity]
  #289 — TopicResolver + vehicle_id namespace   [ipc, modularity]
  #290 — ISysInfo platform abstraction          [infra, modularity]
  
  Epic: #<epic_number>
  Target: main
```

If any issue is closed, warn and ask whether to include it.

**Step 1.2 — Route each issue to an agent role**

Use the same label routing as `/deploy-issue` (from `scripts/orchestrator/config.py`). Present the routing table:

```
  #288 → feature-infra-platform  (labels: platform, deploy)
  #289 → feature-infra-core      (labels: ipc, common)
  #290 → feature-infra-core      (labels: infra, common)
```

User can override any routing.

**Step 1.3 — Create integration branch**

```bash
git fetch origin
branch_name="integration/epic-<epic_number>-wave-<wave_number>"
git checkout -b "$branch_name" "origin/<base_branch>"
git push -u origin "$branch_name"
```

If an integration branch for this epic already exists (`git branch -r --list 'origin/integration/epic-<N>*'`), ask the user whether to reuse it or create a new one.

---

## PHASE 2: Wave Plan [MANDATORY]

This is the key value-add over running `/deploy-issue` N times. The tech-lead plans ALL issues as a coherent batch, analyzes dependencies between them, and groups independent issues for parallel execution.

**Step 2.1 — Read relevant code for all issues**

For each issue, read the source files that will need to change. Build a mental model of the full wave scope.

**Step 2.2 — Produce the wave plan**

Present a unified plan covering all issues. The plan has three parts: per-issue implementation plans, dependency analysis, and execution groups.

**Step 2.2a — Dependency Analysis**

For every pair of issues in the wave, classify the relationship:

- **Independent**: No shared files, no interface dependency (issue A's output is not consumed by issue B). These can run in parallel.
- **Dependent**: Issue B uses types, interfaces, or files created/modified by issue A. B must run after A merges. Record the direction: `A → B` meaning "B depends on A."
- **Independent-with-conflict-risk**: Both issues modify the same file but for orthogonal reasons (e.g., both add entries to a CMakeLists.txt). Mark as independent — can run in parallel — but merge order matters and conflict is expected.

The classification is based on the file lists from Step 2.1. Two issues share a dependency if:
1. Their planned file-change lists overlap (same file modified by both), OR
2. One issue creates a new header/interface that the other issue's plan imports or references

**Step 2.2b — Execution Groups**

Using the dependency graph from 2.2a, partition issues into ordered execution groups:

- An execution group is a set of issues that are mutually independent (no edges between them in the dependency graph).
- Groups execute in sequence: all issues in group N must complete and merge before group N+1 starts.
- Within a group, all issues execute in **parallel** (each in its own worktree).
- Cap: **max 3 parallel agents per group** (gives each agent sufficient context window). If more than 3 independent issues, split into sub-groups of 3.

The partitioning algorithm is a topological sort of the dependency DAG, where each "level" of the topological order becomes one execution group. Issues with no unresolved dependencies go into the current group; after that group completes, remove their edges; repeat.

**Step 2.2c — Present the wave plan**

```
═══ WAVE PLAN — Epic #<N> Wave <W> ═══

Dependency graph:
  #288 (CMake options) — independent
  #290 (ISysInfo)      — independent
  #289 (TopicResolver) — depends on #290

Execution groups:
  Group 1 [PARALLEL]: #288, #290
  Group 2 [SEQUENTIAL]: #289 (depends on #290)

┌── GROUP 1 — Parallel ──────────────────────────┐
│                                                 │
│ Issue #288 — Per-process CMake enable options    │
│ Agent: feature-infra-platform                   │
│ Approach: <concrete implementation plan>        │
│ Files: <list of files to modify/create>         │
│ Tests: <what to test>                           │
│ Scope: ~150 lines                               │
│                                                 │
│ Issue #290 — ISysInfo platform abstraction      │
│ Agent: feature-infra-core                       │
│ Approach: <concrete implementation plan>        │
│ Files: <list of files to modify/create>         │
│ Tests: <what to test>                           │
│ Scope: ~200 lines                               │
│                                                 │
└─────────────────────────────────────────────────┘

┌── GROUP 2 — Sequential (depends on Group 1) ───┐
│                                                 │
│ Issue #289 — TopicResolver + vehicle_id         │
│ Agent: feature-infra-core                       │
│ Approach: <concrete implementation plan>        │
│ Files: <list of files to modify/create>         │
│ Tests: <what to test>                           │
│ Scope: ~250 lines                               │
│                                                 │
└─────────────────────────────────────────────────┘

Shared concerns:
  - Cross-issue interfaces: #290 creates ISysInfo, #289 uses it
  - Conflict risk: none within groups (or: list any independent-with-conflict-risk pairs)

Total estimated scope: ~600 lines across 3 PRs
```

**Parallel justification table** (shown for each parallel group before user approval):

```
═══ Parallel Justification — Group 1 ═══

Why #288 and #290 can run in parallel:

| Criterion           | #288 (CMake options)          | #290 (ISysInfo)                 | Overlap? |
|---------------------|-------------------------------|---------------------------------|----------|
| Files modified      | CMakeLists.txt (root), cmake/ | common/util/include/util/,      | None     |
|                     | process[1-7]_*/CMakeLists.txt | process7_system_monitor/        |          |
| Files created       | cmake/ProcessOptions.cmake    | isys_info.h, jetson_sys_info.h  | None     |
|                     |                               | linux_sys_info.h, mock_sys_info |          |
| Headers included    | None new                      | <sys/sysinfo.h>, <fstream>      | None     |
| Interfaces produced | CMake options (build-only)    | ISysInfo interface              | None     |
| Interfaces consumed | None                          | None                            | None     |
| Config keys         | ENABLE_PROCESS_*              | system_monitor.sys_info_backend | None     |

Verdict: INDEPENDENT — no shared files, no interface dependency, no config overlap.

Why #289 must wait for #290:
  #289 imports ISysInfo (created by #290) in TopicResolver
  → Dependent: #289 runs after #290 merges
```

This justification table gives the user concrete evidence to verify the parallelism classification. If they spot a missed dependency (e.g., "actually #288 also touches common/util/CMakeLists.txt"), they can override before agents launch.

User choices:
- **approve** → proceed to Phase 3
- **modify** → adjust plan (reorder, change approach, drop an issue, reclassify dependency, move issue between groups)
- **abort** → stop

---

## PHASE 3: Execution [GROUP-BASED]

### Step 3.0 — Wave Execution Mode Selection

Before starting execution, present the mode choice:

```
═══ Wave Execution Mode ═══

  [S]tandard — validate after each group (current behavior)
     Safer: failures isolated per-issue, easy to attribute
     Time: ~<N * 15-20> min for <N> issues

  [F]ast — all agents first, single validation pass at end
     ~50-65% faster: agents work in parallel, one build+test+review at end
     Time: ~<20-30> min regardless of issue count

  Recommendation: <Fast|Standard> based on:
    - Issue count: <N> issues
    - Independence: <all independent | has dependencies>
    - Estimated time saved: ~<X> min
```

User choices:
- **standard** → execute Phase 3A (group-based, validate per group)
- **fast** → execute Phase 3B (all agents first, single validation)

---

### PHASE 3A: Standard Mode — Per-Group Execution

Execute each execution group from Phase 2 in order. Within a group, independent issues run their feature agents in **parallel** (each in its own worktree). Between groups, execution is sequential — group N+1 starts only after group N is fully merged and validated.

For each execution group G:

### Step 3A.1 — Pre-group setup

1. Push integration branch to remote (agents branch from it):
   ```bash
   git push origin <integration_branch>
   ```
2. Gather cross-agent context (read if they exist, skip if missing):
   - `tasks/active-work.md` — current in-flight work
   - `tasks/agent-changelog.md` — recent completed work
   - `.claude/shared-context/domain-knowledge.md` — non-obvious pitfalls
3. For groups after Group 1: also prepare summaries of all previously merged issues (from prior groups' AGENT_REPORT.md files) to include in agent prompts.

### Step 3A.2 — Spawn feature agents

**Single-issue group:** Spawn one agent (identical to the sequential case):

```
Agent(
  description: "Implement issue #<N> (wave <W>, group <G>)",
  subagent_type: "<routed-role>",
  isolation: "worktree",
  prompt: <plan for this issue> + acceptance criteria + cross-agent context + pipeline instructions
)
```

**Multi-issue group (parallel):** Spawn ALL agents in the group using **multiple `Agent()` calls in a single message**:

```
// All in a single message — they run in parallel
Agent(
  description: "Implement issue #<N1> (wave <W>, group <G>, 1/<group_size>)",
  subagent_type: "<routed-role-1>",
  isolation: "worktree",
  prompt: <plan for issue N1> + acceptance criteria + ...
)
Agent(
  description: "Implement issue #<N2> (wave <W>, group <G>, 2/<group_size>)",
  subagent_type: "<routed-role-2>",
  isolation: "worktree",
  prompt: <plan for issue N2> + acceptance criteria + ...
)
```

**Context trimming for agent prompts:** Pass only the **acceptance criteria** extracted from the issue body, not the full issue body. The tech-lead's implementation plan (from Phase 2) is the primary spec and already incorporates the relevant context. Only include the full issue body if the plan explicitly references "see issue for details" or the acceptance criteria cannot stand alone. Cross-agent context should only be included when the issue touches shared modules (IPC, HAL, common/).

Pipeline instructions for each agent (same for single or parallel):
- Implement the issue following the plan
- The integration branch is `<branch_name>` — your worktree branches from it
- Run `bash deploy/build.sh` and verify zero warnings
- Run tests with `./tests/run_tests.sh` and verify pass rate
- Write `AGENT_REPORT.md` with: Summary, Changes, Design Decisions, Test Coverage, Build Verification, Remaining Work
- If you deviate from the plan, document why
- Do NOT create a PR or push

**Additional prompt for parallel agents:** "Other issues executing in parallel in this group: #X, #Y. You share no files with them (verified in the dependency analysis). Do not modify files outside your plan."

### Step 3A.3 — Wait for completion

All agents in the group complete before proceeding. The Agent tool handles this implicitly when multiple calls are made in one message — all return before the next step.

### Step 3A.4 — Sequential merge-back

Even when agents run in parallel, merging to the integration branch is **always sequential** (one at a time, in planned merge order). This prevents merge conflicts between parallel agents.

For each completed agent in the group, in the planned merge order:

1. Read `AGENT_REPORT.md` from the agent's worktree
2. Merge the agent's worktree branch into the integration branch:
   ```bash
   git merge <agent_worktree_branch> --no-edit
   ```
3. If merge conflict occurs:
   - For **independent-with-conflict-risk** pairs (identified in Phase 2): attempt auto-resolve for append-only files (CMakeLists.txt entries, config additions are typically trivially resolved)
   - If auto-resolve fails, present the conflict to the user with options:
     a) **Resolve manually** — user fixes the conflict in the current session
     b) **Re-run agent** — re-run the conflicting agent with the current integration branch state as base (it now sees all previously merged agents' work)
     c) **Move to next group** — defer this issue to a later sequential group
     d) **Skip issue** — drop this issue from the wave
4. **Clean up the agent worktree immediately** after merging (or after deciding to skip):
   ```bash
   git worktree remove .claude/worktrees/<agent-id> --force
   git branch -D worktree-<agent-id>
   ```
   Do this right after merging, not at the end of the wave. Stale worktrees pollute the VS Code source control panel and risk accidental commits to dead branches.

### Step 3A.5 — Validate merged group

Run validation on the integration branch after ALL agents in the group are merged. This runs once per group (not once per issue):

- **Build**: `bash deploy/build.sh` (verify exit code 0, zero warnings)
- **Format**: Find changed C++ files and check with `clang-format-18 --dry-run --Werror`
- **Test count**: `ctest -N --test-dir build | grep "Total Tests:"` — compare against baseline in `tests/TESTS.md`
- **Hallucination checks**: includes, report vs diff, config keys, symbols — same as deploy-issue Phase 3

### Step 3A.6 — Group checkpoint

**Single-issue group:** Present per-issue checkpoint (same format as before):

```
═══ Issue — #<N> <title> ═══

[AGENT_REPORT summary]

--- Diff Summary ---
[git diff --stat]

--- Validation ---
Build: PASS/FAIL
Format: PASS/FAIL
Test count: N (expected M)

--- Tech-Lead Assessment ---
[your assessment]
```

**Multi-issue group (parallel):** Present **batch checkpoint** showing each issue's results and combined validation:

```
═══ Group <G>/<total_groups> — <N> Issues (Parallel) ═══

┌── Issue #<N1> — <title> ──────────────────────┐
│ [AGENT_REPORT summary, max 100 words]         │
│ Diff: +<A> -<B> across <C> files              │
│ Merge: clean                                   │
└────────────────────────────────────────────────┘

┌── Issue #<N2> — <title> ──────────────────────┐
│ [AGENT_REPORT summary, max 100 words]         │
│ Diff: +<A> -<B> across <C> files              │
│ Merge: clean                                   │
└────────────────────────────────────────────────┘

--- Combined Validation ---
Build: PASS/FAIL
Format: PASS/FAIL
Test count: N (expected M)

--- Tech-Lead Assessment ---
[assessment of the group as a whole + any cross-issue observations]
```

User choices:
- **accept all** → commit all, create per-issue PRs, move to next group
- **accept #X, reject #Y** → selective accept/reject per issue. Rejected issues: `git revert <merge-commit> --no-edit` to undo their merge from the integration branch.
- **changes #X** → make modifications to a specific issue, re-validate, then re-present the group checkpoint
- **skip group** → skip all issues in this group, move to next
- **abort wave** → stop the entire wave

### Step 3A.7 — Commit and PR (for accepted issues)

For each accepted issue in the group:

1. Commit: `feat(#<N>): <title>`
2. Push to integration branch
3. Create PR: `gh pr create --title "feat(#<N>): <title>" --base <integration_branch>`
4. Apply labels and milestone from source issue

### Step 3A.8 — Loop to next group

Repeat Steps 3A.1–3A.7 for the next execution group. Each subsequent group's agents receive summaries of all previously merged issues (from prior groups) so they have full context and don't duplicate or conflict.

---

### PHASE 3B: Fast Mode — All Agents First, Single Validation

In Fast mode, ALL feature agents run first (in parallel worktrees), then a single validation+review pass runs on the combined result. This eliminates redundant per-group build/test/review cycles.

### Step 3B.1 — Spawn ALL feature agents [PARALLEL]

Push integration branch to remote, then spawn **all** feature agents simultaneously, regardless of execution groups:

```bash
git push origin <integration_branch>
```

```
// All agents in a single message — maximum parallelism
Agent(
  description: "Implement issue #<N1> (wave <W>, fast mode, 1/<total>)",
  subagent_type: "<routed-role-1>",
  isolation: "worktree",
  prompt: <plan for issue N1> + acceptance criteria + pipeline instructions
)
Agent(
  description: "Implement issue #<N2> (wave <W>, fast mode, 2/<total>)",
  subagent_type: "<routed-role-2>",
  isolation: "worktree",
  prompt: <plan for issue N2> + acceptance criteria + pipeline instructions
)
// ... up to all issues in the wave
```

Cap at **3 parallel agents** per spawn. If the wave has more than 3 issues, batch into groups of 3 and wait for each batch before spawning the next. This gives each agent sufficient context window.

Same context trimming rules apply: pass acceptance criteria only (not full issue body), include cross-agent context only for shared modules.

### Step 3B.2 — Wait for all agents

All agents complete before proceeding. Report progress as each finishes:

```
  ✓ #288 (feature-infra-platform)  complete — AGENT_REPORT.md ready
  ✓ #290 (feature-infra-core)      complete — AGENT_REPORT.md ready
  ⏳ #289 (feature-infra-core)      running...
```

### Step 3B.3 — Sequential merge into integration branch

Merge each agent's work one-at-a-time, in dependency order (from Phase 2 dependency graph). After each merge, run a **quick compile check** (not full test suite):

```bash
# For each agent, in dependency order:
git merge <agent_worktree_branch> --no-edit

# Quick compile check — catches obvious breakage early
cd build && cmake --build . --target all -j$(nproc) 2>&1 | tail -5
```

**If compile fails after a merge:**
1. Identify which agent's merge broke the build
2. Present to user with options:
   a) **Fix inline** — manually resolve the issue
   b) **Revert and re-run** — `git revert HEAD --no-edit`, re-run that agent with updated integration branch
   c) **Skip issue** — revert the merge, continue with remaining agents

**Clean up each agent's worktree immediately after merging:**
```bash
git worktree remove .claude/worktrees/<agent-id> --force
git branch -D worktree-<agent-id>
```

### Step 3B.4 — Single full build + test pass

After all agents are merged, run the full validation once:

- **Build**: `bash deploy/build.sh` (verify exit code 0, zero warnings)
- **Format**: Find changed C++ files and check with `clang-format-18 --dry-run --Werror`
- **Test count**: `ctest -N --test-dir build | grep "Total Tests:"` — compare against baseline
- **Hallucination checks**: includes, report vs diff, config keys, symbols — spot-check mode

**If full build/test fails** (but quick compiles passed — indicates cross-issue interaction):
1. Identify the failure (build error, test failure, etc.)
2. Binary search: revert merges one-at-a-time from last to first until build passes
3. The last reverted merge is the culprit (or the interaction between it and prior merges)
4. Present to user with the culprit identified and options to fix or skip

### Step 3B.5 — Batch checkpoint (all issues)

Present all agents' work in a single checkpoint:

```
═══ Fast Mode — All Issues Complete ═══

┌── Issue #<N1> — <title> ──────────────────────┐
│ [AGENT_REPORT summary, max 100 words]         │
│ Diff: +<A> -<B> across <C> files              │
│ Merge: clean                                   │
└────────────────────────────────────────────────┘

┌── Issue #<N2> — <title> ──────────────────────┐
│ [AGENT_REPORT summary, max 100 words]         │
│ Diff: +<A> -<B> across <C> files              │
│ Merge: clean                                   │
└────────────────────────────────────────────────┘

[... all issues ...]

--- Combined Validation ---
Build: PASS/FAIL
Format: PASS/FAIL
Test count: N (expected M)

--- Tech-Lead Assessment ---
[assessment of the wave as a whole + cross-issue observations]
```

User choices:
- **accept all** → proceed to Phase 4 (wave review)
- **accept #X, reject #Y** → revert rejected issues' merge commits, re-validate
- **changes #X** → make modifications to a specific issue, re-validate
- **abort wave** → stop

### Step 3B.6 — Commit and push

For each accepted issue, commit and push to the integration branch. Create per-issue PRs against the integration branch.

After Phase 3B completes, proceed to Phase 4 (Wave Review) — the same combined review runs regardless of execution mode.

---

## PHASE 4: Wave Review [TWO PASSES]

After all issues are committed to the integration branch, run a combined review on the full wave diff.

**Step 4.1 — Get combined diff**

```bash
git diff origin/<base_branch>...HEAD --stat
git diff origin/<base_branch>...HEAD
```

**Step 4.2 — Build review roster**

Same roster logic as `/deploy-issue` Phase 5, but applied to the **combined diff**. This catches cross-issue problems that per-issue reviews would miss:
- Interface mismatches between issues
- Duplicate code across issue boundaries
- Inconsistent naming/patterns

Present the roster for approval (same format as deploy-issue Step 5.1).

**Step 4.3 — Launch Pass 1 agents** [PARALLEL]

Spawn Pass 1 review agents on the combined diff. Cap output to 200 words per agent.

**Step 4.4 — Launch Pass 2 agents** [PARALLEL, after Pass 1]

Spawn Pass 2 review agents with Pass 1 findings as context.

**Step 4.5 — Check bot/CI comments**

Check all per-issue PRs for Copilot/bot review comments:
```bash
for pr_number in <all_pr_numbers>; do
  gh api repos/<owner>/<repo>/pulls/$pr_number/comments --jq '.[].body'
done
```

Categorize each bot comment as: **already fixed** (by our agents), **worth fixing** (net-new), or **false positive** (with reason). Include actionable findings in the checkpoint presentation.

**Step 4.6 — Post findings**

Post the merged review findings as a comment on the **merge PR**. After all fixes are applied, **also post a Copilot triage table** summarizing what was already fixed by our agents, what was fixed from Copilot, and what was a false positive (with reasons). This documents the value-add of each review layer and creates a full audit trail.

### Wave Review Checkpoint

```
═══ WAVE REVIEW — Epic #<N> Wave <W> ═══

Combined diff: +<N> -<N> across <M> files, <K> PRs

--- Review Summary ---
Pass 1: <N> agents, <P1> findings, <P2> findings
Pass 2: <N> agents, <P1> findings, <P2> findings

--- Cross-Issue Findings ---
[anything that spans multiple issues]

--- Per-Issue Findings ---
[grouped by issue]

--- Tech-Lead Assessment ---
[your synthesis]
```

User choices:
- **accept** → proceed to Phase 5
- **fix** → fix issues, re-validate, re-review. **Always re-run BOTH Pass 1 AND Pass 2** — fixes frequently introduce new issues. Loop back to Step 4.2 (roster approval), not Phase 3.
- **defer with rationale** → for P3 findings or comments where we disagree with the recommendation, document each decision in `docs/guides/DESIGN_RATIONALE.md` as a new DR-NNN entry. This creates an audit trail showing the comment was evaluated, trade-offs weighed, and the decision was intentional — not an oversight. Each entry needs: the question, arguments for both sides, our decision, and when to revisit.
- **abort** → stop

---

## PHASE 5: Merge & Cleanup [SEQUENTIAL]

**Step 5.1 — Create merge PR**

```bash
gh pr create \
  --title "Epic #<N> Wave <W>: <description>" \
  --base <base_branch> \
  --head <integration_branch> \
  --body "<summary of all issues, links to per-issue PRs>"
```

The body should include:
- Summary of the wave (what was accomplished)
- Links to all per-issue PRs
- Combined test count verification
- `Closes #<issue1>, Closes #<issue2>, ...` for all issues in the wave

**Step 5.2 — Present final summary**

```
═══ WAVE COMPLETE — Epic #<N> Wave <W> ═══

Integration branch: <branch>
Merge PR: #<N> — <title>
Per-issue PRs: #<N1>, #<N2>, #<N3>

Issues closed: #<N1>, #<N2>, #<N3>
Total changes: +<N> -<N> across <M> files
Test count: <N> (baseline: <M>)

Review findings: <N> P1, <N> P2, <N> P3 (all addressed)
```

User choices:
- **done** → update tracking files, close issues, post summary comments
- **re-review** → re-run Phase 4
- **abort** → preserve branch for manual work

**Step 5.3 — Cleanup**

1. Verify merge PR body contains `Closes #<issue>` for each issue
2. Append wave entry to `tasks/agent-changelog.md`
3. Post summary comment on the epic issue (#<epic_number>) with wave progress
4. Update `tasks/active-work.md` if it exists

---

## Edge Cases

- **Agent conflict**: If agent B touches a file that agent A already modified on the integration branch, the merge in Step 3.4 may conflict. Resolve manually or re-run agent B with awareness of A's changes.
- **Partial wave**: If the user aborts mid-wave, the completed per-issue PRs remain on the integration branch. The wave can be resumed later by re-running with the remaining issues and `--base` pointing to the existing integration branch.
- **Single issue wave**: If only one issue number is provided, the skill degrades gracefully to essentially `/deploy-issue` with an integration branch. Still useful for consistency.
- **Existing integration branch**: Check `git branch -r --list 'origin/integration/epic-<N>*'` before creating. If one exists, offer to reuse it (pick up where a prior wave left off).
- **Context window management**: A full wave with 3 issues + 2-pass review generates significant context. Cap all agent outputs to 200 words. Summarize per-issue results before starting the next issue. If context gets tight, summarize earlier issues aggressively. For parallel groups: each agent's output is capped at 200 words; with the 3-agent-per-group cap, batch checkpoint output stays under ~900 words.
- **Empty per-issue PR**: If an issue's files were committed as part of a bulk operation (e.g., user's gitignore changes, re-tracking commits), the per-issue feature branch may show an empty diff against integration. In that case, skip the per-issue PR for that issue — note it in the wave summary and ensure the changes are covered by the merge PR.
- **Parallel agent failure**: If one agent in a parallel group fails (crashes, times out, or produces no useful output), the other agents' results are still valid. Merge the successful agents, skip the failed one, and note it in the group checkpoint. Offer to re-run the failed agent sequentially after the group merges.
- **Conflict during parallel merge-back**: If merging agent N's branch conflicts with a previously merged agent's changes from the same group, this means the Phase 2 dependency analysis missed a shared-file relationship. Present the conflict to the user with the options listed in Step 3.4. Log this as a lesson for future wave plans.
- **Selective rejection from batch**: If the user rejects one issue from a parallel batch after all have been merged, use `git revert <merge-commit> --no-edit` on that issue's merge commit. This is safe because merges were done sequentially — the revert creates a new commit that undoes only the specified merge. If the rejected issue was merged before other accepted issues, the revert is still clean.
- **All issues independent (single group)**: If all issues are mutually independent, the entire wave executes as one parallel group. Cap at 3 parallel agents per group to give each agent sufficient context window. If more than 3, split into sub-groups of 3.
- **All issues dependent (fully sequential)**: If every issue depends on the previous one, execution degrades to the current sequential model with per-issue checkpoints. No behavioral change from the original deploy-wave design.
- **Re-run after rejection**: If the user rejects an issue from a parallel batch and wants it re-run, spawn a single agent sequentially (it now branches from the updated integration branch with all accepted issues merged). This is effectively a mini-group of size 1.

If the user provided arguments, use them as context: $ARGUMENTS
