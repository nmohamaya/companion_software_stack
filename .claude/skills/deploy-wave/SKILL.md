<!-- SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary -->
<!-- Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md. -->

---
name: deploy-wave
description: Deploy multiple related issues as a wave — plan all upfront, execute sequentially against an integration branch, review the combined diff, merge to main
argument-hint: "<issue-numbers...> --epic <N> [--base <branch>]"
---

# /deploy-wave — Multi-Issue Wave Deployment

Deploy a wave of related issues from an epic. Plans all issues upfront as a batch, executes each against an integration branch, runs a single combined review on the full wave diff, and creates a final merge PR to main.

**This is the orchestrator for multi-issue waves.** It replaces running `/deploy-issue` N times by adding wave-level planning and combined review — catching cross-issue conflicts and shared-interface problems before they compound.

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
│ Identify sequence, shared files, risks      │
└──────────────────────┬──────────────────────┘
                       ▼
┌─────────────────────────────────────────────┐
│ PHASE 3: PER-ISSUE EXECUTION (loop)         │
│ For each issue in sequence:                 │
│   Agent work → Validate → Commit → PR       │
│   User checkpoint per issue                 │
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

This is the key value-add over running `/deploy-issue` N times. The tech-lead plans ALL issues as a coherent batch.

**Step 2.1 — Read relevant code for all issues**

For each issue, read the source files that will need to change. Build a mental model of the full wave scope.

**Step 2.2 — Produce the wave plan**

Present a unified plan covering all issues:

```
═══ WAVE PLAN — Epic #<N> Wave <W> ═══

Execution order: #288 → #290 → #289
Reasoning: CMake options (288) are build-only with no code deps.
           ISysInfo (290) creates interfaces used by 289's TopicResolver.
           TopicResolver (289) goes last as it may reference ISysInfo types.

┌─────────────────────────────────────────────────┐
│ Issue #288 — Per-process CMake enable options    │
│ Agent: feature-infra-platform                   │
│                                                 │
│ Approach: <concrete implementation plan>        │
│ Files: <list of files to modify/create>         │
│ Tests: <what to test>                           │
│ Scope: ~150 lines                               │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│ Issue #290 — ISysInfo platform abstraction      │
│ Agent: feature-infra-core                       │
│                                                 │
│ Approach: <concrete implementation plan>        │
│ Files: <list of files to modify/create>         │
│ Tests: <what to test>                           │
│ Scope: ~200 lines                               │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│ Issue #289 — TopicResolver + vehicle_id         │
│ Agent: feature-infra-core                       │
│                                                 │
│ Approach: <concrete implementation plan>        │
│ Files: <list of files to modify/create>         │
│ Tests: <what to test>                           │
│ Scope: ~250 lines                               │
└─────────────────────────────────────────────────┘

Shared concerns:
  - Files touched by multiple issues: <list, if any>
  - Cross-issue interfaces: <e.g., 290 creates ISysInfo, 289 uses it>
  - Risk: <anything non-obvious>

Total estimated scope: ~600 lines across 3 PRs
```

User choices:
- **approve** → proceed to Phase 3
- **modify** → adjust plan (reorder, change approach, drop an issue)
- **abort** → stop

---

## PHASE 3: Per-Issue Execution [SEQUENTIAL LOOP]

For each issue in the approved execution order:

### Step 3.1 — Gather cross-agent context

Read these files if they exist (skip if missing):
- `tasks/active-work.md` — current in-flight work
- `tasks/agent-changelog.md` — recent completed work
- `.claude/shared-context/domain-knowledge.md` — non-obvious pitfalls

### Step 3.2 — Spawn feature agent

Use the Agent tool with the **approved plan for this specific issue** as the primary spec:

```
Agent(
  description: "Implement issue #<N> (wave <W>, <position>/<total>)",
  subagent_type: "<routed-role>",
  isolation: "worktree",
  prompt: <plan from Phase 2 for this issue> + issue body + cross-agent context + pipeline instructions
)
```

Pipeline instructions for the agent:
- Implement the issue following the plan
- The integration branch is `<branch_name>` — your worktree branches from it
- Run `bash deploy/build.sh` and verify zero warnings
- Run tests with `./tests/run_tests.sh` and verify pass rate
- Write `AGENT_REPORT.md` with: Summary, Changes, Design Decisions, Test Coverage, Build Verification, Remaining Work
- If you deviate from the plan, document why
- Do NOT create a PR or push

### Step 3.3 — Integrate and validate

After the agent completes:

1. Read `AGENT_REPORT.md`
2. Merge agent worktree changes into the integration branch
3. **Clean up the agent worktree immediately** — remove the worktree and delete its local branch:
   ```bash
   git worktree remove .claude/worktrees/<agent-id> --force
   git branch -D worktree-<agent-id>
   ```
   Do this right after merging, not at the end of the wave. Stale worktrees pollute the VS Code source control panel and risk accidental commits to dead branches.
4. Run validation (build + format + test count — same as deploy-issue Phase 3)
5. Run hallucination checks (includes, report vs diff, config keys, symbols — same as deploy-issue)

### Step 3.4 — Per-issue checkpoint

```
═══ Issue <position>/<total> — #<N> <title> ═══

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

User choices:
- **accept** → commit, create PR against integration branch, move to next issue
- **changes** → make modifications, re-validate, then re-present
- **skip** → skip this issue, move to next (issue stays open)
- **abort wave** → stop the entire wave

If accepted:
1. Commit: `feat(#<N>): <title>`
2. Push to integration branch
3. Create PR: `gh pr create --title "feat(#<N>): <title>" --base <integration_branch>`
4. Apply labels and milestone from source issue

### Step 3.5 — Loop

Repeat Steps 3.1–3.4 for the next issue. Each subsequent agent's prompt should mention what the prior issues changed (brief summary) so it doesn't duplicate or conflict.

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
- **fix** → fix issues, re-validate, re-review (loop back to Phase 3 for affected issues only)
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

- **Agent conflict**: If agent B touches a file that agent A already modified on the integration branch, the merge in Step 3.3 may conflict. Resolve manually or re-run agent B with awareness of A's changes.
- **Partial wave**: If the user aborts mid-wave, the completed per-issue PRs remain on the integration branch. The wave can be resumed later by re-running with the remaining issues and `--base` pointing to the existing integration branch.
- **Single issue wave**: If only one issue number is provided, the skill degrades gracefully to essentially `/deploy-issue` with an integration branch. Still useful for consistency.
- **Existing integration branch**: Check `git branch -r --list 'origin/integration/epic-<N>*'` before creating. If one exists, offer to reuse it (pick up where a prior wave left off).
- **Context window management**: A full wave with 3 issues + 2-pass review generates significant context. Cap all agent outputs to 200 words. Summarize per-issue results before starting the next issue. If context gets tight, summarize earlier issues aggressively.
- **Empty per-issue PR**: If an issue's files were committed as part of a bulk operation (e.g., user's gitignore changes, re-tracking commits), the per-issue feature branch may show an empty diff against integration. In that case, skip the per-issue PR for that issue — note it in the wave summary and ensure the changes are covered by the merge PR.

If the user provided arguments, use them as context: $ARGUMENTS
