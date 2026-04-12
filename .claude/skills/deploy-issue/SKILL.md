---
name: deploy-issue
description: Run the full deploy-issue pipeline within the current Claude Code session — routing, agent work, review, PR creation, and cleanup with 5 interactive checkpoints
argument-hint: "<issue-number> [--base <branch>]"
---

# /deploy-issue — In-Session Pipeline

Run the full development pipeline for a GitHub issue within the current Claude Code session. You (Claude) act as **tech-lead** throughout — routing, reviewing, synthesizing, and presenting findings at each checkpoint.

**This replaces the `python -m orchestrator deploy-issue` subprocess workflow.** Instead of launching a separate `claude` process, everything happens inline in this conversation.

## Arguments

Parse `$ARGUMENTS` to extract:
- **issue_number** (required): The GitHub issue number (e.g., `299`)
- **--base \<branch\>** (optional): Target branch for the PR (default: `main`). Use for integration branches like `--base integration/epic-284`.

If no arguments provided, ask the user for the issue number.

## Pipeline Phases

Execute these phases sequentially. At each checkpoint (marked with **CP**), present your findings and wait for the user's decision before proceeding.

---

### PHASE 1: Routing & Setup [SEQUENTIAL]

**Step 1.1 — Fetch issue**

```bash
gh issue view <issue_number> --json title,body,labels,state,milestone
```

Display: issue title, labels, state. If the issue is closed, warn the user and ask whether to proceed.

**Step 1.2 — Triage labels → agent role**

Route the issue to an agent role using the label routing from `scripts/orchestrator/config.py` (`LABEL_ROUTING` + `LABEL_DIRECT_ROLES`):

| Priority | Labels (from `LABEL_ROUTING`) | Role |
|----------|-------------------------------|------|
| P3 (highest) | `perception`, `domain:perception` | `feature-perception` |
| P3 | `nav-planning`, `domain:nav` | `feature-nav` |
| P3 | `comms`, `domain:comms` | `feature-integration` |
| P3 | `ipc` | `feature-infra-core` |
| P2 | `integration`, `domain:integration` | `feature-integration` |
| P2 | `common`, `infra`, `infrastructure`, `modularity`, `domain:infra-core` | `feature-infra-core` |
| P1 | `platform`, `deploy`, `ci`, `bsp`, `domain:infra-platform` | `feature-infra-platform` |
| Direct | `safety-audit` | `review-memory-safety` |
| Direct | `security-audit` | `review-security` |
| Direct | `test-coverage` | `test-unit` |
| Direct | `cross-domain` | `tech-lead` |

Also detect: `bug` label → set `is_bug=true` (affects branch prefix: `fix/` vs `feature/`).

Present your routing recommendation:
```
Routing Recommendation:
  Role:     feature-perception
  Priority: high
  Bug:      no
  Reasoning: Issue labels [perception, fusion] map to P3 perception domain
```

Ask the user: **accept / override with a different role**

**Step 1.3 — Branch strategy**

Ask: "Branch to **main** or an **integration branch**?"

If integration: check for existing integration branches with `git branch -r --list 'origin/integration/*'` and let the user pick or create a new one.

**Step 1.4 — Create branch**

```bash
# Fetch latest remote state to avoid branching from stale local refs
git fetch origin

# Branch naming
branch_name="<fix|feature>/issue-<N>-<slug>"  # slug from title, max 40 chars
base_ref="origin/<base_branch>"               # always use origin/ to avoid stale local ref

# Create branch from remote base
git checkout -b "$branch_name" "$base_ref"
```

Note: The feature agent in Phase 2 runs in an **isolated worktree** (via `isolation: "worktree"`) so it cannot interfere with the current checkout. After the agent completes, its changes are merged back in Step 2.5.

---

### PHASE 2: Agent Work [SEQUENTIAL]

**Step 2.1 — Gather cross-agent context**

Read these files if they exist (skip if missing):
- `tasks/active-work.md` — current in-flight work
- `tasks/agent-changelog.md` — recent completed work
- `.claude/shared-context/domain-knowledge.md` — non-obvious pitfalls

**Step 2.2 — Tech-lead planning (MANDATORY)**

Before spawning any feature agent, **you (as tech-lead) must plan the implementation**. Enter plan mode and produce a concrete implementation plan. This is critical — the feature agent receives your plan as its spec, not the raw issue body.

Planning steps:
1. Read the issue body carefully — understand acceptance criteria and constraints
2. Read the relevant source files — identify what needs to change, where, and how
3. Check for related code patterns — how similar features were implemented
4. Read `domain-knowledge.md` for non-obvious pitfalls in the affected area
5. Identify the specific files to create/modify, the approach, and the test strategy
6. Consider edge cases, backwards compatibility, and safety implications

Present the plan to the user:

```
═══ Implementation Plan — Issue #<N> ═══

Approach: <1-2 sentence summary of the implementation strategy>

Files to modify:
  - <file_path> — <what changes and why>
  - <file_path> — <what changes and why>

New files:
  - <file_path> — <purpose>

Test strategy:
  - <what tests to add/modify>

Edge cases / risks:
  - <anything non-obvious>

Estimated scope: <small / medium / large>
```

User choices:
- **approve** → proceed to agent launch (Step 2.3)
- **modify** → user adjusts the plan, then re-present
- **skip agent** → skip the feature agent entirely. The user wants to implement manually. Jump straight to CP1 with an empty diff (user will make changes themselves during the "changes" flow at CP1).

**Step 2.3 — Push branch before spawning agent**

**IMPORTANT:** Before spawning the feature agent, push the current branch so the agent's worktree can access it:

```bash
git push -u origin <branch_name>
```

This is critical because `isolation: "worktree"` creates a worktree from the repo's default branch, not from your local branch. If you've made local commits (e.g., from prior fix rounds), the agent won't have them unless they're on the remote. Tell the agent in its prompt to `git fetch origin && git merge origin/<branch_name> --no-edit` as its first step.

**Step 2.4 — Confirm and spawn feature agent**

Only after the plan is approved. Present the agent launch and ask for final confirmation:

```
═══ Agent Launch — Feature Implementation ═══

Agent:    <routed-role> (e.g., feature-perception)
Model:    Opus
Task:     Implement issue #<N> — <title>
Isolation: worktree (separate git worktree)
Plan:     <approved plan from Step 2.2>
```

User choices:
- **launch** → spawn the agent
- **override \<role\>** → launch a different agent role instead (e.g., user wants `feature-nav` instead of `feature-integration`)

Use the Agent tool:

```
Agent(
  description: "Implement issue #<N>",
  subagent_type: "<routed-role>",  // e.g., "feature-perception"
  isolation: "worktree",
  prompt: <see below>
)
```

The prompt must include:
1. **Your implementation plan from Step 2.2** — this is the primary spec. Be specific: name files, functions, patterns to follow, and the exact approach. The agent should execute your plan, not re-derive the architecture.
2. The full issue body (for context and acceptance criteria)
3. Cross-agent context from Step 2.1
4. These pipeline instructions:
   - **First:** `git fetch origin && git merge origin/<branch_name> --no-edit` to get the latest branch state
   - Implement the issue following the plan above
   - Run `bash deploy/build.sh` and verify zero warnings
   - Run tests with `./tests/run_tests.sh` and verify pass rate
   - Update documentation as required by DEVELOPMENT_WORKFLOW.md
   - When done, write `AGENT_REPORT.md` in the worktree root with: Summary, Changes (table of files), Design Decisions, Test Coverage, Build Verification, Remaining Work
   - Do NOT create a PR or push — leave changes as local commits
   - If you deviate from the plan, document why in AGENT_REPORT.md

Wait for the agent to complete.

**Step 2.4 — Read results**

After the agent completes, read `AGENT_REPORT.md` from the agent's worktree (the path is returned by the Agent tool).

**Step 2.5 — Integrate worktree changes**

The Agent tool with `isolation: "worktree"` returns the worktree path and branch. Merge the agent's changes into the working branch:

```bash
# The agent committed to its worktree branch — merge those commits into our branch
git merge <agent_worktree_branch> --no-edit

# Verify the merge succeeded
git diff --stat origin/<base_branch>...HEAD
```

If the merge has conflicts (unlikely since the worktree branched from our branch), resolve them and commit. Copy `AGENT_REPORT.md` from the worktree to the current directory if not already present after merge.

---

### CP1: Changes Review [INTERACTIVE]

Present to the user:

```
═══ CHECKPOINT 1/5 — Changes Review ═══

[AGENT_REPORT.md content or summary]

--- Diff Summary ---
[git diff --stat output]

--- Tech-Lead Recommendation ---
Based on the agent report and diff, I recommend: [accept/changes]
Reasoning: [your assessment of completeness, quality, test coverage]
[Any concerns or suggestions]
```

User choices:
- **accept** → proceed to Phase 3
- **changes** → ask the user what modifications they want, then either make them yourself or spawn the agent again with specific instructions. Return to CP1 after changes.
- **reject** → go to ABORT

---

### PHASE 3: Validate & Commit

**Step 3.1 — Validation (8-check hallucination detector)** [PARALLEL where possible]

Run these checks. Checks 1-3 are BLOCKING (must pass to commit). Checks 4-8 are WARNING (report but don't block).

**BLOCKING checks:**

1. **Build** — `bash deploy/build.sh` (verify exit code 0, zero warnings)
2. **Format** — Find changed C++ files and check with `clang-format-18 --dry-run --Werror`
3. **Test count** — `ctest -N --test-dir build | grep "Total Tests:"` — compare against baseline in `tests/TESTS.md`

**Hallucination detection checks (WARNING):**

4. **Include verification** — Scan the diff for new `#include "..."` directives. For each, verify the header file exists in the repo (skip system/third-party prefixes: `zenoh/`, `Eigen/`, `opencv2/`, `gz/`, `mavsdk/`, `spdlog/`, `nlohmann/`, `gtest/`, `gmock/`).
```bash
git diff origin/<base_branch>...HEAD | grep '^\+.*#include "' | sed 's/.*#include "//;s/".*//' | sort -u
```
For each include, search the project dirs (`common/`, `process[1-7]_*/`, `tests/`) for the file.

5. **AGENT_REPORT.md vs actual diff** — If AGENT_REPORT.md exists, extract its "Changed files" table and compare against `git diff --name-only origin/<base_branch>...HEAD`. Flag:
   - Files listed in report but NOT in the diff (phantom changes — agent claimed to change a file it didn't)
   - Files in the diff but NOT in the report (undocumented changes — agent forgot to report)

6. **Config key consistency** — Search the diff for new `cfg.get<>()` or `cfg_key::` references. For each new config key, verify it exists in `config/default.json`. Agents commonly add code that reads a config key but forget to add the default value.
```bash
# Find new config key references in the diff
git diff origin/<base_branch>...HEAD | grep '^\+' | grep -oE 'cfg\.get<[^>]*>\("[^"]*"' | sed 's/.*"//;s/".*//' | sort -u

# Cross-reference against default.json
```

7. **Symbol resolution** — For new function calls or class references added in the diff, spot-check that the symbols actually exist. Focus on:
   - New `#include` files → do the functions used from those headers exist?
   - New class instantiations → does the constructor signature match?
   - New method calls on existing classes → does the method exist?

   This is a best-effort heuristic check. Use `grep -r` to verify key symbols. Don't exhaustively check every line — focus on non-trivial additions (new classes, factory calls, HAL interfaces).

8. **Diff sanity** — Check the overall diff for red flags:
   - Files >1000 lines added in a single commit (may indicate generated/copied code)
   - Deleted files that are still `#include`d elsewhere
   - Test files that don't contain any `TEST` or `TEST_F` macros (phantom test files)
   - `.cpp` files added but not included in any `CMakeLists.txt`

9. **Safety guide compliance** — Scan the diff for patterns that violate the safety-critical C++ practices in CLAUDE.md:
   - Raw owning pointers (`new`/`delete`) or `const float*` where `const std::array<float,N>*` or `std::span` is appropriate
   - `memcpy`/`memset`/`memmove` instead of `std::copy`/`std::fill`/value semantics
   - `std::shared_ptr` in safety-critical paths (acceptable for external library contracts)
   - C-style casts, uninitialized variables, `using namespace` in headers
   - Unguarded signed→unsigned casts on durations/sizes/counts
   
   This catches safety guide violations before a full review round. Flag as WARNING.

### CP2: Commit Approval [INTERACTIVE]

```
═══ CHECKPOINT 2/5 — Commit Approval ═══

--- Validation Results ---
[BLOCKING]
Build:          PASS (zero warnings)
Format:         PASS
Test count:     1259 (expected 1259) ✓

[HALLUCINATION CHECKS]
Includes:       PASS (3 new includes, all resolved)
Report vs diff: WARN (report lists config_validator.h but not in diff)
Config keys:    PASS (2 new keys, both in default.json)
Symbol check:   PASS (spot-checked 4 symbols)
Diff sanity:    PASS

[If any BLOCKING checks fail, recommend going back to CP1]
[If any hallucination checks WARN, show details and recommend review]
```

User choices:
- **commit** → stage all changes, create commit: `feat(#<N>): <title>\n\nCo-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>`
- **back** → return to CP1
- **abort** → go to ABORT

---

### PHASE 4: PR Creation [SEQUENTIAL]

**Step 4.1 — Push**

```bash
git push -u origin <branch_name>
```

**Step 4.2 — Generate PR metadata**

Create PR title and body:
- Title: `feat(#<N>): <issue_title>` (or `fix(#<N>):` for bugs)
- Body: Summary from AGENT_REPORT.md, diff stat, test plan checklist, `Closes #<N>`

### CP3: PR Preview [INTERACTIVE]

```
═══ CHECKPOINT 3/5 — PR Preview ═══

Title: feat(#299): add multi-class height priors for depth estimation

Body:
[generated body]
```

User choices:
- **create** → `gh pr create --title "..." --body "..." --base <base_branch>`, then apply labels and milestone from source issue
- **edit** → let user modify title/body, then loop back
- **back** → return to CP2
- **abort** → go to ABORT

After creating: check for existing open PR for same branch first (`gh pr list --head <branch> --state open`). If one exists, offer to update it instead.

---

### PHASE 5: Review [TWO PASSES]

**Step 5.1 — Build review roster and get approval**

Get the PR diff and determine which reviewers are applicable:

**Pass 1 — Safety & Correctness** (always recommended):
- `review-memory-safety` — RAII, ownership, lifetimes
- `review-security` — input validation, auth, TLS
- `test-unit` — GTest coverage, test count verification

**Pass 1 — Conditional** (recommended based on diff content):
- `review-concurrency` — if diff contains `std::atomic`, `std::mutex`, `std::thread`, `lock_guard`, `memory_order`
- `review-fault-recovery` — if diff touches `process4_*`, `process5_*`, `process7_*`, `watchdog`, `fault`, `health`
- `test-scenario` — if diff touches `common/ipc/`, `common/hal/`, `config/scenarios/`, Gazebo configs

**Pass 2 — Quality & Contracts** (always recommended, runs after Pass 1):
- `review-test-quality` — tests exercise new code paths, assertions meaningful
- `review-api-contract` — docstrings match impl, data consistency
- `review-code-quality` — dead code, DRY violations, complexity, naming
- `review-performance` — unnecessary copies, allocation in hot paths, O(n²)

Present the full roster for approval:

```
═══ Review Agent Roster ═══

--- Pass 1: Safety & Correctness ---
  [1] review-memory-safety    RECOMMENDED   RAII, ownership, lifetimes
  [2] review-security         RECOMMENDED   Input validation, auth, TLS
  [3] test-unit               RECOMMENDED   GTest coverage, test count
  [4] review-concurrency      CONDITIONAL   Diff contains std::atomic usage
  [5] review-fault-recovery   NOT TRIGGERED Diff does not touch P4/P5/P7/watchdog

--- Pass 2: Quality & Contracts (runs after Pass 1) ---
  [6] review-test-quality     RECOMMENDED   Tests exercise new paths
  [7] review-api-contract     RECOMMENDED   Docstrings, data consistency
  [8] review-code-quality     RECOMMENDED   Dead code, DRY, complexity
  [9] review-performance      RECOMMENDED   Copies, allocation, hot paths

Launch: [1] [2] [3] [4] [6] [7] [8] [9]  (8 agents, ~5 min)
Skipped: [5] (not triggered)
```

User choices:
- **launch all** → launch all recommended/conditional agents as shown
- **skip \<numbers\>** → skip specific agents (e.g., `skip 4 9` to skip concurrency and performance). The skipped agents will not run and their findings will show as "SKIPPED" in the report.
- **only \<numbers\>** → launch only the specified agents (e.g., `only 1 2 3` for safety-only review)
- **skip all** → skip the entire review phase, go directly to CP5

**Step 5.2 — Launch Pass 1 agents** [PARALLEL]

Spawn all **approved** Pass 1 agents in **parallel** using multiple Agent tool calls in a single message. Each agent receives:
- The PR diff (or summary of changed files with key snippets)
- Instructions to report findings with severity (P1/P2/P3) and file:line references

Collect all Pass 1 results. Report progress as agents complete:
```
  ✓ review-memory-safety    complete (0 P1, 1 P2, 2 P3)
  ✓ review-security         complete (0 P1, 0 P2, 1 P3)
  ⏳ test-unit              running...
```

**Step 5.3 — Launch Pass 2 agents** [PARALLEL, after Pass 1]

Spawn all **approved** Pass 2 agents in parallel, providing Pass 1 findings as context:
- Each Pass 2 agent receives the merged Pass 1 findings so it can build on them
- Skipped Pass 2 agents show as "SKIPPED" in the final report

Collect Pass 2 results.

**Step 5.4 — Merge findings** from both passes into a combined report. Mark skipped agents clearly.

**Step 5.5 — Check for bot/CI review comments**

Before presenting CP4, check for existing review comments from Copilot or other bots on the PR:

```bash
gh api repos/<owner>/<repo>/pulls/<N>/comments --jq '.[].body' | head -50
```

Include any actionable bot comments in the CP4 findings presentation.

**Step 5.6 — Post findings to PR**

**Always** post the merged review findings as a PR comment — do not wait to be asked. Use a structured format with agent status table, P1/P2 details, and tech-lead assessment. This creates an audit trail on the PR.

### CP4: Review Findings [INTERACTIVE]

```
═══ CHECKPOINT 4/5 — Review Findings ═══

--- Review Summary ---
Pass 1 (Safety & Correctness): 3 ran, 1 skipped, 0 P1, 2 P2, 5 P3
Pass 2 (Quality & Contracts):  3 ran, 1 skipped, 0 P1, 1 P2, 3 P3

--- Agent Status ---
  ✓ review-memory-safety    0 P1, 1 P2, 3 P3
  ✓ review-security         0 P1, 0 P2, 1 P3
  ✓ test-unit               0 P1, 1 P2, 1 P3
  ⊘ review-concurrency      SKIPPED (user)
  ✓ review-test-quality     0 P1, 1 P2, 2 P3
  ✓ review-api-contract     0 P1, 0 P2, 1 P3
  ✓ review-code-quality     0 P1, 0 P2, 0 P3
  ⊘ review-performance      SKIPPED (user)

--- P2 Issues ---
1. [review-memory-safety] occupancy_grid_3d.h:142 — unguarded signed→unsigned cast on cell_count
2. [review-test-quality] test_depth.cpp:87 — test doesn't exercise the new multi-class path (detection class is DEFAULT)
3. [review-api-contract] ukf_fusion_engine.h:45 — estimate_depth() docstring says "returns meters" but returns depth_confidence tier

--- P3 Issues ---
[...]

--- Tech-Lead Assessment ---
Recommendation: fix (2 P2 issues are worth addressing before merge)
Note: 2 agents were skipped by user — those review areas are uncovered.
```

User choices:
- **accept** → proceed to CP5 (findings are acceptable as-is)
- **fix** → spawn feature agent with findings, then re-validate, commit, push, and **re-run the FULL Phase 5 review (BOTH Pass 1 AND Pass 2)**. This is mandatory — do NOT skip Pass 2 or go directly to CP5 after fixes. The fix loop always returns to Step 5.1 (roster approval).
- **fix \<numbers\>** → fix only specific issues by number (e.g., `fix 1 3`), accept the rest as-is. Still re-runs full Phase 5.
- **back** → return to CP3
- **reject** → go to ABORT

---

### CP5: Final Summary [INTERACTIVE]

```
═══ CHECKPOINT 5/5 — Final Summary ═══

--- Commit History ---
[git log --oneline output for this branch]

--- PR ---
URL: https://github.com/<owner>/<repo>/pull/<N>
Status: open
Reviews: [summary]

--- Validation ---
Last build: PASS
Last test count: 1259 ✓
```

User choices:
- **done** → proceed to CLEANUP
- **re-review** → re-run Phase 5 review agents
- **back** → return to CP4
- **abort** → go to ABORT

---

### CLEANUP [PARALLEL]

1. Verify PR body contains `Closes #<issue_number>` — if not, update it
2. Append entry to `tasks/agent-changelog.md`:
   ```
   ### <date> | <role> | <model> | PR #<N>
   - **Issue:** #<issue_number> — <title>
   - **Branch:** <branch_name>
   - **Mode:** skill (/deploy-issue)
   - **Status:** completed
   ```
3. Post a summary comment on the GitHub issue noting the PR
4. Report completion to the user:
   ```
   Pipeline complete. PR #<N> ready for merge.
   URL: <pr_url>
   ```

---

### ABORT (from any checkpoint)

If the user chooses to abort at any checkpoint:

1. Ask: **clean up** (delete branch + any worktree) or **preserve** (keep for manual work)?
2. If clean: `git checkout <base_branch>` and `git branch -D <branch_name>`
3. If preserve: show resume instructions
4. Report abort status

---

## Edge Cases

- **Stale artifacts**: Before spawning the feature agent, check for and remove any existing `AGENT_REPORT.md` in the worktree from prior runs. Also note: `AGENT_REPORT.md` from prior issues may exist in the base branch — after merge, verify the report content matches the current issue, not a stale one.
- **Duplicate PRs**: Before creating a PR, check `gh pr list --head <branch> --state open` — if one exists, offer to update instead of creating a duplicate
- **Push failures**: If push fails, go back to CP2 (commit step), not loop within push
- **Review agent timeout**: If an agent takes too long, report which agents completed and which are pending. Present partial results and ask the user how to proceed.
- **Base branch verification**: If `--base` specifies an integration branch, verify it exists with `git branch -r --list 'origin/<base>'`
- **Context window management**: A full pipeline with 2 review rounds generates ~50K+ tokens of agent output. To avoid context exhaustion:
  - Cap all review agent prompts to request results "under 200 words if clean"
  - When reading agent task outputs, extract only the final summary — do not read full JSONL task files
  - Summarize findings concisely at each checkpoint rather than including full agent output verbatim

If the user provided arguments, use them as context: $ARGUMENTS
