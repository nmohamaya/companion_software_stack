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

Route the issue to an agent role using this label priority system:

| Priority | Labels | Role |
|----------|--------|------|
| P3 (highest) | `perception`, `detection`, `tracking`, `fusion` | `feature-perception` |
| P3 | `nav-planning`, `mission`, `slam`, `vio`, `path-planning` | `feature-nav` |
| P3 | `comms`, `fc-link`, `gcs`, `gimbal`, `payload`, `monitor` | `feature-integration` |
| P3 | `ipc`, `config`, `util`, `common`, `modularity` | `feature-infra-core` |
| P2 | `infrastructure`, `ci`, `deploy`, `systemd`, `cross-compile` | `feature-infra-platform` |
| Direct | `audit`, `security-audit` | `review-security` |
| Direct | `test`, `test-coverage` | `test-unit` |

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

**Step 1.4 — Create branch and worktree**

```bash
# Branch naming
branch_name="<fix|feature>/issue-<N>-<slug>"  # slug from title, max 40 chars

# Create branch from base
git checkout -b "$branch_name" "<base_branch>"
```

Note: we work directly on the branch rather than in a separate worktree since we're in the current session.

---

### PHASE 2: Agent Work [SEQUENTIAL]

**Step 2.1 — Gather cross-agent context**

Read these files if they exist (skip if missing):
- `tasks/active-work.md` — current in-flight work
- `tasks/agent-changelog.md` — recent completed work
- `.claude/shared-context/domain-knowledge.md` — non-obvious pitfalls

**Step 2.2 — Spawn feature agent**

Use the Agent tool to spawn the routed agent:

```
Agent(
  description: "Implement issue #<N>",
  subagent_type: "<routed-role>",  // e.g., "feature-perception"
  isolation: "worktree",
  prompt: <see below>
)
```

The prompt must include:
1. The full issue body
2. Cross-agent context from Step 2.1
3. These pipeline instructions:
   - Implement the issue fully (code, tests, build verification)
   - Run `bash deploy/build.sh` and verify zero warnings
   - Run tests with `./tests/run_tests.sh` and verify pass rate
   - Update documentation as required by DEVELOPMENT_WORKFLOW.md
   - When done, write `AGENT_REPORT.md` in the worktree root with: Summary, Changes (table of files), Design Decisions, Test Coverage, Build Verification, Remaining Work
   - Do NOT create a PR or push — leave changes as local commits

Wait for the agent to complete.

**Step 2.3 — Read results**

After the agent completes, read `AGENT_REPORT.md` from the agent's worktree (the path is returned by the Agent tool). Also run:
```bash
git diff --stat <base_branch>...HEAD
```

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

**Step 3.1 — Validation** [PARALLEL where possible]

Run these checks (use parallel Bash calls where possible):
1. Build: `bash deploy/build.sh` (verify exit code 0, zero warnings)
2. Format: Find changed C++ files and check with `clang-format-18 --dry-run --Werror`
3. Test count: `ctest -N --test-dir build | grep "Total Tests:"` — compare against baseline in `tests/TESTS.md`

### CP2: Commit Approval [INTERACTIVE]

```
═══ CHECKPOINT 2/5 — Commit Approval ═══

--- Validation Results ---
Build:      PASS (zero warnings)
Format:     PASS
Test count: 1259 (expected 1259) ✓

[If any failures, show details and warn]
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

**Pass 1 — Safety & Correctness** [PARALLEL]

Get the PR diff and determine which reviewers to launch:

Always launch:
- `review-memory-safety`
- `review-security`
- `test-unit`

Conditionally launch (check diff content):
- `review-concurrency` — if diff contains `std::atomic`, `std::mutex`, `std::thread`, `lock_guard`, `memory_order`
- `review-fault-recovery` — if diff touches `process4_*`, `process5_*`, `process7_*`, `watchdog`, `fault`, `health`
- `test-scenario` — if diff touches `common/ipc/`, `common/hal/`, `config/scenarios/`, Gazebo configs

Spawn all Pass 1 agents in **parallel** using multiple Agent tool calls in a single message. Each agent receives:
- The PR diff (or summary of changed files with key snippets)
- Instructions to report findings with severity (P1/P2/P3) and file:line references

Collect all Pass 1 results.

**Pass 2 — Quality & Contracts** [PARALLEL, after Pass 1]

Spawn these agents in parallel, providing Pass 1 findings as context:
- `review-test-quality` — validates tests exercise new code paths, assertions are meaningful
- `review-api-contract` — validates docstrings, data consistency, code completeness
- `review-code-quality` — catches dead code, DRY violations, complexity, naming issues
- `review-performance` — catches unnecessary copies, allocation in hot paths, algorithmic complexity

Collect Pass 2 results.

**Merge findings** from both passes into a combined report.

### CP4: Review Findings [INTERACTIVE]

```
═══ CHECKPOINT 4/5 — Review Findings ═══

--- Review Summary ---
Pass 1 (Safety & Correctness): 3 agents, 0 P1, 2 P2, 5 P3
Pass 2 (Quality & Contracts):  4 agents, 0 P1, 1 P2, 3 P3

--- P2 Issues ---
1. [review-memory-safety] occupancy_grid_3d.h:142 — unguarded signed→unsigned cast on cell_count
2. [review-test-quality] test_depth.cpp:87 — test doesn't exercise the new multi-class path (detection class is DEFAULT)
3. [review-api-contract] ukf_fusion_engine.h:45 — estimate_depth() docstring says "returns meters" but returns depth_confidence tier

--- P3 Issues ---
[...]

--- Tech-Lead Assessment ---
Recommendation: fix (2 P2 issues are worth addressing before merge)
```

User choices:
- **accept** → proceed to CP5 (findings are acceptable as-is)
- **fix** → spawn feature agent with findings, then re-validate, commit, push, and re-run reviews (FIX LOOP → back to Phase 5)
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

- **Stale artifacts**: Before spawning the feature agent, check for and remove any existing `AGENT_REPORT.md` in the worktree from prior runs
- **Duplicate PRs**: Before creating a PR, check `gh pr list --head <branch> --state open` — if one exists, offer to update instead of creating a duplicate
- **Push failures**: If push fails, go back to CP2 (commit step), not loop within push
- **Review agent timeout**: If an agent takes too long, report which agents completed and which are pending. Present partial results and ask the user how to proceed.
- **Base branch verification**: If `--base` specifies an integration branch, verify it exists with `git branch -r --list 'origin/<base>'`

If the user provided arguments, use them as context: $ARGUMENTS
