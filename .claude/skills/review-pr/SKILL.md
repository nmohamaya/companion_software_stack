<!-- SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary -->
<!-- Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md. -->

---
name: review-pr
description: Run a two-pass domain-aware code review on a pull request — spawns up to 10 review agents with severity-tagged findings
argument-hint: "<pr-number> [--focus <domain>] [--pass1-only]"
---

# /review-pr — Domain-Aware Two-Pass Code Review

Run the full two-pass review pipeline on a pull request, with Claude acting as tech-lead to synthesize findings.

## Arguments

Parse `$ARGUMENTS` to extract:
- **pr_number** (required): The PR number (e.g., `396`)
- **--focus \<domain\>** (optional): Limit review focus (e.g., `security`, `performance`, `memory-safety`)
- **--pass1-only** (optional): Skip Pass 2 agents

If no arguments provided, ask the user for the PR number.

---

## Steps

### Step 1: Fetch PR Context [PARALLEL]

Run these in parallel:
```bash
gh pr view <pr_number> --json title,body,labels,state,baseRefName,headRefName,files,additions,deletions
gh pr diff <pr_number>
gh pr checks <pr_number>
gh pr view <pr_number> --comments
```

Display: PR title, state, base branch, files changed count, CI status.

If the PR is merged or closed, warn the user and ask whether to proceed.

### Step 1.5: Checkout PR Branch in Worktree [SEQUENTIAL]

Review agents need to read the actual PR branch files, not the current working directory (which may be on `main` or a different branch). Create a temporary git worktree for the PR branch:

```bash
# Fetch the PR branch
git fetch origin <headRefName>

# Create a worktree INSIDE the project directory (not /tmp)
# Agents have restricted file access — paths outside the project tree are denied.
PR_WORKTREE="<project_root>/.pr-review-<pr_number>"
git worktree remove "$PR_WORKTREE" --force 2>/dev/null || true
git worktree add "$PR_WORKTREE" "origin/<headRefName>" --detach

# Verify the worktree has the PR files
ls "$PR_WORKTREE"
```

Store the `PR_WORKTREE` path — all agent prompts will reference it.

**IMPORTANT:** The worktree MUST be inside the project directory (e.g., `.pr-review-472/`), not under `/tmp/`. Review agents have sandboxed file access restricted to the project tree — they cannot read files under `/tmp/` or other external paths. The `.pr-review-*` prefix keeps it hidden and gitignored.

**Why worktree instead of passing the diff?** Passing the full diff inflates agent prompts by thousands of tokens, exhausting context on large PRs. A worktree gives agents direct file access via `Read` with no Bash requirement — they just read from `$PR_WORKTREE/path/to/file` instead of the working directory path. This is cheaper, more accurate (agents can navigate includes and cross-reference), and works for any PR size.

### Step 2: Analyze Diff for Review Routing

Examine the diff to determine which Pass 1 agents to launch:

**Always launch:**
- `review-memory-safety`
- `review-security`
- `test-unit`

**Conditionally launch (check diff content):**
- `review-concurrency` — if diff contains `std::atomic`, `std::mutex`, `std::thread`, `lock_guard`, `condition_variable`, `memory_order`
- `review-fault-recovery` — if diff touches `process4_*`, `process5_*`, `process7_*`, `watchdog`, `fault`, `recovery`
- `test-scenario` — if diff touches `common/ipc/`, `common/hal/`, `config/scenarios/`, Gazebo configs

If `--focus` was specified, prioritize that domain's agent and reduce others to a quick scan.

Present the routing plan:
```
Review Routing:
  Pass 1 (safety & correctness):
    - review-memory-safety (always)
    - review-security (always)
    - test-unit (always)
    - review-concurrency (triggered: diff contains "std::atomic")
  Pass 2 (quality & contracts):
    - review-test-quality (always)
    - review-api-contract (always)
    - review-code-quality (always)
    - review-performance (always)
  Total: 7 agents across 2 passes
```

### Step 3: Pass 1 — Safety & Correctness [PARALLEL]

Spawn Pass 1 agents in **parallel** using `subagent_type` to reference each custom agent role. This automatically loads the role's system prompt, model tier, tools, and review checklist from `.claude/agents/<role>.md`. Do NOT manually reconstruct role context in the prompt.

```
// All in a single message — they run in parallel
Agent(
  description: "Review PR #<N> — memory safety",
  subagent_type: "review-memory-safety",   // loads .claude/agents/review-memory-safety.md
  prompt: "Review PR #<N> for memory safety issues.

IMPORTANT: The PR branch files are checked out at: <PR_WORKTREE>
Read files from that path (e.g., <PR_WORKTREE>/common/hal/include/hal/foo.h), NOT from the main
working directory. The working directory may be on a different branch.

Key files changed in this PR:
<list of changed files from Step 1>

<Brief summary of what the PR does — 2-3 sentences from PR body>

Report findings with severity (P1/P2/P3) and file:line references. Cap output to 200 words if clean."
)
Agent(
  description: "Review PR #<N> — security",
  subagent_type: "review-security",
  prompt: "... (same PR_WORKTREE instructions) ..."
)
Agent(
  description: "Review PR #<N> — unit tests",
  subagent_type: "test-unit",
  prompt: "... (same PR_WORKTREE instructions) ..."
)
// Add conditional agents (review-concurrency, review-fault-recovery, test-scenario) if triggered
```

**Critical:** Every agent prompt MUST include the `PR_WORKTREE` path and explicit instructions to read files from there. Agents that read from the working directory will review stale code from whatever branch is currently checked out.

Wait for all Pass 1 agents to complete. Collect findings.

### Step 4: Pass 2 — Quality & Contracts [PARALLEL, after Pass 1]

Skip if `--pass1-only` was specified.

Spawn Pass 2 agents in **parallel** using `subagent_type`. Provide Pass 1 findings as context AND the `PR_WORKTREE` path:

```
Agent(
  description: "Review PR #<N> — test quality",
  subagent_type: "review-test-quality",    // loads .claude/agents/review-test-quality.md
  prompt: "Review test quality for PR #<N>.

IMPORTANT: The PR branch files are checked out at: <PR_WORKTREE>
Read files from that path, NOT from the main working directory.

Key files changed: <list>

Pass 1 findings (for context):
<merged Pass 1 results — summarized to ~200 words max>

Cap output to 200 words if clean."
)
Agent(
  description: "Review PR #<N> — API contracts",
  subagent_type: "review-api-contract",
  prompt: "... (same PR_WORKTREE instructions + Pass 1 context) ..."
)
Agent(
  description: "Review PR #<N> — code quality",
  subagent_type: "review-code-quality",
  prompt: "... (same PR_WORKTREE instructions + Pass 1 context) ..."
)
Agent(
  description: "Review PR #<N> — performance",
  subagent_type: "review-performance",
  prompt: "... (same PR_WORKTREE instructions + Pass 1 context) ..."
)
```

**Critical:** Same as Pass 1 — every agent prompt must include the `PR_WORKTREE` path.

Wait for all Pass 2 agents to complete. Collect findings.

### Step 4.5: Check Copilot/Bot Review Comments

Before synthesizing, check for existing review comments from Copilot or other bots:

```bash
gh api repos/{owner}/{repo}/pulls/<pr_number>/comments --jq '.[] | select(.user.type == "Bot") | {user: .user.login, body: .body, path: .path, line: .line}' | head -50
gh api repos/{owner}/{repo}/pulls/<pr_number>/reviews --jq '.[] | select(.user.type == "Bot") | {user: .user.login, state: .state, body: .body}'
```

Copilot comments overlap ~60% with our agents. Focus on the unique edge cases they catch that our agents might miss. Include any non-duplicate Copilot findings in the synthesis with `[Copilot]` attribution.

### Step 5: Synthesize Findings

Merge findings from both passes AND Copilot/bot comments. As tech-lead, synthesize:

1. **Deduplicate** — multiple agents may flag the same issue
2. **Prioritize** — rank by severity (P1 > P2 > P3)
3. **Cross-reference** — connect related findings (e.g., memory-safety flagged a raw pointer AND performance flagged unnecessary copy at the same location)
4. **Validate** — check if any P1 findings are false positives based on your understanding of the code

Present structured report:

```
═══ PR Review: #<N> — <title> ═══

--- Review Summary ---
Pass 1 (Safety & Correctness): N agents, X P1, Y P2, Z P3
Pass 2 (Quality & Contracts):  N agents, X P1, Y P2, Z P3

--- P1 Issues (blocks merge) ---
1. [review-memory-safety] file.h:42 — description
   Fix: specific suggestion

--- P2 Issues (should fix) ---
1. [review-concurrency] file.cpp:100 — description
   Fix: specific suggestion

--- P3 Issues (nice to have) ---
[...]

--- Tech-Lead Assessment ---
Recommendation: [approve / request-changes / needs-discussion]
Reasoning: [assessment]

--- PR Template Checklist Verification ---
✓ No raw new/delete (memory-safety verified)
✗ Missing [[nodiscard]] on new Result<> returns (api-contract found)
✓ const correctness (code-quality verified)
[...]
```

### Step 6: Interactive Dialog

User choices:
- **approve** — post summary as PR comment via `gh pr review --approve`
- **request-changes** — post findings as PR comment via `gh pr review --request-changes`
- **comment** — post findings as informational comment (no review decision)
- **discuss** — enter interactive discussion about specific findings
- **post** — post the review as a PR comment without a review decision
- **skip** — don't post anything, just show locally

If posting, format the comment with:
- Severity-tagged findings with file:line references
- Collapsible sections for P3 issues (use `<details>` tags)
- Agent attribution for each finding

### Step 7: Cleanup

Remove the PR branch worktree created in Step 1.5:

```bash
git worktree remove "$PR_WORKTREE" --force 2>/dev/null || true
```

Do this after posting (or after the user chooses "skip"). Don't leave stale worktrees.

**IMPORTANT:** The worktree MUST always be inside the project workspace directory (e.g., `<project_root>/.pr-review-<N>`). Never use `/tmp/`, `~/.claude/`, or any path outside the project root. Agents are sandboxed to the project directory and cannot read external paths.

---

If the user provided arguments, use them as context: $ARGUMENTS
