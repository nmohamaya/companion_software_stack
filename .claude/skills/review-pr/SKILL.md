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

Spawn Pass 1 agents in **parallel** using multiple Agent tool calls in a single message. Each agent receives:
- The PR diff (or summary of changed files with key code snippets for large diffs)
- The PR description for context
- Instructions to report findings with severity (P1/P2/P3) and file:line references

Wait for all Pass 1 agents to complete. Collect findings.

### Step 4: Pass 2 — Quality & Contracts [PARALLEL, after Pass 1]

Skip if `--pass1-only` was specified.

Spawn Pass 2 agents in **parallel**, providing Pass 1 findings as context:
- `review-test-quality` — validates tests exercise new code paths, assertions are meaningful
- `review-api-contract` — validates docstrings match implementation, data consistency
- `review-code-quality` — catches dead code, DRY violations, complexity, naming issues
- `review-performance` — catches unnecessary copies, allocation in hot paths, O(n²)

Wait for all Pass 2 agents to complete. Collect findings.

### Step 5: Synthesize Findings

Merge findings from both passes. As tech-lead, synthesize:

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

If the user provided arguments, use them as context: $ARGUMENTS
