<!-- SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary -->
<!-- Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md. -->

---
name: update-docs
description: Auto-generate documentation updates for all tracking files after a PR — PROGRESS.md, ROADMAP.md, BUG_FIXES.md, TESTS.md, API.md, CI_ISSUES.md, design docs, README.md
argument-hint: "[--pr <number>] [--issue <number>] [--dry-run]"
---

# /update-docs — Auto-Generate Documentation Updates

The #1 manual burden in the development workflow: every merge requires updating up to 8+ documentation files. This skill auto-generates all entries from the current branch's changes, PR metadata, and test output.

**Pain point context:** PROGRESS.md and ROADMAP.md are the #1 source of merge conflicts because every branch touches them. This skill minimizes the time spent on documentation and reduces conflict risk by generating correct entries quickly.

## Arguments

Parse `$ARGUMENTS` to extract:
- **--pr \<number\>** (optional): PR number to pull metadata from. If omitted, detect from current branch.
- **--issue \<number\>** (optional): Issue number. If omitted, extract from branch name (`feature/issue-XX-*`) or PR body (`Closes #XX`).
- **--dry-run** (optional): Show proposed changes without applying them.

## Steps

### Step 1: Gather Context [PARALLEL]

Run these in parallel:
```bash
git branch --show-current
git diff --stat main...HEAD
git log --oneline main...HEAD
```

If `--pr` was provided:
```bash
gh pr view <number> --json title,body,labels,baseRefName,additions,deletions,files
```

If `--issue` was provided:
```bash
gh issue view <number> --json title,body,labels
```

Also gather:
```bash
ctest -N --test-dir build 2>/dev/null | grep "Total Tests:"
```

### Step 2: Determine Which Docs Need Updates

Check each file against the changes:

| File | Condition | Source |
|------|-----------|--------|
| `docs/tracking/PROGRESS.md` | Always (every PR adds an improvement) | PR title + body + diff stat |
| `docs/tracking/ROADMAP.md` | If issue is being closed | Issue number → strikethrough + checkmark |
| `docs/tracking/BUG_FIXES.md` | If `bug` label or branch starts with `fix/` | Issue body + diff for root cause |
| `tests/TESTS.md` | If files in `tests/` were added/modified | ctest output + new test file analysis |
| `docs/design/API.md` | If interfaces in `common/hal/include/` or `common/ipc/` changed | Diff of header files |
| `docs/tracking/CI_ISSUES.md` | If `.github/workflows/` or `deploy/` CI scripts changed | Diff of CI files |
| `docs/design/<process>_design.md` | If major architectural changes to a process | Detect from diff scope |
| `README.md` | If new capabilities, metrics, or architecture changes | Detect from significance |

### Step 3: Generate PROGRESS.md Entry

Read the current `docs/tracking/PROGRESS.md` to determine:
- Current Phase number and section
- Next Improvement number (scan for highest `### Improvement #N`)
- Entry format (follow existing pattern exactly)

Generate entry:
```markdown
### Improvement #<N+1> — <PR title>

**Date:** <today's date YYYY-MM-DD>
**Category:** <detect from labels: Feature/Architecture/Testing/Infrastructure/Bug Fix/Performance>
**Files Added:**
<list new files from git diff --stat>

**Files Modified:**
<list modified files from git diff --stat>

**What:** <2-3 sentence summary from PR body or commit messages>

**Why:** <motivation from issue body if available>

**Test additions:** <list new test files or "No new tests" if none>

---
```

### Step 4: Generate ROADMAP.md Update

If an issue is being closed, find it in `docs/tracking/ROADMAP.md` and:
1. Add strikethrough to the issue line: `~~#XX — description~~`
2. Add checkmark: `✅`
3. Update the "Current State at a Glance" metrics table if any metrics changed:
   - Test count (from ctest output)
   - HAL backends count (if new HAL added)
   - Line coverage (if coverage build was run)
   - Any other metrics that changed

Read the current file first to find the exact line to modify.

### Step 5: Generate BUG_FIXES.md Entry

Only if this is a bug fix (branch starts with `fix/` or issue has `bug` label).

Read `docs/tracking/BUG_FIXES.md` to determine:
- Next Fix number (scan for highest `### Fix #N`)
- Correct section (IPC, Perception, Mission Planner, etc. — determine from files changed)

Generate entry following the exact format:
```markdown
### Fix #<N+1> — <title> (#<issue-number>)

**Date:** <today's date>
**Severity:** <High/Medium/Low — from issue labels or estimate>
**File:** <primary files changed>

**Bug:** <description from issue body>

**Root Cause:** <extract from PR body or commit messages — the WHY>

**Fix:** <what was changed — from commit messages>

**Found by:** <from issue: "Code review", "Scenario test", "CI", "User report", "AI review">

**Regression test:** <name of test added, or "N/A">

---
```

### Step 6: Generate TESTS.md Update

If test files were added or modified:

1. Run `ctest -N --test-dir build 2>/dev/null | grep "Total Tests:"` to get current count
2. Read `tests/TESTS.md` and find the Summary table
3. Update the `**Total**` row with new counts
4. If new test files were added, add entries to the appropriate section following the existing format:
   - Find the right category section
   - Add a new subsection with test file name, test count, and description
   - Grep the test file for `TEST(` and `TEST_F(` to count and list tests

### Step 7: Generate API.md Update

Only if interfaces changed (files in `common/hal/include/hal/` or `common/ipc/include/ipc/`).

Read `docs/design/API.md` and update the relevant tables:
- New IPC message types → add to message type table
- New HAL interfaces → add to interface table
- Modified struct fields → update field lists

### Step 8: Generate CI_ISSUES.md Entry

Only if CI-related files changed (`.github/workflows/`, `deploy/build.sh`, `deploy/run_ci_local.sh`).

Add entry to `docs/tracking/CI_ISSUES.md` following existing format.

### Step 9: Detect Design Doc Updates

Analyze the diff to determine if any **architectural changes** require design document updates. A change is architectural if it:
- Adds a new class, interface, or module
- Changes the data flow between processes (IPC topics, message types)
- Modifies thread structure or synchronization patterns
- Adds or changes a HAL backend
- Changes the config schema significantly (new section, new pattern)
- Modifies the pipeline (detection → tracking → fusion → planning)

Map changed files to design docs:

| Files touched | Design doc |
|--------------|------------|
| `process1_video_capture/` | `docs/design/video_capture_design.md` |
| `process2_perception/` | `docs/design/perception_design.md` |
| `process3_slam_vio_nav/` | `docs/design/slam_vio_nav_design.md` |
| `process4_mission_planner/` | `docs/design/mission_planner_design.md` |
| `process5_comms/` | `docs/design/comms_design.md` |
| `process6_payload_manager/` | `docs/design/payload_manager_design.md` |
| `process7_system_monitor/` | `docs/design/system_monitor_design.md` |
| `common/hal/` | `docs/design/hal_design.md` |
| `common/ipc/` | `docs/design/ipc_design.md` |
| `common/util/` (Result, error handling) | `docs/design/error_handling_design.md` |
| Watchdog, systemd, process management | `docs/design/hardening-design.md` |

For each matched design doc:
1. Read the current doc
2. Identify which sections are affected by the changes
3. Generate a proposed update that adds or modifies the relevant sections
4. If the change is minor (e.g., bug fix within existing architecture), note it but don't propose changes

Present as: "Design doc `<name>` may need updates — [specific sections affected]"

### Step 10: Detect README.md Updates

Check if the changes warrant a `README.md` update. Update is needed if:

- **New capability** — a major new feature visible to users (new process, new HAL backend, new tool)
- **Metrics changed** — test count, HAL backend count, scenario count, coverage, etc.
- **Architecture changed** — new process, new IPC channel, changed dependencies
- **New documentation** — new guide or design doc that should be in the Documentation Index
- **Quick start changed** — build commands, dependencies, or setup steps changed

If needed, propose specific edits to `README.md`:
- Update the metrics in any summary tables
- Add new entries to the Documentation Index table
- Update architecture diagrams (process map, IPC channels) if changed
- Update the Quick Start section if setup steps changed

### Step 11: Present All Proposed Changes

Show a summary of all generated entries:

```
═══ Documentation Updates ═══

--- Tracking Files ---
✓ PROGRESS.md: Improvement #<N> entry (always)
✓ ROADMAP.md: Issue #<X> marked complete + metrics updated
✗ BUG_FIXES.md: Not applicable (not a bug fix)
✓ TESTS.md: Total updated 1259 → 1272, 1 new suite added

--- Design Files ---
✓ API.md: New IPC message type added
⚠ perception_design.md: May need update (new detector backend added)
✗ Other design docs: No changes needed

--- Top-Level ---
✓ README.md: Test count metric, new design doc in index

Files to update: 5 of 8+
```

If `--dry-run`, stop here and show the proposals.

### Step 12: Apply Changes

Ask the user: **apply all / select which / cancel**

If applying:
- Use Edit tool to insert each entry at the correct location in each file
- Verify edits are clean (no formatting issues)
- Show `git diff --stat` after all edits

**Important formatting rules:**
- Match the exact indentation and style of existing entries (read surrounding entries first)
- Use the same date format as existing entries (YYYY-MM-DD)
- Preserve section ordering in ROADMAP.md (don't move items between sections)
- In TESTS.md Summary table, ensure column alignment is preserved
- In README.md, match the existing table formatting and style

If the user provided arguments, use them as context: $ARGUMENTS
