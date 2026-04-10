---
name: create-issue
description: Create a structured GitHub issue with AI-powered domain detection, auto-labeling, and code context — supports bug, feature, refactor, performance, audit types
argument-hint: "<type>: <title> [--milestone <name>] [--epic <number>]"
---

# /create-issue — Structured Issue Filing with Code Context

Create a well-structured GitHub issue with automatic domain detection, label assignment, and relevant code context. This skill analyzes the codebase to enrich issue descriptions beyond what a human would typically include.

## Arguments

Parse `$ARGUMENTS` to extract:
- **type prefix** (required): One of `bug:`, `feat:`, `refactor:`, `perf:`, `audit:`, `test:`
- **title** (required): The issue title after the type prefix
- **--milestone \<name\>** (optional): Assign to a milestone
- **--epic \<number\>** (optional): Link as sub-issue of an epic

If no arguments provided, ask the user for the type and title.

Example invocations:
```
/create-issue bug: snap offset exceeds acceptance_radius during waypoint transitions
/create-issue feat: add multi-class height priors for depth estimation
/create-issue refactor: extract OccupancyGrid3D from mission planner main
/create-issue perf: reduce allocation in per-frame detection loop
/create-issue audit: review thread safety of UKF fusion engine
/create-issue test: add boundary tests for D* Lite planner edge cases
```

## Steps

### Step 1: Parse Type and Detect Domain

Map the type prefix to GitHub template and labels:

| Prefix | Template | Labels |
|--------|----------|--------|
| `bug:` | Bug Report | `bug`, `triage` |
| `feat:` | Feature Request | `enhancement`, `triage` |
| `refactor:` | Refactor | `refactor`, `triage` |
| `perf:` | Performance | `performance`, `triage` |
| `audit:` | Safety/Security Audit | `safety-audit` or `security-audit` |
| `test:` | Test Coverage | `test-coverage`, `triage` |

### Step 2: Detect Domain from Title and Context [PARALLEL]

Analyze the title to detect the affected domain:

**Keyword-based routing** (from the label routing system in `scripts/orchestrator/config.py`):

| Keywords in title | Domain label | Agent |
|-------------------|-------------|-------|
| perception, detection, tracking, fusion, camera, detector, YOLOv8, ByteTrack | `domain:perception` | feature-perception |
| nav, mission, planner, slam, vio, waypoint, obstacle, avoidance, geofence | `domain:nav` | feature-nav |
| comms, fc_link, gcs, gimbal, payload, monitor, mavlink | `domain:comms` | feature-integration |
| ipc, zenoh, message_bus, wire_format, pub/sub | `domain:infra-core` | feature-infra-core |
| config, util, result, error_handling, common | `domain:infra-core` | feature-infra-core |
| deploy, ci, systemd, cross-compile, board, customer | `domain:infra-platform` | feature-infra-platform |

Also search the codebase for relevant files:
```bash
# Search for symbols mentioned in the title
```
Use Grep to find files related to the title keywords. This helps:
1. Identify the exact domain
2. Find relevant code to reference in the issue body
3. Detect related existing issues

### Step 3: Search for Related Issues

Check for duplicates or related work:
```bash
gh issue list --search "<key terms from title>" --limit 10 --json number,title,state,labels
```

If potential duplicates found, show them and ask the user to confirm this is a new issue.

### Step 4: Gather Code Context

Based on the detected domain and title, read relevant files to enrich the issue:

**For bugs:**
- Find the function/class mentioned in the title
- Read the relevant code to understand current behavior
- Check for existing tests that should have caught this
- Look for related comments or TODOs

**For features:**
- Find the module where the feature would be added
- Identify affected interfaces (HAL, IPC types)
- Check if config keys would be needed
- Identify which tests would need to be added

**For refactors:**
- Find the code being refactored
- Measure current complexity (LOC, nesting depth)
- Identify callers/dependents that would be affected

**For performance:**
- Find the hot path or code section
- Check if it's in a per-frame/per-message path
- Identify allocation or copy patterns

### Step 5: Generate Issue Body

Generate a structured issue body based on the type:

**For bugs:**
```markdown
## Bug Report

### Domain
<detected domain>

### Severity
<estimate from description: Critical/High/Medium/Low>

### Description
**What happens:** <from user's title + code analysis>
**Expected behavior:** <inferred from code intent>

### Steps to Reproduce
1. <steps if determinable from code>

### Relevant Code
- `<file>:<line>` — <description of the relevant function/section>

### Root Cause Analysis
<initial analysis based on code reading — where the bug likely originates>

### Suggested Fix
<approach based on code understanding>

### Acceptance Criteria
- [ ] Bug is fixed
- [ ] Regression test added
- [ ] No new warnings
```

**For features:**
```markdown
## Feature Request

### Domain
<detected domain>

### Description
**Problem:** <what limitation this addresses>
**Proposed solution:** <from user's title + code analysis>

### Affected Files
| File | Change needed |
|------|--------------|
| <file> | <what changes> |

### Design Considerations
- **Interfaces affected:** <HAL interfaces, IPC types that would change>
- **Config keys needed:** <new config parameters>
- **Thread safety:** <if the feature touches shared state>
- **IPC impact:** <if new messages or topics needed>

### Implementation Approach
<multi-step plan based on code understanding>

### Acceptance Criteria
- [ ] Feature implemented
- [ ] Unit tests added (target: >80% coverage of new code)
- [ ] Config keys documented
- [ ] Design doc updated (if architectural change)
- [ ] No new warnings
```

**For refactors:**
```markdown
## Refactor

### Domain
<detected domain>

### Description
**Current state:** <what the code looks like now, with metrics>
**Target state:** <what it should look like after>

### Motivation
<why this refactor matters — complexity, maintainability, modularity>

### Affected Files
| File | Current | Target |
|------|---------|--------|
| <file> | <current state> | <target state> |

### Callers/Dependents
<list of files that use the code being refactored>

### Acceptance Criteria
- [ ] Refactor complete
- [ ] No behavior change (existing tests still pass)
- [ ] No new warnings
- [ ] Code complexity reduced (measurable)
```

### Step 6: Estimate Size

Based on the number of files affected and scope of changes:
- **Small** (<200 lines, single PR): Single-file changes, minor additions
- **Medium** (200-600 lines, 1-2 PRs): Multi-file changes, new test suite
- **Large** (600+ lines, multi-phase): New module, cross-cutting refactor, epic

If Large, suggest splitting into sub-issues with a phased approach.

### Step 7: Preview and Create

Present the full issue preview:

```
═══ Issue Preview ═══

Title: <type>(#domain): <title>
Labels: [<labels>]
Milestone: <milestone if provided>

Body:
<generated body>

--- Related Issues ---
#XX — <similar issue title> (open/closed)

--- Suggested Labels ---
<auto-detected labels from domain + type>
```

User choices:
- **create** — create the issue via `gh issue create`
- **edit** — modify title, body, or labels before creating
- **cancel** — don't create

If creating:
```bash
gh issue create --title "<title>" --body "<body>" --label "<labels>"
```

If `--milestone` was provided:
```bash
gh issue edit <number> --milestone "<milestone>"
```

If `--epic` was provided, add a comment on the epic issue linking this as a sub-issue.

### Step 8: Post-Creation

After creating the issue:
1. Show the issue URL
2. If the issue is a bug, suggest: "Create a branch with `/deploy-issue <number>`?"
3. If the issue is part of an epic, update the epic's task list

If the user provided arguments, use them as context: $ARGUMENTS
