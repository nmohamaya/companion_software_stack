# Agent Changelog

Agents append completed work here. Newest entries at top. Keep 30 days.

## Entry Format

<!--
### YYYY-MM-DD | role | model | PR #XXX
**Task:** description
**Files:** key files changed
**Tests:** X added, Y total passing (baseline: see tests/TESTS.md)
**Duration:** ~Xh
**Cost:** $X.XX (if available)
**Safety issues found:** list or "none"
**Hallucination flags:** PASS/WARN/FAIL from validate-session.sh
**Notes:** anything notable
-->

## Log

### 2026-04-12 | tech-lead (deploy-wave) | opus | PR #404

**Task:** Epic #284 Wave 3 — Composition & Testing (#293 EventBus, #291 ProcessBuilder, #292 Integration Harness)
**Files:** common/util/include/util/event_bus.h, common/util/include/util/process_context.h, common/util/include/util/signal_handler.h, common/util/include/util/isys_info.h, common/util/include/util/jetson_sys_info.h, common/util/include/util/linux_sys_info.h, common/util/include/util/mock_sys_info.h, common/util/include/util/sys_info_factory.h, process[1-7]_*/src/main.cpp, tests/test_event_bus.cpp, tests/test_process_context.cpp, tests/test_process_interfaces.cpp, tests/test_system_monitor.cpp, tests/integration/integration_harness.h
**Tests:** 33 added, 1429 total passing (baseline: 1396)
**Safety issues found:** ARM64 memory ordering (relaxed→seq_cst/acquire on g_running), EventBus lifetime contract undocumented, fork-after-Zenoh risk in P7 (deferred)
**Hallucination flags:** PASS
**Notes:** Two-pass review × 2 rounds (9 agents each), 22 findings fixed. Per-issue PRs: #402 (#293), #403 (#291). #292 changes included in wave merge commit. Follow-ups: fork-after-Zenoh, config path sanitization, ThreadWatchdog DRY.

### 2026-04-06 | feature-infra-platform | opus | PR #359

**Task:** Issue #358 — Enable multi-developer GitHub collaboration
**Files:** .gitignore, CLAUDE.md, tasks/active-work.md, tasks/agent-changelog.md, tasks/sessions/.gitignore, scripts/deploy-issue.sh, scripts/validate-session.sh
**Tests:** 0 added, 1286 total passing (baseline: see tests/TESTS.md)
**Safety issues found:** none
**Hallucination flags:** PASS (3 pass, 1 warn — `lessons.md` reference in PR body)
**Notes:** First pipeline test. Fixed `deploy-issue.sh` label routing (infrastructure, domain:*), fixed `validate-session.sh` false positive on "0 tests failed". Created 13 GitHub labels (6 domain + 3 agent + 4 audit).

### 2026-04-07 | feature-infra-platform | opus | PR #374
- **Issue:** #371 — feat: remote pipeline monitoring — tmux sessions + ntfy.sh mobile notifications
- **Branch:** feature/issue-371-feat-remote-pipeline-monitoring--tmux-se
- **Mode:** pipeline
- **Fix iterations:** 0
- **Status:** completed

### 2026-04-07 | feature-infra-platform | opus | PR #375
- **Issue:** #299 — feat: Add --json-logs flag to scenario runners and launch_gazebo.sh
- **Branch:** feature/issue-299-feat-add---json-logs-flag-to-scenario-ru
- **Mode:** pipeline
- **Fix iterations:** 0
- **Status:** completed

### 2026-04-08 | feature-infra-core | opus | PR #380
- **Issue:** #285 — feat(#284-A1): ILogger interface + DRONE_LOG macros
- **Branch:** feature/issue-285-feat284-a1-ilogger-interface--dronelog-m
- **Mode:** pipeline
- **Fix iterations:** 0
- **Status:** completed

### 2026-04-08 | feature-infra-core | opus | PR #382
- **Issue:** #287 — refactor(#284-A3): Config key registry — centralized constexpr keys
- **Branch:** feature/issue-287-refactor284-a3-config-key-registry--cent
- **Mode:** pipeline
- **Fix iterations:** 0
- **Status:** completed

### 2026-04-08 | feature-infra-core | opus | PR #383
- **Issue:** #286 — feat(#284-A2): IClock interface + MockClock
- **Branch:** feature/issue-286-feat284-a2-iclock-interface--mockclock
- **Mode:** pipeline
- **Fix iterations:** 0
- **Status:** completed

### 2026-04-10 | feature-nav | claude-opus-4-6 | PR #397
- **Issue:** #394 — fix(#394): D* Lite snap offset exceeding acceptance_radius
- **Branch:** fix/issue-394-snap-offset-acceptance-radius
- **Mode:** skill (/deploy-issue)
- **Fix iterations:** 2 (review fixes + re-review fixes)
- **Status:** completed
