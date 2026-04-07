# Agent Report: Issue #299 — Add --json-logs flag to scenario runners

**Agent:** feature-infra-platform
**Branch:** feature/issue-299-feat-add---json-logs-flag-to-scenario-ru
**Date:** 2026-04-07

## Summary

Added `--json-logs` flag to `tests/run_scenario.sh` and `tests/run_scenario_gazebo.sh` so structured JSON log output can be enabled when running scenarios. The flag is forwarded through to the underlying launcher scripts (`launch_all.sh` and `launch_gazebo.sh`), which in turn pass it to all 7 companion process binaries.

## Changes

| File | Change |
|------|--------|
| `tests/run_scenario.sh` | Added `--json-logs` to arg parser, help text, `--all` recursive call, and `launch_all.sh` invocation |
| `tests/run_scenario_gazebo.sh` | Added `--json-logs` to arg parser, help text, `--all` recursive call, and `launch_gazebo.sh` invocation |

**Note:** `deploy/launch_gazebo.sh` already forwards unknown args to all binaries via its `EXTRA_ARGS` catch-all (line 39: `*) EXTRA_ARGS="${EXTRA_ARGS} ${arg}" ;;`), so no modification was needed there. The issue description suggested modifying it, but the existing code already handles the pass-through correctly.

## Verification

- Bash syntax check: both scripts pass `bash -n` validation
- Help output: `--json-logs` appears correctly in both `--help` outputs
- Quick CI: 2/2 jobs passed (FMT + BUILD), 1286/1286 tests pass, 0 warnings
- No C++ files modified, so format/clang-tidy not applicable

## Usage

```bash
# Tier 1 scenario with JSON logs
tests/run_scenario.sh config/scenarios/03_battery_degradation.json --json-logs

# Tier 2 Gazebo scenario with JSON logs
tests/run_scenario_gazebo.sh config/scenarios/18_perception_avoidance.json --json-logs

# All scenarios with JSON logs
tests/run_scenario.sh --all --json-logs
tests/run_scenario_gazebo.sh --all --json-logs

# Direct Gazebo launch (already worked before this change)
bash deploy/launch_gazebo.sh --json-logs
```

## Commit

```
feat(#299): add --json-logs flag to scenario runners
```
