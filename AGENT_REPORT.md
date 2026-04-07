# Agent Report — Issue #371: Remote Pipeline Monitoring

## Summary

Implemented tmux session management and ntfy.sh push notifications for the multi-agent pipeline, enabling monitoring and checkpoint approval from any device (phone via SSH, tablet, another terminal).

**Two complementary layers:**
1. **tmux sessions** — Pipeline runs inside a named tmux session (`pipeline-<issue>`). User can detach, walk away, and reattach from any device.
2. **ntfy.sh notifications** — Push notifications sent at each of the 5 checkpoints, on errors, and on completion. User gets a ping on their phone without watching the terminal.

## Changes

### New Files

| File | Purpose |
|------|---------|
| `scripts/orchestrator/pipeline/notifications.py` | ntfy.sh notification module — `Notifier`, `NotifyConfig`, `NotifyEvent` |
| `scripts/orchestrator/pipeline/tmux.py` | tmux session management — `TmuxSession`, `list_pipeline_sessions`, `session_status` |
| `scripts/orchestrator/commands/pipeline_monitor.py` | `pipeline list/attach/status/kill` subcommands |
| `tests/test_orchestrator/test_notifications.py` | 18 unit tests for notification module |
| `tests/test_orchestrator/test_tmux.py` | 20 unit tests for tmux module |
| `tests/test_orchestrator/test_checkpoint_notifications.py` | 10 integration tests for checkpoint + notification |

### Modified Files

| File | Changes |
|------|---------|
| `scripts/orchestrator/pipeline/checkpoints.py` | Added optional `notifier` param to all 5 checkpoint functions; sends notification at each checkpoint |
| `scripts/orchestrator/commands/deploy_issue.py` | Added `--tmux` and `--notify` flags; tmux session wrapping; notifier creation and passing through pipeline |
| `scripts/orchestrator/cli.py` | Added `--tmux`/`--notify` flags to `deploy-issue`; added `pipeline` subcommand group (list/attach/status/kill) |

## Usage

### tmux sessions
```bash
# Launch pipeline in tmux (detachable)
python -m orchestrator deploy-issue 367 --pipeline --tmux

# Detach: Ctrl+B, D
# Reattach from any device:
tmux attach -t pipeline-367

# List active pipelines
python -m orchestrator pipeline list

# Check status
python -m orchestrator pipeline status 367

# Attach to session
python -m orchestrator pipeline attach 367

# Kill session
python -m orchestrator pipeline kill 367
```

### ntfy.sh notifications
```bash
# Via command line flag
python -m orchestrator deploy-issue 367 --pipeline --notify drone-pipeline-nm

# Via environment variable
export NTFY_TOPIC=drone-pipeline-nm
python -m orchestrator deploy-issue 367 --pipeline

# Custom server (self-hosted)
export NTFY_SERVER=https://ntfy.example.com
export NTFY_TOPIC=my-topic
```

### Combined (recommended for on-the-move use)
```bash
python -m orchestrator deploy-issue 367 --pipeline --tmux --notify drone-pipeline-nm
```

## Architecture Decisions

1. **Graceful degradation** — Notifications never block the pipeline. If ntfy.sh is unreachable or curl fails, it logs a warning and continues. tmux unavailability falls back to direct execution.

2. **No new dependencies** — Uses only `curl` (for ntfy) and `tmux`, both standard on Linux. No Python packages needed.

3. **Environment-based config** — `NTFY_TOPIC`, `NTFY_SERVER`, `NTFY_EVENTS` env vars allow persistent config without editing code.

4. **Session naming convention** — `pipeline-<issue-number>` for predictable naming and easy scripting.

5. **Layer 3 deferred** — Interactive ntfy responses (approve/reject from notification) intentionally deferred to a future phase per the issue spec.

## Test Plan

- [ ] `PYTHONPATH=scripts python -m pytest tests/test_orchestrator/test_notifications.py -v` — 18 tests
- [ ] `PYTHONPATH=scripts python -m pytest tests/test_orchestrator/test_tmux.py -v` — 20 tests
- [ ] `PYTHONPATH=scripts python -m pytest tests/test_orchestrator/test_checkpoint_notifications.py -v` — 10 tests
- [ ] Existing tests still pass: `PYTHONPATH=scripts python -m pytest tests/test_orchestrator/ -v`
- [ ] Manual: `--notify` flag sends real notification to ntfy.sh topic
- [ ] Manual: `--tmux` flag creates tmux session and attaches
- [ ] C++ build unaffected (no C++ changes)

## Risks / Review Attention

- **Low risk** — Python-only changes to the orchestrator; no C++ code modified, no ctest count change.
- **External dependency** — ntfy.sh is a free service; self-hosting recommended for production use.
- **tmux availability** — Falls back gracefully if tmux is not installed.
