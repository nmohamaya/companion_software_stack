# Remote Pipeline Monitoring & Mobile Approvals

Monitor and approve multi-agent pipeline checkpoints from your phone or any remote device.

## Overview

The pipeline (`deploy-issue.sh --pipeline`) has 5 human checkpoints where you approve or reject agent work. This guide covers how to manage pipelines remotely using:

1. **tmux** — persistent terminal sessions you can attach to from anywhere
2. **ntfy.sh** — push notifications when checkpoints need attention
3. **Tailscale** — secure SSH access from your phone without port forwarding

## Prerequisites

| Tool | Install | Purpose |
|------|---------|---------|
| tmux | `sudo apt install tmux` | Persistent terminal sessions |
| Tailscale | `curl -fsSL https://tailscale.com/install.sh \| sh` | VPN for remote SSH |
| ntfy (optional) | Phone app store | Push notifications |

**Phone apps:**

| App | Platform | Purpose |
|-----|----------|---------|
| Termux | Android | Full Linux terminal + SSH (free, open-source) |
| Blink Shell | iOS | SSH terminal |
| Tailscale | Android / iOS | VPN |
| ntfy | Android / iOS | Push notifications |

## Quick Start

### 1. Set up Tailscale (one-time)

**On your dev machine:**
```bash
# Install
curl -fsSL https://tailscale.com/install.sh | sh

# Start and authenticate
sudo tailscale up

# Note your Tailscale IP
tailscale ip -4
# Example: 100.64.0.1
```

**On your phone:**
1. Install the Tailscale app
2. Sign in with the same account
3. Your phone and dev machine are now on the same private network

**Verify:** From your phone's SSH app, connect to `100.64.0.1` (your Tailscale IP). If you enabled MagicDNS in the Tailscale admin console, you can use the machine name instead (e.g., `ssh dev-machine`).

### 2. Run pipelines in tmux

**Start a pipeline:**
```bash
# Create a named tmux session and run the pipeline inside it
tmux new -s pipeline-367 "bash scripts/deploy-issue.sh 367 --pipeline"
```

**Detach** (leave it running in the background):
- Press `Ctrl+B`, then `D`
- The pipeline keeps running — it will block at checkpoints until you return

**Reattach** (from your phone or any terminal):
```bash
# SSH into your dev machine
ssh 100.64.0.1

# Reattach to the pipeline session
tmux attach -t pipeline-367
```

You'll see the full terminal output — diffs, AGENT_REPORT.md, validation results — and can type your response (`a` for accept, `r` for reject, etc.).

### 3. Managing multiple pipelines

```bash
# List all active tmux sessions
tmux list-sessions

# Example output:
# pipeline-367: 1 windows (created Mon Apr  7 10:30:00 2026)
# pipeline-368: 1 windows (created Mon Apr  7 10:45:00 2026)

# Attach to a specific one
tmux attach -t pipeline-368

# Switch between sessions (while attached)
# Ctrl+B, then S — shows session list, arrow keys to select
```

### 4. Set up ntfy notifications (optional)

Get push notifications when checkpoints need your attention, so you don't have to keep checking.

**Choose a unique topic name** (acts like a private channel):
```bash
# Pick something unique — anyone who knows the topic can subscribe
export NTFY_TOPIC="drone-pipeline-$(whoami)-$(hostname)"
```

**Test it:**
```bash
# Send a test notification
curl -d "Hello from your pipeline!" ntfy.sh/$NTFY_TOPIC
```

**On your phone:**
1. Open the ntfy app
2. Subscribe to your topic name
3. You should see the test notification

**Add to your pipeline** (manual, until Python rewrite integrates it):
```bash
# Wrapper script: scripts/pipeline-notify.sh
#!/usr/bin/env bash
# Send a notification at each checkpoint
# Usage: source scripts/pipeline-notify.sh

notify() {
    local message="$1"
    local topic="${NTFY_TOPIC:-}"
    if [[ -n "$topic" ]]; then
        curl -s \
            -H "Title: Pipeline Checkpoint" \
            -H "Tags: robot" \
            -d "$message" \
            "https://ntfy.sh/${topic}" >/dev/null 2>&1 &
    fi
}
```

## tmux Cheat Sheet

| Action | Keys |
|--------|------|
| Detach from session | `Ctrl+B`, `D` |
| List sessions | `tmux list-sessions` |
| Attach to session | `tmux attach -t <name>` |
| Switch sessions | `Ctrl+B`, `S` |
| Scroll up | `Ctrl+B`, `[` (then arrow keys, `q` to exit) |
| Split pane horizontal | `Ctrl+B`, `"` |
| Split pane vertical | `Ctrl+B`, `%` |
| Switch panes | `Ctrl+B`, arrow key |
| Kill session | `tmux kill-session -t <name>` |

**Scrolling on phone:** This is the most common need — to read diffs and reports:
1. `Ctrl+B`, then `[` to enter scroll mode
2. Use arrow keys or swipe to scroll
3. Press `q` to exit scroll mode

## Workflow Example

**Morning — start pipelines from your workstation:**
```bash
# Start 3 pipelines for different issues
tmux new -d -s pipeline-370 "bash scripts/deploy-issue.sh 370 --pipeline"
tmux new -d -s pipeline-371 "bash scripts/deploy-issue.sh 371 --pipeline"
tmux new -d -s pipeline-372 "bash scripts/deploy-issue.sh 372 --pipeline"

# The -d flag starts them detached (in the background)
```

**Later — check from your phone at the coffee shop:**
```bash
# SSH in via Tailscale
ssh 100.64.0.1

# See what's running
tmux list-sessions
# pipeline-370: ... (attached)    <-- still working
# pipeline-371: ... (attached)    <-- waiting at CP1
# pipeline-372: ... (attached)    <-- waiting at CP3

# Attach to the one waiting for you
tmux attach -t pipeline-371

# Review the checkpoint, type 'a' to accept
# Detach: Ctrl+B, D
# Check the next one
tmux attach -t pipeline-372
```

**With ntfy notifications** you'd get a phone buzz when each pipeline hits a checkpoint, so you know exactly when to check in.

## Pipeline Resume

If your SSH connection drops or you accidentally close the terminal, the pipeline is still running inside tmux. Just reattach:

```bash
tmux attach -t pipeline-367
```

If the pipeline itself was interrupted (Ctrl+C), the state is saved to `.pipeline-state`. Resume it:

```bash
bash scripts/deploy-issue.sh 367 --pipeline
# "Found saved pipeline state at CP2. Resume? [Y/n]"
```

## Security Notes

- **Tailscale** encrypts all traffic with WireGuard. No data passes through Tailscale's servers (peer-to-peer).
- **ntfy.sh** messages are sent over HTTPS but the service is public — don't include secrets in notifications. For sensitive projects, self-host ntfy (`docker run -p 80:80 binwiederhier/ntfy`).
- **SSH keys** — use key-based auth, not passwords. Most phone SSH clients support importing keys.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Can't SSH from phone | Check Tailscale is connected on both devices (`tailscale status`) |
| tmux session not found | `tmux list-sessions` — it may have a different name |
| Can't scroll in tmux on phone | `Ctrl+B`, `[` enters scroll mode. Some phone keyboards need special key combos for Ctrl. |
| ntfy notification not received | Check topic name matches exactly. Test with `curl -d "test" ntfy.sh/<topic>` |
| Pipeline not resuming | Check `.claude/worktrees/issue-<N>/.pipeline-state` exists |

## Related

- [Multi-Agent Guide](MULTI_AGENT_GUIDE.md) — full pipeline documentation
- [Development Workflow](DEVELOPMENT_WORKFLOW.md) — branch and PR conventions
- [ADR-010](../adr/ADR-010-multi-agent-pipeline-architecture.md) — pipeline architecture decision record
- Issue [#371](https://github.com/nmohamaya/companion_software_stack/issues/371) — implementation tracking
