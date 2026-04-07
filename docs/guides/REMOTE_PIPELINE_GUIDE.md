# Remote Pipeline Monitoring & Mobile Approvals

Monitor and approve multi-agent pipeline checkpoints from your phone or any remote device.

## Overview

The pipeline (`python -m orchestrator deploy-issue <issue> --pipeline`) has 5 human checkpoints where you approve or reject agent work. This guide covers how to manage pipelines remotely using:

1. **tmux** — persistent terminal sessions you can attach to from anywhere
2. **ntfy.sh** — push notifications when checkpoints need attention
3. **NordVPN Meshnet** or **Tailscale** — secure SSH access from your phone without port forwarding

## Prerequisites

| Tool | Install | Purpose |
|------|---------|---------|
| tmux | `sudo apt install tmux` | Persistent terminal sessions |
| openssh-server | `sudo apt install openssh-server` | SSH access to dev machine |
| NordVPN (Meshnet) | `sh <(curl -sSf https://downloads.nordcdn.com/apps/linux/install.sh)` | VPN for remote SSH (if you have NordVPN) |
| Tailscale | `curl -fsSL https://tailscale.com/install.sh \| sh` | VPN for remote SSH (free alternative) |

**Phone apps:**

| App | Platform | Purpose |
|-----|----------|---------|
| Termux (free via [F-Droid](https://f-droid.org)) | Android | Full Linux terminal + SSH (free, open-source) |
| Termux (paid via Google Play Store) | Android | Same app, paid version |
| Blink Shell | iOS | SSH terminal |
| ntfy | Android / iOS | Push notifications |

## Quick Start

### 1. Set up SSH on your dev machine (one-time)

```bash
# Install SSH server
sudo apt install openssh-server

# Enable and start
sudo systemctl enable --now ssh

# Verify it's running
sudo systemctl status ssh
```

### 2. Set up VPN (one-time)

Choose **one** of the following:

#### Option A: NordVPN Meshnet (if you have a NordVPN subscription)

**On your dev machine:**
```bash
# Install NordVPN CLI
sh <(curl -sSf https://downloads.nordcdn.com/apps/linux/install.sh)

# Log in and enable Meshnet
nordvpn login
nordvpn set meshnet on

# List your devices — note the Meshnet name/IP
nordvpn meshnet peer list
```

**On your phone:**
1. Open NordVPN app → **Meshnet** tab
2. Your dev machine should appear as a peer
3. Tap it and enable **"Allow remote access"**

#### Option B: Tailscale (free)

**On your dev machine:**
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4   # Note the IP (e.g., 100.64.0.1)
```

**On your phone:**
1. Install the Tailscale app
2. Sign in with the same account

### 3. Set up Termux on your phone (one-time)

Install Termux (free from F-Droid, or paid from Google Play Store), then:

```bash
# Update packages
pkg update && pkg upgrade

# Install SSH and tmux
pkg install openssh tmux

# Generate SSH key (so you don't need passwords)
ssh-keygen -t ed25519

# Copy key to dev machine (replace with your Meshnet/Tailscale IP)
ssh-copy-id your-username@<meshnet-or-tailscale-ip>
```

### 4. Test the connection

From Termux on your phone:

```bash
# Connect — use your Ubuntu username, not your NordVPN device name
ssh your-username@<meshnet-or-tailscale-ip>
```

> **Common mistake:** SSH uses your **Ubuntu username** (e.g., `nmohanan31`), not your VPN device name. If you see "invalid user", check you're specifying the right username with `@`.

### 5. Run pipelines in tmux

**Start a pipeline inside a tmux session:**
```bash
# Create a named tmux session
tmux new-session -s pipeline-299

# Inside the session, run the pipeline
PYTHONPATH=scripts python3 -m orchestrator deploy-issue 299 --pipeline
```

**Detach** (leave it running in the background):
- Press `Ctrl+B`, then `D`
- The pipeline keeps running — it will block at checkpoints until you return

**Reattach from your phone:**
```bash
# SSH into your dev machine
ssh your-username@<meshnet-or-tailscale-ip>

# Reattach to the pipeline session
tmux attach -t pipeline-299
```

You'll see the full terminal output — diffs, AGENT_REPORT.md, validation results — and can type your response (`a` for accept, `r` for reject, etc.).

### 6. Set up ntfy notifications (optional)

Get push notifications when checkpoints need your attention.

**On your phone:**
1. Install the ntfy app (Play Store, F-Droid, or App Store)
2. Subscribe to a unique topic (e.g., `drone-pipeline-yourname`)

**Test it:**
```bash
curl -d "Hello from your pipeline!" ntfy.sh/drone-pipeline-yourname
```

**Use with the pipeline:**
```bash
# Pass the --notify flag with your topic
PYTHONPATH=scripts python3 -m orchestrator deploy-issue 299 --pipeline --notify drone-pipeline-yourname

# Or set it as an environment variable
export NTFY_TOPIC="drone-pipeline-yourname"
PYTHONPATH=scripts python3 -m orchestrator deploy-issue 299 --pipeline
```

You'll get push notifications at each checkpoint, so you know when to attach and review.

## Managing Multiple Pipelines

```bash
# List all active tmux sessions
tmux list-sessions

# Example output:
# pipeline-299: 1 windows (created Mon Apr  7 10:30:00 2026)
# pipeline-300: 1 windows (created Mon Apr  7 10:45:00 2026)

# Attach to a specific one
tmux attach -t pipeline-300

# Switch between sessions (while attached)
# Ctrl+B, then S — shows session list, arrow keys to select
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
# Start 3 pipelines in separate tmux sessions
tmux new-session -d -s pipeline-301 \
  "PYTHONPATH=scripts python3 -m orchestrator deploy-issue 301 --pipeline"
tmux new-session -d -s pipeline-302 \
  "PYTHONPATH=scripts python3 -m orchestrator deploy-issue 302 --pipeline"
tmux new-session -d -s pipeline-303 \
  "PYTHONPATH=scripts python3 -m orchestrator deploy-issue 303 --pipeline"

# The -d flag starts them detached (in the background)
```

**Later — check from your phone:**
```bash
# SSH in
ssh your-username@<meshnet-or-tailscale-ip>

# See what's running
tmux list-sessions
# pipeline-301: ...    <-- still working
# pipeline-302: ...    <-- waiting at CP1
# pipeline-303: ...    <-- waiting at CP3

# Attach to the one waiting for you
tmux attach -t pipeline-302

# Review the checkpoint, type 'a' to accept
# Detach: Ctrl+B, D
# Check the next one
tmux attach -t pipeline-303
```

**With ntfy notifications** you'd get a phone buzz when each pipeline hits a checkpoint, so you know exactly when to check in.

## Pipeline Resume

If your SSH connection drops, the pipeline is still running inside tmux. Just reattach:

```bash
tmux attach -t pipeline-299
```

If the pipeline itself was interrupted (Ctrl+C), the state is saved. Resume it:

```bash
PYTHONPATH=scripts python3 -m orchestrator deploy-issue 299 --pipeline
# "Found saved pipeline state at CP2. Resume? [Y/n]"
```

## Security Notes

- **NordVPN Meshnet** and **Tailscale** both encrypt traffic with WireGuard. Peer-to-peer, no data through their servers.
- **ntfy.sh** messages are sent over HTTPS but the service is public — don't include secrets in notifications. For sensitive projects, self-host ntfy (`docker run -p 80:80 binwiederhier/ntfy`) and set `NTFY_SERVER` and `NTFY_TOKEN`.
- **SSH keys** — use key-based auth instead of passwords. Generate with `ssh-keygen -t ed25519` and copy with `ssh-copy-id`.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Can't SSH from phone | Check VPN is connected on both devices. For NordVPN: `nordvpn meshnet peer list`. For Tailscale: `tailscale status`. |
| SSH says "permission denied" | Make sure you're using your Ubuntu username (e.g., `nmohanan31@`), not your VPN device name |
| SSH says "unit sshd.service not found" | On Ubuntu the service is called `ssh`, not `sshd`: `sudo systemctl enable --now ssh` |
| tmux says "no such file or directory" | No tmux sessions are running. Start one first: `tmux new-session -s pipeline-<issue>` |
| tmux says "duplicate session" | Kill the stale session: `tmux kill-session -t pipeline-<issue>`, then recreate |
| Can't scroll in tmux on phone | `Ctrl+B`, `[` enters scroll mode. Some phone keyboards need special key combos for Ctrl. |
| ntfy notification not received | Check topic name matches exactly. Test with `curl -d "test" ntfy.sh/<topic>` |
| Pipeline not resuming | Check `.claude/worktrees/issue-<N>/.pipeline-state` exists |

## Related

- [Multi-Agent Guide](MULTI_AGENT_GUIDE.md) — full pipeline documentation
- [Development Workflow](DEVELOPMENT_WORKFLOW.md) — branch and PR conventions
- [ADR-010](../adr/ADR-010-multi-agent-pipeline-architecture.md) — pipeline architecture decision record
- Issue [#371](https://github.com/nmohamaya/companion_software_stack/issues/371) — implementation tracking
