# Security Issues & Mitigations

This document tracks security vulnerabilities, mitigations, and hardening decisions relevant to the companion software stack — particularly the multi-agent development workflow and production deployment.

---

## SEC-001 — Git Bare Repository Attack (CVE-2024-32002)

**Date:** 2026-03-17
**Severity:** Critical
**Affects:** Development workflow (multi-agent git worktrees)
**Status:** Mitigated

### Vulnerability

Git versions before 2.45.1 are vulnerable to arbitrary code execution during `git clone --recurse-submodules` on case-insensitive filesystems (macOS, Windows). An attacker crafts a repository with:

1. A submodule at path `A/modules/x`
2. A symlink `a` → `.git/modules/x`
3. On case-insensitive filesystems, `A/` and `a/` resolve to the same directory
4. Git writes attacker-controlled files through the symlink into `.git/modules/x/hooks/`
5. Hooks like `post-checkout` execute automatically — **arbitrary code execution on clone**

### Why This Matters for This Project

This project uses **git worktrees** for multi-agent isolation (see `CLAUDE.md`). Multiple AI agents clone and operate on branches concurrently. If an agent were tricked into cloning a malicious repository (e.g., via a crafted submodule URL or fork), the attack could compromise the build environment.

Attack surface:
- Agents cloning forks or external repositories
- Submodule references in `.gitmodules` pointing to untrusted remotes
- CI pipelines that clone with `--recurse-submodules`

### Mitigations Applied

| Mitigation | Scope | Notes |
|------------|-------|-------|
| Git ≥ 2.45.1 on all dev machines | Dev | Verify with `git --version` |
| `core.symlinks = false` | Dev | `git config --global core.symlinks false` — prevents symlink traversal |
| `protocol.file.allow = user` | Dev | `git config --global protocol.file.allow user` — restricts local file protocol |
| No submodules in this repo | Repo | All dependencies are system-installed or vendored |
| Agents clone only from trusted remote | Workflow | Agents must not clone arbitrary URLs |
| CI uses shallow clone without submodules | CI | `actions/checkout` with `submodules: false` |

### Verification

```bash
# Check git version (must be >= 2.45.1)
git --version

# Check global config mitigations
git config --global --get core.symlinks        # should be false
git config --global --get protocol.file.allow  # should be user

# Verify no submodules exist
test ! -f .gitmodules && echo "OK: no submodules"
```

### References

- [CVE-2024-32002](https://nvd.nist.gov/vuln/detail/CVE-2024-32002) — Git recursive clone RCE
- [CVE-2024-32004](https://nvd.nist.gov/vuln/detail/CVE-2024-32004) — Local clone arbitrary code execution
- [Git 2.45.1 release notes](https://github.blog/open-source/git/git-security-vulnerabilities-announced-3/) — Security fix announcement

---

## SEC-002 — Zenoh IPC Without TLS (ALLOW_INSECURE_ZENOH)

**Date:** 2026-03-17
**Severity:** Medium
**Affects:** Development and simulation environments
**Status:** Accepted risk (dev only)

### Description

The build flag `ALLOW_INSECURE_ZENOH=ON` disables TLS certificate validation for Zenoh IPC sessions. This is required on development machines that lack provisioned TLS certificates.

### Risk

Without TLS, Zenoh sessions are vulnerable to:
- **Eavesdropping** — IPC messages (FC commands, telemetry, mission data) are transmitted in plaintext
- **Injection** — An attacker on the same network could publish malicious messages on Zenoh topics (e.g., fake FC state, spoofed GCS commands)
- **Replay** — Captured messages could be replayed to trigger unintended drone behaviour

### Mitigations

| Environment | TLS Required | Notes |
|-------------|-------------|-------|
| Unit tests / CI | No | Localhost only, no network exposure |
| Gazebo SITL | No | Localhost only |
| Hardware-in-the-loop | **Yes** | Shared network with FC, GCS |
| Production (Jetson Orin) | **Yes** | `ALLOW_INSECURE_ZENOH` must be OFF |

### Production Requirements

- Build **without** `ALLOW_INSECURE_ZENOH` (defaults to OFF)
- Provision TLS certificates on the Jetson Orin (see `deploy/install_certs.sh`)
- Zenoh config in `config/default.json` must specify `tls.root_ca_certificate`
- CI enforces that production builds fail if `ALLOW_INSECURE_ZENOH=ON`

---

## SEC-003 — Fault Injector Access Control

**Date:** 2026-03-17
**Severity:** Medium
**Affects:** Production deployment
**Status:** Mitigated by design

### Description

The `fault_injector` tool can publish arbitrary messages on any Zenoh topic — battery overrides, FC disconnect, GCS commands. In production, this tool must not be accessible.

### Mitigations

- `fault_injector` is built only when `BUILD_TOOLS=ON` (off by default in production builds)
- Production systemd units do not include the fault injector
- The `fault/overrides` Zenoh topic is only subscribed to when `fault_injection.enabled = true` in config (false in production `default.json`)
- Hardware deploy script (`deploy/launch_hardware.sh`) does not include fault injection tools

---

## Security Checklist for Production Deployment

- [ ] Git ≥ 2.45.1 on build machine
- [ ] `ALLOW_INSECURE_ZENOH=OFF` (default)
- [ ] TLS certificates provisioned and configured in Zenoh config
- [ ] `fault_injection.enabled = false` in production config
- [ ] `fault_injector` binary not present on target
- [ ] No `.git/` directory deployed to target (use release archives)
- [ ] systemd units use `ProtectSystem=strict` and `PrivateTmp=yes`
- [ ] Verify with: `bash deploy/security_check.sh` (if available)

---

*Last updated: 2026-03-17*
