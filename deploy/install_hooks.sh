#!/usr/bin/env bash
# deploy/install_hooks.sh — install the repo's git hooks (ADR-016 Tier-0).
#
# Points git at the tracked .githooks/ dir, so any clone opts in with one
# command (vs the untracked .git/hooks/ that has to be re-created per clone).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

chmod +x .githooks/* 2>/dev/null || true
git config core.hooksPath .githooks

echo "Installed git hooks: core.hooksPath -> .githooks"
echo "Active hooks: $(ls .githooks 2>/dev/null | tr '\n' ' ')"
echo "Bypass a single commit deliberately with: git commit --no-verify"
