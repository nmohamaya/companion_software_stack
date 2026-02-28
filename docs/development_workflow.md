# Development Workflow

## Branching & PR Process

```
main ← feature/issue-<N>-<short-description>   # new features
main ← fix/<short-description>                  # bug fixes
```

1. **Create an issue** for every change (feature, bug fix, or chore).
2. **Branch from `main`:**
   - Features: `git checkout -b feature/issue-<N>-<short-description> main`
   - Bug fixes: `git checkout -b fix/<short-description> main`
3. **Develop, commit, push** to the branch.
4. **Open a PR** targeting `main`. Reference the issue (`Fixes #<N>` for bugs, `Closes #<N>` for features) in the PR body.
5. **CI must pass** — all tests green, `-Werror -Wall -Wextra` clean.
6. **Get review approval**, address all comments.
7. **Update documentation** — update `PROGRESS.md` and `ROADMAP.md` with the changes (see [Documentation Updates](#documentation-updates) below).
8. **Merge the PR** (squash or merge commit).

---

## Bug Fix Workflow

> **Example (Issue #24 → PR #23):** Drone hovered for ~30-60s during RTL instead of landing promptly. Caused by empty RTL/LAND FSM state handlers that relied on PX4's built-in loiter delay.

Bug fixes follow the same process as features, with these conventions:

1. **Create a bug issue first** — describe the observed behavior, expected behavior, and root cause if known. Label it `bug`.
2. **Branch with `fix/` prefix**: `git checkout -b fix/<short-description> main`
3. **Reference with `Fixes #<N>`** in the PR body (not `Closes`) — GitHub convention for bug fixes.
4. **Include before/after comparison** in the PR description when behavior changes are visible.

---

## Documentation Updates

Every PR should include updates to the project's tracking documents:

1. **`PROGRESS.md`** — Add an entry under the current Phase section:
   - Improvement number, title, date, category
   - Files added/modified
   - What was done and why
   - Test coverage additions (if any)
   - Updated summary metrics table

2. **`ROADMAP.md`** — Update the relevant sections:
   - Mark completed issues as done in the phase tables (strikethrough + ✅)
   - Update the issue tracking table (change state from "Open" to "Closed")
   - Update the metrics history table with the new column

3. **When to update:** Include doc updates in the same PR branch, committed before merge.

---

## Pre-Merge Checklist

Before merging any PR, verify:

- [ ] All commits intended for `main` are **on the branch at merge time** (not pushed after merge)
- [ ] CI passes on the final commit SHA
- [ ] `ctest --test-dir build` passes locally (262+ tests)
- [ ] No untracked source files (`git status --short | grep "^??"` should be empty)
- [ ] Deploy/install scripts tested end-to-end if modified
- [ ] `PROGRESS.md` and `ROADMAP.md` updated

---

## Post-Merge Commits — Lessons Learned

> **Incident (PR #20 → Issue #22):** Two commits (`install_dependencies.sh` and its bug fixes) were pushed to the feature branch *after* the PR was already merged. This left critical install script fixes out of `main`, requiring a follow-up PR (#21) to cherry-pick them in.

**Rules to prevent this:**

1. **Never push to a feature branch after its PR is merged.** The branch is dead once merged.
2. **If you discover something needs fixing after merge**, create a new issue + branch + PR from `main`.
3. **Verify the merge SHA matches your latest commit** before deleting the branch:
   ```bash
   # After merge, confirm main has your latest work
   git fetch origin main
   git log origin/main --oneline -5   # should include your final commit
   ```
4. **Use `Closes #<N>` in the PR body** so the issue auto-closes only when the fix actually lands in `main`.

---

## Quick Reference

| Action | Command |
|---|---|
| New feature branch | `git checkout -b feature/issue-<N>-desc main` |
| New bug fix branch | `git checkout -b fix/<desc> main` |
| Run all tests | `cd build && ctest --output-on-failure` |
| CI-equivalent build | `cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)` |
| Check for missed commits | `git log origin/main..HEAD --oneline` (should be empty after merge) |
| Clean simulation state | `pkill -f "px4\|gz sim" && rm -f /dev/shm/drone_*` |
