# ADR-005: Parallel AI Agent Workflow via git worktree

| Field | Value |
|-------|-------|
| **Status** | Accepted |
| **Date** | 2026-03-12 |
| **Author** | Team |
| **Deciders** | Project leads |
| **Supersedes** | — |
| **Related** | Issue #138 |

---

## 1. Context

Two AI coding agents (Claude instances in VS Code) work concurrently on this
codebase.  Each agent may hold uncommitted changes, run builds, and push
branches independently.

### Problems with a single working tree

| Problem | Impact |
|---------|--------|
| **Branch collision** | Agent A has unstaged edits on branch X; agent B runs `git checkout Y` → agent A's work is lost or conflicts arise |
| **Build interference** | Both share `build/`; a cmake reconfigure by one agent can silently reset feature flags (e.g. `ENABLE_ZENOH=OFF`), dropping ~109 tests with no warning |
| **File contention** | Shared doc files (`PROGRESS.md`, `ROADMAP.md`, `TESTS.md`, `BUG_FIXES.md`) are updated at the end of every PR cycle, causing merge conflicts |
| **Stale object files** | Mixing build types (Release vs Coverage) in a single `build/` directory causes `__gcov_init` linker errors |

## 2. Decision

Use `git worktree` to give each agent its own checkout directory, sharing a
single `.git` object store:

```
/home/user/NM/Projects/
├── companion_software_stack/      ← Agent 1 (primary checkout)
│   ├── .git/                      ← single shared repo
│   └── build/                     ← Agent 1's build artifacts
│
└── companion_stack_wt/            ← Agent 2 (worktree)
    ├── .git  (file → symlink back to main .git)
    └── build/                     ← Agent 2's build artifacts
```

### Rules

1. **One branch per worktree.** git enforces this — two worktrees cannot have
   the same branch checked out simultaneously.
2. **Independent builds.** Each worktree has its own `build/` directory.
   Always run the canonical cmake command (see `CLAUDE.md`) inside the
   worktree, never reference the other tree's build dir.
3. **Doc merges.** When both agents update `PROGRESS.md`, `ROADMAP.md`, or
   `TESTS.md`, the second-to-merge PR handles the git conflict — append-only
   doc sections make this trivial.
4. **Worktree lifecycle.** Create before starting work, remove after the
   feature branch is merged:

   ```bash
   # Create
   git worktree add ../companion_stack_wt -b feature/issue-NN-desc main

   # Remove (after PR merge)
   git worktree remove ../companion_stack_wt
   ```

5. **CI compatibility.** All build and test scripts already use relative path
   resolution (`SCRIPT_DIR` / `PROJECT_DIR`), so they work unmodified from
   any worktree directory.

## 3. Alternatives Considered

| Alternative | Pros | Cons |
|-------------|------|------|
| **Full second clone** | Complete isolation | Doubles disk usage (~200 MB+); separate remote tracking; risk of divergent history if not synced |
| **`git stash` before switch** | No extra directory | Fragile — stash can conflict; agents can't work simultaneously; requires coordination |
| **Single tree, different files only** | Simple | Fails on shared docs; shared `build/` causes cmake flag resets; no build isolation |
| **Docker-per-agent** | Full OS-level isolation | Heavy; requires Docker setup; slow build cold-starts; overkill for file-level isolation |

## 4. Consequences

### Positive

- Both agents work simultaneously on different branches with zero risk of
  overwriting each other's uncommitted changes.
- Fully independent builds — no more stale-flag pitfalls.
- No additional tooling or infrastructure required (`git worktree` is built
  into git).
- Negligible disk overhead — only source files are duplicated; the `.git`
  object store is shared.

### Negative

- One extra directory to manage on the filesystem.
- Developers and agents must remember to use the canonical cmake command in
  each worktree independently.
- Doc-file merge conflicts can occur when both agents update the same
  append-only files in the same PR cycle — but these are trivially resolved
  since entries are sequential.

### Neutral

- Pushing from either worktree updates the same remote — both agents see
  each other's branches immediately.
- `git worktree list` shows all active worktrees for visibility.
