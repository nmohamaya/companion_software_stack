# ADR-005: Parallel AI Agent Workflow via git worktree

> **Update note (2026-07):** The core decision — one `git worktree` per agent
> with an independent `build/` dir — remains in force. Some details have drifted
> since 2026-03 and are corrected inline below: the POSIX SHM backend and its
> `ENABLE_ZENOH`/`HAVE_ZENOH` build guards were removed (Zenoh is now always
> built, so the test suite can no longer be silently flag-dropped); the shared
> tracking docs moved out of repo root into `docs/tracking/` and `tests/`;
> `DEVELOPMENT_WORKFLOW.md` moved under `docs/how-to/`; and the original
> two-agent model became a multi-agent pipeline whose coordinated worktrees
> branch from a long-lived integration branch rather than directly from `main`.

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

Multiple AI coding agents (Claude instances) work concurrently on this
codebase.  Each agent may hold uncommitted changes, run builds, and push
branches independently.

### Problems with a single working tree

| Problem | Impact |
|---------|--------|
| **Branch collision** | Agent A has unstaged edits on branch X; agent B runs `git checkout Y` → agent A's work is lost or conflicts arise |
| **Build interference** | Both share `build/`; switching build type in one worktree (e.g. Release ↔ Coverage) leaves stale object files that surface later as confusing `__gcov_init` linker errors — no warning at reconfigure time |
| **File contention** | Shared doc files (`docs/tracking/PROGRESS.md`, `docs/tracking/ROADMAP.md`, `tests/TESTS.md`, `docs/tracking/BUG_FIXES.md`) are updated at the end of every PR cycle, causing merge conflicts |
| **Stale object files** | Mixing build types (Release vs Coverage) in a single `build/` directory causes `__gcov_init` linker errors |

## 2. Decision

Use `git worktree` to give each agent its own checkout directory, sharing a
single `.git` object store:

```
~/Projects/
├── companion_software_stack/              ← primary checkout
│   ├── .git/                              ← single shared repo
│   └── build/                             ← primary build artifacts
│
└── companion_software_stack_worktrees/    ← per-agent worktrees
    └── <branch-name>/                     ← one worktree per agent branch
        ├── .git  (file with "gitdir: ..." pointer into main repo's .git/worktrees/)
        └── build/                         ← that worktree's build artifacts
```

(Short-lived, agent-scoped worktrees may instead live in-repo under
`.claude/worktrees/agent-NNN`; both layouts share the single `.git` store.)

### Rules

1. **One branch per worktree.** git enforces this — two worktrees cannot have
   the same branch checked out simultaneously.
2. **Independent builds.** Each worktree has its own `build/` directory.
   Always run the canonical cmake command (see `docs/how-to/DEVELOPMENT_WORKFLOW.md`) inside the
   worktree, never reference the other tree's build dir.
3. **Doc merges.** When agents update `docs/tracking/PROGRESS.md`,
   `docs/tracking/ROADMAP.md`, or `tests/TESTS.md`, the second-to-merge PR
   handles the git conflict — append-only doc sections make this trivial.
4. **Worktree lifecycle.** Create before starting work, remove after the
   feature branch is merged:

   ```bash
   # Create — coordinated multi-issue work branches from the long-lived
   # integration branch; standalone single-issue work branches from main.
   git worktree add ~/Projects/companion_software_stack_worktrees/feature-issue-NN-desc \
       -b feature/issue-NN-desc integration/epic-NN-desc

   # Remove (after PR merge)
   git worktree remove ~/Projects/companion_software_stack_worktrees/feature-issue-NN-desc
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

- Agents work simultaneously on different branches with zero risk of
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
- Doc-file merge conflicts can occur when agents update the same
  append-only files in the same PR cycle — but these are trivially resolved
  since entries are sequential.

### Neutral

- Pushing from any worktree updates the same remote — all agents see
  each other's branches immediately.
- `git worktree list` shows all active worktrees for visibility.
