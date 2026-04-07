"""Branch and worktree cleanup command.

Replaces cleanup-branches.sh (119 lines).
Finds merged branches and stale worktrees, optionally removes them.
"""

from __future__ import annotations

from orchestrator.config import resolve_project_dir
from orchestrator.console import Console
from orchestrator.git import Git


PROTECTED_BRANCHES = {"main", "develop"}


def run(
    io: Console | None = None,
    git: Git | None = None,
    *,
    dry_run: bool = False,
    auto_yes: bool = False,
) -> int:
    """Clean up merged branches and stale worktrees."""
    if io is None:
        io = Console()
    if git is None:
        git = Git(resolve_project_dir())

    io.header("Branch & Worktree Cleanup")

    # Find merged branches
    merged = git.merged_branches("main")
    merged = [b for b in merged if b not in PROTECTED_BRANCHES]

    # Find worktrees and their branches
    worktrees = git.worktree_list()
    wt_branch_map: dict[str, str] = {}
    for wt in worktrees:
        parts = wt.split()
        if len(parts) >= 3:
            path = parts[0]
            # Extract branch from [branch_name]
            for part in parts:
                if part.startswith("[") and part.endswith("]"):
                    wt_branch_map[part[1:-1]] = path

    # Stale worktrees: worktrees whose branch is merged
    stale_wts = [
        wt_branch_map[b] for b in merged if b in wt_branch_map
    ]

    if not merged and not stale_wts:
        io.print("Nothing to clean up.")
        return 0

    if merged:
        io.print(f"Merged branches to delete ({len(merged)}):")
        for b in merged:
            io.print(f"  - {b}")
        io.print("")

    if stale_wts:
        io.print(f"Stale worktrees to remove ({len(stale_wts)}):")
        for wt in stale_wts:
            io.print(f"  - {wt}")
        io.print("")

    if dry_run:
        io.print("(dry run — no changes made)")
        return 0

    if not auto_yes:
        if not io.confirm("Proceed with cleanup?"):
            io.print("Aborted.")
            return 0

    wt_removed = 0
    br_removed = 0

    for wt in stale_wts:
        io.print(f"Removing worktree: {wt}")
        try:
            git.worktree_remove(wt)
            wt_removed += 1
        except Exception:
            io.warn(f"Could not remove worktree {wt}")

    for branch in merged:
        io.print(f"Deleting branch: {branch}")
        try:
            git.delete_branch(branch)
            br_removed += 1
        except Exception:
            io.warn(f"Could not delete branch {branch}")

    io.print("")
    io.print(
        f"Summary: {br_removed} branches removed, "
        f"{wt_removed} worktrees cleaned"
    )
    return 0
