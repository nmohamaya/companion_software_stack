"""Git subprocess wrapper — all git operations go through this module.

Wraps git CLI calls so they are mockable in tests and provide consistent
error handling. Methods correspond to git commands used across the 12 bash
scripts.
"""

from __future__ import annotations

import subprocess
from pathlib import Path


class GitError(Exception):
    """Raised when a git command fails."""

    def __init__(self, cmd: list[str], returncode: int, stderr: str) -> None:
        self.cmd = cmd
        self.returncode = returncode
        self.stderr = stderr
        super().__init__(f"git failed (exit {returncode}): {stderr.strip()}")


class Git:
    """Thin wrapper around git CLI subprocess calls."""

    def __init__(self, project_dir: Path) -> None:
        self.project_dir = project_dir

    def _run(
        self,
        *args: str,
        check: bool = True,
        capture: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        """Run a git command in the project directory."""
        cmd = ["git", "-C", str(self.project_dir), *args]
        result = subprocess.run(
            cmd,
            capture_output=capture,
            text=True,
            timeout=60,
        )
        if check and result.returncode != 0:
            raise GitError(cmd, result.returncode, result.stderr)
        return result

    # ── Repository state ───────────────────────────────────────────────────

    def is_repo(self) -> bool:
        """Check if project_dir is a git repository."""
        result = self._run("rev-parse", "--git-dir", check=False)
        return result.returncode == 0

    def current_branch(self) -> str:
        """Get the current branch name, or 'detached' if HEAD is detached."""
        result = self._run("branch", "--show-current", check=False)
        branch = result.stdout.strip()
        return branch if branch else "detached"

    def has_uncommitted_changes(self) -> bool:
        """Check for uncommitted changes (staged or unstaged)."""
        result = self._run("status", "--porcelain", check=False)
        return bool(result.stdout.strip())

    def uncommitted_count(self) -> int:
        """Count of files with uncommitted changes."""
        result = self._run("status", "--porcelain", check=False)
        lines = [l for l in result.stdout.strip().splitlines() if l.strip()]
        return len(lines)

    def head_sha(self) -> str:
        """Get the current HEAD commit SHA."""
        result = self._run("rev-parse", "HEAD")
        return result.stdout.strip()

    def behind_remote(self) -> int:
        """Count commits the current branch is behind its upstream. Returns -1 if no upstream."""
        result = self._run(
            "rev-list", "--count", "HEAD..@{upstream}", check=False
        )
        if result.returncode != 0:
            return -1
        try:
            return int(result.stdout.strip())
        except ValueError:
            return -1

    def branch_exists(self, branch: str) -> bool:
        """Check if a branch exists locally."""
        result = self._run("rev-parse", "--verify", branch, check=False)
        return result.returncode == 0

    # ── Diff and log ───────────────────────────────────────────────────────

    def diff_stat(self, base: str, head: str = "HEAD") -> str:
        """Get diff --stat between base and head."""
        result = self._run("diff", "--stat", f"{base}...{head}", check=False)
        return result.stdout.strip()

    def diff_name_only(self, base: str, head: str = "HEAD") -> list[str]:
        """Get list of changed file paths between base and head."""
        result = self._run(
            "diff", "--name-only", f"{base}...{head}", check=False
        )
        return [f for f in result.stdout.strip().splitlines() if f.strip()]

    def diff_full(self, base: str, head: str = "HEAD") -> str:
        """Get full diff between base and head."""
        result = self._run("diff", f"{base}...{head}", check=False)
        return result.stdout

    def diff_uncommitted(self) -> str:
        """Get diff of uncommitted changes (staged + unstaged)."""
        result = self._run("diff", "HEAD", check=False)
        return result.stdout

    def diff_stat_working_tree(self, base: str = "HEAD") -> str:
        """Get diff --stat including uncommitted changes (staged + unstaged).

        Uses two-dot diff (base..HEAD plus working tree) so uncommitted
        changes show up — unlike three-dot which only compares commits.
        """
        result = self._run("diff", "--stat", base, check=False)
        return result.stdout.strip()

    def log_oneline(
        self,
        n: int = 10,
        since_commit: str | None = None,
        since: str | None = None,
        ref_range: str | None = None,
    ) -> list[str]:
        """Get recent commit log as one-line entries."""
        args = ["log", "--oneline"]
        if ref_range:
            args.append(ref_range)
        elif since_commit:
            args.append(f"{since_commit}..HEAD")
        elif since:
            args.extend(["--since", since])
        else:
            args.extend(["-n", str(n)])
        result = self._run(*args, check=False)
        return [l for l in result.stdout.strip().splitlines() if l.strip()]

    def commit_count_since(self, since_commit: str) -> int:
        """Count commits since a given commit SHA."""
        result = self._run(
            "rev-list", "--count", f"{since_commit}..HEAD", check=False
        )
        try:
            return int(result.stdout.strip())
        except ValueError:
            return 0

    def log_grep(self, pattern: str, since: str | None = None) -> list[str]:
        """Search commit messages for a pattern."""
        args = ["log", "--oneline", f"--grep={pattern}"]
        if since:
            args.append(f"--since={since}")
        result = self._run(*args, check=False)
        return [l for l in result.stdout.strip().splitlines() if l.strip()]

    # ── Branch operations ──────────────────────────────────────────────────

    def list_branches(self, pattern: str = "") -> list[str]:
        """List branches, optionally filtered by a glob pattern."""
        args = ["branch", "--list"]
        if pattern:
            args.append(pattern)
        result = self._run(*args, check=False)
        branches = []
        for line in result.stdout.strip().splitlines():
            branch = line.strip().lstrip("* ")
            if branch:
                branches.append(branch)
        return branches

    def create_branch(self, name: str, base: str = "main") -> None:
        """Create a new branch from base."""
        self._run("branch", name, base)

    def checkout(self, branch: str) -> None:
        """Check out a branch."""
        self._run("checkout", branch)

    def merged_branches(self, target: str = "main") -> list[str]:
        """List branches that have been merged into target."""
        result = self._run("branch", "--merged", target, check=False)
        branches = []
        for line in result.stdout.strip().splitlines():
            branch = line.strip().lstrip("* ")
            if branch and branch != target:
                branches.append(branch)
        return branches

    # ── Worktree operations ────────────────────────────────────────────────

    def worktree_add(
        self, path: Path, branch: str, base: str | None = None
    ) -> None:
        """Create a git worktree. If base is given, creates a new branch."""
        args = ["worktree", "add", str(path)]
        if base:
            args.extend(["-b", branch, base])
        else:
            args.append(branch)
        self._run(*args)

    def worktree_remove(self, path: Path, force: bool = False) -> None:
        """Remove a git worktree."""
        args = ["worktree", "remove", str(path)]
        if force:
            args.append("--force")
        self._run(*args, check=False)

    def worktree_list(self) -> list[str]:
        """List all worktrees."""
        result = self._run("worktree", "list", check=False)
        return [l for l in result.stdout.strip().splitlines() if l.strip()]

    # ── Commit and push ────────────────────────────────────────────────────

    def add_all(self) -> None:
        """Stage all changes."""
        self._run("add", "-A")

    def add_files(self, files: list[str]) -> None:
        """Stage specific files."""
        if files:
            self._run("add", "--", *files)

    def commit(self, message: str) -> bool:
        """Create a commit. Returns True if commit was created, False if nothing to commit."""
        result = self._run("commit", "-m", message, check=False)
        if result.returncode != 0:
            if "nothing to commit" in result.stdout or "nothing to commit" in result.stderr:
                return False
            raise GitError(["git", "commit"], result.returncode, result.stderr)
        return True

    def push(
        self, branch: str | None = None, set_upstream: bool = False
    ) -> None:
        """Push to remote."""
        args = ["push"]
        if set_upstream:
            args.append("-u")
            args.append("origin")
        if branch:
            args.append(branch)
        self._run(*args)

    # ── Cleanup ────────────────────────────────────────────────────────────

    def delete_branch(self, branch: str, force: bool = False) -> None:
        """Delete a local branch."""
        flag = "-D" if force else "-d"
        self._run("branch", flag, branch)

    def delete_remote_branch(self, branch: str) -> None:
        """Delete a remote branch."""
        self._run("push", "origin", "--delete", branch, check=False)
