"""Agent boundary enforcement command.

Replaces check-agent-boundaries.sh (180 lines).
Checks that changed files are within the allowed boundaries for the
detected agent role.
"""

from __future__ import annotations

import fnmatch

from orchestrator.config import ALWAYS_ALLOWED_PATTERNS, ROLE_BOUNDARY_PATTERNS, resolve_project_dir
from orchestrator.console import Console
from orchestrator.git import Git
from orchestrator.routing import role_for_branch


def check_boundaries(
    changed_files: list[str],
    role: str,
) -> list[str]:
    """Return list of files that violate role boundaries.

    A file is allowed if it matches any ALWAYS_ALLOWED pattern or any
    role-specific pattern from ROLE_BOUNDARY_PATTERNS.
    """
    role_patterns = ROLE_BOUNDARY_PATTERNS.get(role, [])
    violations = []

    for filepath in changed_files:
        if _matches_any(filepath, ALWAYS_ALLOWED_PATTERNS):
            continue
        if _matches_any(filepath, role_patterns):
            continue
        violations.append(filepath)

    return violations


def _matches_any(filepath: str, patterns: list[str]) -> bool:
    """Check if filepath matches any of the patterns.

    Supports:
      - Directory prefixes (ending with /): "docs/" matches "docs/foo.md"
      - Glob patterns: "tests/test_*perception*" matches "tests/test_perception_unit.cpp"
      - Exact matches: "CLAUDE.md" matches "CLAUDE.md"
    """
    for pattern in patterns:
        if pattern.endswith("/"):
            if filepath.startswith(pattern):
                return True
        elif "*" in pattern:
            if fnmatch.fnmatch(filepath, pattern):
                return True
        else:
            if filepath == pattern:
                return True
    return False


def run(
    io: Console | None = None,
    git: Git | None = None,
    *,
    base: str = "origin/main",
) -> int:
    """Check agent boundaries for changed files on current branch."""
    if io is None:
        io = Console()
    if git is None:
        git = Git(resolve_project_dir())

    changed = git.diff_name_only(base, "HEAD")
    if not changed:
        io.print(f"No changed files detected against {base}")
        return 0

    branch = git.current_branch()
    role = role_for_branch(branch)

    if not role:
        io.print("No agent role detected from branch name, skipping boundary check")
        return 0

    io.print(f"Branch: {branch}")
    io.print(f"Detected role: {role}")
    io.print(f"Base ref: {base}")
    io.print("")

    violations = check_boundaries(changed, role)

    if not violations:
        io.pass_(
            f"All {len(changed)} changed files are within {role} boundaries."
        )
        return 0

    io.fail(f"BOUNDARY VIOLATIONS for role {role}:")
    io.print("")
    for v in violations:
        io.print(f"  - {v}")
    io.print("")
    io.print(
        f"{len(violations)} file(s) outside allowed boundaries for {role}"
    )
    return 1
