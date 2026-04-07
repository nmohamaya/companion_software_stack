"""Session validation command — 5-check post-session hallucination detector.

Replaces validate-session.sh (304 lines).
Checks: build, test count, test execution, includes, diff sanity.
"""

from __future__ import annotations

import re
from pathlib import Path

from orchestrator.build import BuildSystem
from orchestrator.config import resolve_project_dir
from orchestrator.console import Console
from orchestrator.git import Git
from orchestrator.github import GitHub


def run(
    io: Console | None = None,
    git: Git | None = None,
    github: GitHub | None = None,
    build: BuildSystem | None = None,
    *,
    branch: str = "",
) -> int:
    """Run 5-check post-session validation.

    Returns 0 on pass/warn, 1 on failure.
    """
    if io is None:
        io = Console()
    if git is None:
        git = Git()
    if github is None:
        github = GitHub()

    project_dir = resolve_project_dir()
    if build is None:
        build = BuildSystem(project_dir)

    if not branch:
        branch = git.current_branch()

    io.header(f"Session Validation: {branch}")

    pass_count = 0
    warn_count = 0
    fail_count = 0

    def pass_(msg: str) -> None:
        nonlocal pass_count
        io.pass_(msg)
        pass_count += 1

    def warn(msg: str) -> None:
        nonlocal warn_count
        io.warn(msg)
        warn_count += 1

    def fail(msg: str) -> None:
        nonlocal fail_count
        io.fail(msg)
        fail_count += 1

    # ── 1. Build verification ──────────────────────────────────────────
    io.print("[1/5] Build verification")
    if not build.has_build_dir():
        fail("No build/ directory — run cmake first")
    else:
        try:
            result = build.build()
            if result.returncode == 0:
                if "warning:" in result.stdout.lower():
                    warn("Build succeeded but warnings detected")
                else:
                    pass_("Build succeeded (zero warnings)")
            else:
                fail("Build failed")
        except Exception as e:
            fail(f"Build error: {e}")
    io.print("")

    # ── 2. Test count ───────────────────────────────────────────────────
    io.print("[2/5] Test count verification")
    actual = build.test_count() if build.has_build_dir() else 0
    expected = build.expected_test_count()

    if not expected:
        warn(f"Test count: {actual} (could not determine baseline)")
    elif actual == expected:
        pass_(f"Test count: {actual} matches expected {expected}")
    elif actual > expected:
        pass_(f"Test count: {actual} ({expected} expected — tests added)")
    else:
        fail(f"Test count: {actual} < expected {expected} — tests may be missing")
    io.print("")

    # ── 3. Test execution ───────────────────────────────────────────────
    io.print("[3/5] Test execution")
    if not build.has_build_dir():
        fail("Cannot run tests — no build directory")
    else:
        try:
            test_result = build.run_tests()
            parsed = build.parse_test_results(test_result.stdout)
            if parsed.get("failed", 0) > 0:
                fail(f"Test failures: {parsed['failed']} tests failed")
            elif parsed.get("passed_pct") == 100:
                pass_(f"All tests passed ({parsed.get('total', '?')} tests)")
            else:
                pass_(f"Tests completed: {parsed.get('passed_pct', '?')}% passed")
        except Exception as e:
            fail(f"Test error: {e}")
    io.print("")

    # ── 4. Include verification ─────────────────────────────────────────
    io.print("[4/5] Include verification")
    try:
        diff = git.diff_full("main", branch)
        new_includes = re.findall(r'^\+.*#include\s+"([^"]+)"', diff, re.MULTILINE)
        new_includes = sorted(set(new_includes))

        if not new_includes:
            pass_("No new #include directives in diff")
        else:
            # Check if includes can be found
            system_prefixes = (
                "zenoh/", "Eigen/", "opencv2/", "gz/", "mavsdk/",
                "spdlog/", "nlohmann/", "gtest/", "gmock/"
            )
            missing = []
            for inc in new_includes:
                if inc.startswith(system_prefixes):
                    continue
                # Search in project dirs
                found = False
                for search_dir in (
                    "common", "process1_video_capture", "process2_perception",
                    "process3_slam_vio_nav", "process4_mission_planner",
                    "process5_comms", "process6_payload_manager",
                    "process7_system_monitor", "tests",
                ):
                    search_path = project_dir / search_dir
                    if search_path.exists():
                        matches = list(search_path.rglob(f"*{Path(inc).name}"))
                        if matches:
                            found = True
                            break
                if not found:
                    missing.append(inc)

            if not missing:
                pass_(f"All {len(new_includes)} new includes resolved")
            else:
                fail(f"{len(missing)} include(s) not found:")
                for mi in missing:
                    io.print(f"    - {mi}")
    except Exception:
        warn("Could not check includes (no diff available)")
    io.print("")

    # ── 5. Diff sanity ──────────────────────────────────────────────────
    io.print("[5/5] Diff sanity")
    try:
        prs = github.pr_list_for_branch(branch)
        if not prs:
            warn(f"No PR found for branch '{branch}'")
        else:
            pr = prs[0]
            io.info(f"PR #{pr.number} found for branch")

            diff_stat = git.diff_stat("main", branch)
            io.info(f"Diff stats: {diff_stat.splitlines()[-1] if diff_stat else 'unknown'}")

            if pr.body:
                # Check for phantom file references
                mentioned = re.findall(r"[a-zA-Z0-9_/]+\.(?:cpp|h|md)", pr.body)
                diff_files = git.diff_name_only("main", branch)
                phantom = [
                    f for f in set(mentioned)
                    if f not in diff_files and not (project_dir / f).exists()
                ]
                if not phantom:
                    pass_("PR body file references check out")
                else:
                    warn(f"PR body mentions {len(phantom)} file(s) not in diff or repo")
            else:
                warn("PR body is empty")
    except Exception:
        warn("Could not check PR diff sanity")
    io.print("")

    # ── Verdict ─────────────────────────────────────────────────────────
    io.header("Verdict")
    io.print(f"  Passed: {pass_count}")
    io.print(f"  Warns:  {warn_count}")
    io.print(f"  Failed: {fail_count}")
    io.print("")

    if fail_count > 0:
        io.fail(f"FAIL — {fail_count} check(s) failed. Investigate before merging.")
        return 1
    if warn_count > 0:
        io.warn(f"WARN — All critical checks passed, {warn_count} warning(s).")
        return 0
    io.pass_("PASS — All checks passed.")
    return 0
