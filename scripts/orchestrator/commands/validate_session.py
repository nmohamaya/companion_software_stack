"""Session validation command — 8-check post-session hallucination detector.

Replaces validate-session.sh (304 lines).
Checks: build, test count, test execution, includes, diff sanity,
        config key consistency, symbol resolution, diff sanity (extended).
"""

from __future__ import annotations

import json
import re
import subprocess
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
    """Run 8-check post-session validation.

    Returns 0 on pass/warn, 1 on failure.
    """
    if io is None:
        io = Console()
    if git is None:
        git = Git(resolve_project_dir())
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
    io.print("[1/8] Build verification")
    if not build.has_build_dir():
        fail("No build/ directory — run cmake first")
    else:
        try:
            success, output, warning_count = build.build()
            if success:
                if warning_count > 0:
                    warn(f"Build succeeded but {warning_count} warning(s) detected")
                else:
                    pass_("Build succeeded (zero warnings)")
            else:
                fail("Build failed")
        except Exception as e:
            fail(f"Build error: {e}")
    io.print("")

    # ── 2. Test count ───────────────────────────────────────────────────
    io.print("[2/8] Test count verification")
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
    io.print("[3/8] Test execution")
    if not build.has_build_dir():
        fail("Cannot run tests — no build directory")
    else:
        try:
            test_output, test_exit = build.run_tests()
            parsed = build.parse_test_results(test_output)
            if "failures" in parsed and "0 tests" not in parsed["failures"]:
                fail(f"Test failures: {parsed['failures']}")
            elif "pass_rate" in parsed:
                pass_(f"Tests completed: {parsed['pass_rate']}")
            elif test_exit == 0:
                pass_("All tests passed")
            else:
                fail("Tests failed")
        except Exception as e:
            fail(f"Test error: {e}")
    io.print("")

    # ── 4. Include verification ─────────────────────────────────────────
    io.print("[4/8] Include verification")
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
    io.print("[5/8] Diff sanity")
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

    # ── 6. Config key consistency ─────────────────────────────────────
    io.print("[6/8] Config key consistency")
    try:
        diff = git.diff_full("main", branch)
        # Find new cfg.get<> calls in added lines
        new_keys: list[str] = []
        for line in diff.splitlines():
            if not line.startswith("+"):
                continue
            # Match cfg.get<Type>("key" or cfg.get<Type>("section.key"
            for m in re.finditer(r'cfg\.get<[^>]*>\(\s*"([^"]+)"', line):
                new_keys.append(m.group(1))
            # Match cfg_key:: constants used as get arguments
            for m in re.finditer(r'cfg_key::(\w+)::(\w+)', line):
                new_keys.append(f"{m.group(1)}.{m.group(2)}")

        if not new_keys:
            pass_("No new config key references in diff")
        else:
            new_keys = sorted(set(new_keys))
            # Load default.json and check keys exist
            default_json = project_dir / "config" / "default.json"
            missing_keys: list[str] = []
            if default_json.exists():
                try:
                    config_data = json.loads(default_json.read_text())
                    for key in new_keys:
                        parts = key.split(".")
                        node = config_data
                        found = True
                        for part in parts:
                            if isinstance(node, dict) and part in node:
                                node = node[part]
                            else:
                                found = False
                                break
                        if not found:
                            missing_keys.append(key)
                except json.JSONDecodeError:
                    warn("Could not parse config/default.json")
                    missing_keys = []

            if not missing_keys:
                pass_(
                    f"All {len(new_keys)} config key(s) found in default.json"
                )
            else:
                warn(
                    f"{len(missing_keys)} config key(s) not in default.json:"
                )
                for mk in missing_keys:
                    io.print(f"    - {mk}")
    except Exception:
        warn("Could not check config key consistency")
    io.print("")

    # ── 7. Symbol resolution (spot-check) ─────────────────────────────
    io.print("[7/8] Symbol resolution (spot-check)")
    try:
        diff = git.diff_full("main", branch)
        # Find new class instantiations and method calls on common types
        # Focus on make_unique/make_shared — common hallucination points
        phantom_symbols: list[str] = []
        new_instantiations: list[str] = []
        for line in diff.splitlines():
            if not line.startswith("+"):
                continue
            # std::make_unique<ClassName> or std::make_shared<ClassName>
            for m in re.finditer(
                r'(?:make_unique|make_shared)<(\w+)>', line
            ):
                new_instantiations.append(m.group(1))

        new_instantiations = sorted(set(new_instantiations))
        if new_instantiations:
            # Spot-check: grep for class/struct definitions
            search_dirs = [
                str(project_dir / d)
                for d in (
                    "common", "process1_video_capture",
                    "process2_perception", "process3_slam_vio_nav",
                    "process4_mission_planner", "process5_comms",
                    "process6_payload_manager", "process7_system_monitor",
                )
                if (project_dir / d).exists()
            ]
            for sym in new_instantiations[:10]:  # cap at 10 checks
                # Skip standard library / third-party types
                if sym in (
                    "string", "vector", "mutex", "thread",
                    "promise", "future",
                ):
                    continue
                result = subprocess.run(
                    ["grep", "-rl", f"class {sym}", *search_dirs],
                    capture_output=True, text=True, timeout=10,
                )
                if not result.stdout.strip():
                    # Also check for struct
                    result2 = subprocess.run(
                        ["grep", "-rl", f"struct {sym}", *search_dirs],
                        capture_output=True, text=True, timeout=10,
                    )
                    if not result2.stdout.strip():
                        phantom_symbols.append(sym)

        if not new_instantiations:
            pass_("No new class instantiations to verify")
        elif not phantom_symbols:
            pass_(
                f"Spot-checked {len(new_instantiations)} instantiation(s) "
                f"— all resolve"
            )
        else:
            warn(
                f"{len(phantom_symbols)} class(es) not found in codebase:"
            )
            for ps in phantom_symbols:
                io.print(f"    - {ps}")
    except Exception:
        warn("Could not run symbol resolution check")
    io.print("")

    # ── 8. Extended diff sanity ───────────────────────────────────────
    io.print("[8/8] Extended diff sanity")
    try:
        changed_files = git.diff_name_only("main", branch)
        issues: list[str] = []

        # 8a. Test files without TEST macros
        new_test_files = [
            f for f in changed_files
            if f.startswith("tests/") and f.endswith(".cpp")
        ]
        for tf in new_test_files:
            tf_path = project_dir / tf
            if tf_path.exists():
                content = tf_path.read_text()
                if "TEST(" not in content and "TEST_F(" not in content:
                    issues.append(
                        f"Test file {tf} has no TEST/TEST_F macros"
                    )

        # 8b. New .cpp files not in any CMakeLists.txt
        new_cpp = [
            f for f in changed_files
            if f.endswith(".cpp") and not f.startswith("tests/")
        ]
        for cpp_file in new_cpp:
            basename = Path(cpp_file).name
            # Search CMakeLists.txt in the file's directory and parent
            cpp_dir = project_dir / Path(cpp_file).parent
            found_in_cmake = False
            for cmake_dir in (cpp_dir, cpp_dir.parent):
                cmake_path = cmake_dir / "CMakeLists.txt"
                if cmake_path.exists() and basename in cmake_path.read_text():
                    found_in_cmake = True
                    break
            if not found_in_cmake:
                issues.append(
                    f"New source {cpp_file} not found in CMakeLists.txt"
                )

        # 8c. Deleted files still included elsewhere
        diff = git.diff_full("main", branch)
        deleted_files = re.findall(
            r'^--- a/(.+)\n\+\+\+ /dev/null', diff, re.MULTILINE
        )
        for df in deleted_files:
            header_name = Path(df).name
            if header_name.endswith(".h"):
                result = subprocess.run(
                    ["grep", "-rl", f'#include.*{header_name}',
                     str(project_dir / "common"),
                     str(project_dir / "tests")],
                    capture_output=True, text=True, timeout=10,
                )
                # Exclude the deleted file itself from matches
                remaining = [
                    l for l in result.stdout.strip().splitlines()
                    if l and df not in l
                ]
                if remaining:
                    issues.append(
                        f"Deleted {df} still #included in: "
                        f"{', '.join(Path(r).name for r in remaining[:3])}"
                    )

        if not issues:
            pass_("Extended diff sanity checks passed")
        else:
            for issue in issues:
                warn(issue)
    except Exception:
        warn("Could not run extended diff sanity checks")
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
