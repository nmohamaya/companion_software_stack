"""Regression coverage for local CI failure reporting."""

import os
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def test_format_failure_reaches_local_runner_exit_status(tmp_path):
    formatter = tmp_path / "clang-format-18"
    formatter.write_text("#!/bin/sh\nexit 1\n")
    formatter.chmod(0o755)

    result = subprocess.run(
        ["bash", str(ROOT / "deploy/run_ci_local.sh"), "--job", "FMT"],
        cwd=ROOT,
        env={**os.environ, "PATH": f"{tmp_path}{os.pathsep}{os.environ['PATH']}"},
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 1
    assert "FAIL FMT" in result.stdout
    assert "PASS FMT" not in result.stdout


def test_quick_run_keeps_independent_jobs_after_a_failure(tmp_path):
    runner = tmp_path / "run_ci_local.sh"
    shutil.copyfile(ROOT / "deploy/run_ci_local.sh", runner)
    (tmp_path / "ci_job.sh").write_text(
        '#!/bin/bash\nset -e\n'
        'if [[ "$1" == FMT ]]; then false; echo "masked failure"; fi\n'
        'echo "ran $1"\n'
    )

    result = subprocess.run(
        ["bash", str(runner), "--quick"], text=True, capture_output=True, check=False,
    )

    assert result.returncode == 1
    assert "masked failure" not in result.stdout
    assert "PASS DOC" in result.stdout
    assert "PASS BUILD" in result.stdout
    assert "Failed jobs: FMT" in result.stderr
