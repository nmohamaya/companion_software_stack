#!/usr/bin/env bash
# deploy/lib_evidence.sh
# ═══════════════════════════════════════════════════════════════
# Shared helper for the evidence/ report store (Issue #804).
#
# Every generated assurance report (safety audit, test logs, coverage,
# static analysis) lands under a single canonical folder with consistent
# naming and provenance:
#
#   evidence/<category>/<YYYY-MM-DD_HHMMSSZ>_<git-short-sha>/
#     manifest.txt          ← provenance stamp (written by this lib)
#     <report files...>     ← written by the caller
#   evidence/<category>/latest  → newest run dir (convenience symlink)
#
# evidence/ is fully gitignored; CI uploads each category as a
# workflow artifact.  Layout spec: docs/how-to/EVIDENCE.md.
#
# Usage (from any deploy/tests script):
#   source "$(dirname "${BASH_SOURCE[0]}")/lib_evidence.sh"   # adjust path
#   dir=$(evidence_run_dir safety-audit)
#   my_tool > "${dir}/report.md"
#
# The helper never fails the caller: if the evidence dir cannot be
# created (read-only checkout, exotic CI), it falls back to a temp dir
# and warns on stderr — report generation must not break the build.
# ═══════════════════════════════════════════════════════════════

# Repo root = parent of the directory containing this lib (deploy/).
_EVIDENCE_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-$(dirname "${_EVIDENCE_LIB_DIR}")/evidence}"

# evidence_run_dir <category> [generator-label]
# Creates evidence/<category>/<ts>_<sha>/ with a manifest.txt, refreshes
# the category's `latest` symlink, and echoes the absolute run-dir path.
evidence_run_dir() {
    local category="$1"
    local generator="${2:-${0##*/}}"

    local ts sha dirty branch run_dir
    ts="$(date -u +%Y-%m-%d_%H%M%SZ)"
    sha="$(git -C "${_EVIDENCE_LIB_DIR}" rev-parse --short HEAD 2>/dev/null || echo nogit)"
    branch="$(git -C "${_EVIDENCE_LIB_DIR}" branch --show-current 2>/dev/null || echo unknown)"
    if git -C "${_EVIDENCE_LIB_DIR}" diff --quiet HEAD 2>/dev/null; then
        dirty="clean"
    else
        dirty="dirty"
    fi

    run_dir="${EVIDENCE_ROOT}/${category}/${ts}_${sha}"
    if ! mkdir -p "${run_dir}" 2>/dev/null; then
        run_dir="$(mktemp -d "/tmp/evidence-${category}-XXXXXX")"
        echo "WARN [lib_evidence] cannot write ${EVIDENCE_ROOT}; using ${run_dir}" >&2
    fi

    {
        echo "generated_utc: ${ts}"
        echo "git_sha:       ${sha}"
        echo "git_branch:    ${branch}"
        echo "git_tree:      ${dirty}"
        echo "generator:     ${generator}"
        echo "host:          $(hostname 2>/dev/null || echo unknown)"
        echo "kernel:        $(uname -sr 2>/dev/null || echo unknown)"
        # CI provenance when running under GitHub Actions.
        # (if-statements, not `[[ ]] &&` — a false && list here would end the
        #  redirection group non-zero and kill `set -e` callers locally.)
        if [[ -n "${GITHUB_RUN_ID:-}" ]]; then
            echo "github_run:    ${GITHUB_REPOSITORY:-?}/actions/runs/${GITHUB_RUN_ID}"
        fi
        if [[ -n "${GITHUB_JOB:-}" ]]; then
            echo "github_job:    ${GITHUB_JOB}"
        fi
    } > "${run_dir}/manifest.txt"

    # Convenience symlink — local only. Skipped in CI: upload-artifact
    # follows symlinks, so `latest` would duplicate the run dir's content
    # inside every evidence artifact.
    if [[ -z "${GITHUB_ACTIONS:-}" ]]; then
        ln -sfn "${run_dir}" "${EVIDENCE_ROOT}/${category}/latest" 2>/dev/null || true
    fi

    echo "${run_dir}"
}
