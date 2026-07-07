#!/usr/bin/env bash
# deploy/fetch_ci_evidence.sh
# ═══════════════════════════════════════════════════════════════
# Download a CI run's evidence-* artifacts into the local evidence/
# tree (Issue #804).
#
# Usage:
#   bash deploy/fetch_ci_evidence.sh              # newest CI run on this branch
#   bash deploy/fetch_ci_evidence.sh <run-id>     # a specific workflow run
#   bash deploy/fetch_ci_evidence.sh --pr <N>     # newest CI run of a PR's branch
#
# Result:
#   evidence/ci/<run-id>/evidence-tests-default/<ts>_<sha>/ctest_default.log
#   evidence/ci/<run-id>/evidence-coverage/<ts>_<sha>/{coverage.info,summary.txt,html/}
#   evidence/ci/<run-id>/evidence-static-analysis/...
#   evidence/ci/<run-id>/evidence-safety-audit/...
#
# Each run dir carries the manifest.txt written by the CI job itself
# (deploy/lib_evidence.sh), so provenance travels with the download.
#
# Requires: gh (authenticated).
# ═══════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-${PROJECT_DIR}/evidence}"

RUN_ID=""
case "${1:-}" in
    --pr)
        [[ -n "${2:-}" ]] || { echo "ERROR: --pr needs a PR number" >&2; exit 2; }
        BRANCH="$(gh pr view "$2" --json headRefName -q .headRefName)"
        ;;
    "")
        BRANCH="$(git -C "$PROJECT_DIR" branch --show-current)"
        ;;
    *)
        RUN_ID="$1"
        ;;
esac

if [[ -z "$RUN_ID" ]]; then
    # Repo slug from the origin remote (works for ssh + https + host aliases).
    REPO="$(git -C "$PROJECT_DIR" remote get-url origin \
        | sed -E 's#(\.git)$##; s#^.*[:/]([^/]+/[^/]+)$#\1#')"
    RUN_ID="$(gh api "repos/${REPO}/actions/runs?branch=${BRANCH}" \
        --jq '[.workflow_runs[] | select(.name=="CI")][0].id // empty')"
    if [[ -z "$RUN_ID" ]]; then
        echo "ERROR: no CI workflow run found for branch '${BRANCH}'" >&2
        exit 1
    fi
    echo "Newest CI run on '${BRANCH}': ${RUN_ID}"
fi

DEST="${EVIDENCE_ROOT}/ci/${RUN_ID}"
mkdir -p "$DEST"

# Enumerate the run's evidence-* artifact names via the REST API — this
# gh version has no `gh run download --pattern`, only repeatable `-n`.
REPO="${REPO:-$(git -C "$PROJECT_DIR" remote get-url origin \
    | sed -E 's#(\.git)$##; s#^.*[:/]([^/]+/[^/]+)$#\1#')}"
mapfile -t NAMES < <(gh api "repos/${REPO}/actions/runs/${RUN_ID}/artifacts" \
    --jq '.artifacts[] | select(.name | startswith("evidence-")) | .name')
if [[ ${#NAMES[@]} -eq 0 ]]; then
    echo "ERROR: run ${RUN_ID} has no evidence-* artifacts — run may still be" >&2
    echo "       in progress, predate #804, or artifacts expired (14 days)." >&2
    exit 1
fi

NAME_FLAGS=()
for n in "${NAMES[@]}"; do NAME_FLAGS+=(-n "$n"); done
echo "Downloading ${#NAMES[@]} evidence artifacts of run ${RUN_ID} → ${DEST}"
if ! gh run download "$RUN_ID" "${NAME_FLAGS[@]}" --dir "$DEST"; then
    echo "ERROR: download failed." >&2
    exit 1
fi

echo ""
echo "Fetched:"
find "$DEST" -maxdepth 1 -mindepth 1 -type d -printf '  %f\n' | sort
echo ""
echo "Browse: ${DEST}"
