#!/usr/bin/env bash
# Run the same repository checks as GitHub Actions with independent job results.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
all_jobs=(FMT DOC BUILD ASAN TSAN UBSAN COV CPPCHECK AUDIT PYTHON PR_SIZE BOUNDARIES)
selected=("${all_jobs[@]}")

case "${1:-}" in
    '') ;;
    --quick|-q) selected=(FMT DOC BUILD) ;;
    --job|-j)
        case "${2:-}" in
            FMT|DOC|BUILD|ASAN|TSAN|UBSAN|COV|CPPCHECK|AUDIT|PYTHON|PR_SIZE|BOUNDARIES|TIDY|COVDELTA)
                selected=("$2") ;;
            *) echo "Unknown CI job: ${2:-<missing>}" >&2; exit 2 ;;
        esac
        shift
        ;;
    --help|-h)
        echo "Usage: bash deploy/run_ci_local.sh [--quick | --job TAG]"
        echo "Full run: ${all_jobs[*]}"
        echo "Quick run: FMT DOC BUILD"
        echo "Standalone advisory jobs: TIDY COVDELTA"
        exit 0
        ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then
    echo "Use --help for supported arguments" >&2
    exit 2
fi

failed=()
for job in "${selected[@]}"; do
    echo "=== $job ==="
    launcher=()
    if [[ "$job" == TSAN ]]; then
        mmap_bits="$(sysctl -n vm.mmap_rnd_bits 2>/dev/null || true)"
        if [[ ! "$mmap_bits" =~ ^[0-9]+$ ]] || (( mmap_bits > 28 )); then
            echo "TSan runner: using process-local ASLR workaround (Actions uses vm.mmap_rnd_bits=28)"
            launcher=(setarch "$(uname -m)" -R)
        fi
    fi
    # A separate Bash process preserves errexit inside the job while allowing
    # the orchestrator to keep running other independent jobs after failure.
    if "${launcher[@]}" bash "$script_dir/ci_job.sh" "$job"; then
        echo "PASS $job"
    else
        failed+=("$job")
        echo "FAIL $job"
    fi
done

if (( ${#failed[@]} > 0 )); then
    echo "Failed jobs: ${failed[*]}" >&2
    exit 1
fi
echo "Passed jobs: ${selected[*]}"
