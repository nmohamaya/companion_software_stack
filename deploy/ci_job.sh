#!/usr/bin/env bash
# Checks shared by GitHub Actions and run_ci_local.sh.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source deploy/lib_evidence.sh

build_root="$(realpath -m "${CI_BUILD_ROOT:-build-ci}")"
base="${CI_BASE:-origin/main}"
jobs="${CI_JOBS:-$(nproc)}"
zenoh_version=1.7.2
tsan_exclude='Zenoh|Mavlink|Yolo|Liveliness|MessageBus|TopicResolver|ProcessContext|IpcRoundTrip|FaultCascade'

# Extract the same released C package used by Actions. Keep the paired C++
# headers in this prefix so an installed development SDK cannot shadow them.
prepare_zenoh() {
    local scratch="${build_root}/deps/zenoh-${zenoh_version}"
    zenoh_prefix="${scratch}/prefix/usr"
    mkdir -p "$scratch"
    (
        flock 9
        if [[ -f "$scratch/ready" ]]; then
            exit 0
        fi
        local archive="libzenohc-${zenoh_version}-x86_64-unknown-linux-gnu-debian.zip"
        wget -nv -O "$scratch/zenoh.zip" \
            "https://github.com/eclipse-zenoh/zenoh-c/releases/download/${zenoh_version}/${archive}"
        unzip -oq "$scratch/zenoh.zip" -d "$scratch/debs"
        for package in "$scratch"/debs/*.deb; do
            dpkg-deb -x "$package" "$scratch/prefix"
        done
        if [[ ! -d "$scratch/zenoh-cpp/.git" ]]; then
            git clone --depth 1 --branch "$zenoh_version" \
                https://github.com/eclipse-zenoh/zenoh-cpp.git "$scratch/zenoh-cpp"
        fi
        cmake -S "$scratch/zenoh-cpp" -B "$scratch/cpp-build" \
            -DZENOHCXX_ZENOHC=ON -DZENOHCXX_ZENOHPICO=OFF \
            -DZENOHCXX_ENABLE_TESTS=OFF -DZENOHCXX_ENABLE_EXAMPLES=OFF \
            -Dzenohc_DIR="$zenoh_prefix/lib/cmake/zenohc" \
            -DCMAKE_INSTALL_PREFIX="$zenoh_prefix"
        cmake --install "$scratch/cpp-build"
        touch "$scratch/ready"
    ) 9>"$scratch/install.lock"
}

build_variant() {
    local variant="$1"
    local build_type=Debug
    local cxx_flags
    local -a flags=()
    case "$variant" in
        default) build_type=Release ;;
        asan) flags=(-DENABLE_ASAN=ON) ;;
        tsan) flags=(-DENABLE_TSAN=ON) ;;
        ubsan) flags=(-DENABLE_UBSAN=ON) ;;
        coverage) flags=(-DENABLE_COVERAGE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON) ;;
    esac
    prepare_zenoh
    cxx_flags="-isystem \"$zenoh_prefix/include\""
    if [[ "$variant" != coverage ]]; then
        cxx_flags="-Werror -Wall -Wextra $cxx_flags"
    fi

    cmake -S . -B "$build_root/$variant" \
        -DCMAKE_BUILD_TYPE="$build_type" \
        -DCMAKE_CXX_FLAGS="$cxx_flags" \
        -Dzenohc_DIR="$zenoh_prefix/lib/cmake/zenohc" \
        -DCMAKE_BUILD_RPATH="$zenoh_prefix/lib" \
        -DALLOW_INSECURE_ZENOH=ON \
        -DCI_DEPENDENCY_PROFILE=ON \
        "${flags[@]}"
    cmake --build "$build_root/$variant" --parallel "$jobs"
}

run_tests() {
    local variant="$1"
    local evidence
    local -a filters=()
    if [[ "$variant" == tsan ]]; then
        filters=(-E "$tsan_exclude")
    fi
    evidence="$(evidence_run_dir tests "ci-$variant")"
    ctest --test-dir "$build_root/$variant" --no-tests=error \
        --output-on-failure --parallel "$jobs" "${filters[@]}" 2>&1 \
        | tee "$evidence/ctest_$variant.log"
}

advisory() {
    if ! "$@"; then
        echo "::warning::Advisory check reported findings: $*"
    fi
}

coverage_delta() {
    python3.12 deploy/coverage_delta.py --coverage "$build_root/coverage/coverage.info" \
        --base "$base"
}

coverage() {
    local coverage_dir="$build_root/coverage"
    local evidence summary line_coverage
    build_variant coverage
    advisory env CI_COMPILE_DB="$coverage_dir" bash deploy/run_clang_tidy.sh --changed "$base"
    lcov --zerocounters --directory "$coverage_dir"
    run_tests coverage
    lcov --capture --directory "$coverage_dir" --output-file "$coverage_dir/coverage-raw.info" \
        --ignore-errors mismatch,negative --rc lcov_branch_coverage=1
    evidence="$(evidence_run_dir coverage ci-coverage)"
    lcov --remove "$coverage_dir/coverage-raw.info" \
        '/usr/*' "$zenoh_prefix/*" '*/googletest/*' '*/gtest/*' '*/test/*' '*/tests/*' \
        '*_test.cpp' '*_test.h' \
        --output-file "$coverage_dir/coverage.info" \
        --ignore-errors unused --rc lcov_branch_coverage=1
    cp "$coverage_dir/coverage.info" "$evidence/coverage.info"
    lcov --list "$coverage_dir/coverage.info" --rc lcov_branch_coverage=1
    advisory coverage_delta
    summary="$(LC_ALL=C lcov --summary "$coverage_dir/coverage.info" 2>&1)" || true
    printf '%s\n' "$summary" > "$evidence/summary.txt"
    line_coverage="$(sed -n 's/.*lines\.\.\.\.\.\.: \([0-9.]*\)%.*/\1/p' <<< "$summary")"
    if [[ ! "$line_coverage" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "::error::Failed to parse line coverage"
        echo "$summary"
        return 1
    fi
    echo "Line coverage: ${line_coverage}%"
    if LC_ALL=C awk -v pct="$line_coverage" 'BEGIN { exit !(pct < 70.0) }'; then
        echo "::error::Line coverage ${line_coverage}% is below 70% threshold"
        return 1
    fi
    genhtml "$coverage_dir/coverage.info" --output-directory "$evidence/html" \
        --title "Companion Stack Coverage" --rc genhtml_branch_coverage=1
}

cppcheck_job() {
    local evidence
    evidence="$(evidence_run_dir static-analysis ci-cppcheck)"
    bash deploy/run_cppcheck.sh 2>&1 | tee "$evidence/cppcheck.log"
}

python_tests() {
    python3.12 -m venv "$build_root/python"
    "$build_root/python/bin/python" -m pip install '.[dev]'
    "$build_root/python/bin/python" -m pytest tests/test_orchestrator/ -v --tb=short
}

case "${1:-}" in
    FMT)
        find common process[1-7]_* tests tools \( -name '*.h' -o -name '*.cpp' \) -print0 \
            | xargs -0 clang-format-18 --dry-run --Werror
        ;;
    DOC)
        git merge-base "$base" HEAD > /dev/null
        bash deploy/lint_docs_ssot.sh --changed "$base"
        ;;
    BUILD) build_variant default; run_tests default ;;
    ASAN) build_variant asan; run_tests asan ;;
    TSAN) build_variant tsan; run_tests tsan ;;
    UBSAN) build_variant ubsan; run_tests ubsan ;;
    COV) coverage ;;
    CPPCHECK) advisory cppcheck_job ;;
    AUDIT) advisory bash deploy/safety_audit.sh ;;
    PYTHON) python_tests ;;
    PR_SIZE|BOUNDARIES) bash deploy/ci_pr_check.sh "$1" "$base" "${CI_BRANCH:-$(git branch --show-current)}" ;;
    TIDY) advisory bash deploy/run_clang_tidy.sh --changed "$base" ;;
    COVDELTA) advisory coverage_delta ;;
    *) echo "Unknown CI job: ${1:-<missing>}" >&2; exit 2 ;;
esac
