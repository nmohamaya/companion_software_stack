# CI Pipeline Setup Guide

> How the GitHub Actions CI pipeline is structured, why each piece exists,
> and how to modify it. Aimed at DevOps newcomers and contributors adding
> new CI jobs.

**Workflow file:** [`.github/workflows/ci.yml`](../.github/workflows/ci.yml)  
**Runner:** `ubuntu-24.04` (GitHub-hosted)  
**Trigger:** push to `main`/`develop`, pull requests to `main`

---

## Table of Contents

- [CI Pipeline Setup Guide](#ci-pipeline-setup-guide)
  - [Table of Contents](#table-of-contents)
  - [Pipeline Overview](#pipeline-overview)
  - [Job Dependency Graph](#job-dependency-graph)
  - [Job 1 — format-check (Fast Gate)](#job-1--format-check-fast-gate)
  - [Job 2 — build-and-test (7-Leg Matrix)](#job-2--build-and-test-7-leg-matrix)
    - [Why both Release (plain) and Debug (sanitizer) legs?](#why-both-release-plain-and-debug-sanitizer-legs)
    - [Matrix naming](#matrix-naming)
    - [Steps breakdown](#steps-breakdown)
    - [Why `fail-fast: false`?](#why-fail-fast-false)
  - [Job 3 — coverage](#job-3--coverage)
  - [CMake Build Options](#cmake-build-options)
  - [Zenoh Installation on CI](#zenoh-installation-on-ci)
  - [Sanitizer Details](#sanitizer-details)
    - [AddressSanitizer (ASan)](#addresssanitizer-asan)
    - [ThreadSanitizer (TSan)](#threadsanitizer-tsan)
    - [UndefinedBehaviorSanitizer (UBSan)](#undefinedbehaviorsanitizer-ubsan)
  - [Known Issues \& Workarounds](#known-issues--workarounds)
  - [Adding a New CI Job](#adding-a-new-ci-job)
  - [Adding a New Matrix Leg](#adding-a-new-matrix-leg)
  - [Local Reproduction](#local-reproduction)
    - [Automated: `deploy/run_ci_local.sh`](#automated-deployrun_ci_localsh)
      - [Available Job Tags](#available-job-tags)
    - [Manual: Reproducing a Single CI Leg](#manual-reproducing-a-single-ci-leg)

---

## Pipeline Overview

The CI pipeline has **4 jobs** totalling **10 parallel runners** at peak:

| Job | Purpose | Runners | Gate? |
|-----|---------|---------|-------|
| `format-check` | Verify all C++ files match `.clang-format` | 1 | Yes — blocks `build-and-test` |
| `build-and-test` | Compile + run tests for each backend × sanitizer combo | 7 | No — `fail-fast: false`, all legs run |
| `coverage` | SHM coverage: build with gcov, run tests, upload lcov report | 1 | No — independent |
| `coverage-zenoh` | Zenoh coverage: same as above with Zenoh backend enabled | 1 | No — independent |

**Total wall-clock time:** ~4–6 minutes (format-check ~30 s, then build matrix ~3–5 min in parallel).

---

## Job Dependency Graph

```
                  ┌─────────────┐
                  │format-check │  ← Fast gate (~30 s)
                  └──────┬──────┘
                         │ needs: format-check
          ┌──────────────┼──────────────┐
          ▼              ▼              ▼
   ┌─────────────┐ ┌──────────┐ ┌──────────┐
   │ build (shm) │ │build(shm,│ │build(shm,│  ... (7 legs total)
   │             │ │  asan)   │ │  tsan)   │
   └─────────────┘ └──────────┘ └──────────┘

   ┌────────────────┐   ┌──────────────────┐
   │ coverage (shm) │   │ coverage (zenoh) │  ← Independent, no gate dependency
   └────────────────┘   └──────────────────┘
```

---

## Job 1 — format-check (Fast Gate)

**Purpose:** Catch formatting drift before spending CI minutes on builds.

**How it works:**
1. Installs `clang-format-18` (pinned version for reproducibility).
2. Runs `find` across `common/`, `process[1-7]_*/`, and `tests/` for `*.h` and `*.cpp` files.
3. Invokes `clang-format-18 --dry-run --Werror` — exits non-zero if any file differs from the formatted version.

**Key details:**
- The `find` command uses `\( -name '*.h' -o -name '*.cpp' \)` with explicit parentheses — without them, `find`'s operator precedence causes only `.cpp` files to be checked (see PR #72 review).
- Uses `-print0 | xargs -0` for safe handling of paths with spaces.
- The `build-and-test` matrix has `needs: format-check`, so formatting failures short-circuit the entire pipeline.

**Config file:** [`.clang-format`](../.clang-format) (4-space indent, K&R braces, 100-col limit, left pointer alignment).

**Fixing a format failure locally:**
```bash
# Check which files need formatting:
find common process[1-7]_* tests \( -name '*.h' -o -name '*.cpp' \) -print0 \
  | xargs -0 clang-format-18 --dry-run --Werror

# Auto-fix all files:
find common process[1-7]_* tests \( -name '*.h' -o -name '*.cpp' \) -print0 \
  | xargs -0 clang-format-18 -i
```

---

## Job 2 — build-and-test (7-Leg Matrix)

The build matrix tests every combination of IPC backend and sanitizer that makes sense:

| # | `ipc_backend` | `sanitizer` | Build Type | Notes |
|---|---------------|-------------|------------|-------|
| 1 | `shm` | `none` | Release | Baseline — POSIX SHM, no instrumentation |
| 2 | `zenoh` | `none` | Release | Zenoh backend, pre-built zenohc 1.7.2 debs |
| 3 | `shm` | `asan` | Debug | AddressSanitizer — memory errors, leaks |
| 4 | `shm` | `tsan` | Debug | ThreadSanitizer — data races |
| 5 | `shm` | `ubsan` | Debug | UndefinedBehaviorSanitizer |
| 6 | `zenoh` | `asan` | Debug | ASan on Zenoh code paths |
| 7 | `zenoh` | `ubsan` | Debug | UBSan on Zenoh code paths |

**Why no Zenoh + TSan?** The pre-built `zenohc` library is not compiled with TSan instrumentation. TSan flags false positives in Zenoh's internal threading, making results unreliable. See [CI_ISSUES.md CI-005](CI_ISSUES.md#ci-005-tsan-build-fails--atomic_thread_fence-not-supported-with--fsanitize-thread).

### Why both Release (plain) and Debug (sanitizer) legs?

The plain and sanitizer legs build in different modes and catch **different classes of problems**. Neither is a superset of the other.

| Leg type | Build mode | What it catches |
|----------|-----------|------------------|
| **Plain** (legs 1–2) | `Release -O2` | Optimisation-triggered warnings (e.g. `-Wstringop-truncation`, `-Wmaybe-uninitialized`), Release-only codepaths (`#ifdef NDEBUG`, `assert()` removal), and ensures the **actual shipping binary** compiles cleanly |
| **Sanitizer** (legs 3–7) | `Debug -O0` | Memory errors (ASan), data races (TSan), undefined behaviour (UBSan) — runtime correctness issues that `-O0` makes easier to diagnose with precise stack traces |

**Why the difference matters:**

1. **GCC performs deeper static analysis at `-O2`.** Interprocedural optimisations like inlining and constant propagation let GCC track values across function boundaries, enabling warnings that are invisible at `-O0`. Example: `-Wstringop-truncation` fires in Release when GCC statically determines a `strncpy` source is longer than the destination — this analysis is skipped at `-O0` (see [CI_ISSUES.md CI-008](CI_ISSUES.md)).

2. **Release and Debug builds produce different binaries.** `NDEBUG` is defined in Release, disabling `assert()`. Code behind `#ifndef NDEBUG` (debug logging, extra validation) only compiles in Debug. Both paths need testing.

3. **Sanitizers require `-O0`** (or `-O1` at most) for accurate diagnostics. At `-O2`, the compiler reorders, inlines, and eliminates code, making sanitizer reports harder to interpret and occasionally producing false positives.

In short: plain legs answer "does our shipping binary compile and pass tests?", while sanitizer legs answer "does our code have memory/thread/UB correctness issues?". Both are necessary.

### Matrix naming

The `name` field uses a GitHub Actions expression to produce clean labels:
```yaml
name: build (${{ matrix.ipc_backend }}${{ matrix.sanitizer != 'none' && format(', {0}', matrix.sanitizer) || '' }})
```
This produces names like `build (shm)`, `build (shm, asan)`, `build (zenoh, ubsan)`.

### Steps breakdown

1. **Checkout** — `actions/checkout@v4`
2. **Install dependencies** — apt packages: `build-essential cmake libspdlog-dev libeigen3-dev nlohmann-json3-dev libgtest-dev`
3. **Install Zenoh** (conditional) — Pre-built zenohc debs + zenoh-cpp headers (see [Zenoh Installation](#zenoh-installation-on-ci))
4. **Fix ASLR for TSan** (conditional) — `sudo sysctl vm.mmap_rnd_bits=28` (see [Known Issues](#known-issues--workarounds))
5. **Configure CMake** — Sets `ENABLE_ASAN/TSAN/UBSAN`, build type, Zenoh flags
6. **Build** — `cmake --build build -j$(nproc)`
7. **Run tests** — `ctest --output-on-failure -j$(nproc)` (TSan excludes `Zenoh|Mavlink|Yolo|Liveliness`)

### Why `fail-fast: false`?

Without this, GitHub cancels all matrix legs as soon as one fails. We want all 7 legs to finish so we can see the full picture — e.g., a TSan failure doesn't hide an ASan failure in a different leg.

---

## Job 3 — coverage

**Purpose:** Measure test coverage and produce a downloadable HTML report.

**How it works:**
1. Builds with `ENABLE_COVERAGE=ON` (adds `--coverage -fprofile-arcs -ftest-coverage` flags).
2. Zeros gcov counters, runs the full test suite.
3. `lcov --capture` collects raw `.gcda` data.
4. `lcov --remove` strips system headers (`/usr/*`), googletest, and test source files.
5. `genhtml` produces an HTML report.
6. `actions/upload-artifact@v4` uploads the report (14-day retention).

**Downloading the report:** Go to the workflow run → Artifacts → click `coverage-report` → unzip → open `index.html`.

**Current baseline:** ~75% line coverage, ~85% function coverage.

**Why SHM-only?** Coverage is about measuring *our* code paths, not third-party library code. The SHM backend exercises all first-party code without requiring Zenoh installation.

---

## CMake Build Options

These options control CI behaviour and are also available for local development:

| Option | Default | Purpose | CI usage |
|--------|---------|---------|----------|
| `ENABLE_ASAN` | `OFF` | AddressSanitizer | Legs 3, 6 |
| `ENABLE_TSAN` | `OFF` | ThreadSanitizer | Leg 4 |
| `ENABLE_UBSAN` | `OFF` | UBSan | Legs 5, 7 |
| `ENABLE_COVERAGE` | `OFF` | gcov instrumentation | Coverage job |
| `ENABLE_ZENOH` | `OFF` | Build with Zenoh IPC backend | Legs 2, 6, 7 |
| `ALLOW_INSECURE_ZENOH` | `OFF` | Skip TLS config requirement | All Zenoh legs (CI/dev only) |
| `CMAKE_BUILD_TYPE` | — | `Release` (normal) / `Debug` (sanitizer/coverage) | Set per leg |

**Mutual exclusivity:** `ENABLE_ASAN` and `ENABLE_TSAN` cannot both be `ON` — CMake emits `FATAL_ERROR`.

**Example local commands:**
```bash
# ASan build:
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=ON
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure

# Coverage build:
cmake -B build-cov -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON
cmake --build build-cov -j$(nproc)
ctest --test-dir build-cov --output-on-failure
lcov --capture --directory build-cov --output-file coverage.info --ignore-errors mismatch
lcov --remove coverage.info '/usr/*' '*/tests/*' --output-file coverage.info --ignore-errors unused
genhtml coverage.info --output-directory coverage-report
# Open coverage-report/index.html
```

---

## Zenoh Installation on CI

Zenoh is not available via `apt`, so the CI installs it from GitHub releases:

1. **zenohc** (C library) — Pre-built `.deb` packages from [eclipse-zenoh/zenoh-c releases](https://github.com/eclipse-zenoh/zenoh-c/releases). Currently pinned to **v1.7.2**.
   - These debs are built **without** the `shared-memory` Cargo feature, so `PosixShmProvider` returns `nullptr` at runtime. SHM-specific tests use `GTEST_SKIP()`. See [CI_ISSUES.md CI-001](CI_ISSUES.md).
   - Building from source with `-DZENOHC_BUILD_WITH_SHARED_MEMORY=ON` was attempted but fails with opaque-type size mismatches. See [CI_ISSUES.md CI-003](CI_ISSUES.md).

2. **zenoh-cpp** (header-only C++ bindings) — Cloned from GitHub at the matching version tag, installed via `cmake --install`.

**Upgrading Zenoh version:**
```yaml
# In ci.yml, update:
ZENOH_VERSION="1.8.0"  # example
# And update the zenoh-cpp clone tag to match
```

Test locally first — Zenoh API breaking changes are common between minor versions.

---

## Sanitizer Details

### AddressSanitizer (ASan)

**Detects:** heap/stack buffer overflow, use-after-free, double-free, memory leaks  
**Flags:** `-fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls`  
**Overhead:** ~2× slower, ~2× more memory  
**Build type:** Debug (for useful stack traces)

### ThreadSanitizer (TSan)

**Detects:** data races, deadlocks, thread leaks  
**Flags:** `-fsanitize=thread -fno-omit-frame-pointer -fno-optimize-sibling-calls -Wno-tsan`  
**Overhead:** ~5–15× slower, ~5–10× more memory  
**Build type:** Debug  
**Special handling:**
- ASLR fix: `sudo sysctl vm.mmap_rnd_bits=28` (kernel 6.17+ uses 32 bits, TSan only supports ≤28)
- `-Wno-tsan` suppresses GCC 13's warning about `atomic_thread_fence` under TSan + `-Werror`
- Tests depending on uninstrumented libraries (`Zenoh|Mavlink|Yolo|Liveliness`) are excluded via `ctest -E`

### UndefinedBehaviorSanitizer (UBSan)

**Detects:** signed integer overflow, null pointer dereference, misaligned access, shift errors  
**Flags:** `-fsanitize=undefined -fno-omit-frame-pointer`  
**Overhead:** minimal (~10–20%)  
**Build type:** Debug

---

## Known Issues & Workarounds

These are summarized here for quick reference. Full details are in [CI_ISSUES.md](CI_ISSUES.md).

| ID | Issue | Workaround | Ref |
|----|-------|-----------|-----|
| CI-001 | Zenoh SHM tests fail (pre-built debs lack SHM feature) | `GTEST_SKIP()` when `shm_provider()` is `nullptr` | CI_ISSUES.md |
| CI-002 | Unused parameter `-Werror` failure | Fix the code, don't suppress the warning | CI_ISSUES.md |
| CI-003 | zenoh-c source build fails (opaque-type size mismatch) | Use pre-built debs instead | CI_ISSUES.md |
| CI-004 | Unused typedef in SHM build | `[[maybe_unused]]` or `(void)` cast | CI_ISSUES.md |
| CI-005 | TSan + GCC 13 `-Wtsan` warning | `-Wno-tsan` in ENABLE_TSAN block | CI_ISSUES.md |
| — | TSan crash on kernel ≥6.17 (`FATAL: unexpected memory mapping`) | `sudo sysctl vm.mmap_rnd_bits=28` | BUG_FIXES.md #11 |
| — | TSan false positives with Zenoh/MAVSDK/OpenCV | Exclude those tests via `ctest -E` regex | ci.yml |

---

## Adding a New CI Job

1. Add a new job block in `ci.yml` after the existing jobs:
   ```yaml
   my-new-job:
     runs-on: ubuntu-24.04
     name: my-new-job
     steps:
     - name: Checkout
       uses: actions/checkout@v4
     # ... your steps ...
   ```

2. If it should block the build matrix, add `needs: my-new-job` to `build-and-test`.

3. If it should run after tests pass, add `needs: build-and-test` to your job.

4. Test the workflow syntax: push to a feature branch and check the Actions tab.

---

## Adding a New Matrix Leg

To add a new backend or sanitizer combination:

1. Add a new entry to the `matrix.include` array:
   ```yaml
   - ipc_backend: my_backend
     sanitizer: none
   ```

2. If the new backend needs dependencies, add a conditional install step:
   ```yaml
   - name: Install MyBackend
     if: matrix.ipc_backend == 'my_backend'
     run: |
       # Install commands here
   ```

3. Update the `Configure CMake` step if new CMake flags are needed.

4. Update this document with the new leg.

---

## Local Reproduction

### Automated: `deploy/run_ci_local.sh`

The easiest way to verify CI will pass is the local CI runner script. It mirrors
the exact same jobs, flags, and exclusions as `.github/workflows/ci.yml`.

```bash
# Quick check — format + SHM build+test (~40 s)
bash deploy/run_ci_local.sh --quick

# Full CI matrix — all 10 jobs (format + 7 build combos + 2 coverage)
bash deploy/run_ci_local.sh

# Run a single job by tag
bash deploy/run_ci_local.sh --job ASAN        # SHM ASAN
bash deploy/run_ci_local.sh --job ZENOH       
bash deploy/run_ci_local.sh --job FMT         # Clang-tidy format check  
bash deploy/run_ci_local.sh --job SHM
bash deploy/run_ci_local.sh --job TSAN        # SHM TSAN
bash deploy/run_ci_local.sh --job UBSAN       # SHM UBSAN
bash deploy/run_ci_local.sh --job ASAN_Z      # Zenoh ASAN
bash deploy/run_ci_local.sh --job UBSAN_Z     # Zenoh UBSAN
bash deploy/run_ci_local.sh --job COV         # SHM Coverage
bash deploy/run_ci_local.sh --job COV_Z       # Zenoh Coverage     
```

#### Available Job Tags

| Tag | CI Equivalent | Description |
|-----|---------------|-------------|
| `FMT` | `format-check` | clang-format-18 dry-run with `--Werror` |
| `SHM` | `build (shm)` | SHM backend, Debug build + all tests |
| `ZENOH` | `build (zenoh)` | Zenoh backend, Debug build + all tests |
| `ASAN` | `build (shm, asan)` | SHM + AddressSanitizer |
| `TSAN` | `build (shm, tsan)` | SHM + ThreadSanitizer (excludes Zenoh/Mavlink/Yolo tests) |
| `UBSAN` | `build (shm, ubsan)` | SHM + UndefinedBehaviorSanitizer |
| `ASAN_Z` | `build (zenoh, asan)` | Zenoh + AddressSanitizer |
| `UBSAN_Z` | `build (zenoh, ubsan)` | Zenoh + UndefinedBehaviorSanitizer |
| `COV` | `coverage (shm)` | SHM coverage build + lcov report |
| `COV_Z` | `coverage (zenoh)` | Zenoh coverage build + lcov report |

The script prints a color-coded summary at the end:

```
  CI Summary
  Total : 10
  Passed: 10
  Failed: 0
  Time  : 312s

✓ CI would PASS
```

### Manual: Reproducing a Single CI Leg

To reproduce a specific CI failure manually, match the CI environment:

```bash
# 1. Use the same Ubuntu version (or close to it)
#    CI uses ubuntu-24.04

# 2. Install the same dependencies
sudo apt-get install -y build-essential cmake libspdlog-dev \
  libeigen3-dev nlohmann-json3-dev libgtest-dev

# 3. Build with the same flags as the failing leg
#    Example: reproducing a shm+asan failure
cmake -B build \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
  -DENABLE_ASAN=ON
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure -j$(nproc)
```

**Common local vs CI differences:**
- Anaconda `LD_LIBRARY_PATH` can mask system libraries locally
- Local machine may have additional packages (MAVSDK, OpenCV, Gazebo) that CI lacks
- Zenoh SHM works locally (built from source) but not on CI (pre-built debs)
- Kernel ASLR bits differ (local kernel ≥6.17 may need `vm.mmap_rnd_bits=28` for TSan)

---

*Last updated: 2026-03-05 — added Zenoh coverage job (COV_Z) and `deploy/run_ci_local.sh` documentation.*
