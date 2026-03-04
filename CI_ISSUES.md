# CI Issues Log

This document records CI pipeline failures, their root causes, and the fixes applied. It serves as a knowledge base so newcomers can quickly understand past pitfalls and avoid repeating them.

**Format:** Each entry includes the symptoms, root cause analysis, fix, and prevention strategy.

---

## CI-001: ZenohShmProvider tests fail on CI (Zenoh build)

| Field | Value |
|---|---|
| **Date** | 2026-02-28 |
| **Branch** | `feat/48-zenoh-phase-c` |
| **PR** | #54 |
| **Affected tests** | `ZenohShmProvider.ProviderCreatesSuccessfully`, `AllocAndWriteBuffer`, `AllocLargeVideoFrameBuffer`, `PoolSizeConfiguration` |
| **CI matrix** | `zenoh` backend only (SHM backend unaffected) |

### Symptoms

```
99% tests passed, 4 tests failed out of 277

The following tests FAILED:
    195 - ZenohShmProvider.ProviderCreatesSuccessfully (Failed)
    196 - ZenohShmProvider.AllocAndWriteBuffer (Failed)
    197 - ZenohShmProvider.AllocLargeVideoFrameBuffer (Failed)
    198 - ZenohShmProvider.PoolSizeConfiguration (Failed)
Error: Process completed with exit code 8.
```

All 4 tests passed locally but failed on CI. The rest of the Zenoh test suite (273 tests) passed fine.

### Root Cause

The CI workflow installed **pre-built zenohc debian packages** from the [zenoh-c GitHub releases](https://github.com/eclipse-zenoh/zenoh-c/releases). These packages are built with the **default** Cargo features, which do **not** include `shared-memory`.

In zenoh-c's `Cargo.toml`:
```toml
[features]
shared-memory = ["zenoh/shared-memory"]   # ← opt-in, NOT in default
default = [
    "auth_pubkey",
    "auth_usrpwd",
    "transport_multilink",
    # ... transport features only — no shared-memory
]
```

Without the `shared-memory` feature, the `z_posix_shm_provider_new()` FFI call returns an error, `PosixShmProvider` creation throws a `ZException`, and `ZenohSession::shm_provider()` returns `nullptr`.

**Why it passed locally:** The local machine had zenohc built from source with `ZENOHC_BUILD_WITH_SHARED_MEMORY=ON`, so the SHM symbols were present and functional. This difference between local and CI environments was the root cause.

### How to Verify

You can check whether a zenohc library has SHM support by inspecting its exported symbols:

```bash
# Should show the symbol if SHM is supported:
nm -D /usr/lib/libzenohc.so | grep z_posix_shm_provider_new

# Count all SHM-related symbols (expect ~80+ if enabled):
nm -D /usr/lib/libzenohc.so | grep -i shm | wc -l
```

### Fix Applied

**Commit:** `f0f402a` (initial attempt) → reverted in CI-003

1. **Tests** (`tests/test_zenoh_ipc.cpp`):
   - Added `GTEST_SKIP()` guards: if `shm_provider()` returns `nullptr`, affected tests skip gracefully instead of hard-failing.

2. **CI workflow** — initially switched to building zenohc from source with `-DZENOHC_BUILD_WITH_SHARED_MEMORY=ON`, but this failed due to opaque-type size mismatches (see CI-003). Reverted to pre-built debs.

### Current Status

**SHM tests are SKIPPED on CI.** The following 7 tests skip when the SHM provider is unavailable:
- `ZenohShmProvider.ProviderCreatesSuccessfully`
- `ZenohShmProvider.AllocAndWriteBuffer`
- `ZenohShmProvider.AllocLargeVideoFrameBuffer`
- `ZenohShmProvider.PoolSizeConfiguration`
- `ZenohShmPublish.LargeVideoFrameUsesShmPath`
- `ZenohShmPublish.StereoFrameUsesShmPath`
- `ZenohShmPublish.SustainedVideoPublish`

The remaining Zenoh tests still run (pub/sub falls back to the bytes path).

### Prevention

- When adding features that depend on **opt-in** capabilities of a third-party library, always check whether CI installs a version that includes that capability.
- Add `GTEST_SKIP()` guards for tests that depend on runtime-optional capabilities (SHM, GPU, hardware) so they degrade gracefully.

---

## CI-002: Unused parameter `-Werror` failure (SHM build)

| Field | Value |
|---|---|
| **Date** | 2026-02-28 |
| **Branch** | `feat/48-zenoh-phase-c` |
| **PR** | #54 |
| **Affected file** | `common/ipc/include/ipc/message_bus_factory.h` |
| **CI matrix** | `shm` backend only (Zenoh backend unaffected) |

### Symptoms

```
error: unused parameter 'shm_pool_mb' [-Werror,-Wunused-parameter]
```

The SHM build compiled with `-Werror -Wall -Wextra` and failed because `shm_pool_mb` was only used inside `#ifdef HAVE_ZENOH`, but the parameter existed in all builds.

### Root Cause

`create_message_bus()` accepted a `shm_pool_mb` parameter. When `HAVE_ZENOH` is not defined (SHM-only build), the parameter was never referenced, triggering `-Wunused-parameter` promoted to an error by `-Werror`.

### Fix Applied

**Commit:** `63160d9`

Added `(void)shm_pool_mb;` cast at the top of the function body to suppress the warning in non-Zenoh builds.

### Prevention

- When adding parameters used only inside conditional compilation (`#ifdef`), always add a `(void)param;` cast in the else branch, or use `[[maybe_unused]]` (C++17).
- Run **both** CI build configurations locally before pushing:
  ```bash
  # SHM-only build
  cmake -B build-shm -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" && cmake --build build-shm

  # Zenoh build
  cmake -B build-zenoh -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
    -DENABLE_ZENOH=ON -DALLOW_INSECURE_ZENOH=ON && cmake --build build-zenoh
  ```

---

## CI-003: zenoh-c source build fails with opaque-type size mismatches

| Field | Value |
|---|---|
| **Date** | 2026-02-28 |
| **Branch** | `feat/48-zenoh-phase-c` |
| **PR** | #54 |
| **Affected step** | "Build Zenoh from source" CI step |
| **CI matrix** | `zenoh` backend only |
| **Urgency** | **Medium** — SHM zero-copy is not tested in CI until this is resolved |
| **Status** | **OPEN** — workaround in place (tests skip), but SHM coverage gap exists |

### Symptoms

```
error[E0080]: evaluation of constant value failed
  --> src/lib.rs:57:1
   |
57 | get_opaque_type_data!(ZBytes, z_owned_bytes_t);
   | ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ the evaluated program panicked at
   | 'type: z_owned_bytes_t, align: 8, size: 40'

error: could not compile `opaque-types` (lib) due to 66 previous errors
gmake[2]: *** [CMakeFiles/cargo.dir/build.make:149:
  /tmp/zenoh-c/target/release/libzenohc.so] Error 101
```

66 `get_opaque_type_data!` macro invocations fail with size/alignment mismatches.

### Root Cause

zenoh-c uses a build-time Rust macro (`get_opaque_type_data!`) that verifies the C header struct sizes match the compiled Rust type sizes. When building from source with `-DZENOHC_BUILD_WITH_SHARED_MEMORY=ON`:

1. The `shared-memory` cargo feature changes internal Rust type layouts (additional fields for SHM metadata).
2. The **pre-generated C headers** shipped in the zenoh-c repo were generated for the **default** feature set (no SHM).
3. The size assertions panic because the compiled Rust structs are larger than what the headers declare.

This is a **zenoh-c upstream issue**: their release process generates headers only for the default feature combination. Building with non-default features requires regenerating headers via `cbindgen`, which in turn requires matching the exact Rust toolchain version pinned in `rust-toolchain.toml`.

The `dtolnay/rust-toolchain@stable` on CI may install a different Rust version than what zenoh-c 1.7.2 was built with, compounding the mismatch.

### Current Workaround

Reverted CI to use **pre-built debian packages** (which lack SHM support). All SHM-dependent tests use `GTEST_SKIP()` so CI passes, but **7 tests are skipped** — zero-copy publishing is not validated in CI.

### Impact

| What works on CI | What does NOT work on CI |
|---|---|
| Zenoh pub/sub (bytes path) | PosixShmProvider creation |
| Topic mapping | SHM buffer allocation |
| Factory pattern | Zero-copy publish path |
| Small-message round-trips | SHM publish counter assertions |
| Fallback from SHM → bytes | Sustained video SHM delivery |

The SHM code path is validated **locally** (where zenohc is built from source with SHM), but not in CI. This creates a risk of SHM regressions going undetected.

### When to Fix

| Priority | Trigger | Action |
|---|---|---|
| **P2 — Medium** | Before Phase D (Epic #45) | Must be resolved before adding more SHM-dependent features |
| **Ideal** | When zenoh-c 1.8+ releases | Check if SHM is included in default features or if pre-built packages include it |
| **Alternative** | If blocked on upstream | Build zenohc with exact pinned Rust toolchain + regenerate headers in CI |

**Recommended timeline:** Resolve before starting Phase D (#49). Without CI SHM coverage, any regression in the zero-copy path would only be caught by manual local testing.

### Possible Solutions (in order of preference)

1. **Wait for zenoh-c upstream fix** — check each new zenoh-c release to see if `shared-memory` is added to default features or if pre-built packages include it. This is the lowest-effort fix.

2. **Pin exact Rust toolchain in CI** — read `rust-toolchain.toml` from the zenoh-c repo, install that exact version, and build from source. This should avoid the opaque-type mismatch.
   ```yaml
   # Extract pinned version from zenoh-c's rust-toolchain.toml
   RUST_VERSION=$(grep channel /tmp/zenoh-c/rust-toolchain.toml | cut -d'"' -f2)
   rustup install "$RUST_VERSION"
   rustup default "$RUST_VERSION"
   ```

3. **Use zenoh-c's Docker build image** — the zenoh project may provide CI images with the correct toolchain pre-installed.

4. **Pre-build and host our own deb** — build zenohc+SHM once locally, package as a `.deb`, host on GitHub Releases in our repo, and install from there in CI.

---

<!-- Template for new entries:

## CI-NNN: Short description

| Field | Value |
|---|---|
| **Date** | YYYY-MM-DD |
| **Branch** | `branch-name` |
| **PR** | #NN |
| **Affected tests/files** | ... |
| **CI matrix** | which backend(s) affected |

### Symptoms
What error message or failure was observed in CI?

### Root Cause
Why did it fail? What was different between local and CI?

### Fix Applied
**Commit:** `hash`
What changes were made?

### Prevention
How to avoid this class of issue in the future?

-->

---

## CI-004: Unused typedef / parameter in SHM build (`bus_create_client` / `bus_create_server`)

| Field | Value |
|---|---|
| **Date** | 2026-03-01 |
| **Branch** | `feat/49-zenoh-phase-d` |
| **PR** | #55 |
| **Affected file** | `common/ipc/include/ipc/message_bus_factory.h` |
| **CI matrix** | `shm` backend only (Zenoh backend unaffected) |

### Symptoms

```
error: typedef 'using BusType = ...' locally defined but not used [-Werror=unused-local-typedefs]
error: unused parameter 'b' [-Werror=unused-parameter]
```

Both `bus_create_client()` and `bus_create_server()` failed to compile in the SHM-only CI build.

### Root Cause

The `using BusType` typedef and the lambda parameter `b` were declared outside the `#ifdef HAVE_ZENOH` guard but only referenced inside it. In non-Zenoh builds, both were unreferenced, triggering `-Wunused-local-typedefs` and `-Wunused-parameter` (promoted to errors by `-Werror`).

Same class of issue as CI-002 — conditional compilation with strict warnings.

### Fix Applied

1. Moved `using BusType = ...` inside `#ifdef HAVE_ZENOH`.
2. Added `(void)b;` cast in the fallback path.

### Prevention

See CI-002. Always test both build configurations with `-Werror -Wall -Wextra` before pushing:
```bash
cmake -B build-shm -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" && cmake --build build-shm
cmake -B build-zenoh -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
  -DENABLE_ZENOH=ON -DALLOW_INSECURE_ZENOH=ON && cmake --build build-zenoh
```

---

## CI-005: TSan build fails — `atomic_thread_fence` not supported with `-fsanitize=thread`

| Field | Value |
|---|---|
| **Date** | 2026-03-03 |
| **Branch** | `infra/issue-67-sanitizers` |
| **PR** | #71 |
| **Affected leg** | `build (shm, tsan)` |
| **Affected files** | `common/ipc/include/ipc/shm_reader.h`, `process2_perception/src/main.cpp`, `process3_slam_vio_nav/src/main.cpp` |

### Symptoms

```
/usr/include/c++/13/bits/atomic_base.h:144:26: error: 'atomic_thread_fence' is not supported
with '-fsanitize=thread' [-Werror=tsan]
  144 |   { __atomic_thread_fence(int(__m)); }
      |     ~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~

cc1plus: all warnings being treated as errors
```

The TSan CI leg (`shm + tsan`) failed at the **Build** step. The `shm` and `zenoh` legs (without sanitizers) passed fine. Local TSan builds also passed because they did not use `-Werror`.

### Root Cause

GCC 13 (shipped with Ubuntu 24.04) emits a `-Wtsan` diagnostic whenever `std::atomic_thread_fence()` is compiled with `-fsanitize=thread`. This is because ThreadSanitizer **replaces** fence operations with its own instrumentation — the fence in the source code is effectively a no-op at runtime under TSan.

The warning is **informational**, not a correctness issue. TSan still correctly detects data races around the fenced memory accesses.

However, the CI pipeline passes `-Werror` (via `CMAKE_CXX_FLAGS`), which promotes `-Wtsan` into a hard compile error. Our `ShmReader::read()` method uses `std::atomic_thread_fence(std::memory_order_acquire)` as part of the seqlock read pattern — a legitimate and intentional use.

### Fix Applied

Added `-Wno-tsan` to the TSan compile options in `CMakeLists.txt`:

```cmake
if(ENABLE_TSAN)
    message(STATUS "  Sanitizer    : ThreadSanitizer (TSan) ENABLED")
    add_compile_options(-fsanitize=thread -Wno-tsan)
    add_link_options(-fsanitize=thread)
endif()
```

This suppresses the informational warning without affecting TSan's ability to detect real data races.

### Prevention

When adding new sanitizer flags, always test with the **exact same flags** used in CI:

```bash
# Match CI conditions: -Werror + sanitizer
cmake -B build-tsan -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
  -DENABLE_TSAN=ON && cmake --build build-tsan -j$(nproc)
```

**Note:** TSan on kernel 6.x also requires `sudo sysctl vm.mmap_rnd_bits=28` to avoid ASLR-related crashes (see BUG_FIXES.md Fix #11).

---

## CI-007: Merge Conflicts — PR #77 Branch vs `main`

| Field | Value |
|---|---|
| **Date** | 2026-03-03 |
| **Branch** | `infra/issue-70-nodiscard-audit` |
| **PR** | #77 |
| **Affected files** | `DEVELOPMENT_WORKFLOW.md`, `ROADMAP.md`, `common/util/include/util/result.h` |
| **CI status** | `DIRTY` / `CONFLICTING` — GitHub blocked merging |

### Symptoms

GitHub showed PR #77 as "This branch has conflicts that must be resolved" with `mergeStateStatus: DIRTY` and `mergeable: CONFLICTING`. The PR could not be merged.

### Root Cause

PR #74 (Tier 1 docs + deploy scripts) merged into `main` while PRs #75–#77 were still open on branches that were created *before* PR #74 merged. The overlapping files:

1. **`DEVELOPMENT_WORKFLOW.md`** — PR #74 added a `docs/CI_SETUP.md` row to the documentation table; this branch added a `tests/TESTS.md` row at the same location.

2. **`ROADMAP.md`** (5 conflict regions) — PR #74 updated the metrics table with detailed per-phase columns and Tier 1 data (400 tests, 23 suites). This branch had a condensed table with Tier 2 data (464 tests, 26 suites, all issues closed). Both sides also edited the "Current State at a Glance" summary and the footer.

3. **`result.h`** (2 conflict regions) — PR #75 (merged into `main`) added `static_assert` guards in `and_then()` and removed the duplicate early-return before them. This branch still had the old code with the duplicate early-return.

### Fix Applied

Merged `origin/main` into the feature branch and resolved all conflicts:

```bash
git fetch origin main
git merge origin/main
# Resolve conflicts in 3 files, then:
git add -A && git commit
git push
```

**Resolution strategy per file:**

| File | Strategy |
|------|----------|
| `DEVELOPMENT_WORKFLOW.md` | Kept both doc rows (additive — `tests/TESTS.md` + `docs/CI_SETUP.md`) |
| `ROADMAP.md` | Took `main`'s detailed per-phase table, updated final column with this branch's data (464 tests, 26 suites, all issues closed), added new rows (`[[nodiscard]]`, config schemas, `Result<T,E>`, line coverage) |
| `result.h` | Took `main`'s version — removed the duplicate early-return before `static_assert` (correct call already exists after the assertion) |

Commit: `7514538`

### Prevention

See [DEVELOPMENT_WORKFLOW.md § Avoiding Merge Conflicts](#avoiding-merge-conflicts) for the full best-practices guide. Key points:

1. **Rebase feature branches frequently** — `git fetch origin main && git rebase origin/main` at least daily
2. **Use stacked PR base branches** for chained work (PR #76 base = PR #75's branch, not `main`)
3. **Keep doc updates on `main`** — project-wide metrics files (ROADMAP.md, PROGRESS.md) are especially conflict-prone
4. **Keep branches short-lived** — aim for <3 days, merge in order promptly

---

## CI-008: `-Wstringop-truncation` failure in Release builds (SHM + Zenoh)

| Field | Value |
|---|---|
| **Date** | 2026-03-04 |
| **Branch** | `feature/issue-89-thread-heartbeat-watchdog` |
| **PR** | #94 |
| **Affected legs** | `shm, none` and `zenoh, none` (Release `-O2`) |
| **Passed locally** | Yes (local build uses `deploy/build.sh` which doesn't pass `-Werror`) |

### Symptoms

```
thread_heartbeat.h:99:21: error: 'char* __builtin___strncpy_chk(char*, const char*,
  long unsigned int, long unsigned int)' output truncated copying 31 bytes from a
  string of length 50 [-Werror=stringop-truncation]
```

Build failed only on the **plain** (non-sanitizer) SHM and Zenoh CI legs. All sanitizer legs (ASAN/TSAN/UBSAN) passed because they build in **Debug** mode (`-O0`).

### Root Cause

`ThreadHeartbeatRegistry::register_thread()` used `std::strncpy()` to copy a thread name into a fixed-size `char[32]` buffer, intentionally truncating long names to 31 chars + null terminator. This is correct behaviour — the truncation is by design.

However, GCC's `-Wstringop-truncation` warning (enabled by `-Wall`) detects that `strncpy` will truncate and flags it. With `-Werror`, this becomes a hard error.

This warning only fires in **Release** mode (`-O2`) because GCC's interprocedural static analysis at higher optimization levels performs deeper string length tracking. In **Debug** mode (`-O0`), the analysis is less aggressive and the warning is not triggered.

### Fix Applied

**Commit:** `712b63c`

Replaced `strncpy` + manual null-termination with `snprintf`:

```cpp
// Before (triggers -Wstringop-truncation):
std::strncpy(beats_[idx].name, name, sizeof(beats_[idx].name) - 1);
beats_[idx].name[sizeof(beats_[idx].name) - 1] = '\0';

// After (silent, same semantics):
std::snprintf(beats_[idx].name, sizeof(beats_[idx].name), "%s", name);
```

`snprintf` with `%s` provides identical truncation + null-termination without triggering the warning because GCC recognises `snprintf` truncation as intentional by design.

Also added `#include <cstdio>` for `snprintf`.

### Prevention

- Prefer `snprintf(dst, sizeof(dst), "%s", src)` over `strncpy(dst, src, n)` for intentional truncation into fixed-size char buffers.
- Always test with Release + `-Werror -Wall -Wextra` locally before pushing, especially for new code that manipulates C strings. The `deploy/build.sh` script does NOT pass `-Werror` — CI does.
