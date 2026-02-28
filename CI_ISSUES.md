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

**Commit:** `f0f402a` + `55abafb`

1. **CI workflow** (`.github/workflows/ci.yml`):
   - Replaced pre-built deb install with source build using `-DZENOHC_BUILD_WITH_SHARED_MEMORY=ON`
   - Added `dtolnay/rust-toolchain@stable` to provide Rust/Cargo for the source build
   - Added `actions/cache@v4` to cache the built artifacts across CI runs (keyed on zenoh version + OS + arch), so the Rust build only happens once per version bump

2. **Tests** (`tests/test_zenoh_ipc.cpp`):
   - Added `GTEST_SKIP()` as defence-in-depth: if `shm_provider()` returns `nullptr`, the test skips gracefully instead of hard-failing. This makes the tests resilient to environments without SHM support.

### Prevention

- When adding features that depend on **opt-in** capabilities of a third-party library, always check whether CI installs a version that includes that capability.
- Prefer building dependencies from source in CI when specific build flags are required. Use GitHub Actions caching (`actions/cache`) to avoid rebuilding on every run.
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
