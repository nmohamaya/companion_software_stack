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
