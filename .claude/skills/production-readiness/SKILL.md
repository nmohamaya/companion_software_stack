---
name: production-readiness
description: Pre-deployment audit for hardware — build verification, safety audit, test coverage, security scan, performance check, config audit, debug code gate, systemd validation
argument-hint: "[--target <jetson-orin|localhost>] [--quick] [--section <name>]"
---

# /production-readiness — Pre-Deployment Audit

Comprehensive production readiness assessment before deploying to hardware. Checks 11 categories covering build, safety, test coverage, sanitizers, security, performance, configuration, debug code, systemd, scenario tests, and dependencies.

**This is the final gate before code goes on a real drone.** Every finding is either BLOCKING (must fix) or ADVISORY (should fix, document risk if not).

## Arguments

Parse `$ARGUMENTS` to extract:
- **--target \<platform\>** (optional): `jetson-orin` (default) or `localhost` (dev machine audit)
- **--quick** (optional): Run only blocking checks (skip advisory sections)
- **--section \<name\>** (optional): Run a specific section only (e.g., `safety`, `security`, `performance`)

## Audit Sections

### Section 1: Build Verification [BLOCKING]

Run the build with maximum strictness:

```bash
bash deploy/build.sh
```

Verify:
- [ ] Exit code 0
- [ ] Zero compiler warnings (`-Werror -Wall -Wextra`)
- [ ] All `[[nodiscard]]` enforced
- [ ] Build output size reasonable (check binary sizes in `build/bin/`)

If `--target jetson-orin`, also check:
- [ ] Cross-compilation config exists (`boards/jetson-orin/`)
- [ ] ARM64 toolchain available

Report binary sizes:
```
Binary sizes:
  video_capture:    1.2 MB
  perception:       3.4 MB
  slam_vio_nav:     2.1 MB
  mission_planner:  2.8 MB
  comms:            1.5 MB
  payload_manager:  0.8 MB
  system_monitor:   0.9 MB
  Total:           12.7 MB
```

### Section 2: Safety Audit [BLOCKING]

Run the existing safety audit script:

```bash
bash deploy/safety_audit.sh
```

This checks 29 safety rules from CLAUDE.md:
- **16 AVOID rules**: memcpy, raw new/delete, shared_ptr, C-style casts, reinterpret_cast without static_assert, volatile, goto, atoi/atof, bare std::thread, using namespace in headers, exit/abort, unscoped enum, global mutable state, memory_order_relaxed, signed→unsigned casts
- **12 PREFER rules**: [[nodiscard]], RAII locks, = delete, override, noexcept on moves, fixed-width types, default member initializers, strong types

Parse the report and present:
```
Safety Audit: 28/29 PASS, 1 WARN, 0 FAIL
  WARN: Rule 30 (signed→unsigned casts) — 2 instances in process4_mission_planner/
```

### Section 3: Test Coverage [BLOCKING]

Run tests and verify coverage:

```bash
# Run all tests
./tests/run_tests.sh

# Get test count
ctest -N --test-dir build | grep "Total Tests:"

# Compare to baseline
```

Read `tests/TESTS.md` for the expected baseline count.

Verify:
- [ ] All tests pass (100% pass rate)
- [ ] Test count matches baseline (currently 1259 C++ + 42 shell + 250+ scenario)
- [ ] No RESOURCE_LOCK warnings (Zenoh session exhaustion)

If coverage build is available (`build-coverage/`):
```bash
bash deploy/view_coverage.sh --no-open
```
- [ ] Line coverage ≥ 75% (current baseline: 75.1%)
- [ ] No uncovered safety-critical paths (mission planner, fault recovery, comms)

### Section 4: Sanitizer Verification [BLOCKING]

Run all three sanitizers:

```bash
bash deploy/build.sh --asan && ./tests/run_tests.sh    # AddressSanitizer
bash deploy/build.sh --tsan && ./tests/run_tests.sh    # ThreadSanitizer
bash deploy/build.sh --ubsan && ./tests/run_tests.sh   # UBSanitizer
```

This is slow (~15 min total). If `--quick`, skip and warn that sanitizers were not run.

Verify:
- [ ] ASan: zero heap-buffer-overflow, use-after-free, memory leaks
- [ ] TSan: zero data races (note: Zenoh/MAVSDK tests excluded under TSan)
- [ ] UBSan: zero undefined behavior (signed overflow, null deref, alignment)

### Section 5: Security Scan [BLOCKING]

Check for security issues:

**Debug/insecure code:**
```bash
# ALLOW_INSECURE_ZENOH should NOT be in production builds
grep -r "ALLOW_INSECURE_ZENOH" CMakeLists.txt build/
```

**Secrets exposure:**
```bash
# Check for hardcoded credentials, tokens, API keys
grep -rn "password\|secret\|api_key\|token\|credential" common/ process[1-7]_* --include='*.h' --include='*.cpp' -i
```

**Network exposure:**
```bash
# Check Zenoh config for open listeners
grep -r "listen\|connect" config/default.json config/gazebo_sitl.json
```

**Input validation:**
- [ ] All IPC structs have `validate()` methods
- [ ] MAVLink messages validated before processing
- [ ] GCS commands bounds-checked
- [ ] Config values validated (no negative altitudes, no zero timeouts)

Verify:
- [ ] No `ALLOW_INSECURE_ZENOH` in production config
- [ ] No hardcoded secrets
- [ ] Zenoh TLS configured (or documented exception)
- [ ] All external inputs validated

### Section 6: Performance Check [ADVISORY]

Check for performance issues in hot paths:

**Allocation in frame loop:**
```bash
# Search for new/make_unique/make_shared in perception and mission planner hot paths
grep -n "make_unique\|make_shared\|new " process2_perception/src/*.cpp process4_mission_planner/src/*.cpp
```

**Lock scope:**
- Check for mutexes held during I/O or allocation
- Check for wide lock scopes in per-frame code

**Binary size:**
- Compare against previous release (if available)
- Flag binaries > 10 MB (may indicate bloat from template instantiation or debug symbols)

### Section 7: Configuration Audit [BLOCKING]

Verify production config:

1. Read `config/default.json` — check all tunables have sensible defaults
2. Verify no test/debug overrides:
   - `ipc_backend` must be `"zenoh"` (not `"shm"`)
   - `detector.backend` should not be `"simulated"` in production
   - `video_capture.mission_cam.backend` should not be `"simulated"`
3. Verify safety parameters:
   - `geofence.enabled` should be `true`
   - `battery.rtl_threshold` should be > 0
   - `watchdog` timeout values reasonable (not too short, not disabled)

If `--target jetson-orin`, check customer configs:
```bash
ls config/customers/
```

### Section 8: Debug Code Gate [BLOCKING]

Check for debug/diagnostic code that should be gated or removed before production.

Search for known debug patterns:

```bash
# DIAG logging that should be disabled in Release
grep -rn "DIAG\|DEBUG_ONLY\|#ifdef DEBUG" common/ process[1-7]_* --include='*.h' --include='*.cpp'

# Test-only code paths
grep -rn "TEST_ONLY\|TESTING\|#ifdef TESTING" common/ process[1-7]_* --include='*.h' --include='*.cpp'

# Temporary/experimental code
grep -rn "TODO.*remove\|HACK\|FIXME\|TEMPORARY\|EXPERIMENTAL" common/ process[1-7]_* --include='*.h' --include='*.cpp'
```

Verify:
- [ ] No DIAG logging at ERROR/WARN level (should be DEBUG/TRACE only)
- [ ] No test-only code compiled in Release
- [ ] All TODOs/HACKs tracked as issues or documented as acceptable

### Section 9: systemd Validation [BLOCKING for hardware]

Verify systemd service units:

```bash
# Check syntax
for f in deploy/systemd/*.service; do
    systemd-analyze verify "$f" 2>&1
done
```

Verify:
- [ ] All 7 service files + target file present
- [ ] `Type=notify` with `WatchdogSec=10s` on all services
- [ ] `BindsTo=` dependencies correct (P4 depends on P2, P5, etc.)
- [ ] `ExecStart=` points to correct binary paths
- [ ] `Restart=on-failure` with reasonable `RestartSec`

If `--target localhost`, skip and note "systemd validation skipped (not hardware target)".

### Section 10: Scenario Test Results [ADVISORY]

Check recent scenario test results:

```bash
# Find latest run results
ls -lt drone_logs/scenarios_gazebo/*/latest/run_metadata.json 2>/dev/null | head -25
```

If results exist, parse and summarize:
- [ ] All 20 Tier 1 scenarios pass
- [ ] All 5 Tier 2 scenarios pass (if Gazebo available)
- [ ] No regressions from previous run

If no recent results, suggest: "Run `/run-scenario all` before deploying."

### Section 11: Dependency Audit [ADVISORY]

Check dependency versions:

```bash
# Zenoh version
dpkg -l | grep zenoh 2>/dev/null || echo "Zenoh not installed via dpkg"

# OpenCV version
pkg-config --modversion opencv4 2>/dev/null || echo "OpenCV not found"

# MAVSDK version
grep "MAVSDK" CMakeLists.txt
```

Compare against known-good versions in `deploy/install_dependencies.sh`.

Verify:
- [ ] Zenoh version matches CI (1.7.2)
- [ ] No CVEs in current dependency versions (check recent advisories)
- [ ] All dependencies available on target platform

---

## Final Report

Present the complete audit report:

```
═══════════════════════════════════════════════════════
  PRODUCTION READINESS AUDIT
  Target: <platform>
  Date: <today>
  Branch: <branch>
  Commit: <commit>
═══════════════════════════════════════════════════════

--- BLOCKING ---
[1]  Build Verification     PASS
[2]  Safety Audit           PASS (28/29 rules, 1 WARN)
[3]  Test Coverage          PASS (1259 tests, 100% pass)
[4]  Sanitizers             PASS (ASan + TSan + UBSan clean)
[5]  Security Scan          PASS
[7]  Config Audit           WARN (geofence disabled in default.json)
[8]  Debug Code Gate        PASS (2 TODOs tracked as issues)
[9]  systemd Validation     PASS (7 services verified)

--- ADVISORY ---
[6]  Performance Check      WARN (3 allocations in frame loop)
[10] Scenario Tests         PASS (25/25 green)
[11] Dependency Audit       PASS (versions match CI)

═══════════════════════════════════════════════════════
  VERDICT: CONDITIONAL PASS
  Blocking issues: 0
  Advisory warnings: 2
  
  Action items:
  1. [ADVISORY] Fix 3 allocations in perception frame loop
  2. [ADVISORY] Enable geofence in production config
═══════════════════════════════════════════════════════

Pre-Flight Checklist for Hardware Deployment:
  □ Flash Jetson Orin with JetPack 6.x
  □ Install dependencies: bash deploy/install_dependencies.sh --all
  □ Build: bash deploy/build.sh
  □ Install systemd units: bash deploy/install_systemd.sh
  □ Configure customer config: config/customers/<customer>.json
  □ Enable geofence with mission-specific polygon
  □ Verify sensor connections (cameras, IMU, radar)
  □ Test arm/disarm cycle via GCS
  □ Run nominal_mission scenario on bench
  □ Verify RTL triggers correctly
```

User choices:
- **generate-checklist** — output the pre-flight checklist as a standalone markdown file
- **file-issues** — create GitHub issues for each blocking/advisory item
- **re-run \<section\>** — re-run a specific section after fixing issues
- **done** — acknowledge the audit

If the user provided arguments, use them as context: $ARGUMENTS
