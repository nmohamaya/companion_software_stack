# Deploy Scripts — Usage Guide

Quick reference for all build, test, and launch scripts in the `deploy/` folder.

---

## `build.sh` — Flexible Project Build

The main build script. Supports sanitizers, coverage, format checking, and
integrated test runs.

```bash
# ── Basic builds ─────────────────────────────────────────────
bash deploy/build.sh                          # Release build
bash deploy/build.sh Debug                    # Debug build
bash deploy/build.sh --clean                  # Delete build/ before building

# ── Sanitizers (auto-switches to Debug) ──────────────────────
bash deploy/build.sh --asan                   # AddressSanitizer
bash deploy/build.sh --tsan                   # ThreadSanitizer
bash deploy/build.sh --ubsan                  # UndefinedBehaviorSanitizer

# ── Code coverage (auto-switches to Debug) ───────────────────
bash deploy/build.sh --coverage               # Build with gcov instrumentation

# ── Tests ────────────────────────────────────────────────────
bash deploy/build.sh --test                   # Build + run all tests
bash deploy/build.sh --test-filter watchdog   # Build + run module tests

# ── Format check (no build — validates formatting only) ──────
bash deploy/build.sh --format-check           # Dry-run clang-format-18 check

# ── Combinations ─────────────────────────────────────────────
bash deploy/build.sh --clean --asan           # Clean + ASan
```

| Flag             | Effect                                              |
| ---------------- | --------------------------------------------------- |
| `Debug\|Release` | Set CMake build type (default: `Release`)           |
| `--clean`        | Remove `build/` before configuring                  |
| `--asan`         | Enable AddressSanitizer (forces Debug)               |
| `--tsan`         | Enable ThreadSanitizer (forces Debug)                |
| `--ubsan`        | Enable UndefinedBehaviorSanitizer (forces Debug)     |
| `--coverage`     | Enable gcov code-coverage instrumentation (forces Debug) |
| `--format-check` | Run `clang-format-18 --dry-run --Werror` and exit   |
| `--test`         | Build then run all tests via `ctest`                 |
| `--test-filter`  | Build then run tests matching a module name          |

> **Note:** Sanitizers are mutually exclusive — only pass one of `--asan`,
> `--tsan`, or `--ubsan` at a time.
>
> **Note:** The `--zenoh` flag is accepted but ignored — Zenoh is always enabled
> (it is the sole IPC backend since PR #151).

---

## `clean_build_and_run.sh` — Full Build + Gazebo SITL Pipeline

Performs a **complete cycle**: kill stale processes → clean build → unit tests →
Gazebo SITL launch.

```bash
bash deploy/clean_build_and_run.sh               # Headless, Release
bash deploy/clean_build_and_run.sh --gui          # With 3-D Gazebo GUI
bash deploy/clean_build_and_run.sh --asan         # + AddressSanitizer
bash deploy/clean_build_and_run.sh --ubsan        # + UBSan
bash deploy/clean_build_and_run.sh --coverage     # + code coverage
```

| Flag         | Effect                               |
| ------------ | ------------------------------------ |
| `--gui`      | Open the Gazebo 3-D visualisation    |
| `--asan`     | AddressSanitizer (forces Debug)      |
| `--ubsan`    | UBSan (forces Debug)                 |
| `--coverage` | gcov instrumentation (forces Debug)  |

> **Note:** `--tsan` is intentionally omitted because the `zenohc` library
> triggers ThreadSanitizer false-positives in its internal threading.

---

## `launch_all.sh` — Launch All Companion Processes

Starts all seven companion processes in the background using a given config
file. Typically called by the clean-build script, but can be used standalone:

```bash
CONFIG_FILE=config/gazebo_sitl.json bash deploy/launch_all.sh
```

---

## `launch_gazebo.sh` — Launch Gazebo SITL Simulation

Starts PX4 SITL + Gazebo + the companion stack:

```bash
bash deploy/launch_gazebo.sh              # Headless
bash deploy/launch_gazebo.sh --gui        # With 3-D visualisation
```

Set `CONFIG_FILE` to select a Gazebo/SITL config profile:

```bash
CONFIG_FILE=config/gazebo_sitl.json   bash deploy/launch_gazebo.sh --gui   # Default SITL profile
```

| Environment Variable | Default | Description |
|---------------------|---------|-------------|
| `CONFIG_FILE` | `config/gazebo_sitl.json` | JSON config for Gazebo/SITL profile + process settings |
| `PX4_DIR` | `~/PX4-Autopilot` | Path to PX4-Autopilot source |
| `GZ_WORLD` | `sim/worlds/test_world.sdf` | Gazebo SDF world file |
| `LOG_DIR` | `drone_logs/` | Log output directory |

> **Known Issue #30:** Gazebo may SIGSEGV (exit 139) when relaunched immediately
> after a previous run. Wait 10–15 seconds between runs.

---

## `launch_hardware.sh` — Launch on Real Hardware

Deploys the companion stack to physical hardware (no Gazebo):

```bash
bash deploy/launch_hardware.sh
```

---

## `install_dependencies.sh` — Install System Dependencies

One-time setup script that installs all required system packages:

```bash
bash deploy/install_dependencies.sh
```

---

## `install_systemd.sh` / `uninstall_systemd.sh` — systemd Service Management

Install or remove the drone companion stack as a systemd service target
(7 service units + 1 target for auto-start on boot).

```bash
# Install (requires root)
sudo ./deploy/install_systemd.sh                              # defaults
sudo ./deploy/install_systemd.sh --bin-dir /usr/local/bin      # custom binary path
sudo ./deploy/install_systemd.sh --config /etc/drone/my.json   # custom config
sudo ./deploy/install_systemd.sh --no-enable                   # install without auto-start

# Uninstall
sudo ./deploy/uninstall_systemd.sh                             # stop + remove all units
sudo ./deploy/uninstall_systemd.sh --keep-logs                 # preserve /var/log/drone
```

---

## `run_ci_local.sh` — Run CI Checks Locally

Runs the same matrix of checks as GitHub Actions CI, locally. Useful for
validating before pushing.

```bash
bash deploy/run_ci_local.sh               # All jobs (format + build + sanitizers + coverage)
bash deploy/run_ci_local.sh --quick       # Format + build only (fast)
bash deploy/run_ci_local.sh --job FMT     # Single job by tag
bash deploy/run_ci_local.sh --job ASAN    # ASan only
bash deploy/run_ci_local.sh 2>&1 | tee ci_results.log   # Save output
```

| Tag | Description |
|-----|-------------|
| `FMT` | clang-format-18 check |
| `BUILD` | Debug build + test |
| `ASAN` | AddressSanitizer |
| `TSAN` | ThreadSanitizer |
| `UBSAN` | UndefinedBehaviorSanitizer |
| `COV` | Coverage build + lcov report |

---

## `view_coverage.sh` — Code Coverage Report

Builds with coverage instrumentation, runs tests, and generates an HTML report.

```bash
bash deploy/view_coverage.sh              # Full pipeline: build → test → report
bash deploy/view_coverage.sh --open       # Auto-open report in browser
bash deploy/view_coverage.sh --report     # Skip build/test, just regenerate report
bash deploy/view_coverage.sh --summary    # Terminal summary only (no HTML)
```

**Output:**
- `build/coverage-report/index.html` — HTML coverage report
- `build/coverage.info` — lcov tracefile

---

## Common Developer Workflows

### Quick format check before committing

```bash
bash deploy/build.sh --format-check
```

### Run tests under AddressSanitizer to find memory bugs

```bash
bash deploy/build.sh --clean --asan
ctest --test-dir build --output-on-failure
```

### Generate a code-coverage report

```bash
bash deploy/view_coverage.sh --open
```

Or manually:

```bash
bash deploy/build.sh --clean --coverage
cd build
ctest --output-on-failure
lcov --capture --directory . --output-file coverage.info --ignore-errors mismatch
lcov --remove coverage.info '/usr/*' '*/tests/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_html
echo "Open: ${PWD}/coverage_html/index.html"
```

### Full end-to-end with Gazebo GUI

```bash
bash deploy/clean_build_and_run.sh --gui
```

### Run all Gazebo SITL scenarios

```bash
./tests/run_scenario_gazebo.sh --all           # headless
./tests/run_scenario_gazebo.sh --all --gui     # with 3-D window
```

### Run a single scenario (e.g. obstacle avoidance)

```bash
./tests/run_scenario_gazebo.sh config/scenarios/02_obstacle_avoidance.json --gui
```

### Run Tier 1 scenarios without Gazebo

```bash
./tests/run_scenario.sh --all --tier 1
```

### Run full CI locally before pushing

```bash
bash deploy/run_ci_local.sh --quick    # fast: format + build
bash deploy/run_ci_local.sh            # full: all CI jobs
```

### Auto-fix formatting on all source files

```bash
find common process[1-7]_* tests \( -name '*.h' -o -name '*.cpp' \) -print0 \
  | xargs -0 clang-format-18 -i
```

### Install as systemd service on target hardware

```bash
bash deploy/build.sh --clean
sudo ./deploy/install_systemd.sh --bin-dir $(pwd)/build/bin --config config/hardware.json
sudo systemctl start drone-companion.target
sudo journalctl -u drone-mission-planner -f   # watch logs
```
