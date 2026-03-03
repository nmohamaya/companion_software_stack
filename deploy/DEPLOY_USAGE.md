# Deploy Scripts — Usage Guide

Quick reference for all build, test, and launch scripts in the `deploy/` folder.

---

## `build.sh` — Flexible Project Build

The main build script. Supports all IPC backends, sanitizers, coverage, and
format checking.

```bash
# ── Basic builds ─────────────────────────────────────────────
bash deploy/build.sh                          # Release, SHM backend
bash deploy/build.sh Debug                    # Debug, SHM backend
bash deploy/build.sh --zenoh                  # Release, Zenoh backend
bash deploy/build.sh Debug --zenoh            # Debug, Zenoh backend
bash deploy/build.sh --clean                  # Delete build/ before building

# ── Sanitizers (auto-switches to Debug) ──────────────────────
bash deploy/build.sh --asan                   # AddressSanitizer
bash deploy/build.sh --tsan                   # ThreadSanitizer
bash deploy/build.sh --ubsan                  # UndefinedBehaviorSanitizer
bash deploy/build.sh --asan --zenoh           # ASan + Zenoh backend

# ── Code coverage (auto-switches to Debug) ───────────────────
bash deploy/build.sh --coverage               # Build with gcov instrumentation
bash deploy/build.sh --coverage --zenoh       # Coverage + Zenoh backend

# ── Format check (no build — validates formatting only) ──────
bash deploy/build.sh --format-check           # Dry-run clang-format-18 check

# ── Combinations ─────────────────────────────────────────────
bash deploy/build.sh --clean --asan --zenoh   # Clean + ASan + Zenoh
```

| Flag             | Effect                                              |
| ---------------- | --------------------------------------------------- |
| `Debug\|Release` | Set CMake build type (default: `Release`)           |
| `--zenoh`        | Enable Zenoh IPC backend (default: POSIX SHM)       |
| `--clean`        | Remove `build/` before configuring                  |
| `--asan`         | Enable AddressSanitizer (forces Debug)               |
| `--tsan`         | Enable ThreadSanitizer (forces Debug)                |
| `--ubsan`        | Enable UndefinedBehaviorSanitizer (forces Debug)     |
| `--coverage`     | Enable gcov code-coverage instrumentation (forces Debug) |
| `--format-check` | Run `clang-format-18 --dry-run --Werror` and exit   |

> **Note:** Sanitizers are mutually exclusive — only pass one of `--asan`,
> `--tsan`, or `--ubsan` at a time.

---

## `clean_build_and_run_shm.sh` — Full SHM Pipeline

Performs a **complete cycle**: cleanup → clean build → unit tests → Gazebo SITL
launch, all using the POSIX shared-memory IPC backend.

```bash
bash deploy/clean_build_and_run_shm.sh                # Headless, Release
bash deploy/clean_build_and_run_shm.sh --gui           # With 3-D Gazebo GUI
bash deploy/clean_build_and_run_shm.sh --asan          # + AddressSanitizer
bash deploy/clean_build_and_run_shm.sh --tsan          # + ThreadSanitizer
bash deploy/clean_build_and_run_shm.sh --ubsan         # + UBSan
bash deploy/clean_build_and_run_shm.sh --coverage      # + code coverage
bash deploy/clean_build_and_run_shm.sh --gui --asan    # GUI + ASan
```

| Flag         | Effect                               |
| ------------ | ------------------------------------ |
| `--gui`      | Open the Gazebo 3-D visualisation    |
| `--asan`     | AddressSanitizer (forces Debug)      |
| `--tsan`     | ThreadSanitizer (forces Debug)       |
| `--ubsan`    | UBSan (forces Debug)                 |
| `--coverage` | gcov instrumentation (forces Debug)  |

---

## `clean_build_and_run_zenoh.sh` — Full Zenoh Pipeline

Same four-step cycle as the SHM variant, but using the **Zenoh** IPC backend.
Requires `zenohc` to be installed.

```bash
bash deploy/clean_build_and_run_zenoh.sh               # Headless, Release
bash deploy/clean_build_and_run_zenoh.sh --gui          # With 3-D Gazebo GUI
bash deploy/clean_build_and_run_zenoh.sh --asan         # + AddressSanitizer
bash deploy/clean_build_and_run_zenoh.sh --ubsan        # + UBSan
bash deploy/clean_build_and_run_zenoh.sh --coverage     # + code coverage
```

> **`--tsan` is intentionally blocked** for Zenoh builds because the `zenohc`
> library triggers ThreadSanitizer false-positives in its internal threading.
> Use `--tsan` with the SHM backend instead.

---

## `launch_all.sh` — Launch All Companion Processes

Starts all seven companion processes in the background using a given config
file. Typically called by the clean-build scripts, but can be used standalone:

```bash
CONFIG_FILE=config/gazebo.json bash deploy/launch_all.sh
```

---

## `launch_gazebo.sh` — Launch Gazebo SITL Simulation

Starts PX4 SITL + Gazebo + the companion stack:

```bash
bash deploy/launch_gazebo.sh              # Headless
bash deploy/launch_gazebo.sh --gui        # With 3-D visualisation
```

Set `CONFIG_FILE` to choose the IPC backend config:

```bash
CONFIG_FILE=config/gazebo.json       bash deploy/launch_gazebo.sh --gui   # SHM
CONFIG_FILE=config/gazebo_zenoh.json bash deploy/launch_gazebo.sh --gui   # Zenoh
```

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
bash deploy/build.sh --clean --coverage
cd build
ctest --output-on-failure
lcov --capture --directory . --output-file coverage.info --ignore-errors mismatch
lcov --remove coverage.info '/usr/*' '*/tests/*' --output-file coverage_filtered.info
genhtml coverage_filtered.info --output-directory coverage_html
echo "Open: ${PWD}/coverage_html/index.html"
```

### Full end-to-end with ThreadSanitizer and Gazebo GUI

```bash
bash deploy/clean_build_and_run_shm.sh --gui --tsan
```

### Auto-fix formatting on all source files

```bash
find common process[1-7]_* tests \( -name '*.h' -o -name '*.cpp' \) -print0 \
  | xargs -0 clang-format-18 -i
```
