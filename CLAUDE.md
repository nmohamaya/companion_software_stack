# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Production-grade C++17 autonomous drone software stack for companion computers (NVIDIA Jetson Orin or any Linux box). Runs as 7 independent Linux processes (21 threads total) communicating via Zenoh zero-copy pub/sub IPC.

## Multi-Agent Environment

Multiple AI agents work concurrently in this codebase using **git worktrees** for isolation. Each agent operates in its own worktree (a separate checked-out copy of the repo) so that concurrent work on different branches does not interfere. When you identify changes you did not make, review them for correctness and functional accuracy before proceeding — treat it as an opportunity to catch bugs or inconsistencies.

**Worktree awareness:**
- Your working directory may be a worktree (check with `git worktree list`)
- Never modify files in another agent's worktree
- Coordinate via branches and PRs, not direct file access across worktrees
- When merging or rebasing, be aware that other worktrees may have in-flight work on shared files

## Proactive Improvement Suggestions

When working on any task, **always look for and suggest improvements** you notice — even if they are outside the immediate scope of the task. This includes:
- **Infrastructure:** CI pipeline, build system, deployment scripts, dev tooling
- **Architecture:** Module boundaries, IPC design, error handling patterns, config structure
- **Code quality:** Performance, readability, safety, test coverage gaps
- **Functional:** Missing features, edge cases, untested code paths
- **Testing infrastructure:** Test runner bugs, scenario config gaps, flaky tests, coverage blind spots
- **Design patterns:** Better abstractions, RAII opportunities, template patterns, API ergonomics
- **Tools:** Developer experience improvements, debugging aids, profiling scripts, automation gaps
- **Documentation:** Stale numbers (test counts, version refs), broken links, missing install steps, unclear examples, contradictory guidance across docs
- **Scripts:** Install scripts, deploy scripts, CI workflows, build scripts — stale assumptions, missing error handling, outdated commands

Flag these as suggestions (don't silently implement them). Use your judgement on severity — mention critical issues immediately, batch minor suggestions at the end of your response.

**Where deferred items are logged:** two destinations, never mixed:

- **Proactive findings I noticed myself** (not in response to a review comment) → `docs/tracking/IMPROVEMENTS.md`. Backlog of nice-to-haves for quiet windows. Entries are dated, prioritised (P1/P2/P3), categorised, and moved to a **Resolved** section with a PR/commit reference once addressed.
- **Declining or disagreeing with a review comment** (from agents, Copilot, or humans) → `docs/tracking/DESIGN_RATIONALE.md` as a new DR-NNN entry. This is the audit trail proving the comment was evaluated and the call was intentional.

When writing "Review Fixes" tables on a PR and marking an item deferred, always include a DR-NNN reference. When flagging improvements in an end-of-task summary, add them to IMPROVEMENTS.md — don't leave them floating in conversation only.

**Safety issues are critical** — any memory safety violations, undefined behaviour, race conditions, missing error handling on flight-critical paths, or other issues that could cause loss of vehicle must be reported to the user **immediately** when noticed, not batched.

## Build Commands

**Canonical CMake build:**
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra" \
         -DALLOW_INSECURE_ZENOH=ON
make -j$(nproc)
```

`ALLOW_INSECURE_ZENOH=ON` is required on dev machines without TLS certificates. Zenoh is the only IPC backend and is always enabled.

**Via deploy script (wraps cmake):**
```bash
bash deploy/build.sh            # Release
bash deploy/build.sh --asan     # Debug + AddressSanitizer
bash deploy/build.sh --tsan     # Debug + ThreadSanitizer
bash deploy/build.sh --ubsan    # Debug + UBSanitizer
bash deploy/build.sh --coverage # Debug + coverage
```

All executables land in `build/bin/`.

**After any clean rebuild (`rm -rf build/*`):**
1. Reconfigure with the canonical command above
2. Verify test count: `ctest -N --test-dir build | grep "Total Tests:"` → must show **1259** (see [tests/TESTS.md](tests/TESTS.md) for current baseline)

## Test Commands

```bash
# Run all tests
./tests/run_tests.sh

# Run by module
./tests/run_tests.sh ipc
./tests/run_tests.sh watchdog
./tests/run_tests.sh perception
./tests/run_tests.sh mission
./tests/run_tests.sh comms
./tests/run_tests.sh hal
./tests/run_tests.sh util
./tests/run_tests.sh monitor
./tests/run_tests.sh quick          # Skip slow/E2E tests

# Options
./tests/run_tests.sh watchdog --verbose
./tests/run_tests.sh ipc --repeat 5
./tests/run_tests.sh --build --asan

# Build and test in one step
bash deploy/build.sh --test
bash deploy/build.sh --test-filter watchdog

# Via ctest
ctest --test-dir build --output-on-failure -j$(nproc)
```

**Before reporting "all tests pass" — verify:**
- [ ] Correct branch? (`git branch --show-current`)
- [ ] Test count matches baseline? (currently **1707** — see [tests/TESTS.md](tests/TESTS.md))
- [ ] Zero compiler warnings? (build uses `-Werror -Wall -Wextra`)
- [ ] clang-format clean? (`git diff --name-only | xargs clang-format-18 --dry-run --Werror`)

See `tests/TESTS.md` for the full test inventory.

## Lint and Format

```bash
# Check formatting (CI gate)
bash deploy/build.sh --format-check

# Auto-fix formatting
find common process[1-7]_* tests tools \( -name '*.h' -o -name '*.cpp' \) -print0 \
  | xargs -0 clang-format-18 -i
```

Code style: clang-format-18 (100-char columns, Attach braces, 4-space indents — see `.clang-format`).
Static analysis: clang-tidy-18 (see `.clang-tidy`). CI enforces zero warnings.

## Architecture

### Process Map

| Process | Threads | Role |
|---------|---------|------|
| P1 `process1_video_capture` | 3 | Camera frame acquisition |
| P2 `process2_perception` | 6 | Detection → Tracking → Sensor fusion |
| P3 `process3_slam_vio_nav` | 4 | Visual-inertial odometry + navigation |
| P4 `process4_mission_planner` | 1 | FSM + path planning + obstacle avoidance |
| P5 `process5_comms` | 5 | Flight controller & GCS communication |
| P6 `process6_payload_manager` | 1 | Gimbal & camera control |
| P7 `process7_system_monitor` | 1 | Health monitoring & process supervision |

### IPC Channels

Processes communicate exclusively through typed IPC channels (no shared memory outside the bus):

```
P1 → /drone_mission_cam   → P2
P1 → /drone_stereo_cam    → P3
P2 → /detected_objects    → P4
P3 → /slam_pose           → P4, P5, P6
P4 → /trajectory_cmd      → P5
P4 → /fc_commands         → P5
P5 → /fc_state            → P4, P7
P5 → /gcs_commands        → P4
P6 → /payload_status      → P4, P7
P7 → /system_health       → P4
```

IPC backend is Zenoh (the only supported backend). Runtime config in `config/default.json → ipc_backend` must be `"zenoh"`.

### Common Libraries (`common/`)

- **`common/ipc/`** — Zenoh publisher/subscriber/MessageBus (wire format, liveliness, network config)
- **`common/hal/`** — Hardware Abstraction Layer: ICamera, IDetector, IFCLink, IGCSLink, IGimbal, IIMUSource, IPathPlanner, IObstacleAvoider, IProcessMonitor + all backends + factory
- **`common/util/`** — Config, `Result<T,E>`, arg_parser, correlation IDs, diagnostic, JSON log, latency tracker, process graph, restart policy, ThreadHeartbeat, ThreadWatchdog, sd_notify, signal handler

### Hardware Abstraction

All hardware access goes through HAL interfaces. Implementations:
- **Simulated** — runs on any Linux box, used by unit tests and quick dev cycles
- **Gazebo** — Gazebo SITL (gz-transport13/gz-msgs10) for integration tests
- **Real** — V4L2/libargus, MAVSDK (MAVLink), UART/PWM, SPI/I2C (Jetson Orin)

HAL backends are selected via `config/default.json` (e.g., `video_capture.mission_cam.backend`).

### Error Handling Pattern

The codebase uses `Result<T,E>` (monadic, no exceptions) throughout. All public APIs return `[[nodiscard]] Result<...>`. Chain with `.map()`, `.and_then()`, `.map_err()`.

### Three-Layer Watchdog / Fault Recovery

1. **Thread** — `ThreadHeartbeat` (lock-free atomic touch, ~1 ns); `ThreadWatchdog` detects stuck threads
2. **Process** — `ProcessManager` (fork+exec, exponential backoff, dependency graph) in P7
3. **OS** — systemd service units (`Type=notify`, `WatchdogSec=10s`, `BindsTo=` deps) in `deploy/systemd/`

### Simulation Config Principle

**Always maximise stack coverage in simulation.** Gazebo configs should use the full planning/avoidance stack (A* planner, 3D obstacle avoider, HD-map static obstacles) — the same code paths as real hardware. Only simplify when a scenario specifically tests something else (e.g. fault injection) and the full stack would interfere — in that case, disable only the minimum necessary (e.g. clear `static_obstacles: []` rather than downgrading the planner backend).

### Configuration

Runtime config via `config/default.json` (95+ parameters). Access in C++:
```cpp
drone::Config cfg;
cfg.load("config/default.json");
int w = cfg.get<int>("video_capture.mission_cam.width", 1920);
auto section = cfg.section("mission_planner");
```

## Development Workflow (from `docs/guides/DEVELOPMENT_WORKFLOW.md`)

### Branch & Commit Conventions
- Branch naming: `feature/issue-XX-description`, `fix/issue-XX-description`, `refactor/issue-XX-description`, `docs/issue-XX-description`
- Commit format: `type(#issue): subject` — types: `feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf`
- PR title: `feat(#XX): description`, body must link issue with `Closes #XX`

### PR Size Guidelines
- **Keep PRs small and focused** — each PR should address a single concern (one feature, one bug fix, one refactor). Small PRs are faster to review, easier to reason about, and less likely to introduce regressions.
- **Target: <400 lines changed** (excluding auto-generated files and test fixtures). If a PR exceeds this, consider splitting it.
- **Split large features into phases** — use sub-issues (e.g., `#100-part-1`, `#100-part-2`) with sequential PRs. Each phase should be independently buildable and testable.
- **Commits within a PR should be logical units** — each commit should compile and pass tests on its own. Don't mix unrelated changes in a single commit.
- **When splitting isn't practical** (e.g., tightly coupled refactor), explain the scope in the PR description and organize commits to make review tractable.

### Implementation Workflow (Steps 1–9)
1. **Create a GitHub Issue** with description, acceptance criteria, labels
2. **Branch from main:** `git checkout main && git pull && git checkout -b feature/issue-XX-desc`
3. **Implement** — commit frequently, reference issue numbers, add/update tests, use `cfg.get<>()` for all tunables
4. **Pre-push verification (all required):**
   - Build: `bash deploy/build.sh Release` (zero warnings)
   - Format: `find common process[1-7]_* tests \( -name '*.h' -o -name '*.cpp' \) -print0 | xargs -0 clang-format-18 --dry-run --Werror`
   - Tests: `./tests/run_tests.sh` (100% pass rate)
   - Local CI: `bash deploy/run_ci_local.sh --quick` (minimum before every push)
   - Full local CI: `bash deploy/run_ci_local.sh` (before creating PR)
5. **Create PR** — push branch, CI runs automatically
6. **Address review** — batch by priority (P1: critical bugs, P2: memory/perf, P3: code quality, P4: tests/docs)
7. **Update documentation before merge** (see Documentation Updates below)
8. **Merge** — Squash and merge via GitHub UI preferred
9. **Cleanup** — delete feature branch locally and on remote

### Local CI Check (mirrors GitHub Actions)
```bash
bash deploy/run_ci_local.sh              # Full: format + build + sanitizers + coverage
bash deploy/run_ci_local.sh --quick      # Quick: format + build + tests
bash deploy/run_ci_local.sh --job FMT    # Format only
bash deploy/run_ci_local.sh --job BUILD  # Build + tests
bash deploy/run_ci_local.sh --job ASAN   # AddressSanitizer
bash deploy/run_ci_local.sh --job TSAN   # ThreadSanitizer
bash deploy/run_ci_local.sh --job UBSAN  # UBSanitizer
bash deploy/run_ci_local.sh --job COV    # Coverage report
```

### Documentation Updates (required before merge)
Every PR must update the relevant docs:
- **`docs/tracking/PROGRESS.md`** — improvement entry (number, title, date, files, what/why, test additions)
- **`docs/tracking/ROADMAP.md`** — mark issues done (strikethrough + checkmark), update metrics table
- **`docs/tracking/BUG_FIXES.md`** — entry for every bug fix (root cause, fix, "found by")
- **`docs/design/API.md`** — when interfaces or IPC classes change
- **`tests/TESTS.md`** — when adding/modifying tests (new file entry, update counts)
- **`docs/tracking/CI_ISSUES.md`** — for CI-specific failures (symptoms, root cause, fix, prevention)

### Multi-Phase Features
- Each phase gets a separate sub-issue and PR
- For sequential phases: branch from updated main after prior phase merges (preferred), or branch from predecessor and rebase after it merges
- Never branch a later phase from stale main when both phases touch the same files
- Optional dependencies use compile guards: `find_package(X QUIET)` + `#ifdef HAVE_X`
- CI must always pass without optional dependencies

### Code Patterns
- All tunables via `drone::Config` — no hardcoded magic numbers
- RAII for all resources (file descriptors, threads)
- `MessageBusFactory::create_message_bus()` — never hardcode Zenoh in process code
- All IPC structs must be trivially copyable
- `std::memory_order_acquire/release` for lock-free patterns
- Zenoh tests must use `RESOURCE_LOCK` to prevent parallel session exhaustion
- New bugs get a regression test before the fix

### Review Fix Protocol
After addressing review comments:
1. Commit with list of fixes in body: `fix: address PR #N review comments`
2. Update PR body with "Review Fixes" table mapping each comment to the fix
3. Post PR comment summarizing fixes grouped by category
4. Re-run build + tests before pushing

**Codebase-wide scope:** When a review comment identifies an issue that could apply elsewhere in the codebase (e.g., misleading comments, inconsistent log levels, missing test patterns), always search for and fix all occurrences — not just the one the reviewer pointed out. A review comment on one file is a signal to check the whole codebase for the same pattern.

**Disagreeing with review comments:** When declining to fix a review comment (from Copilot, review agents, or humans) based on a justified rationale, **always document the decision in `docs/tracking/DESIGN_RATIONALE.md`** as a new DR-NNN entry. This creates an audit trail showing the comment was evaluated, the trade-offs were weighed, and the decision was intentional — not an oversight. Include the question, arguments for both sides, our decision, and when to revisit. This applies to both P3 deferrals and genuine "we disagree" situations.

### Planning & Self-Correction (from `docs/guides/work_instructions.md`)
- Enter plan mode for any non-trivial task (3+ steps or architectural decisions)
- If something goes sideways, stop and re-plan — don't keep pushing
- Write plans to `tasks/todo.md` with checkable items; mark items complete as you go
- Use subagents to keep the main context window clean
- After any correction from the user: update `tasks/lessons.md` with the pattern
- Review `tasks/lessons.md` at session start
- When given a bug report: fix it directly — don't ask for hand-holding

### Safety-Critical C++ Practices

This is a **safety-critical drone software stack**. Use appropriate C++ constructs and design patterns at all times.

**Constructs to AVOID** (unless used for fully non-safety, non-functional code):
- `memcpy` / `memset` / `memmove` — use `std::copy`, `std::fill`, value semantics, constructors, or `std::array`
- Raw owning pointers (`new`/`delete`) — use RAII, `std::unique_ptr`, or value types
- `std::shared_ptr` — prefer `std::unique_ptr` or value semantics; shared ownership is a code smell in safety code (acceptable for external library contracts like spdlog/MAVSDK/Zenoh callbacks)
- C-style casts `(int)x` — use `static_cast`, `dynamic_cast`, `reinterpret_cast`
- `reinterpret_cast` on safety-relevant data — only acceptable for IPC wire-format serialization of trivially copyable structs
- `volatile` — not a synchronization primitive; use `std::atomic`
- `goto` — use structured control flow
- `atoi` / `atof` / `strtol` — no error checking; use `std::from_chars` (C++17) or `std::stoi`
- Bare `std::thread` without join/detach guarantees — use RAII wrappers
- `using namespace` in header files — namespace pollution
- Uninitialized variables — especially Eigen types (`= ::Zero()`) and numeric types in flight-critical paths
- Hardcoded magic numbers — all tunables via `drone::Config`
- `exit()` / `abort()` / `std::terminate()` — in library code, prevents graceful shutdown
- Unscoped `enum` — use `enum class` for type safety
- Unbounded recursion
- Global mutable state outside of explicitly documented singletons
- **Unguarded signed→unsigned casts on durations/sizes/counts** — negative `int64_t` cast to `uint64_t` wraps to ~2^64, causing infinite waits or massive allocations. Always clamp: `static_cast<uint64_t>(std::max(int64_t{0}, val))`. Also watch for unsigned subtraction underflow (`a - b` where `a < b`) and timestamp addition overflow (`now + offset` near max)

**Constructs to PREFER:**
- `[[nodiscard]]` on all functions returning `Result<T,E>` or error codes
- `const` correctness — parameters, member functions, local variables
- `constexpr` where possible for compile-time evaluation
- RAII for all resources (files, threads, locks, sockets)
- `std::atomic` with explicit memory ordering (`acquire`/`release`) for lock-free patterns — never use `relaxed` unless you can prove no synchronization is needed
- Strong types over bare primitives where confusion is possible (e.g., `Meters` vs `float`)
- `= delete` on copy for non-copyable resources
- `override` on all virtual overrides
- `noexcept` on move constructors/operators and destructors
- Scoped enums (`enum class`) over unscoped enums
- Value types and move semantics
- Design patterns: Strategy, Factory, Template Method
- Fixed-width integer types (`uint32_t`, `int16_t`) for protocol/wire data
- Default member initializers on all struct/class fields
- `static_assert(std::is_trivially_copyable_v<T>)` on all wire-format/IPC structs before `reinterpret_cast`

**Concurrency tiering** (use the simplest mechanism that provides correct synchronization):
- **Lock-free (`std::atomic`):** Hot path only — `ThreadHeartbeat`, inter-thread flags. Always specify memory order explicitly (`acquire`/`release`).
- **Mutex (`std::lock_guard`):** Non-hot-path shared state — config access, ring buffers, logging. Always use RAII (`lock_guard`/`unique_lock`), never manual `lock()`/`unlock()`.
- **Avoid:** Recursive mutexes (restructure code instead), `memory_order_relaxed` without justification, bare `lock()`/`unlock()` calls.

**Observability on flight-critical threads:** Mutex-protected observability primitives (loggers, profilers, metrics collectors — e.g. `LatencyProfiler`, `JsonLogSink`) SHOULD NOT be called from flight-critical or real-time threads (P2 detector/tracker hot paths, P3 VIO backend, P4 planner tick, IPC callbacks, watchdog touch paths) *without documented justification*. The default hazards are (a) **priority inversion** — a lower-priority thread holding the observability mutex can block a higher-priority control thread, and (b) **observation affecting measurement** — the mutex cost contaminates the latency being measured. If a control-loop thread needs to emit telemetry, the preferred pattern is to buffer into a lock-free primitive (`LatencyTracker`, `SPSCRing`, `TripleBuffer`) and let a dedicated IO thread drain into the shared observability. If mutex-protected observability is used on a real-time thread anyway (e.g. benchmark-harness profiler wiring), record the analysis as a DR-NNN entry in `docs/tracking/DESIGN_RATIONALE.md` showing (1) all recorders share similar priority (no inversion risk), (2) mutex-hold-time is bounded and dominated by the measured work, and (3) the usage is gated behind an explicit configuration flag so production builds don't pay the cost. Each observability primitive's header must document the constraint (e.g. "For >10 kHz hot loops, prefer `LatencyTracker` directly").

### Root Cause Analysis

**Always fix the root cause** of issues rather than fixing symptoms or updating tests without understanding the underlying problem. When a test fails, investigate *why* the code produced that output before changing the test. If a fix requires changing test expectations, explain why the new expectation is correct. Treating symptoms masks real bugs that will resurface in production.

### Known Pitfalls (from `tasks/lessons.md`)

**Don't trust test counts — verify after build**
After any build, compare test count to the known baseline (see [tests/TESTS.md](tests/TESTS.md) for current count). A passing run at the wrong count is not "all tests pass".

**Stale object files from different build types**
Mixing Release and Coverage builds in the same directory causes `__gcov_init` linker errors. Use `rm -rf build/*` and a clean rebuild when switching build types. Consider separate directories (`build-release/`, `build-coverage/`).

**PR body updates — do it immediately after fixes**
After pushing review-fix commits, update the PR body right away. Don't wait to be asked.

### Common Issues

| Issue | Solution |
|-------|----------|
| Eigen uninitialized warnings | Always `= Eigen::Vector3f::Zero()` |
| Zenoh test SIGABRT under `ctest -j` | Add `RESOURCE_LOCK "zenoh_session"` |
| CI fails but local passes | Check for Anaconda `LD_LIBRARY_PATH` masking |

## Key Documentation

- `README.md` — Full system overview with architecture diagrams
- `docs/tracking/PROGRESS.md` / `docs/tracking/ROADMAP.md` — Improvement history and planned work
- `docs/design/perception_design.md` — P2 pipeline detail
- `docs/design/hardening-design.md` — Watchdog and systemd integration
- `docs/guides/CPP_PATTERNS_GUIDE.md` — Project C++17 patterns (Result<T,E>, ScopedGuard, thread safety)
- `docs/tracking/DESIGN_RATIONALE.md` — Gray-area design decisions where both sides are defensible (DR-NNN entries)
- `docs/tracking/BUG_FIXES.md` — 29 documented bugs fixed (good reference for common pitfalls)
- `docs/adr/` — Architecture Decision Records
- `docs/design/API.md` — IPC message type reference
- `tests/TESTS.md` — Full test inventory

## Deployment

```bash
bash deploy/launch_all.sh          # 7 processes locally (simulated backends)
bash deploy/launch_gazebo.sh       # 7 processes + Gazebo SITL + PX4
bash deploy/launch_hardware.sh     # Real hardware (Jetson Orin)
bash deploy/install_systemd.sh     # Install systemd units
bash deploy/view_coverage.sh       # Generate + open coverage HTML report
```

Scenario integration tests: `tests/run_scenario.sh` (8 JSON configs in `config/scenarios/`).

## Multi-Agent Pipeline (ADR-010)

This project uses a **17-agent pipeline** orchestrated via Claude Agent SDK with a **two-pass review architecture**. See `docs/adr/ADR-010-multi-agent-pipeline-architecture.md` for the full architecture decision record.

### Agent Roster

| # | Role | Model | Scope |
|---|------|-------|-------|
| 1 | `tech-lead` | Opus | Orchestration, routing, merge decisions |
| 2 | `feature-perception` | Opus | P1/P2, camera/detector HAL |
| 3 | `feature-nav` | Opus | P3/P4, planner/avoider HAL |
| 4 | `feature-integration` | Opus | P5/P6/P7, IPC, HAL backends |
| 5 | `feature-infra-core` | Opus | common/, CMake, config |
| 6 | `feature-infra-platform` | Opus | deploy/, CI, boards/, certification |
| 7 | `review-memory-safety` | Opus | RAII, ownership, lifetimes (Pass 1, read-only) |
| 8 | `review-concurrency` | Opus | Races, atomics, deadlocks (Pass 1, read-only) |
| 9 | `review-fault-recovery` | Sonnet | Watchdog, degradation (Pass 1, read-only) |
| 10 | `review-security` | Sonnet | Input validation, auth, TLS (Pass 1, read-only) |
| 11 | `test-unit` | Sonnet | GTest, coverage delta (Pass 1, tests/ only) |
| 12 | `test-scenario` | Sonnet | Gazebo SITL, integration (Pass 1, tests/ only) |
| 13 | `review-test-quality` | Opus | Tests exercise new paths, assertions meaningful (Pass 2, read-only) |
| 14 | `review-api-contract` | Sonnet | Docstrings match impl, data consistency (Pass 2, read-only) |
| 15 | `review-code-quality` | Sonnet | Dead code, DRY, complexity, naming (Pass 2, read-only) |
| 16 | `review-performance` | Sonnet | Copies, allocation, hot paths, O(n^2) (Pass 2, read-only) |
| 17 | `ops-github` | Haiku | Issue triage, milestones, board (gh CLI) |

### Quick Commands

```bash
# All commands use the Python orchestrator (PYTHONPATH=scripts is required)
export PYTHONPATH=scripts

# List available agents
python3 -m orchestrator list

# Launch an interactive session for a role
python3 -m orchestrator start feature-perception

# Launch with a specific task (non-interactive)
python3 -m orchestrator start feature-nav "Implement issue #315: add version fields"

# Deploy an agent for a GitHub issue (auto-routes by labels)
python3 -m orchestrator deploy-issue 123

# Deploy with pipeline mode (5 checkpoints, automated between)
python3 -m orchestrator deploy-issue 123 --pipeline

# Deploy with mobile notifications at checkpoints
python3 -m orchestrator deploy-issue 123 --pipeline --notify drone-pipeline-yourname

# Launch review agents for a PR (auto-routes by diff content)
python3 -m orchestrator deploy-review 456

# Run a full orchestrated session
python3 -m orchestrator session feature-nav "Implement VIO health fault escalation"

# Validate a session (hallucination detection)
python3 -m orchestrator validate

# View agent dashboard
python3 -m orchestrator dashboard                     # team-wide (default)
python3 -m orchestrator dashboard feature-perception  # per-agent
```

### Review Routing (Two-Pass)

**Pass 1 — Safety & Correctness** (parallel, diff-routed):

| Change touches... | Agents triggered |
|-------------------|-----------------|
| Any file | Memory Safety + Security + Unit Test |
| `std::atomic`, `mutex`, `thread` | + Concurrency |
| P4/P5/P7, watchdog, fault handling | + Fault Recovery |
| IPC, HAL, Gazebo configs | + Scenario Test |

**Pass 2 — Quality & Contracts** (parallel after Pass 1, always-on):

All 4 agents run on every PR and receive Pass 1 findings as context:
Test Quality, API Contract, Code Quality, Performance

### Shared State

- `tasks/active-work.md` — Live work tracker (read at session start)
- `tasks/agent-changelog.md` — Completed work log (append at session end)
- `.claude/shared-context/domain-knowledge.md` — Non-obvious pitfalls all agents should know
- `docs/guides/AGENT_HANDOFF.md` — Cross-domain handoff protocol

## graphify

This project has a graphify knowledge graph at graphify-out/.

Rules:
- Before answering architecture or codebase questions, read graphify-out/GRAPH_REPORT.md for god nodes and community structure
- If graphify-out/wiki/index.md exists, navigate it instead of reading raw files
- After modifying code files in this session, run `python3 -c "from graphify.watch import _rebuild_code; from pathlib import Path; _rebuild_code(Path('.'))"` to keep the graph current
