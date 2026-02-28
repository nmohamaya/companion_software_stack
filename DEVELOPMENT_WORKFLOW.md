## Development Workflow

This project follows a structured development process designed to catch issues early and maintain code quality. All changes must go through this workflow.

**Stack:** C++17 · CMake 3.16+ · Google Test · spdlog · Eigen3 · nlohmann/json  
**Repo:** https://github.com/nmohamaya/companion_software_stack  
**CI:** GitHub Actions (Ubuntu 24.04, `-Werror -Wall -Wextra`)

---

### 🔄 Complete Development Workflow

#### Step 1: Issue Management & Planning
Before starting any work:
1. **Create an Issue** (required)
   - Title format: `[Type] Feature/fix description`
   - Add detailed description, acceptance criteria, and expected behavior
   - Add relevant labels (`bug`, `feature`, `enhancement`, `testing`, `refactor`)

2. **Set Issue Status**
   - Mark as "In Progress" when you start working

#### Step 2: Create a Feature Branch
```bash
git checkout main && git pull
git checkout -b feature/issue-XX-short-description
```

**Branch Naming Convention:**
- Feature: `feature/issue-XX-description`
- Bug fix: `fix/issue-XX-description`
- Refactor: `refactor/issue-XX-description`
- Documentation: `docs/issue-XX-description`

#### Step 3: Implement Changes
- Write code following C++17 conventions (RAII, header-only where appropriate)
- Commit frequently with clear messages
- Reference issue numbers in commits: `fix(#XX): description`
- Add or update unit tests for any changed logic
- Use `cfg.get<>()` for all tunables — no magic numbers in process code

#### Step 4: Pre-Push Verification

**All PRs must pass these checks locally before pushing.**

##### 4a. Build (zero warnings required)
```bash
cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DGTest_DIR=/usr/lib/x86_64-linux-gnu/cmake/GTest ..
make -j$(nproc)
```
- All 7 binaries + all test targets must compile
- Zero compiler warnings

##### 4b. Tests (100% pass rate required)
```bash
LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH" \
  ctest --output-on-failure -j$(nproc)
```
- All tests must pass (currently 121 across 10 suites)
- No regressions in existing tests
- New features must include tests

> **Note:** The `LD_LIBRARY_PATH` and `GTest_DIR` overrides are only needed on machines with Anaconda installed. On clean Ubuntu or in CI, plain `cmake .. && make && ctest` works.

##### 4c. Smoke Test (for IPC/process changes)
```bash
cd build/bin
./video_capture &
sleep 1 && ./perception &
sleep 2
kill %1 %2
```
- Verify SHM segments created/cleaned up
- No segfaults or assertion failures

##### 4d. Integration / Simulation Tests (when applicable)
For changes that interact with external simulators or hardware:
```bash
# Example: run Gazebo SITL integration test
./build/bin/integration_test_gazebo --config config/gazebo_sitl.json
```
- Only required when the change involves HAL backends or end-to-end pipelines
- These tests are **not** part of the CI gate (simulators may not be installed)
- Mark simulation-dependent tests with a GTest filter tag: `TEST(Integration, ...)`
- Document any required environment setup in the PR description

#### Step 5: Create Pull Request
1. Push branch: `git push origin feature/issue-XX-description`
2. Create PR with:
   - Clear title: `feat(#XX): description`
   - Description linking issue: `Closes #XX`
   - Acceptance criteria checklist
3. **CI pipeline runs automatically** — build with `-Werror` + all tests on Ubuntu 24.04
4. Wait for CI green status

#### Step 6: Address Review Feedback
- Respond to all reviewer comments
- Make requested changes in new commits
- Re-run local build + tests after fixes
- Mark conversations as resolved

#### Step 7: Update Documentation
Before merging, update the project's tracking documents:

1. **`PROGRESS.md`** — Add an entry under the current Phase section:
   - Improvement number, title, date, category
   - Files added/modified
   - What was done and why
   - Test coverage additions (if any)
   - Updated summary metrics table

2. **`ROADMAP.md`** — Update the relevant sections:
   - Mark completed issues as done in the phase tables (strikethrough + ✅)
   - Update the issue tracking table (change state from "Open" to "Closed")
   - Update the metrics history table with the new column

3. **`API.md`** — Update when interfaces or IPC classes change:
   - Add/update API tables for new or modified interfaces
   - Update wrapper class documentation (purpose, "Why a wrapper?" rationale)
   - Update topic mapping tables if IPC channels change
   - Update test coverage table with new test counts and suites
   - Mark planned items as implemented when delivered

4. **`BUG_FIXES.md`** — Add an entry for every bug fix:
   - Fix number, date, severity, affected file(s)
   - Bug description with root cause analysis
   - Fix description and approach
   - "Found by" field (test name, CI failure, manual testing, etc.)

5. **`CI_ISSUES.md`** — Add an entry for every CI-specific failure:
   - Symptoms (exact error message from CI logs)
   - Root cause analysis (why CI differs from local)
   - Fix applied (commits, files changed)
   - Prevention strategy (how to avoid this class of issue)
   - Use the template at the bottom of the file for consistent formatting

6. **When to update:** Include doc updates in the same PR branch, committed before merge.

#### Step 8: Merge to Main
Once CI passes and review is approved:

```bash
# Via GitHub UI (recommended):
# 1. Click "Squash and merge"
# 2. Use commit message: "feat(#XX): description"

# OR manually:
git checkout main && git pull
git merge --squash feature/issue-XX-description
git commit -m "feat(#XX): description"
git push origin main
```

#### Step 9: Post-Merge Cleanup
```bash
git checkout main && git pull
git branch -d feature/issue-XX-description
git push origin --delete feature/issue-XX-description
```

---

## 🎯 Multi-Phase Feature Development

For large features broken into multiple phases (e.g., Hardware Abstraction Layer), use this workflow:

### Phase Implementation Strategy

Each phase is a separate sub-issue with its own PR, allowing incremental development and review.

```
Parent Issue (#XX): Hardware Abstraction Layer
├─ Phase 1 (#YY): Define interfaces (ICamera, IFCLink, IGimbal)
│  ├─ Branch: feature/issue-XX-hal
│  └─ PR: #ZZ
├─ Phase 2 (#YY): Factory pattern + config-driven selection
│  └─ PR: #ZZ
├─ Phase 3 (#YY): Integration tests
│  └─ PR: #ZZ
└─ Phase 4+: Real hardware backends
```

### Dependency Graph

When creating an epic, include a dependency graph showing which phases can run in parallel and which are sequential. This prevents wasted effort and clarifies the critical path.

```
Phase 0 (Environment Setup)
   │
   ├──→ Phase 1 (Backend A)  ──┐
   ├──→ Phase 2 (Backend B)  ──┼──→ Phase 4 (Integration Test)
   └──→ Phase 3 (Backend C)  ──┘
```

- Phases without arrows between them can be developed in parallel
- Mark blocking dependencies explicitly in sub-issue descriptions
- Update the graph as phases complete or new dependencies emerge

### Sequential Phase Branching Strategy

When phases are **sequential** (Phase N depends on Phase N-1), choosing the right branching point avoids merge conflicts:

| Strategy | When to Use | Trade-off |
|----------|-------------|-----------|
| **Wait for merge** | Default — Phase N-1 review is quick | Zero conflicts, cleanest history |
| **Branch from predecessor** | Phase N-1 is in review, you want to start early | Works well, but requires rebase after N-1 merges |
| **Branch from stale main** | ❌ Avoid this | Both phases modify the same files → painful conflicts |

**Preferred workflow (wait for merge):**
```bash
# Phase A merged to main via PR
git checkout main && git pull
git checkout -b feat/48-phase-b         # Branch from up-to-date main
# ... implement Phase B, create PR, merge ...

git checkout main && git pull
git checkout -b feat/49-phase-c         # Branch from main that includes Phase B
# ... no conflicts with Phase B code
```

**Early-start workflow (branch from predecessor):**
```bash
# Phase B PR is open, review in progress — you want to start Phase C now
git checkout feat/47-phase-b
git checkout -b feat/48-phase-c         # Branch FROM Phase B, not main
# ... implement Phase C ...

# After Phase B merges to main:
git checkout feat/48-phase-c
git fetch origin
git rebase origin/main                  # Rebase onto main (which now has Phase B)
# Conflicts are minimal because your base already had Phase B's code
git push --force-with-lease origin feat/48-phase-c
```

**Why this matters — a real example:**

In the Zenoh migration (Epic #45), Phase B added 10 per-channel round-trip tests to `tests/test_zenoh_ipc.cpp`, and Phase C added 10 SHM zero-copy tests to the same file. Phase C was branched from `main` *before* Phase B merged. When Phase B merged and Phase C tried to rebase, git found 3 conflict regions because both phases inserted code after the same anchor point (`RoundTripViaFactory` test).

The fix was straightforward (keep both sets of tests), but the conflict could have been avoided entirely by either:
1. Waiting for Phase B's PR to merge before branching Phase C, or
2. Branching Phase C from Phase B's branch

**Rule of thumb:** If two phases touch the same files (especially test files or shared headers), *never branch the later phase from a main that doesn't include the earlier phase*.

### Steps

1. **Create Sub-Issues for Each Phase**
   ```markdown
   Title: feat(#XX): [Phase N] Description
   Description:
   - Detailed requirements
   - Dependencies on previous phases
   - Expected deliverables
   - Success criteria
   ```

2. **Single Feature Branch for All Phases** (or one per phase for large changes)
   ```bash
   git checkout -b feature/issue-XX-hal
   ```

3. **Implement, Test, PR**
   ```bash
   make -j$(nproc)                # Build
   ctest --output-on-failure      # Tests pass
   git push origin feature/...    # Create PR
   ```

4. **Track Deferred Work**
   - If leaving TODOs for later phases, create a GitHub issue
   - Link in code: `// TODO(#YY): implement real V4L2 backend`
   - Label as `deferred` or `technical-debt`

5. **Optional Dependencies (compile guards)**
   When a phase introduces an external dependency (e.g., MAVSDK, Gazebo), keep it **optional** so the core stack always builds without it:
   ```cmake
   # CMakeLists.txt — look for dependency, don't fail if absent
   find_package(MAVSDK QUIET)
   if(MAVSDK_FOUND)
     target_compile_definitions(my_target PRIVATE HAVE_MAVSDK)
     target_link_libraries(my_target PRIVATE MAVSDK::mavsdk)
   endif()
   ```
   ```cpp
   // hal_factory.h — guard backend registration
   #ifdef HAVE_MAVSDK
   #include "hal/mavlink_fc_link.h"
   #endif
   ```
   - CI must always pass **without** the optional dependency installed
   - Document install steps in `docs/` for developers who need the backend
   - Use `HAVE_<LIB>` naming convention for all compile guards

### Review Comment Handling

Batch review comments by severity:

| Priority | Category | Response Time | Examples |
|---|---|---|---|
| **P1** | Critical bugs | Same-day | Use-after-free, data corruption, crash |
| **P2** | Memory/Performance | Within PR | Unbounded allocations, race conditions |
| **P3** | Code quality | Within PR | Unused variables, naming, style |
| **P4** | Tests/Docs | Before merge | Missing edge-case tests, comments |

### Review Fix Documentation

When addressing review comments, maintain a clear record of what was changed and why:

1. **Commit message** — list each fix in the commit body:
   ```
   fix: address PR #N review comments

   - Fix A: description
   - Fix B: description
   ```

2. **Update PR body** — add a "Review Fixes" section with a table mapping each comment to the fix applied:
   ```markdown
   ## Review Fixes (commit `abc1234`)
   | # | File | Issue | Fix |
   |---|------|-------|-----|
   | 1 | file.h | Description of problem | What was changed |
   ```

3. **Post a PR comment** — summarize all fixes grouped by category (Correctness, Robustness, Testing, Documentation) so reviewers can quickly verify each one.

4. **Re-run build + tests** — always verify `cmake --build build -j$(nproc)` and `ctest --output-on-failure` pass before pushing review fixes.

This ensures:
- Reviewers can efficiently re-review without re-reading all changed files
- Future contributors understand *why* certain patterns were chosen
- The PR serves as a permanent reference for design decisions

---

## Commit Message Standards

```
type(#issue): subject

body (optional)
```

**Types:**
| Type | Usage |
|---|---|
| `feat` | New feature |
| `fix` | Bug fix |
| `refactor` | Code restructuring (no behavior change) |
| `test` | Test additions or fixes |
| `docs` | Documentation |
| `chore` | Build, dependencies, tooling |
| `perf` | Performance improvement |

**Examples:**
```
feat(#12): add V4L2 camera backend
fix(#7): resolve ShmWriter double-unlink on move
refactor(#15): extract obstacle avoidance into separate class
test(#20): add integration tests for video→perception pipeline
chore(#25): upgrade spdlog to 1.13.0
```

---

## Development Best Practices

### Code Quality
- ✅ All tunables via `drone::Config` — no hardcoded magic numbers in process code
- ✅ RAII for all resources (SHM, file descriptors, threads)
- ✅ Header-only for small utility classes; `.cpp` for implementations with dependencies
- ✅ Keep functions focused and small
- ✅ Document complex logic with comments

### Git Hygiene
- ✅ Commit frequently (1 commit per logical unit)
- ✅ Keep branches short-lived (<3 days)
- ✅ Always pull before pushing
- ✅ Never force-push to `main`

### Testing Strategy
- ✅ Write tests alongside code, not after
- ✅ Test both happy path and error cases
- ✅ Use meaningful test names: `TEST(Component, BehaviorUnderCondition)`
- ✅ Keep tests isolated — no cross-test dependencies
- ✅ New bugs get a regression test before the fix

### IPC & Concurrency
- ✅ All SHM structs must be trivially copyable
- ✅ Use `PoseDoubleBuffer` pattern for cross-thread exchange (not raw pointers)
- ✅ `SPSCRing<T, N>` for producer-consumer queues
- ✅ `std::memory_order_acquire`/`release` for lock-free patterns

---

## Common Workflow Issues & Solutions

| Issue | Cause | Solution |
|---|---|---|
| Merge conflicts | Long-lived branch | Rebase frequently: `git rebase main` |
| Test failures after merge | Local testing incomplete | Run full `ctest` before creating PR |
| CI fails but local passes | Anaconda `LD_LIBRARY_PATH` masking issues | CI uses clean Ubuntu; check for Anaconda-specific workarounds in local build |
| SHM segment leak | Process killed without cleanup | Use `launch_all.sh` which traps signals, or run `rm /dev/shm/drone_*` |
| Eigen warnings | Uninitialized members | Always default-initialize Eigen types: `= Eigen::Vector3f::Zero()` |

---

## Testing & Quality Standards

| Metric | Requirement |
|---|---|
| **Test pass rate** | 100% (all tests must pass) |
| **Compiler warnings** | 0 (CI builds with `-Werror -Wall -Wextra`) |
| **New features** | Must include unit tests |
| **Bug fixes** | Must include regression test |
| **Config values** | Must use `cfg.get<>()` with sensible defaults |

---

## Documentation Requirements

| Document | Purpose | Update Frequency |
|---|---|---|
| [PROGRESS.md](PROGRESS.md) | Track all improvements & features | After each improvement |
| [BUG_FIXES.md](BUG_FIXES.md) | Document all bug fixes | After each bug fix |
| [docs/API.md](docs/API.md) | Interface & IPC API reference | When interfaces or IPC classes change |
| [config/default.json](config/default.json) | Default configuration | When adding new tunables |
| [README.md](README.md) | Project overview & build instructions | As architecture changes |
| [CI_ISSUES.md](CI_ISSUES.md) | CI failure log & root cause analysis | After every CI-specific failure |
| [DEVELOPMENT_WORKFLOW.md](DEVELOPMENT_WORKFLOW.md) | Workflow & best practices | When new practices are discovered |

> **Living Document:** This workflow document is meant to evolve with the project.
> When you discover a new best practice, debugging technique, or useful convention
> during development, add it here so the team benefits from it going forward.
