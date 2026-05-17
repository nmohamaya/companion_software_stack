<!-- SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary -->
<!-- Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md. -->

# Lessons Learned

## Session: 2026-05-13 — Git verification before pushing (commit hygiene)

### 1. `git stash pop` restores staging state silently — always `git status` afterwards

**Mistake:** Cut a clean branch off main for an ADR commit. Ran `git stash pop` to recover the ADR file from the prior branch. Ran `git add docs/adr/ADR-014-...md`. Committed. The commit *also* contained a stray `business` submodule reference (mode 160000) because the stash had restored prior staging state that included the submodule. Force-pushed the bad commit before noticing.

**Why this happens:** `git stash pop` restores **both** the working-directory changes and the **staging-area state** from when the stash was created. If anything was staged when the stash was made, it's silently re-staged on pop. A subsequent `git add <file>` does not replace the staging area — it *adds to* it.

**Rule:** After any `git stash pop`, run `git status` AND `git diff --staged` before committing. Verify the index contains exactly what you expect and nothing else. Do not trust that `git add <name>` resulted in only `<name>` being staged.

### 2. `git commit --amend` runs even when prerequisite `git rm --cached` failed silently — always re-verify with `git show HEAD --stat`

**Mistake:** Ran `git rm --cached business` to remove the stray submodule from the commit. The command failed with `fatal: please stage your changes to .gitmodules or stash them to proceed` because `.gitmodules` had unstaged modifications. Did not check the exit code. Ran `git commit --amend --no-edit`, which succeeded on the unchanged index. Force-pushed the still-broken commit.

**Why this happens:** `git rm --cached` aborts when there are unstaged dependent changes. The error is visible but easy to miss in mid-sequence output. `git commit --amend` then operates on whatever the index actually is — which in this case had not changed.

**Rule:** After any `git rm --cached`, `git restore --staged`, or other index-mutation command, immediately verify with `git status` or `git diff --staged`. After any `git commit --amend`, immediately verify with `git show HEAD --stat` BEFORE pushing. Treat `--force-with-lease` as a verification gate, not a habitual finish.

### 3. Public-repo commits — verify private content (paths, URLs, submodule entries) isn't leaking

**Near-miss:** The accidental `business` submodule entry could have leaked the private repo URL `git@github.com:nmohamaya/companion_software_stack_business.git` if `.gitmodules` had been included in the same commit (it wasn't, by luck). A future broken-submodule commit might leak the URL.

**Rule:** Before pushing to a public repo, `git diff origin/<base>..HEAD` and explicitly scan for:
- Any submodule reference (`mode 160000` entries) — confirm they're intentional and point to public repos
- Any `.gitmodules` entries — confirm URLs are public
- Any reference to `business/`, `tasks/`, customer-specific paths, or other private content
- Any URL to a private GitHub repo
- Any file from the private business-submodule path

If anything looks unintentional, **stop, fix, re-verify, then push**. Never use `git push --force` blindly; always `--force-with-lease`, and only after `git show HEAD --stat` confirms the diff is what you expect.

### 4. Three-step verification checklist for any commit on a public repo

Apply before every push (especially when stashes were used or rebases happened):

1. `git status` — confirm working tree state matches intent
2. `git diff --staged` — confirm the diff to be committed matches intent
3. `git show HEAD --stat` (after commit) — confirm the commit actually contains what was intended

For force-pushes, also:
4. `git log origin/<branch>..HEAD --oneline` — confirm the rewrite is what you expect
5. `--force-with-lease` (never bare `--force`) — protects against overwriting concurrent updates

These five checks take ~30 seconds. They prevent a class of mistake that costs hours to clean up if it lands on a shared branch.



## Session: 2026-02-24 — Issue #4 API-Driven Development

### 1. Always check actual struct field names before writing tests
**Mistake:** Wrote tests using `cpu_usage_pct`, `memory_usage_pct`, `cpu_temp_celsius` when actual fields are `cpu_usage_percent`, `memory_usage_percent`, `cpu_temp_c`.
**Rule:** Before writing any test that references struct fields, `grep_search` or `read_file` the struct definition first. Do NOT guess field names from memory.

### 2. Always check method signatures before writing tests
**Mistake:** Assumed `IProcessMonitor::collect(ShmSystemHealth&)` (void, out-param) when it's actually `collect() -> ShmSystemHealth` (returns by value). Also assumed `IPathPlanner::plan()` and `IObstacleAvoider::avoid()` took Eigen vectors when they take SHM types.
**Rule:** Read the actual interface header before writing test code. Never assume signatures.

### 3. Fully qualify or `using namespace` all relevant namespaces in tests
**Mistake:** Used `using namespace drone` but interfaces were in `drone::slam`, `drone::planner`, `drone::monitor`. Qualified `ipc::ShmImuData` which failed because `ipc` is actually `drone::ipc` and none of the `using` declarations covered it.
**Rule:** At the top of every test file, explicitly `using namespace` every sub-namespace you'll reference. Or use full qualification consistently.

### 4. Account for `-Werror` when initializing structs
**Mistake:** Initialized `Waypoint{10.0f, 0.0f, 0.0f, 0.0f, 2.0f, 5.0f}` missing the `trigger_payload` field, which triggers `-Wmissing-field-initializers` → error under `-Werror`.
**Rule:** When aggregate-initializing structs in a `-Werror` build, always include ALL fields or use designated initializers.

### 5. Missing `<random>` and `<cmath>` includes
**Mistake:** Used `std::mt19937`, `std::normal_distribution`, `std::cos/sin` without the right includes in `ivisual_frontend.h`.
**Rule:** Every standard library type or function needs its header explicitly included. Don't rely on transitive includes.

### 6. Follow `tasks/todo.md` workflow from the start
**Mistake:** Started implementation without creating `tasks/todo.md` and `tasks/lessons.md` as required by `prompt_instructions.md`.
**Rule:** At session start, read `prompt_instructions.md`, create `tasks/todo.md` with the plan, and create `tasks/lessons.md`. Do this BEFORE writing any code.

### 7. Always build locally with exact CI flags before pushing
**Mistake:** Pushed code that compiled locally but failed CI because `ensure_shm_exists()` called `ftruncate()` without checking the return value, which triggers `-Werror=unused-result` under `-Werror -Wall -Wextra`.
**Rule:** Before every push, do a clean build with `-DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra"` to match CI. A `(void)` cast does NOT suppress `warn_unused_result` — must actually branch on the return value.
