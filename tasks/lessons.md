# Lessons Learned

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
