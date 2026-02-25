# Real Perception Pipeline — Color/Contour-Based Object Detection from Gazebo Camera

## Context
Currently `SimulatedDetector` generates random fake bounding boxes — the Gazebo-rendered camera frames are captured but never processed. The entire detect→track→avoid pipeline runs on meaningless data. This issue replaces the simulated detector with a real color-segmentation + contour detector that processes actual Gazebo RGB frames, and adds more visually distinct obstacles to the world so there's something meaningful to detect.

## Plan

### Phase 0: Setup
- [x] Create GitHub issue #19 for real perception pipeline
- [x] Create feature branch `feature/issue-19-real-perception`
- [x] Update this todo with issue number

### Phase 1: Add Visually Distinct Obstacles to Gazebo World
- [x] Add 6 brightly colored obstacles to `sim/worlds/test_world.sdf` along the flight path
- [x] Use distinct, saturated colors (red, blue, yellow, green, orange, magenta) that contrast with the ground
- [x] Place obstacles at different heights near waypoints
- [x] Removed 3 old muted obstacles, replaced with 6 new vivid ones

### Phase 2: Implement ColorContourDetector
- [x] Created header-only `process2_perception/include/perception/color_contour_detector.h` (~320 lines)
- [x] Algorithm: RGB→HSV → binary mask per color → union-find connected-component labeling → bounding box extraction
- [x] No OpenCV dependency — pure C++ (rgb_to_hsv, UnionFind, ComponentBBox)
- [x] 6 default color ranges with hue wrap-around support (red spans 340-20°)
- [x] All tunable via `drone::Config` (HSV ranges, min_contour_area, max_detections)

### Phase 3: Detector Factory & Config Integration
- [x] Added `"color_contour"` backend to factory in `detector_interface.h`
- [x] Updated `config/gazebo.json`: `"backend": "color_contour"`, fixed cx=320/cy=240
- [x] `config/default.json` unchanged — still uses `"simulated"` (no regression)

### Phase 4: Wire Into Perception Main Loop
- [x] Updated `process2_perception/src/main.cpp` inference thread to use factory-created detector
- [x] Detector receives real frame_data from ShmVideoFrame

### Phase 5: Unit Tests
- [x] Created `tests/test_color_contour_detector.cpp` — 42 tests across 6 suites
- [x] Updated `tests/CMakeLists.txt`
- [x] All 238 tests pass (196 existing + 42 new)

### Phase 6: Integration Verification
- [x] Build clean: 51 targets, zero warnings
- [x] All 238 tests pass
- [ ] Gazebo SITL flight verification (manual step — requires PX4 + Gazebo running)

### Phase 7: Documentation & Delivery
- [x] Updated PROGRESS.md with Improvement #7
- [ ] Commit with proper message
- [ ] Push to feature branch
- [ ] Create PR referencing issue #19

## Design Decisions

### Why Color Segmentation (not ML)?
- **Zero external dependencies** — project principle is "written from scratch, no external ML/CV frameworks"
- HSV thresholding is simple, fast, deterministic, and testable
- Gazebo objects have perfectly uniform colors — ideal for color-based detection
- Algorithm is ~200 lines of pure C++ with no library dependencies

### Why Not OpenCV?
- Project explicitly avoids external CV frameworks
- HSV conversion is trivial (~10 lines)
- Contour finding via flood-fill connected components is straightforward
- Keeps the build simple and dependency-free

### Contour Detection Approach
- Binary mask per color → connected-component labeling (union-find or flood-fill)
- Bounding box from component pixel extents
- Area = pixel count of component
- This is simpler than Suzuki-Abe border tracing but sufficient for blocky Gazebo objects

## Review
_(To be filled after PR review)_

## Review

### Delivered
- **12 new header files**: 7 IPC interfaces/implementations, 4 process strategy interfaces, 1 API doc
- **7 process main.cpp files refactored**: all use abstract interfaces via ShmMessageBus
- **2 new test files**: 42 new tests (163 total, 100% pass, 0.62s)
- **PR #5** created on branch `feature/issue-4-api-interfaces` closing issue #4

### Metrics
| Metric | Before | After |
|--------|--------|-------|
| Tests | 121 | 163 |
| Pass rate | 100% | 100% |
| New files | — | 14 |
| Modified files | — | 10 |
| Lines added | — | ~2184 |

### Corrections & Lessons
1. Wrote tests using wrong field names (e.g. `cpu_usage_pct` vs `cpu_usage_percent`) — lesson: always read struct definitions first
2. Wrote tests with wrong method signatures (e.g. `collect(health)` vs `collect()`) — lesson: read interface headers before writing tests
3. Bootstrap ShmWriter in service channel unlinked SHM on destruction — fixed with raw `ensure_shm_exists()` helper
4. Missing `<random>`, `<cmath>` includes in ivisual_frontend.h — caught by `-Werror`
5. Forgot to create `tasks/todo.md` and `tasks/lessons.md` at session start — fixed mid-session after re-reading prompt_instructions.md

All lessons documented in `tasks/lessons.md`.
