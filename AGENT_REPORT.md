# Change Report — Issue #363

## Summary
Added a `--version` flag to `scripts/start-agent.sh` that prints "multi-agent-pipeline v1.0" and exits with code 0. The flag is documented in the usage help text. A new shell test script (`tests/test_start_agent.sh`) validates the `--version`, `--help`, and `--list` flags.

## Files Changed
| File | Change | Reason |
|------|--------|--------|
| `scripts/start-agent.sh` | Added `PIPELINE_VERSION` variable, `--version` case in arg parser, `--version` line in usage help | Implements the `--version` flag per acceptance criteria |
| `tests/test_start_agent.sh` | New file — 6 shell-based assertions | Validates `--version` output/exit code, `--help` mentions `--version`, `--list` works |

## Tests Added/Modified
- **`tests/test_start_agent.sh`** (new): Shell test with 6 assertions:
  - `--version` exits 0
  - `--version` prints exact string "multi-agent-pipeline v1.0"
  - `--help` exits 0
  - `--help` output contains "--version"
  - `--help` output contains "--list"
  - `--list` exits 0 and shows "tech-lead"

## Decisions Made
- **Version string format**: Used "multi-agent-pipeline v1.0" as specified in the issue description.
- **Shell tests over GTest**: Since this is a bash script, shell-based tests are the natural fit. The test uses a simple assert_eq/assert_contains pattern consistent with shell testing best practices.
- **Version variable placement**: Added `PIPELINE_VERSION` as a top-level constant near the color definitions, making it easy to find and update.

## Risks / Review Attention
- Very low risk — 3 lines added to an existing script, no behavior changes to existing flags.
- The test script is new and not yet integrated into the C++ test suite (ctest). It should be run manually with `bash tests/test_start_agent.sh`.

## Build & Test Status
- This is a shell-only change — no C++ build impact, no change to the ctest count (remains 1259).
- Shell tests need manual execution (`bash tests/test_start_agent.sh`) for verification.
- Awaiting approval to run tests and create git commit.
