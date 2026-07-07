# ADR-016: Tool-First Verification — Reserve Agent Tokens for Judgment

**Status:** Accepted
**Date:** 2026-06-15
**Issue:** #765 (triggering evidence); refines ADR-010 (multi-agent pipeline), complements ADR-005 (parallel agent worktrees)

## Context

ADR-010 gave us a 17-agent review pipeline. It is powerful, but two things became undeniable during the #765 work:

1. **Full fan-out is expensive and can be self-defeating.** A single full two-pass review with adversarial verification (~45 sub-agents) consumed ~2.7M tokens and **exhausted the operator's Claude Max session allowance mid-run** — leaving the verification pass *incomplete*. An over-thorough review that cannot finish is worse than a scoped review that does.

2. **Most review checks are mechanical, and a tool that *proves* a property beats an agent that *reasons* about it.** A ThreadSanitizer run does not "think" a path is race-free — it executes the code and reports the race (or its absence). An LLM agent reading the same code produces a *probabilistic* judgment. For any property a deterministic tool can decide, spending a token is strictly worse: more costly, non-reproducible, and less reliable.

Concrete evidence from this session:

| Approach | Cost | Outcome |
|---|---|---|
| Full roster + verify fan-out (~45 agents) | ~2.7M tokens | Hit session limit; verify pass incomplete |
| One token-optimised agent on one PR | ~70K tokens | Clean review, 1 trivial nit (38× cheaper) |
| 4 specialized agents across two PRs | ~295K tokens | Clean (9× cheaper) |
| TSan build + 5× run of 4 targets | $0 tokens (local compute) | **Proved** concurrency-clean where agents could only argue it |
| Copilot + CI (free) | $0 tokens | Caught the rate-limit floor-0 bug and SSOT test-count drift the roster missed |

The lesson is not "stop using agents." It is "stop using *tokens* where a *tool* is cheaper and more authoritative, and spend the saved budget on judgment only an agent can provide."

## Decision

Adopt a **tiered, tool-first verification policy**. Each tier runs before — and gates — the next; you only escalate to a more expensive tier for what the cheaper tiers cannot decide.

- **Tier 0 — Deterministic tools (the authoritative gate for mechanical properties).** Compiler `-Werror -Wall -Wextra`; sanitizers (TSan / ASan / UBSan); `clang-format`; `clang-tidy`; a doc-SSOT linter (test counts, scenario counts, version refs — see CLAUDE.md §Single Sources of Truth); coverage (lcov/gcovr) for the test-exercises-new-paths question; `cppcheck`. These *prove* their property and are the first gate. For signal-handler / atomic / lock-free changes, a real TSan **build** (separate `build-tsan/` dir) + repeated run is mandatory — read-only review agents are not a substitute (per the PR #782 "Lessons learned" in `docs/tracking/BUG_FIXES.md`: agents caught the logic bugs but only TSan instrumentation caught the contract-violation race).
- **Tier 1 — Free external review.** GitHub Copilot review + CI. Lean on these before spending any agent tokens; they catch ~60% and have repeatedly found defects the roster missed.
- **Tier 2 — Minimal specialized agents (default review path).** 1–2 specialized review agents per PR, routed to the lenses tools cannot cover (e.g. `review-memory-safety` + `review-concurrency` for threading code). Direct `Agent` calls under operator control — **not** the `Workflow` fan-out.
- **Tier 3 — Full ADR-010 roster.** Only on explicit operator request, with the token cost stated up front, for high-risk / large / cross-domain changes.

**Reserve agent tokens for judgment tools cannot provide:** design and architecture trade-offs; intent-vs-implementation ("does the code do what the docstring/PR claims"); data-plumbing semantics (producer writes the field a consumer reads); test *meaningfulness* (an assertion that would fail if the feature broke); and cross-domain reasoning.

**Guiding principle:** *If a deterministic tool can decide it, do not spend a probabilistic token on it.*

## Consequences

**Positive**
- Cost control — routine verification no longer risks the subscription allowance.
- Reproducibility — a sanitizer/linter result is identical run-to-run; an agent's is not.
- Stronger guarantees for mechanical properties — tools prove; agents argue.
- Earlier feedback — Tier 0 can run pre-commit and in CI, before any agent or human looks.
- Agent budget concentrates where it has unique value (judgment), improving signal-to-noise.

**Negative / costs**
- Upfront tooling investment to build and maintain the Tier-0 gates (done — issue #785; see status below). Advisory gates (cppcheck, clang-tidy, coverage-delta) still need their backlogs burned down before promotion to hard gates.
- Tools cannot judge design or intent — Tier 2/3 agents remain essential, not optional.
- Requires discipline: the reflex to "fan out the whole roster" must be replaced by "what is the cheapest tier that can decide this?"

## Tier-0 tooling status

Implemented for issue #785 — recurring agent/LLM checks turned into deterministic gates:

| Tool | Path | CI surface | Status |
|---|---|---|---|
| Doc-SSOT linter (no hardcoded test-count totals outside `tests/TESTS.md`) | `deploy/lint_docs_ssot.sh` | `doc-lint` job | **hard gate** (changed-lines) |
| cppcheck static analysis | `deploy/run_cppcheck.sh` | `cppcheck` job | advisory |
| clang-tidy on changed `.cpp` | `deploy/run_clang_tidy.sh` | step in `coverage` job | advisory |
| Changed-line coverage delta | `deploy/coverage_delta.py` | step in `coverage` job | advisory |
| Pre-commit hooks (clang-format + doc-SSOT + security guards) | `.githooks/pre-commit`, `deploy/install_hooks.sh` | local (`core.hooksPath`) | opt-in |
| Repeated-TSan helper (signal-handler/lock-free rule) | `tools/tsan_repeat.sh` | local / pre-push | helper |

All are also runnable via `deploy/run_ci_local.sh` (jobs `DOC`/`CPPCHECK`/`TIDY`/`COVDELTA`).

### Remaining (promotion path)

1. **Burn down backlogs, then promote advisory → hard gate.** The doc-SSOT linter found ~96 pre-existing hardcoded totals; cppcheck/clang-tidy have unmeasured backlogs. The changed-files/changed-lines scoping prevents growth; clean the backlog incrementally, then flip the advisory jobs to blocking. Tracked in `docs/tracking/IMPROVEMENTS.md`.
2. **Adopt a changed-line coverage floor** via `coverage_delta.py --min` once a threshold is agreed.

## Revisit when

- Tool-coverage gaps produce escaped bugs that a full-roster review would have caught (re-balance the tiers).
- Agent token economics change materially (cheaper models / higher allowances) such that fan-out is no longer a budget risk.
- A new class of defect emerges that neither tools nor the minimal roster reliably catch.
