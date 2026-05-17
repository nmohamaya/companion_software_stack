# Safety-Critical Shell Discipline

Operational discipline for working with shell commands, git operations, and multi-step workflows on this safety-critical codebase.

**Stack:** C++17 safety-critical drone autonomy — code defects can cause loss of vehicle, mishandled commits can leak private content into public repositories, and a "green" CI checkpoint earlier does not mean the current state is green.

This guide is the **abstract principle** behind the operational checklists in [`tasks/lessons.md`](../../tasks/lessons.md) and the inline cautions in [`CLAUDE.md`](../../CLAUDE.md). It exists because individual rules accumulate; a single load-bearing principle makes them coherent.

---

## 1. The Core Principle

> **Don't trust prior-step success — verify each step's actual effect.**

Multi-step workflows fail when an intermediate step succeeds silently, or fails silently, and the next step proceeds on the assumption that the previous one did what was intended. The mitigation is to read the actual state after each step, not to rely on the prior step's exit code or assumed-correct behaviour.

This is the same principle as safety-critical software: never trust a sensor reading without bounds-checking, never proceed past a fault-detection gate without confirming the gate cleared, never assume an operation succeeded because the function returned. Shell work on a safety-critical codebase deserves the same discipline as the code itself.

---

## 2. Concrete Anti-Patterns (Observed, Not Hypothetical)

Each of these has caused real damage or near-damage on this project. The first occurrence is dated; the pattern is general.

### 2.1 `git stash pop` silently restores staging state

**Observed:** 2026-05-13 — a `git stash pop` restored prior staging-area state, including a stray `business` submodule reference. A subsequent `git add docs/adr/...` *added to* the index rather than replacing it. The resulting commit included content the operator never intentionally staged. Force-pushed before the contamination was caught.

**Generalised pattern:** `git stash` saves both working-directory AND staging-area state. `git stash pop` restores both. A subsequent `git add` is additive, not replacive. Any tool that has a "save state" and "restore state" pair will exhibit this class of bug if the operator doesn't realise both states are being restored.

**Discipline:** after any `git stash pop`, run `git status` AND `git diff --staged` before any commit. Trust nothing about the index until inspected.

### 2.2 Prerequisite commands fail silently in mid-sequence

**Observed:** 2026-05-13 — `git rm --cached business` failed with `fatal: please stage your changes to .gitmodules or stash them to proceed`. The error was visible in stdout but unread. The subsequent `git commit --amend` proceeded on the unchanged index, producing a commit identical to the broken one.

**Generalised pattern:** in a sequence of commands intended to compose, any one of them can refuse to do its job and the sequence continues anyway. Exit codes are only useful if checked; output is only useful if read.

**Discipline:** after any state-mutation command (`git rm`, `git restore`, `git checkout`, `sed -i`, `mv`, `rm`, `cmake`, `make`, etc.), verify the effect on the next command's prerequisite — don't move on until the previous step's success is positively confirmed.

### 2.3 Trusting upstream green when downstream is red

**Observed:** 2026-05-13 — the `format-check` CI gate failed on a PR that touched only docs. The violations were entirely in files the PR did not touch, inherited from an earlier integration merge that landed without the format-check enforcing its gate. The downstream PR became responsible for cleaning up upstream's mess.

**Generalised pattern:** trust in a CI gate is binary. If a "green" PR can branch off a "red" main, the green checkmark stops meaning anything. The fire alarm activates after the fire is already in the next room.

**Discipline:**
- Treat every PR's CI failures as a question, not a conclusion. Read the actual error before assuming the PR introduced it.
- For large integration merges, run the full CI matrix on the integration branch's tip *before* the merge button, not after.
- Required-status-check rules on `main` are the only durable enforcement. Pre-commit hooks help individuals; required-status-checks help the project.

### 2.4 Multi-edit operations on overlapping issue / file numbers

**Observed:** 2026-05-12 — a batch of `gh issue edit` calls operated on issues `#28-#35` with title/body content intended for the next-numbered issue, because the original issue creation order had silently shifted by one (a creation in the middle of the batch had failed silently). Result: titles and bodies were assigned to the wrong issues. Caught when the user explicitly asked "did you do that?"

**Generalised pattern:** when an operation runs across a numbered range (`for i in $(seq ...)`, indexed sed, line-based edits), an off-by-one in the assumed mapping pollutes the whole range. The fix is to re-fetch the mapping from authoritative state after every mutating step, not to compute it once from a memory of the intended order.

**Discipline:** after any batch creation (issues, files, commits, branches), re-query the actual identifiers. Don't assume `issue N+1` is the one that was just created; verify with `gh issue list` or equivalent.

### 2.5 Untracked headers referenced by tracked source files

**Observed:** 2026-05-XX — `tests/test_helpers.h` was `#include`d by two committed test files but the header itself was never `git add`ed. Build broke on a separate machine. Caught only when a colleague tried to compile.

**Generalised pattern:** files that look present locally because they're on disk can still be absent from the repository. `git status` shows the gap, but only if read. CI catches it only if the build runs on a clean checkout.

**Discipline:** after every commit that adds source files, run the include-resolution check:

```bash
# every #include "..." in newly-added .h/.cpp files must resolve to a tracked file
git diff origin/main --diff-filter=AM --name-only | \
  grep -E '\.(h|cpp)$' | \
  xargs grep -h '#include "' 2>/dev/null | \
  grep -oP '#include "\K[^"]+' | sort -u | \
  while read inc; do
    git ls-files | grep -q "$inc$" || echo "MISSING: $inc"
  done
```

---

## 3. The Verification Checklist

A five-step checklist, applied before every push (and especially when stashes, amends, rebases, or interactive operations are in play):

1. **`git status`** — confirm working tree state matches intent
2. **`git diff --staged`** — confirm the diff to be committed matches intent (especially scan for stray submodule entries, `.gitmodules` additions, references to `business/` or private paths, untracked-private-content infiltration)
3. **`git show HEAD --stat`** (after `git commit` or `--amend`) — confirm the commit actually contains what was intended
4. For force-pushes: **`git log origin/<branch>..HEAD --oneline`** — confirm the rewrite is what you expect
5. **`--force-with-lease`** (never bare `--force`) — protects against overwriting concurrent updates

These five checks take ~30 seconds. They prevent a class of mistake that costs hours to clean up if it lands on a shared branch.

For public-repo commits, also scan the staged diff for:

- Stray submodule references (`mode 160000` lines) — confirm intentional and pointing to public repos
- `.gitmodules` additions pointing to private repo URLs (e.g. `git@github.com:.../*business*.git` patterns) — these leak the private repo URL
- Files from `business/`, `tasks/pipeline-state.json`, or other private paths that should not be in the public repo
- Customer-specific paths or content
- Any private-repo content brought in via stash-pop or cherry-pick

If anything looks unintentional: **stop, fix, re-verify with all five steps, then push**.

---

## 4. Where to Apply This

The principle generalises beyond git. Some places where the same discipline matters:

### 4.1 Build verification

After `make` / `cmake --build` / `bash deploy/build.sh`:

- Run `ctest -N --test-dir build | grep "Total Tests:"` to confirm the test count matches the baseline in [`tests/TESTS.md`](../../tests/TESTS.md). A passing run at the wrong count is not "all tests pass."
- Run `find ... | xargs clang-format-18 --dry-run --Werror` independently — don't trust that the CI ran it.
- Inspect the build log for compiler warnings even when exit code is 0 (we use `-Werror` but local CMake configurations vary).

### 4.2 Deployment / install scripts

After `bash deploy/install_systemd.sh` or any deployment action:

- `systemctl status drone-stack.target` — verify every service actually started, not just that the install script exited 0.
- `journalctl -u drone-<process>.service -b --since "5 minutes ago"` — verify no early-fault messages.
- Run a known-good smoke test scenario before declaring "deployed."

### 4.3 Config / scenario edits

After editing `config/default.json` or `config/scenarios/*.json`:

- Validate with `python3 -m json.tool config/default.json > /dev/null` — silent success means well-formed.
- Run at least one scenario that exercises the modified path, not just the build.
- Compare diff against a known-good baseline.

### 4.4 Multi-agent / pipeline operations

When deploying issues via the multi-agent pipeline (`/deploy-issue`, `/deploy-wave`):

- After every Agent tool invocation, read the agent's report explicitly. Don't trust "agent completed" — read what it actually did.
- After every batch operation across issue numbers, re-query the actual IDs.
- After every commit by an agent, run the five-step git verification before pushing.

### 4.5 External-service operations

When calling `gh api`, `curl`, or other external commands:

- Exit code 0 from `gh api` does not mean the response payload is what you expect — parse and inspect.
- A 504 / 502 / 429 response may have partial success — re-check state via a separate query before retrying.
- API rate-limited responses may silently succeed for some entries and fail for others — verify per-entry.

---

## 5. The Trust-Inversion Principle

The standard mental model is: *commands succeed by default; investigate when they fail.*

The safety-critical mental model inverts this: *commands fail by default; investigate to prove success.*

In practice this means:

- Default-deny: assume the previous step did not do what you wanted, until the evidence is in.
- Read the actual state, not the inferred state.
- Treat exit code 0 as one signal among many, not as proof of success.
- The marginal time cost of re-verification is small; the marginal cost of acting on a wrong assumption can be hours of cleanup or a real safety incident.

This is the same inversion that drove the project's adoption of `Result<T,E>` over exceptions, `[[nodiscard]]` on every error-returning function, RAII over manual cleanup, and the three-tier watchdog over single-layer fault detection. The principle is identical — trust nothing, verify everything, fail loudly.

---

## 6. When to Apply Maximum Discipline

Not every command needs this treatment. The five-step verification is mandatory when:

- The operation is **destructive** (`git push --force-with-lease`, `rm`, `git rebase`, `git reset --hard`, branch delete)
- The operation crosses a **trust boundary** (public-repo push, GitHub API call, deployment to shared infrastructure)
- The operation followed a **state-mutating prerequisite** (`stash pop`, `rm --cached`, `restore --staged`, prior amend)
- The operation is part of a **batch / loop** where any single iteration's failure could pollute the whole batch
- The state is **load-bearing** for the next operation (commit message references files; PR description references commit SHAs; deployment references built artefacts)

For routine, reversible, local-only operations (`git add` on a feature branch, `bash deploy/build.sh` during dev cycle, file edits in a worktree), the discipline can relax to `git status` only. The cost-benefit shifts.

---

## 7. Honest Limitations

This guide is operational discipline, not infallibility. It will not catch:

- Bugs in the tools themselves (a `git` bug, a clang-format-18 bug, a CI runner misconfiguration)
- Mistakes in the verification steps themselves (running `git status` but skimming the output)
- Issues introduced before the verification began (the `business` submodule reference contamination came from a stash created before the current session; if the session had started clean, the discipline alone would not have prevented the original stash creation)
- Bugs that pass all the verification steps because the verification steps don't check the right thing

The mitigation is to evolve the checklist when a new failure pattern emerges, not to assume the current checklist is complete. The lessons file at [`tasks/lessons.md`](../../tasks/lessons.md) is the running log of new patterns; this guide is the abstract principle behind them.

---

## 8. Related Documentation

- [`tasks/lessons.md`](../../tasks/lessons.md) — running operational log of session-by-session lessons; this guide is the abstraction layer above
- [`CLAUDE.md`](../../CLAUDE.md) "Known Pitfalls" section — quick-reference cautions for AI agents working in this repo
- [`docs/how-to/DEVELOPMENT_WORKFLOW.md`](../how-to/DEVELOPMENT_WORKFLOW.md) — the broader development workflow that this discipline sits inside
- [`docs/reference/CPP_PATTERNS_GUIDE.md`](../reference/CPP_PATTERNS_GUIDE.md) — the codebase's safety-critical patterns at the language level (`Result<T,E>`, `[[nodiscard]]`, RAII) — same discipline, applied to code instead of shell
- [`docs/tracking/IMPROVEMENTS.md`](../tracking/IMPROVEMENTS.md) — the backlog of CI/process improvements that would enforce parts of this discipline automatically

---

## 9. Document History

- 2026-05-13: Initial guide created — distilled from the operational lessons accumulated in `tasks/lessons.md` after a session that produced multiple near-misses (stash-pop submodule contamination, silent `git rm --cached` failure, inherited format-check breakage). All examples in §2 are real incidents from this project, not hypothetical.
