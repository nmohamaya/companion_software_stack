<!-- SPDX-License-Identifier: LicenseRef-Pipeline-Proprietary -->
<!-- Copyright (c) 2025-2026 Naveen Mohanan. All Rights Reserved. See PIPELINE_LICENSE.md. -->

# Agent skill file — authoring conventions

Guidance for anyone editing `.claude/agents/*.md` files (review-*, test-*, feature-*, tech-lead, ops-github).  Agent system prompts are loaded by the runtime on every spawn — the conventions below keep them **timeless, stable, and reviewable** as the codebase evolves.

## The core principle

Agent skill files capture **rules and patterns**, not **provenance**.  Provenance — who added a rule, when, why, via which PR — belongs in the audit-trail artefacts (commit messages, PR bodies, issue comments, `docs/tracking/PROGRESS.md` entries).  Mixing the two ties the skill file's lifetime to a moving PR landscape.

## What to reference (✅)

| Reference type | Why it works |
|----------------|--------------|
| **Issue numbers** (`#740`, `#727`, `#722`) | Represent *problems* and *epics*, which are stable historical anchors.  Survive merges, force-pushes, and renumbering. |
| **Canonical file paths** (`process4_mission_planner/include/planner/mission_state_tick.h::tick_preflight`) | Stable once code merges — the actual artefact the agent should `Read` for context. |
| **CLAUDE.md / CPP_PATTERNS_GUIDE.md section names** (e.g. "Constructs to AVOID", "Cold-start data hygiene") | Section names survive merges; §-numbers can drift, so prefer names. |
| **Rule descriptions** ("debounce N seconds before ARM", "first observations are suspect") | Timeless — the *behaviour* the agent should look for, written so it's actionable even if the originating PR is forgotten. |

## What to AVOID (❌)

| Anti-pattern | Why it breaks |
|--------------|---------------|
| **PR numbers in inline rule text** (`PR #750 introduces X`, `landing via PR #N`) | PR numbers drift during development (off-by-one errors are common), get reassigned, get superseded, or simply become noise once merged.  Three cleanup PRs in one session (#753 → #754 → #755) corrected the same fileset because PR-number references kept drifting. |
| **"Currently open" / "Once merged" temporal language** | Fragile.  The phrasing is correct today, misleading next week, and stale clutter in six months — at which point a future maintainer must decide whether the ref still means anything. |
| **References to in-flight branches** (`feature/cold-start-hardening`) | Branches are deleted after merge.  The text becomes a dead pointer. |
| **§-number citations** (`CPP_PATTERNS_GUIDE.md §5.4`) | §-numbers shift when sections are added, removed, or reordered.  Use the section *name* instead. |

## Worked example

**Bad — temporal + PR-coupled (this is what existed pre-cleanup):**

```markdown
- [ ] **Cold-start data hygiene** — Wrapper-level birth-time filtering for types with
  `timestamp_ns` lands via PR #750 (currently open against `feature/cold-start-hardening`)
  — until then, the per-site filter in `process4_mission_planner/src/main.cpp`
  (PR #721, merged) protects the pose topic only.
```

**Good — issue + file + rule:**

```markdown
- [ ] **Cold-start data hygiene** ([Issue #722](https://github.com/.../issues/722)) —
  When a subscriber re-connects to a Zenoh topic, the last-value cache can deliver
  a historic message from a previous publisher session.  Defense-in-depth: filter
  incoming messages whose `timestamp_ns` predates the subscriber's birth.  Canonical
  implementation: `common/ipc/include/ipc/zenoh_subscriber.h::on_sample` (apply
  per-site at `process4_mission_planner/src/main.cpp` if the wrapper isn't yet in
  place for the topic in question).
```

The "good" version states the *rule* and points at *stable artefacts*.  A reader can follow the issue link for problem context or `Read` the file for the implementation, regardless of which PR introduced either.

## Updating existing rules

When you genuinely need to capture the **history** of a rule (e.g. an epic that introduced several agents-affecting changes), keep the historical narrative in `docs/tracking/PROGRESS.md` or the originating issue, and let the agent file remain a forward-looking checklist.  If you must mention "introduced by epic X", reference the *issue*, not the PR.

## Output conventions (unchanged)

These rules (already in every agent file) remain canonical:

- **Anti-hallucination block** at the bottom — "Before citing a function/file/API, verify it exists by reading the file."
- **P1/P2/P3 severity grading** with explicit "blocks merge / should fix / follow-up" semantics.
- **`Ref:` lines** point at stable canonical sources (CLAUDE.md sections, file paths, issue numbers) — never at PRs.

## When in doubt

If you're tempted to write "see PR #N" in an agent file, ask:

- Is there an *issue* number for the problem this rule addresses?  Cite that.
- Is there a *file path* for the canonical implementation or the artefact under review?  Cite that.
- Is there a *section* in CLAUDE.md / CPP_PATTERNS_GUIDE.md with the rule's full text?  Cite the section name.

If the answer to all three is no, the rule probably belongs in a commit message or PR body, not in an agent skill file.
