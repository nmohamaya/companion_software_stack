# Shared Context

This directory contains **non-obvious domain knowledge that ALL agents should know**. It is updated over time as agents discover pitfalls, unintuitive behaviors, and time-wasting gotchas.

## What belongs here

- Surprising behaviors that are not evident from reading the code
- Unintuitive API quirks (e.g., methods that don't do what their name suggests)
- Known flaky tests and their root causes
- Hardware-specific gotchas (Jetson Orin, PX4, Zenoh session limits)
- "Things that waste 30+ minutes if you don't know them"
- Cross-cutting concerns that span multiple processes or modules

## What does NOT belong here

- **Code patterns** -- read the code itself or `docs/guides/CPP_PATTERNS_GUIDE.md`
- **Git history** -- use `git log` and `git blame`
- **Debugging recipes** -- the fix should be in the code; if it keeps recurring, fix the root cause
- **Anything already in CLAUDE.md** -- that file is the single source of truth for build commands, architecture, and workflow
- **Temporary session state** -- use `tasks/active-work.md` or `tasks/sessions/` instead

## How to update

When you discover something that cost you significant time and would trip up the next agent, append it to the appropriate file in this directory. Use the existing format. Do not duplicate information that is already documented elsewhere.
