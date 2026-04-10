---
name: commit
description: Create a conventional commit with pre-commit validation — format check, sensitive file detection, test count verification
argument-hint: "[optional: description of changes]"
---

# /commit — Smart Commit with Validation

Create a git commit following this project's conventions, with automatic pre-commit validation.

## Steps

### 1. Gather State

Run these in parallel:
- `git status` — see all untracked and modified files
- `git diff --staged` — see what's already staged
- `git diff` — see unstaged changes
- `git log --oneline -10` — recent commit messages for style reference
- `git branch --show-current` — current branch name

### 2. Determine What to Commit

- If nothing is staged and nothing is modified, report "nothing to commit" and stop
- If there are unstaged changes but nothing staged, show the changes and ask what to stage
- If there are both staged and unstaged changes, show both and ask if unstaged changes should be included
- **Never stage sensitive files**: `.env`, `credentials.json`, `*.pem`, `*.key`, secrets. Warn if these appear in the diff.
- Prefer staging specific files by name rather than `git add -A`

### 3. Pre-Commit Validation

Before committing, run these checks:

**Format check** — find all changed C/C++ files and verify formatting:
```bash
git diff --cached --name-only --diff-filter=ACMR | grep -E '\.(h|cpp)$' | xargs -r clang-format-18 --dry-run --Werror 2>&1
```

If format issues are found:
1. Show the specific files with issues
2. Offer to auto-fix: `clang-format-18 -i <files>`
3. If user accepts, format and re-stage the fixed files

**Sensitive file check** — scan staged filenames AND diff content for secrets:
```bash
# Check filenames for known secret file patterns
git diff --cached --name-only | grep -iE '\.env|credential|secret|\.pem|\.key|token'

# Check staged diff content for hardcoded secrets (passwords, API keys, tokens)
git diff --cached -U0 | grep -iE 'password\s*=|api_key\s*=|secret\s*=|token\s*=|-----BEGIN' | head -20
```

If either check finds matches, warn the user with the specific matches and ask for explicit confirmation before proceeding.

**Test count check** (only if test files changed) — if any files in `tests/` are staged:
```bash
ctest -N --test-dir build 2>/dev/null | grep "Total Tests:"
```
Compare against the baseline in `tests/TESTS.md`. Warn if mismatched.

### 4. Draft Commit Message

Follow the project's conventional commit format: `type(#issue): description`

**Types:** `feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf`

**Rules:**
- Extract issue number from branch name if it follows `feature/issue-XX-*` or `fix/issue-XX-*` pattern
- Keep the subject line under 72 characters
- Use imperative mood ("add", "fix", "update", not "added", "fixes", "updated")
- The subject should describe the **why**, not the **what** (the diff shows what)
- Add a body (separated by blank line) if the change is non-trivial — explain motivation, not mechanics
- Always end with: `Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>`

If the user provided a description in `$ARGUMENTS`, use it as the basis for the commit message.

### 5. Create the Commit

Show the draft message and ask for approval. Then commit using a heredoc:

```bash
git commit -m "$(cat <<'EOF'
type(#issue): subject line

Optional body explaining why.

Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
EOF
)"
```

### 6. Post-Commit Verification

Run `git status` to confirm the commit succeeded and show the result.

If the commit fails due to a pre-commit hook:
- Read the hook output to understand what failed
- Fix the issue (usually formatting)
- Re-stage the fixed files and re-run `git commit` with the same message
- Note: a failed hook means no commit was created, so `--amend` would modify the previous unrelated commit — only use `--amend` if a commit already exists and you explicitly want to update it

If the user provided arguments, use them as context: $ARGUMENTS
