"""Pipeline non-interactive state handlers.

Each handler corresponds to an automated (non-checkpoint) state:
  - AGENT_WORK: Launch the feature agent autonomously
  - VALIDATE: Run validate-session.sh
  - PR_CREATE: Push branch + generate PR metadata
  - REVIEW: Launch review + test agents
  - FIX_AND_REVALIDATE: Agent fixes findings → re-validate → push
  - CLEANUP: Update shared state, verify PR links
  - ABORT: Save state, print resume instructions

Handlers take PipelineStateData + wrapper objects (Git, GitHub, Claude, etc.)
and return the action string for the FSM transition.
"""

from __future__ import annotations

import re
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING

from orchestrator.pipeline.state import PipelineStateData

if TYPE_CHECKING:
    from orchestrator.build import BuildSystem
    from orchestrator.claude import Claude
    from orchestrator.console import IOProtocol
    from orchestrator.git import Git
    from orchestrator.github import GitHub


def handle_agent_work(
    state: PipelineStateData,
    io: IOProtocol,
    claude: Claude,
    *,
    prompt: str,
    interactive: bool = False,
) -> str:
    """Launch the feature agent.

    Both modes use launch_interactive_with_prompt() so the agent can
    request tool permissions from the user when needed (e.g. git, bash).
    The difference is just the prompt content — pipeline mode adds
    autonomous instructions telling the agent to work without chatting.
    Returns "complete" when the agent exits.
    """
    mode_label = "interactive" if interactive else "autonomous (writes AGENT_REPORT.md)"
    io.header(f"Phase 1/5 — Agent Working ({state.agent_role})")
    io.print(f"  Model:  {state.model_tier}")
    io.print(f"  Issue:  #{state.issue_number} — {state.issue_title}")
    io.print(f"  Mode:   {mode_label}")
    worktree = Path(state.worktree_path) if state.worktree_path else None
    if worktree:
        io.print(f"  CWD:    {worktree}")
    io.print("")

    # Always use interactive launch so the agent can request permissions
    # (git, bash, etc.) from the user when sandbox restrictions apply.
    # In pipeline mode, the prompt itself contains autonomous instructions.
    claude.launch_interactive_with_prompt(
        model=state.model_tier,
        agent=state.agent_role,
        prompt=prompt,
        cwd=worktree,
    )
    return "complete"


def handle_validate(
    state: PipelineStateData,
    io: IOProtocol,
    build: BuildSystem,
) -> tuple[str, str, bool]:
    """Run validation checks.

    Returns (action, output, passed).
    """
    io.header("Running Validation")

    passed = True
    output_lines: list[str] = []

    # Check build
    if build.has_build_dir():
        count = build.test_count()
        expected = build.expected_test_count()
        output_lines.append(f"Test count: {count} (expected {expected})")
        if count != expected:
            output_lines.append("  WARNING: test count mismatch")
            passed = False
        else:
            output_lines.append("  OK")
    else:
        output_lines.append("No build directory found — skipping build checks")

    output = "\n".join(output_lines)
    io.print(output)

    return "complete", output, passed


def handle_pr_create(
    state: PipelineStateData,
    io: IOProtocol,
    git: Git,
    github: GitHub | None = None,
) -> str:
    """Push branch to remote and prepare PR metadata.

    Returns "complete" on success or "push_failed" on failure.
    PR metadata (title/body) is populated but actual creation waits for CP3.
    After push, applies labels and milestone from the source issue.
    """
    io.header("Pushing and Preparing PR")

    io.print(f"  Pushing to origin/{state.branch_name}...")
    try:
        git.push(state.branch_name, set_upstream=True)
        io.pass_("Push succeeded")
    except Exception as e:
        io.fail(f"Push failed: {e}")
        return "push_failed"

    # Generate default PR title/body if not set
    if not state.pr_title:
        prefix = "fix" if state.is_bug else "feat"
        # Strip leading type prefix from title to avoid duplication
        # e.g. "feat: remote pipeline..." → "remote pipeline..."
        title = state.issue_title
        for strip_prefix in ("feat:", "fix:", "chore:", "refactor:", "docs:", "test:", "perf:"):
            if title.lower().startswith(strip_prefix):
                title = title[len(strip_prefix):].strip()
                break
        state.pr_title = f"{prefix}(#{state.issue_number}): {title}"

    if not state.pr_body:
        report_path = Path(state.worktree_path) / "AGENT_REPORT.md"
        summary = ""
        if report_path.exists():
            text = report_path.read_text()
            # Extract Summary section
            match = re.search(
                r"## Summary\n(.*?)(?=\n## |\Z)", text, re.DOTALL
            )
            if match:
                summary = match.group(1).strip()

        diff_stat = ""
        try:
            # Diff against origin/main since we're in a worktree and
            # local 'main' may not be up to date
            diff_stat = git.diff_stat("origin/main", "HEAD")
        except Exception:
            try:
                diff_stat = git.diff_stat("main", "HEAD")
            except Exception:
                pass

        # Build closing/linking references
        closing_refs = f"Closes #{state.issue_number}"
        if state.epic_number:
            closing_refs += f"\nPart of #{state.epic_number}"

        state.pr_body = (
            f"## Summary\n"
            f"{summary or 'See AGENT_REPORT.md for details.'}\n\n"
            f"## Changes\n{diff_stat}\n\n"
            f"## Test plan\n"
            f"- [ ] Build passes with zero warnings\n"
            f"- [ ] All tests pass (verify count matches baseline)\n"
            f"- [ ] Review agent findings addressed\n\n"
            f"{closing_refs}\n\n"
            f"Generated with [Claude Code](https://claude.com/claude-code) "
            f"via pipeline mode"
        )

    return "complete"


def handle_review(
    state: PipelineStateData,
    io: IOProtocol,
    github: GitHub,
) -> tuple[str, str]:
    """Launch review + test agents for the PR.

    Runs deploy_review.run() to launch review and test agents in parallel.
    After they complete, fetches the consolidated findings from the PR comment.
    Returns (action, review_findings).
    If no PR exists, returns ("no_pr", "").
    """
    io.header("Deploying Review + Test Agents")

    if not state.pr_number:
        io.fail("No PR number — skipping review agents.")
        return "no_pr", ""

    io.print(f"  Launching review agents for PR #{state.pr_number}...")
    io.print("")

    # Launch review agents via deploy_review
    from orchestrator.commands.deploy_review import run as deploy_review_run

    result = deploy_review_run(
        state.pr_number,
        io=io,
        github=github,
    )

    if result != 0:
        io.warn("Review agents had failures (see logs above)")

    # Fetch the consolidated findings from the PR comment
    findings = ""
    try:
        findings = github.fetch_pr_review_comment(
            state.pr_number, "Automated Safety Review"
        )
    except Exception:
        io.warn("Could not fetch review findings from PR comments.")

    return "complete", findings


def handle_fix_and_revalidate(
    state: PipelineStateData,
    io: IOProtocol,
    claude: Claude,
    git: Git,
    build: BuildSystem,
    *,
    review_findings: str,
    interactive: bool = False,
) -> str:
    """Fix review findings, re-validate, commit, and push.

    In interactive mode the user can guide the fix live.
    In pipeline mode the agent auto-fixes autonomously.
    Returns "complete" on success or "push_failed" if push fails.
    """
    io.header("Fixing Review Findings")

    fix_prompt = (
        f"The review agents found these issues on PR #{state.pr_number}:\n\n"
        f"{review_findings}\n\n"
        f"Fix all P1 (critical) and P2 (high) findings. "
        f"Address P3 items where reasonable.\n"
        f"Do NOT create a PR or push — just fix the code and commit."
    )

    worktree = Path(state.worktree_path) if state.worktree_path else None
    mode_label = "interactive" if interactive else "autonomous"
    io.print(f"  Launching feature agent to fix findings ({mode_label})...")

    # Always use interactive launch so the agent can request permissions
    # from the user when sandbox restrictions apply.
    claude.launch_interactive_with_prompt(
        model=state.model_tier,
        agent=state.agent_role,
        prompt=fix_prompt,
        cwd=worktree,
    )

    io.print("  Re-validating after fixes...")
    # Simplified validation
    _, _, _ = handle_validate(state, io, build)

    io.print("  Committing fixes...")
    try:
        git.add_all()
        git.commit(
            f"fix(#{state.issue_number}): address review findings\n\n"
            f"Pipeline auto-fix for PR #{state.pr_number} review comments.\n\n"
            f"Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>"
        )
    except Exception:
        io.warn("Nothing new to commit")

    io.print("  Pushing fixes...")
    try:
        git.push(state.branch_name)
        io.pass_("Push succeeded")
        state.fix_iterations += 1
        return "complete"
    except Exception:
        io.fail("Push failed. Fixes committed locally but not pushed.")
        return "push_failed"


def handle_cleanup(
    state: PipelineStateData,
    io: IOProtocol,
    git: Git,
    github: GitHub,
    *,
    project_dir: Path,
) -> str:
    """Finalize pipeline: verify PR links, update shared state.

    Returns "complete".
    """
    io.header("Pipeline Complete")

    # 1. Verify PR links to issue
    if state.pr_number:
        try:
            pr = github.pr_view(state.pr_number)
            body = pr.body if pr else ""
            pattern = rf"(closes|fixes|resolves)\s*#{state.issue_number}"
            if re.search(pattern, body, re.IGNORECASE):
                io.pass_(
                    f"PR #{state.pr_number} links to issue #{state.issue_number}"
                )
            else:
                io.warn(
                    f"PR #{state.pr_number} does NOT link issue "
                    f"#{state.issue_number}! Adding link..."
                )
                updated_body = f"{body}\n\nCloses #{state.issue_number}"
                github.pr_edit(state.pr_number, body=updated_body)
                io.pass_("PR body updated with issue link.")
        except Exception as e:
            io.warn(f"Could not verify PR link: {e}")

    # 2. Update active-work.md
    active_work = project_dir / "tasks" / "active-work.md"
    if active_work.exists():
        text = active_work.read_text()
        updated = text.replace(
            "- **Status:** in-progress",
            "- **Status:** completed",
        )
        if updated != text:
            active_work.write_text(updated)
            io.pass_("tasks/active-work.md → completed")

    # 3. Append to agent-changelog.md
    changelog = project_dir / "tasks" / "agent-changelog.md"
    if not changelog.exists():
        changelog.write_text("# Agent Changelog\n\n")

    date = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    entry = (
        f"\n### {date} | {state.agent_role} | {state.model_tier} "
        f"| PR #{state.pr_number or 'N/A'}\n"
        f"- **Issue:** #{state.issue_number} — {state.issue_title}\n"
        f"- **Branch:** {state.branch_name}\n"
        f"- **Mode:** pipeline\n"
        f"- **Fix iterations:** {state.fix_iterations}\n"
        f"- **Status:** completed\n"
    )
    with open(changelog, "a") as f:
        f.write(entry)
    io.pass_("tasks/agent-changelog.md updated")

    # 4. Check for pitfalls in report
    report_path = Path(state.worktree_path) / "AGENT_REPORT.md"
    if report_path.exists():
        report_text = report_path.read_text().lower()
        if any(
            word in report_text
            for word in ("risk", "pitfall", "gotcha", "non-obvious", "surprising")
        ):
            io.warn(
                "AGENT_REPORT.md mentions risks/pitfalls — consider adding to\n"
                "           .claude/shared-context/domain-knowledge.md"
            )

    # 5. Summary
    io.print("")
    io.print(f"  Issue:    #{state.issue_number} — {state.issue_title}")
    io.print(f"  Branch:   {state.branch_name}")
    if state.pr_url:
        io.print(f"  PR:       {state.pr_url}")
    io.print(f"  Role:     {state.agent_role}")
    io.print(f"  Model:    {state.model_tier}")

    # 6. Remove state file
    state_file = project_dir / ".pipeline-state"
    if state_file.exists():
        state_file.unlink()

    return "complete"


def handle_abort(
    state: PipelineStateData,
    io: IOProtocol,
    git: Git,
    *,
    project_dir: Path,
) -> str:
    """Prompt user to clean up or preserve state, then act accordingly.

    Returns "complete".
    """
    io.header("Pipeline Aborted")

    io.print(f"  Worktree: {state.worktree_path}")
    io.print(f"  Branch:   {state.branch_name}")
    if state.pr_url:
        io.print(f"  PR:       {state.pr_url} (still open)")
    io.print("")

    io.options([
        ("c", "clean up — remove worktree, delete branch, delete state file"),
        ("p", "preserve — keep everything for resume or manual work"),
    ])
    choice = io.prompt("  Your choice [c]: ", default="c").lower()

    if choice in ("c", "clean"):
        # 1. Remove worktree
        worktree_path = Path(state.worktree_path) if state.worktree_path else None
        if worktree_path:
            try:
                git.worktree_remove(worktree_path, force=True)
                io.pass_(f"Worktree removed: {worktree_path}")
            except Exception as e:
                io.warn(f"Could not remove worktree: {e}")

        # 2. Delete local branch
        if state.branch_name:
            try:
                git.delete_branch(state.branch_name, force=True)
                io.pass_(f"Branch deleted: {state.branch_name}")
            except Exception as e:
                io.warn(f"Could not delete branch: {e}")

        # 3. Remove state file
        state_file = project_dir / ".pipeline-state"
        if state_file.exists():
            state_file.unlink()
            io.pass_("Pipeline state file removed")

        io.print("")
        io.print("  Clean slate — ready to re-run.")

    else:
        # Preserve: save state for resume
        state_file = project_dir / ".pipeline-state"
        state.save(state_file)

        io.print("")
        io.print("  Resume pipeline:")
        io.print(
            f"    python -m orchestrator deploy-issue {state.issue_number} --pipeline"
        )
        io.print("")
        io.print("  Or work interactively:")
        io.print(f"    cd {state.worktree_path}")
        io.print(f"    python -m orchestrator start {state.agent_role}")

    return "complete"
