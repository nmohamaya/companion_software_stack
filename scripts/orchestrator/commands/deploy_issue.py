"""Deploy issue command — fetch, route, worktree, launch.

Replaces deploy-issue.sh (1316 lines).
Three modes:
  - Interactive (default): Tech-lead orchestrated pipeline, agent runs
    interactively — user can chat with it and guide modifications live.
  - Pipeline (--pipeline): Tech-lead orchestrated pipeline, agent runs
    autonomously (print mode) — for when you're on the move.
  - Headless (--headless): Fire-and-forget, no checkpoints, no tech-lead.

Both interactive and pipeline use the same 12-state FSM with 5 checkpoints.
"""

from __future__ import annotations

import sys
from pathlib import Path

from orchestrator.config import get_role, resolve_project_dir
from orchestrator.console import Console
from orchestrator.context import format_context, gather_context
from orchestrator.git import Git
from orchestrator.github import GitHub
from orchestrator.claude import Claude
from orchestrator.build import BuildSystem
from orchestrator.routing import branch_name_for_issue, triage_issue_labels
from orchestrator.tech_lead import TechLead
from orchestrator.pipeline.fsm import (
    PipelineState,
    is_checkpoint,
    is_terminal,
    transition,
)
from orchestrator.pipeline.state import PipelineStateData
from orchestrator.pipeline.checkpoints import (
    cp1_review,
    cp2_commit,
    cp3_pr_preview,
    cp4_findings,
    cp5_final,
)
from orchestrator.pipeline.handlers import (
    handle_abort,
    handle_agent_work,
    handle_cleanup,
    handle_fix_and_revalidate,
    handle_pr_create,
    handle_review,
    handle_validate,
)


def _prompt_branch_strategy(io: Console, git: Git, issue_number: int) -> str:
    """Prompt user for branch strategy: merge to main or integration branch."""
    # Check for existing integration branches
    integration_branches = git.list_branches("integration/*")

    io.print("Branch strategy:")
    io.options([
        ("m", "main — PR merges directly to main"),
        ("i", "integration branch — PR targets an integration branch"),
    ])

    choice = io.prompt("  Your choice [m]: ", default="m").lower()

    if choice in ("m", "main"):
        return "main"

    # Integration branch flow
    if integration_branches:
        io.print("")
        io.print("  Existing integration branches:")
        for idx, branch in enumerate(integration_branches, 1):
            io.print(f"    {idx}. {branch}")
        io.print(f"    n. Create new integration branch")
        io.print("")

        pick = io.prompt("  Select branch (number or 'n'): ", default="1")
        if pick != "n":
            try:
                idx = int(pick) - 1
                if 0 <= idx < len(integration_branches):
                    return integration_branches[idx]
            except ValueError:
                pass
            io.warn("Invalid selection, creating new branch.")

    # Create new integration branch
    default_name = f"integration/epic-{issue_number}"
    name = io.prompt(f"  Integration branch name [{default_name}]: ", default=default_name)

    if not git.branch_exists(name):
        io.print(f"  Creating branch '{name}' from main...")
        git.create_branch(name, "main")
        git.push(name, set_upstream=True)
        io.pass_(f"Integration branch '{name}' created and pushed")
    else:
        io.info(f"Branch '{name}' already exists")

    return name


def _tech_lead_routing(
    io: Console,
    claude: Claude,
    github: GitHub,
    project_dir: Path,
    issue_number: int,
    auto_role: str,
) -> str:
    """Run tech-lead analysis and let human approve/override routing.

    Returns the final role (may differ from auto_role if tech-lead or human overrides).
    """
    tech_lead = TechLead(claude=claude, github=github, io=io)
    decision = tech_lead.analyze_issue(issue_number, project_dir=project_dir)

    io.print("")
    io.print("--- Tech-Lead Routing ---")
    io.print(f"  Recommended role:     {decision.recommended_role}")
    io.print(f"  Recommended priority: {decision.recommended_priority}")
    io.print(f"  Reasoning:            {decision.reasoning}")
    if decision.sequencing_notes:
        io.print(f"  Sequencing:           {decision.sequencing_notes}")
    if decision.dependencies:
        io.print(f"  Dependencies:         {decision.dependencies}")
    io.print("")

    io.options([
        ("a", f"accept — use {decision.recommended_role}"),
        ("o", f"override — keep auto-triage ({auto_role})"),
        ("c", "custom — enter a different role"),
    ])

    choice = io.prompt("  Your choice [a]: ", default="a").lower()
    if choice in ("o", "override"):
        io.info(f"Using auto-triage role: {auto_role}")
        return auto_role
    if choice in ("c", "custom"):
        custom = io.prompt("  Role: ")
        if custom:
            return custom
    # Default: accept tech-lead recommendation
    return decision.recommended_role


def run(
    issue_number: int,
    *,
    io: Console | None = None,
    git: Git | None = None,
    github: GitHub | None = None,
    claude: Claude | None = None,
    pipeline: bool = False,
    headless: bool = False,
    skip_tech_lead: bool = False,
    base_branch: str = "main",
    dry_run: bool = False,
) -> int:
    """Deploy an agent for a GitHub issue.

    Both interactive (default) and pipeline modes run the full 12-state FSM
    with 5 checkpoints and tech-lead orchestration. The difference:
      - Interactive: agent runs interactively, user can chat and guide live.
      - Pipeline: agent runs autonomously (print mode), for on-the-move use.
      - Headless: fire-and-forget, no FSM, no checkpoints, no tech-lead.
    """
    if io is None:
        io = Console()
    project_dir = resolve_project_dir()
    if git is None:
        git = Git(project_dir)
    if github is None:
        github = GitHub()
    if claude is None:
        claude = Claude()
    build = BuildSystem(project_dir)

    # 1. Fetch issue
    io.print(f"Fetching issue #{issue_number}...")
    try:
        issue = github.fetch_issue(issue_number)
    except Exception as e:
        io.error(f"Failed to fetch issue #{issue_number}: {e}")
        return 1

    io.print(f"  Title:  {issue.title}")
    io.print(f"  Labels: {', '.join(issue.labels)}")
    io.print("")

    # 2. Triage labels → role (fast, deterministic)
    triage = triage_issue_labels(issue.labels)
    if not triage.role:
        io.error(
            f"Cannot determine agent role from labels: {issue.labels}. "
            f"Add a domain label (perception, nav-planning, comms, etc.)"
        )
        return 1

    role = triage.role

    # 3. Tech-lead routing (unless skipped or headless)
    if not skip_tech_lead and not headless and not dry_run:
        role = _tech_lead_routing(
            io, claude, github, project_dir,
            issue_number, auto_role=role,
        )

    try:
        role_config = get_role(role)
    except KeyError:
        io.error(f"Unknown role '{role}' from triage")
        return 1

    io.print(f"  Routed to: {role} ({role_config.tier.value})")

    # 4. Branch strategy
    branch_name = branch_name_for_issue(
        issue_number, issue.title, is_bug=triage.is_bug
    )

    # If no explicit --base was given and not headless/dry-run, prompt for strategy
    if base_branch == "main" and not headless and not dry_run:
        base_branch = _prompt_branch_strategy(io, git, issue_number)
        io.print("")

    io.print(f"  Branch:    {branch_name}")
    io.print(f"  Base:      {base_branch}")
    io.print("")

    if dry_run:
        io.info("Dry-run mode — not creating worktree or launching agent.")
        return 0

    # 5. Create worktree
    worktree_dir = project_dir / ".claude" / "worktrees" / branch_name.replace("/", "-")
    io.print(f"Creating worktree at {worktree_dir}...")

    try:
        if not git.branch_exists(branch_name):
            git.create_branch(branch_name, base_branch)
        git.worktree_add(str(worktree_dir), branch_name)
        io.pass_("Worktree created")
    except Exception as e:
        io.warn(f"Worktree setup: {e} (may already exist)")

    # 6. Build prompt with cross-agent context
    ctx = gather_context(project_dir)
    ctx_text = format_context(ctx)

    prompt = (
        f"Implement GitHub issue #{issue_number}: {issue.title}\n\n"
        f"{issue.body}\n\n"
        f"Labels: {', '.join(issue.labels)}"
    )
    if ctx_text:
        prompt += ctx_text

    # Add codebase context so agents know about the Python orchestrator
    prompt += (
        f"\n\nCODEBASE CONTEXT:\n"
        f"- The multi-agent pipeline is orchestrated by a Python package at "
        f"scripts/orchestrator/ (invoked via `python -m orchestrator`).\n"
        f"- The legacy bash scripts in scripts/*.sh are being replaced — "
        f"do NOT modify them. All new orchestration features go in "
        f"scripts/orchestrator/.\n"
        f"- For C++ drone code: follow patterns in common/, process[1-7]_*.\n"
        f"- Read CLAUDE.md for full build commands, test commands, and "
        f"coding standards.\n"
    )

    # 7. Dispatch by mode
    if headless:
        # Fire-and-forget: no FSM, no checkpoints, no tech-lead
        io.print("Launching agent in headless mode...")
        claude.launch_print(
            model=role_config.model,
            agent=role,
            prompt=prompt,
            cwd=worktree_dir,
        )
        return 0

    # Both interactive and pipeline use the FSM
    # interactive=True (default): user chats with agent live
    # interactive=False (--pipeline): agent works autonomously
    interactive = not pipeline

    # Create tech-lead for checkpoint reviews (unless skipped)
    tech_lead = None
    if not skip_tech_lead:
        tech_lead = TechLead(claude=claude, github=github, io=io)

    # Gather issue metadata for PR linking
    issue_labels = issue.labels
    milestone = ""
    epic_number = 0
    try:
        milestone = github.issue_milestone(issue_number)
    except Exception:
        pass
    # Detect epic from labels (e.g., "epic:modularity" → find parent epic)
    for label in issue.labels:
        if label.startswith("epic:"):
            # Epic label present but we need the epic issue number
            # This will be populated by tech-lead routing if available
            break

    return _run_pipeline(
        io=io,
        git=git,
        github=github,
        claude=claude,
        build=build,
        project_dir=project_dir,
        issue_number=issue_number,
        issue_title=issue.title,
        issue_labels=issue_labels,
        milestone=milestone,
        epic_number=epic_number,
        role=role,
        role_config=role_config,
        branch_name=branch_name,
        worktree_dir=worktree_dir,
        base_branch=base_branch,
        is_bug=triage.is_bug,
        prompt=prompt,
        interactive=interactive,
        tech_lead=tech_lead,
    )


def _run_pipeline(
    *,
    io: Console,
    git: Git,
    github: GitHub,
    claude: Claude,
    build: BuildSystem,
    project_dir: Path,
    issue_number: int,
    issue_title: str,
    issue_labels: list[str] | None = None,
    milestone: str = "",
    epic_number: int = 0,
    role: str,
    role_config,
    branch_name: str,
    worktree_dir: Path,
    base_branch: str,
    is_bug: bool,
    prompt: str,
    interactive: bool = True,
    tech_lead: TechLead | None = None,
) -> int:
    """Run the 12-state pipeline FSM.

    interactive=True: agent runs interactively (user can chat).
    interactive=False: agent runs in print mode (autonomous).
    tech_lead: if provided, used for advisory review at CP1 and CP4.
    """

    # Check for resume state
    state_file = project_dir / ".pipeline-state"
    resumed = False
    if state_file.exists():
        loaded = PipelineStateData.load(state_file)
        # Don't resume from terminal states (ABORT/CLEANUP) — start fresh
        loaded_state = loaded.get_state()
        if (
            loaded.issue_number == issue_number
            and not is_terminal(loaded_state)
        ):
            state = loaded
            resumed = True
            io.info(f"Resuming pipeline from {state.current_state}")
        else:
            state_file.unlink()

    if not resumed:
        state = PipelineStateData(
            issue_number=issue_number,
            issue_title=issue_title,
            issue_labels=issue_labels or [],
            agent_role=role,
            model_tier=role_config.tier.value,
            branch_name=branch_name,
            worktree_path=str(worktree_dir),
            is_bug=is_bug,
            milestone=milestone,
            epic_number=epic_number,
            pipeline_version="2.0.0",
        )

    # Build the agent prompt (pipeline adds extra instructions for autonomous mode)
    if interactive:
        agent_prompt = prompt
    else:
        agent_prompt = (
            f"{prompt}\n\n"
            f"IMPORTANT — PIPELINE MODE INSTRUCTIONS:\n"
            f"1. Implement the issue fully (code, tests, build verification).\n"
            f"2. Update documentation (REQUIRED before completion).\n"
            f"3. Do NOT create a PR or push — leave changes as local commits.\n"
            f"4. When done, write AGENT_REPORT.md in the worktree root.\n"
        )

    mode_label = "interactive" if interactive else "pipeline (autonomous)"
    io.info(f"Mode: {mode_label}")

    # Git instance for worktree operations (commits, pushes, diffs).
    # The main `git` is rooted at project_dir; worktree has its own checkout.
    wt_git = Git(worktree_dir)

    # Track validation state across checkpoints
    validation_output = ""
    validation_passed = True
    review_findings = ""

    # FSM loop
    while True:
        current = state.get_state()
        state.save(state_file)

        if is_terminal(current):
            if current == PipelineState.CLEANUP:
                action = handle_cleanup(
                    state, io, git, github, project_dir=project_dir
                )
                return 0
            else:  # ABORT
                action = handle_abort(state, io, git, project_dir=project_dir)
                return 1

        # Dispatch to handler or checkpoint
        if current == PipelineState.AGENT_WORK:
            action = handle_agent_work(
                state, io, claude,
                prompt=agent_prompt,
                interactive=interactive,
            )

        elif current == PipelineState.CP1_REVIEW:
            report_path = worktree_dir / "AGENT_REPORT.md"
            diff_stat = ""
            try:
                # Use working-tree diff against base so uncommitted changes
                # also appear (agent may not have committed everything).
                diff_stat = wt_git.diff_stat_working_tree(base_branch)
                if not diff_stat:
                    # Fall back to committed-only diff
                    diff_stat = wt_git.diff_stat(base_branch, "HEAD")
            except Exception:
                pass
            action = cp1_review(
                state, io,
                report_path=report_path,
                diff_stat=diff_stat,
                tech_lead=tech_lead,
            )
            if action == "changes":
                # Open interactive session for modifications
                claude.launch_interactive(
                    model=role_config.model, agent=role, cwd=worktree_dir
                )
                # Stay at CP1 (changes loops back)

        elif current == PipelineState.VALIDATE:
            action, validation_output, validation_passed = handle_validate(
                state, io, build
            )

        elif current == PipelineState.CP2_COMMIT:
            action = cp2_commit(
                state, io,
                validation_output=validation_output,
                validation_passed=validation_passed,
            )
            if action == "commit":
                # Stage and commit in the worktree (where agent made changes)
                try:
                    wt_git.add_all()
                    msg = (
                        f"feat(#{issue_number}): {issue_title}\n\n"
                        f"Pipeline commit for issue #{issue_number}.\n"
                        f"Agent role: {role}\n\n"
                        f"Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>"
                    )
                    wt_git.commit(msg)
                    io.pass_("Changes committed")
                except Exception:
                    io.warn("Nothing to commit (may already be committed)")

        elif current == PipelineState.PR_CREATE:
            action = handle_pr_create(state, io, wt_git, github)

        elif current == PipelineState.CP3_PR_PREVIEW:
            action = cp3_pr_preview(state, io)
            if action == "create":
                try:
                    pr = github.create_pr(
                        title=state.pr_title,
                        body=state.pr_body,
                        base=base_branch,
                        head=branch_name,
                    )
                    state.pr_number = pr.number
                    state.pr_url = pr.url
                    io.pass_(f"PR created: {pr.url}")
                    # Apply labels and milestone from source issue
                    if state.issue_labels:
                        try:
                            github.pr_add_labels(pr.number, state.issue_labels)
                            io.pass_(f"Labels applied: {', '.join(state.issue_labels)}")
                        except Exception:
                            io.warn("Could not apply labels to PR")
                    if state.milestone:
                        try:
                            github.pr_set_milestone(pr.number, state.milestone)
                            io.pass_(f"Milestone set: {state.milestone}")
                        except Exception:
                            io.warn("Could not set milestone on PR")
                except Exception as e:
                    io.fail(f"Failed to create PR: {e}")
                    action = "edit"  # re-show preview
            elif action == "update" and state.pr_number:
                try:
                    github.pr_edit(
                        state.pr_number,
                        title=state.pr_title,
                        body=state.pr_body,
                    )
                    io.pass_(f"PR #{state.pr_number} updated")
                except Exception as e:
                    io.fail(f"Failed to update PR: {e}")

        elif current == PipelineState.REVIEW:
            action, review_findings = handle_review(state, io, github)

        elif current == PipelineState.CP4_FINDINGS:
            action = cp4_findings(
                state, io,
                review_findings=review_findings,
                tech_lead=tech_lead,
            )

        elif current == PipelineState.FIX_AND_REVALIDATE:
            action = handle_fix_and_revalidate(
                state, io, claude, wt_git, build,
                review_findings=review_findings,
                interactive=interactive,
            )

        elif current == PipelineState.CP5_FINAL:
            commit_log = ""
            try:
                log_lines = wt_git.log_oneline()
                commit_log = "\n".join(log_lines[:20])
            except Exception:
                pass
            action = cp5_final(
                state, io,
                commit_log=commit_log,
                validation_passed=validation_passed,
            )

        else:
            io.error(f"BUG: unhandled state {current.name}")
            return 2

        # Record and transition
        if is_checkpoint(current):
            state.record_checkpoint(action)

        next_state = transition(current, action)
        if next_state is None:
            return 0
        state.set_state(next_state)
