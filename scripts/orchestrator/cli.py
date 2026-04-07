"""CLI entry point — argparse subcommands for all orchestrator commands.

Usage: python -m orchestrator <command> [args]

Replaces all 12 bash scripts with a unified Python CLI.
"""

from __future__ import annotations

import argparse
import sys

from orchestrator import __version__
from orchestrator.config import ALL_ROLES, PIPELINE_VERSION, ROLES


# ── Command handlers ──────────────────────────────────────────────────────


def _cmd_list(_args: argparse.Namespace) -> int:
    from orchestrator.console import Console

    io = Console()
    rows = [(r, ROLES[r].tier.value, ROLES[r].model) for r in ALL_ROLES]
    io.role_table(rows)
    return 0


def _cmd_version(_args: argparse.Namespace) -> int:
    print(f"{PIPELINE_VERSION} (orchestrator {__version__})")
    return 0


def _cmd_start(args: argparse.Namespace) -> int:
    from orchestrator.commands.start_agent import run

    return run(
        args.role,
        args.task or "",
        interactive=args.interactive,
        dry_run=args.dry_run,
        skip_preflight=args.skip_preflight,
    )


def _cmd_deploy_issue(args: argparse.Namespace) -> int:
    from orchestrator.commands.deploy_issue import run

    return run(
        args.issue,
        pipeline=args.pipeline,
        headless=args.headless,
        skip_tech_lead=args.skip_tech_lead,
        base_branch=args.base,
        dry_run=args.dry_run,
        use_tmux=args.tmux,
        notify_topic=args.notify or "",
    )


def _cmd_deploy_review(args: argparse.Namespace) -> int:
    from orchestrator.commands.deploy_review import run

    return run(
        args.pr,
        force_all=args.all,
        dry_run=args.dry_run,
    )


def _cmd_validate(args: argparse.Namespace) -> int:
    from orchestrator.commands.validate_session import run

    return run(branch=args.branch or "")


def _cmd_session(args: argparse.Namespace) -> int:
    from orchestrator.commands.run_session import run

    return run(args.role, args.task, issue=args.issue)


def _cmd_dashboard(args: argparse.Namespace) -> int:
    from orchestrator.commands.dashboard import run

    return run(agent=args.agent or "", team=args.team, since=args.since or "")


def _cmd_boundaries(args: argparse.Namespace) -> int:
    from orchestrator.commands.boundaries import run

    return run(base=args.base)


def _cmd_stats(args: argparse.Namespace) -> int:
    from orchestrator.commands.stats import run

    return run(since=args.since or "30 days ago", all_time=args.all)


def _cmd_metrics(args: argparse.Namespace) -> int:
    from orchestrator.commands.metrics import run

    return run(logfile=args.logfile, summary=args.summary)


def _cmd_cleanup(args: argparse.Namespace) -> int:
    from orchestrator.commands.cleanup import run

    return run(dry_run=args.dry_run, auto_yes=args.yes)


def _cmd_health(args: argparse.Namespace) -> int:
    from orchestrator.commands.health import run

    return run()


def _cmd_log(args: argparse.Namespace) -> int:
    from orchestrator.commands.log_session import run

    return run(args.role, args.command)


def _cmd_sync_status(_args: argparse.Namespace) -> int:
    from orchestrator.commands.sync_status import run

    return run()


def _cmd_orchestrate(args: argparse.Namespace) -> int:
    from orchestrator.commands.orchestrate import run

    return run(
        args.number,
        epic=args.epic,
        dry_run=args.dry_run,
    )


def _cmd_pipeline(args: argparse.Namespace) -> int:
    from orchestrator.commands.pipeline_monitor import (
        run_attach,
        run_kill,
        run_list,
        run_status,
    )

    subcmd = args.pipeline_command
    if subcmd == "list":
        return run_list()
    if subcmd == "attach":
        return run_attach(args.issue)
    if subcmd == "status":
        return run_status(issue=args.issue if hasattr(args, "issue") else None)
    if subcmd == "kill":
        return run_kill(args.issue)

    # No subcommand — show list
    return run_list()


# ── Parser ─────────────────────────────────────────────────────────────────


def build_parser() -> argparse.ArgumentParser:
    """Build the top-level argument parser with all subcommands."""
    parser = argparse.ArgumentParser(
        prog="orchestrator",
        description="Multi-agent pipeline orchestration for the drone companion stack.",
    )
    parser.add_argument(
        "--version", action="store_true", help="Print version and exit"
    )

    sub = parser.add_subparsers(dest="command", help="Available commands")

    # ── list ───────────────────────────────────────────────────────────
    sub.add_parser("list", help="Print all available agent roles")

    # ── start ──────────────────────────────────────────────────────────
    p = sub.add_parser("start", help="Launch an agent session")
    p.add_argument("role", help="Agent role")
    p.add_argument("task", nargs="?", default="", help="Task description")
    p.add_argument("--interactive", action="store_true", help="Interactive mode")
    p.add_argument("--dry-run", action="store_true", help="Print config, don't launch")
    p.add_argument("--skip-preflight", action="store_true", help="Skip preflight checks")

    # ── deploy-issue ───────────────────────────────────────────────────
    p = sub.add_parser("deploy-issue", help="Deploy agent for a GitHub issue")
    p.add_argument("issue", type=int, help="Issue number")
    p.add_argument("--pipeline", action="store_true",
                   help="Pipeline mode (agent works autonomously, for on-the-move use)")
    p.add_argument("--headless", action="store_true",
                   help="Headless mode (fire-and-forget, no checkpoints)")
    p.add_argument("--skip-tech-lead", action="store_true",
                   help="Skip tech-lead analysis (use auto-triage only)")
    p.add_argument("--base", default="main", help="Base branch (default: main)")
    p.add_argument("--dry-run", action="store_true", help="Show routing without launching")
    p.add_argument("--tmux", action="store_true",
                   help="Run pipeline inside a named tmux session (pipeline-<issue>)")
    p.add_argument("--notify", metavar="TOPIC",
                   help="ntfy.sh topic for push notifications (or set NTFY_TOPIC env var)")

    # ── deploy-review ──────────────────────────────────────────────────
    p = sub.add_parser("deploy-review", help="Launch review agents for a PR")
    p.add_argument("pr", type=int, help="PR number")
    p.add_argument("--all", action="store_true", help="Force all review agents")
    p.add_argument("--dry-run", action="store_true", help="Show routing without launching")

    # ── validate ───────────────────────────────────────────────────────
    p = sub.add_parser("validate", help="Post-session hallucination detector")
    p.add_argument("--branch", default="", help="Branch to validate")

    # ── session ────────────────────────────────────────────────────────
    p = sub.add_parser("session", help="Full orchestrated session (6 phases)")
    p.add_argument("role", help="Agent role")
    p.add_argument("task", help="Task description")
    p.add_argument("--issue", type=int, default=None, help="GitHub issue number")

    # ── dashboard ──────────────────────────────────────────────────────
    p = sub.add_parser("dashboard", help="Agent metrics dashboard")
    p.add_argument("--agent", default="", help="Per-agent report")
    p.add_argument("--team", action="store_true", help="Team-wide report")
    p.add_argument("--since", default="", help="Filter by date")

    # ── boundaries ─────────────────────────────────────────────────────
    p = sub.add_parser("boundaries", help="Check agent boundary violations")
    p.add_argument("--base", default="origin/main", help="Base ref")

    # ── stats ──────────────────────────────────────────────────────────
    p = sub.add_parser("stats", help="Agent commit statistics")
    p.add_argument("--since", default="", help="Period filter")
    p.add_argument("--all", action="store_true", help="All time")

    # ── metrics ────────────────────────────────────────────────────────
    p = sub.add_parser("metrics", help="Parse session log metrics")
    p.add_argument("logfile", nargs="?", default=None, help="Session log file")
    p.add_argument("--summary", action="store_true", help="All sessions")

    # ── cleanup ────────────────────────────────────────────────────────
    p = sub.add_parser("cleanup", help="Clean merged branches and stale worktrees")
    p.add_argument("--dry-run", action="store_true", help="Show what would be cleaned")
    p.add_argument("--yes", "-y", action="store_true", help="Skip confirmation")

    # ── health ─────────────────────────────────────────────────────────
    sub.add_parser("health", help="Health report (tests, coverage, build)")

    # ── log ────────────────────────────────────────────────────────────
    p = sub.add_parser("log", help="Log a session transcript")
    p.add_argument("role", help="Agent role")
    p.add_argument("command", nargs=argparse.REMAINDER, help="Command to run")

    # ── sync-status ────────────────────────────────────────────────────
    sub.add_parser("sync-status", help="Auto-update project-status.md")

    # ── orchestrate ────────────────────────────────────────────────────
    p = sub.add_parser("orchestrate", help="Tech-lead orchestrated workflow")
    p.add_argument("number", type=int, help="Issue or epic number")
    p.add_argument("--epic", action="store_true", help="Treat as epic (multi-issue)")
    p.add_argument("--dry-run", action="store_true", help="Show plan without executing")

    # ── pipeline ──────────────────────────────────────────────────────
    p = sub.add_parser("pipeline", help="Monitor and manage pipeline sessions")
    pipeline_sub = p.add_subparsers(dest="pipeline_command", help="Pipeline commands")

    pipeline_sub.add_parser("list", help="List active pipeline tmux sessions")

    ps = pipeline_sub.add_parser("attach", help="Attach to a pipeline session")
    ps.add_argument("issue", type=int, help="Issue number")

    ps = pipeline_sub.add_parser("status", help="Show pipeline checkpoint status")
    ps.add_argument("issue", type=int, nargs="?", default=None,
                    help="Issue number (omit for all)")

    ps = pipeline_sub.add_parser("kill", help="Kill a pipeline session")
    ps.add_argument("issue", type=int, help="Issue number")

    return parser


# ── Command dispatch ──────────────────────────────────────────────────────

COMMAND_HANDLERS = {
    "list": _cmd_list,
    "start": _cmd_start,
    "deploy-issue": _cmd_deploy_issue,
    "deploy-review": _cmd_deploy_review,
    "validate": _cmd_validate,
    "session": _cmd_session,
    "dashboard": _cmd_dashboard,
    "boundaries": _cmd_boundaries,
    "stats": _cmd_stats,
    "metrics": _cmd_metrics,
    "cleanup": _cmd_cleanup,
    "health": _cmd_health,
    "log": _cmd_log,
    "sync-status": _cmd_sync_status,
    "orchestrate": _cmd_orchestrate,
    "pipeline": _cmd_pipeline,
}


def main(argv: list[str] | None = None) -> None:
    """Main entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.version:
        _cmd_version(args)
        sys.exit(0)

    if not args.command:
        parser.print_help()
        sys.exit(1)

    handler = COMMAND_HANDLERS.get(args.command)
    if handler is None:
        print(f"Command '{args.command}' not yet implemented.", file=sys.stderr)
        sys.exit(1)

    sys.exit(handler(args))
