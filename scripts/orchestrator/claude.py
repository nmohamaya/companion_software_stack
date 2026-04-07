"""Claude CLI subprocess wrapper — all agent launches go through this module.

Wraps `claude` CLI invocations for agent launching in different modes:
  - Interactive: exec replaces the process (user converses with agent)
  - Print mode: subprocess captures output (non-interactive, for automation)
  - Continue: resumes a previous session
  - Background: async subprocess for parallel review agents

Corresponds to the claude invocations in:
  - start-agent.sh (lines 295-307)
  - deploy-issue.sh (lines 523, 741, 777, 1024)
  - deploy-review.sh (lines 233-239)
"""

from __future__ import annotations

import asyncio
import os
import subprocess
import sys
from pathlib import Path


class ClaudeError(Exception):
    """Raised when a claude CLI command fails unexpectedly."""

    def __init__(self, returncode: int, stderr: str = "") -> None:
        self.returncode = returncode
        self.stderr = stderr
        super().__init__(f"claude exited with code {returncode}")


class Claude:
    """Thin wrapper around the claude CLI."""

    def launch_interactive(
        self,
        model: str,
        agent: str,
        prompt: str | None = None,
        cwd: Path | None = None,
    ) -> None:
        """Launch an interactive claude session (exec, replaces current process).

        Used by start-agent for interactive mode.
        """
        if cwd:
            os.chdir(cwd)
        cmd = ["claude", "--model", model, "--agent", agent]
        if prompt:
            cmd.append(prompt)
        os.execvp("claude", cmd)

    def launch_print(
        self,
        model: str,
        agent: str,
        prompt: str,
        permission_mode: str = "acceptEdits",
        log_file: Path | None = None,
        capture: bool = False,
        continue_session: bool = False,
        cwd: Path | None = None,
    ) -> int | str:
        """Launch claude in print mode (non-interactive, captures output).

        Used by deploy-issue --auto/--pipeline, deploy-review, and tech-lead.
        Returns the exit code normally, or captured stdout if capture=True.
        """
        cmd = [
            "claude",
            "--model", model,
            "--agent", agent,
            "--permission-mode", permission_mode,
        ]
        if continue_session:
            cmd.append("--continue")
        cmd.extend(["-p", prompt])

        run_kwargs: dict = {}
        if cwd:
            run_kwargs["cwd"] = str(cwd)

        if log_file:
            with open(log_file, "w") as f:
                result = subprocess.run(
                    cmd, stdout=f, stderr=subprocess.STDOUT, **run_kwargs
                )
            return result.returncode

        if capture:
            result = subprocess.run(
                cmd, capture_output=True, text=True, **run_kwargs
            )
            return result.stdout

        result = subprocess.run(cmd, **run_kwargs)
        return result.returncode

    def launch_continue(
        self,
        model: str,
        agent: str,
        prompt: str | None = None,
        permission_mode: str | None = None,
    ) -> int:
        """Resume a previous claude session with --continue.

        Used by pipeline FIX_AND_REVALIDATE and auto mode review loop.
        Returns the exit code.
        """
        cmd = ["claude", "--model", model, "--agent", agent, "--continue"]
        if permission_mode:
            cmd.extend(["--permission-mode", permission_mode])
        if prompt:
            cmd.extend(["-p", prompt])

        result = subprocess.run(cmd)
        return result.returncode

    def launch_interactive_with_prompt(
        self,
        model: str,
        agent: str,
        prompt: str,
        cwd: Path | None = None,
    ) -> int:
        """Launch an interactive session with an initial prompt (first message).

        Used by start-agent --interactive with a task. Agent opens
        interactively — user can converse after the initial response.
        Returns exit code (only if the session ends, not exec).
        """
        cmd = ["claude", "--model", model, "--agent", agent, prompt]
        run_kwargs: dict = {}
        if cwd:
            run_kwargs["cwd"] = str(cwd)
        result = subprocess.run(cmd, **run_kwargs)
        return result.returncode

    def continue_interactive(
        self,
        model: str,
        agent: str,
    ) -> int:
        """Resume a session interactively (for changes review in auto mode).

        Returns exit code.
        """
        cmd = ["claude", "--model", model, "--agent", agent, "--continue"]
        result = subprocess.run(cmd)
        return result.returncode


class AsyncClaude:
    """Async wrapper for parallel agent launches (deploy-review).

    Uses asyncio.create_subprocess_exec for concurrent agent runs
    with timeout and cancellation support.
    """

    async def launch_agent(
        self,
        model: str,
        agent: str,
        prompt: str,
        log_file: Path,
        permission_mode: str = "acceptEdits",
        timeout: int = 1800,  # 30 minutes default
    ) -> int:
        """Launch a claude agent asynchronously with timeout.

        Output is captured to log_file. Returns exit code.
        Raises asyncio.TimeoutError if timeout is exceeded.
        """
        cmd = [
            "claude",
            "--model", model,
            "--agent", agent,
            "--permission-mode", permission_mode,
            "-p", prompt,
        ]

        with open(log_file, "w") as f:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=f,
                stderr=asyncio.subprocess.STDOUT,
            )
            try:
                await asyncio.wait_for(proc.wait(), timeout=timeout)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
                raise

        return proc.returncode or 0

    async def launch_parallel(
        self,
        agents: list[dict],
        timeout: int = 1800,
    ) -> list[tuple[str, int | None, str | None]]:
        """Launch multiple agents in parallel.

        Args:
            agents: list of dicts with keys: name, model, agent, prompt, log_file
            timeout: max seconds per agent

        Returns:
            list of (agent_name, exit_code_or_None, error_msg_or_None)
        """
        results: list[tuple[str, int | None, str | None]] = []

        async def _run_one(spec: dict) -> tuple[str, int | None, str | None]:
            name = spec["name"]
            try:
                exit_code = await self.launch_agent(
                    model=spec["model"],
                    agent=spec["agent"],
                    prompt=spec["prompt"],
                    log_file=Path(spec["log_file"]),
                    timeout=timeout,
                )
                return (name, exit_code, None)
            except asyncio.TimeoutError:
                return (name, None, "timeout")
            except Exception as e:
                return (name, None, str(e))

        tasks = [asyncio.create_task(_run_one(a)) for a in agents]
        completed = await asyncio.gather(*tasks, return_exceptions=True)

        for item in completed:
            if isinstance(item, Exception):
                results.append(("unknown", None, str(item)))
            else:
                results.append(item)

        return results
