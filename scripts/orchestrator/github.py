"""GitHub CLI (gh) subprocess wrapper — all GitHub operations go through this module.

Wraps `gh` CLI calls for issues, PRs, labels, and comments. Used by
deploy-issue (issue fetch, PR create), deploy-review (PR diff, comment),
and pipeline cleanup (PR body verification).
"""

from __future__ import annotations

import json
import subprocess
from dataclasses import dataclass


class GitHubError(Exception):
    """Raised when a gh command fails."""

    def __init__(self, cmd: list[str], returncode: int, stderr: str) -> None:
        self.cmd = cmd
        self.returncode = returncode
        self.stderr = stderr
        super().__init__(f"gh failed (exit {returncode}): {stderr.strip()}")


@dataclass
class Issue:
    """Parsed GitHub issue."""

    number: int
    title: str
    body: str
    labels: list[str]


@dataclass
class PullRequest:
    """Parsed GitHub pull request."""

    number: int
    title: str
    body: str
    head_branch: str
    state: str


class GitHub:
    """Thin wrapper around gh CLI subprocess calls."""

    @staticmethod
    def _run(
        *args: str,
        check: bool = True,
        input_text: str | None = None,
    ) -> subprocess.CompletedProcess[str]:
        """Run a gh command."""
        cmd = ["gh", *args]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            input=input_text,
            timeout=30,
        )
        if check and result.returncode != 0:
            raise GitHubError(cmd, result.returncode, result.stderr)
        return result

    # ── Issues ─────────────────────────────────────────────────────────────

    def fetch_issue(self, number: int) -> Issue:
        """Fetch a GitHub issue by number."""
        result = self._run(
            "issue", "view", str(number),
            "--json", "title,body,labels,number",
        )
        data = json.loads(result.stdout)
        labels = [l["name"] for l in data.get("labels", [])]
        body = (data.get("body") or "")[:4096]  # cap length
        return Issue(
            number=data["number"],
            title=data["title"],
            body=body,
            labels=labels,
        )

    def edit_issue_labels(
        self, number: int, add: list[str] | None = None, remove: list[str] | None = None
    ) -> None:
        """Add or remove labels on an issue."""
        args = ["issue", "edit", str(number)]
        if add:
            args.extend(["--add-label", ",".join(add)])
        if remove:
            args.extend(["--remove-label", ",".join(remove)])
        self._run(*args)

    def issue_list(
        self,
        state: str = "open",
        labels: list[str] | None = None,
        limit: int = 30,
        json_fields: str = "number,title,labels,state",
    ) -> list[dict]:
        """List issues with optional filters."""
        args = ["issue", "list", "--state", state, "--limit", str(limit),
                "--json", json_fields]
        if labels:
            args.extend(["--label", ",".join(labels)])
        result = self._run(*args, check=False)
        if result.returncode != 0:
            return []
        return json.loads(result.stdout)

    # ── Pull Requests ──────────────────────────────────────────────────────

    @dataclass
    class CreatedPR:
        """Result of creating a PR."""

        number: int
        url: str

    def create_pr(
        self,
        base: str,
        title: str,
        body: str,
        head: str | None = None,
    ) -> CreatedPR:
        """Create a pull request. Returns CreatedPR with number and url."""
        args = ["pr", "create", "--base", base, "--title", title, "--body", body]
        if head:
            args.extend(["--head", head])
        result = self._run(*args)
        url = result.stdout.strip()
        # Extract PR number from URL (e.g., .../pull/123)
        number = 0
        if "/pull/" in url:
            try:
                number = int(url.rstrip("/").rsplit("/", 1)[-1])
            except ValueError:
                pass
        return self.CreatedPR(number=number, url=url)

    def pr_edit(
        self,
        number: int,
        title: str | None = None,
        body: str | None = None,
    ) -> None:
        """Edit a pull request title and/or body."""
        args = ["pr", "edit", str(number)]
        if title:
            args.extend(["--title", title])
        if body:
            args.extend(["--body", body])
        self._run(*args)

    def pr_view(self, number: int) -> PullRequest:
        """Fetch pull request details."""
        result = self._run(
            "pr", "view", str(number),
            "--json", "number,title,body,headRefName,state",
        )
        data = json.loads(result.stdout)
        return PullRequest(
            number=data["number"],
            title=data["title"],
            body=data.get("body", ""),
            head_branch=data.get("headRefName", ""),
            state=data.get("state", ""),
        )

    def pr_diff(self, number: int) -> str:
        """Get the full diff for a pull request."""
        result = self._run("pr", "diff", str(number))
        return result.stdout

    def pr_comment(self, number: int, body: str) -> None:
        """Post a comment on a pull request."""
        self._run("pr", "comment", str(number), "--body", body)

    def pr_list_for_branch(self, branch: str) -> int | None:
        """Find the PR number for a given head branch. Returns None if no PR."""
        result = self._run(
            "pr", "list", "--head", branch,
            "--json", "number",
            "--jq", ".[0].number",
            check=False,
        )
        if result.returncode != 0:
            return None
        num = result.stdout.strip()
        if not num or num == "null":
            return None
        try:
            return int(num)
        except ValueError:
            return None

    def pr_list(
        self,
        state: str = "open",
        base: str | None = None,
        limit: int = 10,
        json_fields: str = "number,title,state",
    ) -> list[dict]:
        """List pull requests with optional filters."""
        args = ["pr", "list", "--state", state, "--limit", str(limit),
                "--json", json_fields]
        if base:
            args.extend(["--base", base])
        result = self._run(*args, check=False)
        if result.returncode != 0:
            return []
        return json.loads(result.stdout)

    # ── PR metadata helpers ─────────────────────────────────────────────────

    def pr_add_labels(self, number: int, labels: list[str]) -> None:
        """Add labels to a pull request."""
        if labels:
            self._run("pr", "edit", str(number), "--add-label", ",".join(labels))

    def pr_set_milestone(self, number: int, milestone: str) -> None:
        """Set milestone on a pull request."""
        if milestone:
            self._run("pr", "edit", str(number), "--milestone", milestone)

    def issue_milestone(self, number: int) -> str:
        """Get the milestone title for an issue. Returns empty string if none."""
        result = self._run(
            "issue", "view", str(number),
            "--json", "milestone",
            check=False,
        )
        if result.returncode != 0:
            return ""
        data = json.loads(result.stdout)
        milestone = data.get("milestone")
        if milestone and isinstance(milestone, dict):
            return milestone.get("title", "")
        return ""

    # ── API ────────────────────────────────────────────────────────────────

    def api(self, endpoint: str, jq: str | None = None) -> str:
        """Call gh api with an endpoint and optional jq filter."""
        args = ["api", endpoint]
        if jq:
            args.extend(["--jq", jq])
        result = self._run(*args, check=False)
        return result.stdout.strip()

    def fetch_pr_review_comment(self, pr_number: int, marker: str) -> str:
        """Fetch the last PR comment containing a marker string."""
        jq_filter = (
            f'[map(select(.body | test("{marker}"))) | last | .body // empty]'
            " | first // empty"
        )
        return self.api(
            f"repos/{{owner}}/{{repo}}/issues/{pr_number}/comments",
            jq=jq_filter,
        )
