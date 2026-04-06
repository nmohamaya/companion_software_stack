#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

DRY_RUN=false
AUTO_YES=false
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=true ;;
        --yes|-y)  AUTO_YES=true ;;
        --help|-h) echo "Usage: $(basename "$0") [--dry-run] [--yes]"; exit 0 ;;
        *)         echo "Unknown argument: $arg" >&2; exit 1 ;;
    esac
done

PROTECTED_BRANCHES="main|develop"

# Collect merged branches (excluding protected and current)
MERGED_BRANCHES=()
while IFS= read -r branch; do
    branch=$(echo "$branch" | xargs)  # trim whitespace
    [[ -z "$branch" ]] && continue
    if echo "$branch" | grep -qE "^($PROTECTED_BRANCHES)$"; then
        continue
    fi
    MERGED_BRANCHES+=("$branch")
done < <(git branch --merged main 2>/dev/null | grep -v '\*' || true)

# Collect worktrees
declare -A WORKTREE_MAP
while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    # Use porcelain-compatible parsing: first field is path (may contain spaces with awk,
    # but standard git worktree list separates fields by whitespace with path first)
    WT_PATH=$(echo "$line" | awk '{print $1}')
    WT_BRANCH=$(echo "$line" | grep -oP '\[.*?\]' | tr -d '[]' || true)
    if [[ -n "$WT_BRANCH" ]]; then
        WORKTREE_MAP["$WT_BRANCH"]="$WT_PATH"
    fi
done < <(git worktree list --porcelain 2>/dev/null | awk '/^worktree /{path=$0; sub(/^worktree /, "", path)} /^branch /{br=$0; sub(/^branch refs\/heads\//, "", br); print path " [" br "]"}')

# Identify stale worktrees (worktrees whose branch is merged)
STALE_WORKTREES=()
for branch in "${MERGED_BRANCHES[@]}"; do
    if [[ -n "${WORKTREE_MAP[$branch]:-}" ]]; then
        STALE_WORKTREES+=("${WORKTREE_MAP[$branch]}")
    fi
done

echo "=========================================="
echo "  Branch & Worktree Cleanup"
echo "=========================================="
echo ""

if [[ ${#MERGED_BRANCHES[@]} -eq 0 && ${#STALE_WORKTREES[@]} -eq 0 ]]; then
    echo "Nothing to clean up."
    exit 0
fi

if [[ ${#MERGED_BRANCHES[@]} -gt 0 ]]; then
    echo "Merged branches to delete (${#MERGED_BRANCHES[@]}):"
    for b in "${MERGED_BRANCHES[@]}"; do
        echo "  - $b"
    done
    echo ""
fi

if [[ ${#STALE_WORKTREES[@]} -gt 0 ]]; then
    echo "Stale worktrees to remove (${#STALE_WORKTREES[@]}):"
    for wt in "${STALE_WORKTREES[@]}"; do
        echo "  - $wt"
    done
    echo ""
fi

if [[ "$DRY_RUN" == true ]]; then
    echo "(dry run — no changes made)"
    exit 0
fi

# Prompt for confirmation (skip if --yes)
if [[ "$AUTO_YES" == "true" ]]; then
    CONFIRM="y"
else
    read -rp "Proceed with cleanup? [y/N] " CONFIRM
fi
if [[ ! "$CONFIRM" =~ ^[yY]$ ]]; then
    echo "Aborted."
    exit 0
fi

WT_REMOVED=0
BR_REMOVED=0

# Remove stale worktrees first
for wt in "${STALE_WORKTREES[@]}"; do
    echo "Removing worktree: $wt"
    if git worktree remove "$wt" 2>/dev/null; then
        WT_REMOVED=$((WT_REMOVED + 1))
    else
        echo "  Warning: could not remove worktree $wt (may have uncommitted changes)"
    fi
done

# Delete merged branches
for branch in "${MERGED_BRANCHES[@]}"; do
    echo "Deleting branch: $branch"
    if git branch -d "$branch" 2>/dev/null; then
        BR_REMOVED=$((BR_REMOVED + 1))
    else
        echo "  Warning: could not delete branch $branch"
    fi
done

echo ""
echo "Summary: $BR_REMOVED branches removed, $WT_REMOVED worktrees cleaned"
