# Cross-Domain Agent Handoff Protocol

When work spans domains (e.g., perception needs a new IPC type that the integration layer must publish, or navigation changes require mission planner updates), agents must coordinate handoffs explicitly.

## Standard Handoff: Sequential Issue Chain

Use this when Agent A's work unblocks Agent B's work and both can proceed independently.

### Steps

1. **Agent A completes and merges** their PR through the normal review process.

2. **Agent A creates a follow-up issue** with the label `<domain>-followup` (e.g., `perception-followup`, `nav-followup`, `comms-followup`). The issue must contain:
   - **API contract:** The exact IPC struct definition, topic name, and message semantics (publish rate, units, coordinate frame, validity conditions).
   - **Acceptance criteria:** What "done" looks like for the receiving domain.
   - **Link to the merged PR:** So Agent B can see the implementation context.
   - **Example usage:** A code snippet showing how to subscribe and interpret the new data.

3. **Agent B picks up the follow-up issue** as a normal work item. Agent B should read the linked PR to understand the full context before starting.

### Example

Agent A (perception) adds a new `RadarTrack` IPC struct and publishes it on `/radar_tracks`:

```
Issue #230: nav-followup: consume /radar_tracks in P3 SLAM fusion
- Struct: RadarTrack (see common/ipc/shm_types.h, PR #228)
- Topic: /radar_tracks, published at 20 Hz
- Semantics: body-frame range/bearing/velocity, NaN = invalid
- Acceptance: P3 fuses radar tracks into pose estimate, regression test added
- Context: PR #228 (merged)
```

## Emergency Cross-Domain: Tech-Lead Coordinated

Use this when two domains must change simultaneously and cannot be sequenced (e.g., an IPC struct change that breaks both publisher and subscriber).

### Steps

1. **Tech lead creates a single coordination issue** with sub-tasks for each domain.
2. Both agents work on the same integration branch (see below).
3. Both agents update `tasks/active-work.md` with their sub-task status.
4. Tech lead merges the integration branch to main once all sub-tasks pass CI.

## Integration Branch Pattern

For multi-issue epics that span domains, use an integration branch between feature worktrees and main. This keeps main demo-ready at all times.

### How It Works

```
main (always demo-ready)
  |
  +-- integration/epic-XXX (collects all sub-issue PRs)
        |
        +-- feature/issue-AAA (Agent A's worktree)
        +-- feature/issue-BBB (Agent B's worktree)
        +-- feature/issue-CCC (Agent C's worktree)
```

1. Create the integration branch from main: `git checkout -b integration/epic-XXX main`
2. Each agent branches from the integration branch, not from main.
3. Sub-issue PRs target the integration branch (not main).
4. Once all sub-issues are merged to the integration branch and CI passes, create a single PR from the integration branch to main.
5. Delete the integration branch after merging to main.

### Rules

- The integration branch must pass CI at all times. Do not merge broken sub-issue PRs into it.
- Agents rebase their feature branches on the integration branch regularly to pick up other agents' changes.
- Only the tech lead (or designated coordinator) merges the integration branch to main.

### Project Examples

- **Epic #263 (Autonomous Intelligence):** Used an integration branch with 3 waves of sub-issues across perception, mission planning, and comms domains. 8 agents worked concurrently in separate worktrees, all targeting `integration/epic-263-autonomous-intelligence`.

- **Radar Fusion (Issues #210, #229, #237):** Required coordinated changes across perception (radar HAL, UKF fusion) and navigation (pose estimation). The radar ground filter fix in the HAL affected both P2 perception and P3 navigation consumers.

## Checklist for Handoff Issues

Every handoff issue must include:

- [ ] Domain label (`perception-followup`, `nav-followup`, `comms-followup`, `mission-followup`, `monitor-followup`)
- [ ] IPC contract (struct name, topic, rate, semantics, coordinate frame)
- [ ] Acceptance criteria (measurable, testable)
- [ ] Link to originating PR
- [ ] Any config keys the receiving domain needs to read
- [ ] Test expectations (what tests should the receiving agent add)
