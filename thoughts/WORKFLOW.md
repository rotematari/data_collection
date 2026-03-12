# Workflow Reference — data_collection

How to work with Claude Code + Linear + Git on this project.

---

## The Big Picture

Every task follows three locked phases. Claude cannot skip or self-advance between them.

```
Linear issue (Todo)
       │
       ▼
  1. RESEARCH          ← Claude reads issue + codebase. No code written.
       │ you say "APPROVED" (or reject/edit the plan)
       ▼
  2. PLAN              ← Claude writes a numbered plan. Waits for your go-ahead.
       │ you say "APPROVED"
       ▼
  3. EXECUTE           ← Claude codes, commits, verifies. Opens PR when done.
       │
       ▼
Linear issue (In Review) + PR open
       │ you review + merge PR
       ▼
Linear issue (Done)
```

---

## Your Responsibilities (what Claude will NOT do)

| Action | Who does it |
|---|---|
| Create a Linear issue | You |
| Write acceptance criteria | You (required — Claude blocks without them) |
| Approve the plan | You |
| Review + merge the PR | You |
| Mark issue Done / Closed | You (or auto on PR merge if configured) |

---

## Setting Up a Task — Checklist

Before handing an issue to Claude, make sure the Linear issue has:

- [ ] **Clear title** — action verb + noun (e.g. "Add retry logic to DIGIT publisher")
- [ ] **Description** — problem, current behavior, desired behavior (3–5 sentences)
- [ ] **Acceptance criteria** — numbered checklist, each item objectively verifiable
- [ ] **Scope note** — what is NOT in scope (prevents scope creep)
- [ ] **Label** — type (bug / feature / chore) + component

Example acceptance criteria:
```
1. DIGIT publisher reconnects automatically if USB is unplugged and replugged.
2. Reconnect is attempted max 3 times with 2s delay between attempts.
3. A ROS warning is logged on each failed attempt.
4. A ROS error is logged if all retries fail, then node shuts down cleanly.
```

---

## Starting a Task with Claude

Just give Claude the issue ID:

> "Work on ENG-42"

Claude will:
1. Read the issue via Linear MCP
2. Explore the relevant codebase
3. Present a research summary + implementation plan
4. Wait for your `APPROVED` before touching any file

---

## Git Flow

```
main
 └── ENG-42/add-digit-retry       ← one branch per issue
      ├── feat(digit_pub): add retry loop         ← atomic commit
      ├── test(digit_pub): add reconnect tests    ← atomic commit
      └── → PR → you review → merge → Done
```

**Branch naming**: `<linear-issue-id>/<short-slug>`
- lowercase, hyphens, max 5 words
- e.g. `ENG-42/add-digit-retry`, `ENG-17/fix-natnet-ip`

**Commit format** (conventional commits):
```
feat(digit_pub): add retry logic on USB connection loss
fix(natnet): correct server IP default in config
chore(bringup): update throttle rate to 15 Hz
```
Types: `feat` | `fix` | `refactor` | `test` | `chore` | `docs`

**Dependent issues** (B requires A to be done first):
- Branch B off A's branch (not main)
- Merge A first, then rebase B onto main and merge

---

## Linear Status Flow

```
Todo → In Progress → In Review → Done
        ↑                ↑         ↑
    Claude sets     Claude sets  YOU set
    (start Execute) (PR opened)  (PR merged)
```

Claude will **never**:
- Mark an issue Done or Closed
- Create new issues without your instruction
- Post more than one comment per issue (summary at PR open)

---

## PR Format

Claude opens PRs with this structure:

```
Title:  [ENG-42] Add retry logic to DIGIT publisher

Body:
## What
- Added reconnect loop in digit_pub_node.py
- Max 3 retries, 2s delay, clean shutdown on failure

## Why
Fixes issue where node crashes on USB unplug (ENG-42)

## How to test
1. Run digit_pub_node
2. Unplug a DIGIT sensor
3. Confirm warning logs appear and node reconnects on replug

Linear: ENG-42
```

---

## When Claude Gets Stuck

If mid-execution Claude discovers that the plan needs to change (e.g. a dependency is missing, the codebase works differently than expected), it will **stop and tell you** rather than improvise. You'll see something like:

> "Step 3 revealed that X. This changes the plan — should I revise it or proceed differently?"

Say what you want and Claude continues.

---

## Filling in Linear IDs (one-time setup)

Open `CLAUDE.md` and fill in the placeholder comments near the Linear Integration section:

```
<!-- Team ID: <YOUR_LINEAR_TEAM_ID> -->
<!-- Default project ID: <YOUR_LINEAR_PROJECT_ID> -->
```

Get these from the Linear URL: `linear.app/<team-slug>/settings` → copy the IDs from the URL or API settings page. This prevents Claude from accidentally writing to the wrong workspace.

---

## Quick Reference Card

| Situation | What to do |
|---|---|
| New task | Create Linear issue with acceptance criteria, give Claude the ID |
| Claude presents plan | Read it, say `APPROVED` or give feedback |
| Claude opens PR | Review code + test, merge, Linear auto-updates or you mark Done |
| Claude says plan changed mid-task | Read the update, decide, tell Claude to continue or revise |
| Something broken | Check the branch — every step is a commit, easy to bisect |
| Want to cancel mid-task | Tell Claude to stop, delete the branch, reset Linear to Todo |
