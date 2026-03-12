# CLAUDE.md — Claude Code Instructions for data_collection

## Workflow — MANDATORY

Every task follows three phases. **Never skip a phase or advance without explicit user approval.**

### Phase 1 — RESEARCH (read-only)

- If a Linear issue ID is provided, read it fully via MCP: title, description, acceptance criteria, comments, linked issues.
- Explore the codebase. Do NOT write or edit any file.
- Understand the affected packages, topics, configs, and dependencies.
- Identify every file that will need to change.
- If acceptance criteria are missing from the issue — ask before proceeding.
- Surface any other ambiguities before assuming.
- **Output**: write a short summary of findings inline (or to `thoughts/research/<task>.md` for complex tasks).

### Phase 2 — PLAN

- Translate findings into a numbered, step-by-step implementation plan.
- Each step should be atomic (single file, single concern).
- List the files that will change and what each change accomplishes.
- Include test/verify criteria for the task.
- **Wait for the user to say "APPROVED" (or equivalent) before writing any code.**

### Phase 3 — EXECUTE

- Create a branch from `main` using the format `<issue-id>/<short-slug>` (e.g. `ENG-42/add-digit-retry`).
- Move the Linear issue to **In Progress**.
- Implement one step at a time, exactly as approved.
- Run tests / verify after each step.
- Commit after each atomic step using conventional commits (see Git Conventions below).
- If a step reveals new information that changes the plan — **stop and surface it**. Do not self-authorize scope changes.
- When implementation is complete and tests pass: move the Linear issue to **In Review**, post one summary comment on the issue, and open a PR targeting `main`.
- Do not merge the PR and do not mark the issue Done — that is the human's responsibility.

---

## Project Overview

ROS2 Jazzy workspace for collecting synchronized multi-modal sensor data from a robotic manipulation setup. See `AGENTS.md` for full architecture reference.

## Key Conventions

- **ROS2 version**: Jazzy
- **Python packages**: use `uv` (see `pyproject.toml` and `uv.lock`)
- **Recording format**: MCAP via rosbag2
- **Build**: `colcon build --symlink-install` from workspace root
- **Source workspace**: `. install/setup.bash` (alias: `r2` / `setupr2`)

## Package Layout

All custom packages live under `src/my_code/`. Do not touch packages outside `my_code/` unless explicitly asked.


| Package                   | Purpose                                                           |
| ------------------------- | ----------------------------------------------------------------- |
| `data_collection_bringup` | Launch files + YAML configs — start here for system-level changes |
| `digit_pub`               | DIGIT tactile sensor publisher                                    |
| `natnet_pub`              | OptiTrack motion capture publisher                                |
| `kinova_state_pub`        | Kinova EE pose via TF                                             |
| `dlo_inference`           | Optional DLO inference from DIGIT + pose topics                   |
| `my_msgs`                 | Custom ROS2 message definitions                                   |
| `robotiq_2f_85_driver`    | Robotiq gripper driver                                            |
| `full_data_pub`           | **DEPRECATED** — do not modify                                    |


## Config Files

- Main node params: `src/my_code/data_collection_bringup/config/data_collection_bringup.yaml`
- DIGIT-specific: `src/my_code/data_collection_bringup/config/digit_config.yaml`
- Recording uses throttled raw topics from `digit_pub`, `natnet_pub`, and `kinova_state_pub` via `record.launch.py`

## Style Guidelines

- Follow existing node patterns in `digit_pub_node.py` / `natnet_pub_node.py`
- QoS: live data = `BEST_EFFORT/VOLATILE`; reference images = `RELIABLE/TRANSIENT_LOCAL`
- Keep solutions minimal — no abstractions for one-time operations

## Git Conventions

**Branches:**

- Always branch from `main`: `git checkout -b <issue-id>/<short-slug> main`
- One branch per Linear issue. Never commit directly to `main`.
- For dependent issues (B requires A): branch from A's branch, not main. Stack the PRs.
- Short slug: lowercase, hyphens only, max 5 words (e.g. `add-digit-retry`, `fix-natnet-ip`)

**Commits** (conventional commits format):

```
<type>(<scope>): <short description>

# Types: feat | fix | refactor | test | chore | docs
# Scope: package name or component (e.g. digit_pub, natnet, bringup)

# Examples:
feat(digit_pub): add retry logic on USB connection loss
fix(natnet): correct server IP default in config
chore(bringup): update record.launch.py throttle rate to 15 Hz
```

**PRs:**

- Title: mirrors the Linear issue title
- Body: what changed, why, how to test, link to Linear issue
- Target branch: always `main`
- Do not merge — the human reviews and merges

---

## Linear Integration

The project uses Linear for task management. The MCP server is available in this session.

linear ID: ROTEM_PHD
Linear project: Neural Jacobian fields





**Status flow**: Todo → In Progress → In Review → Done

**Rules**:

- Move issue to **In Progress** when you begin work (start of Execute phase).
- Move issue to **In Review** when tests pass and a PR is opened.
- **Never move to Done or Closed** — only a merged PR or explicit user instruction does that.
- **Never create new issues** without explicit user instruction.
- **Post at most one comment per issue** — a summary when the PR is opened.
- Always use the issue ID in the branch name: `<issue-id>/<short-slug>`.
- If no acceptance criteria exist on the issue, ask the user before starting the plan.

## What NOT to Do

- Do not modify `full_data_pub` — deprecated
- Do not commit large binaries or bag files
- Do not push to remote without explicit user confirmation
- Do not make changes outside the approved plan without surfacing them first
- Do not close, cancel, or mark Done any Linear issue
- Do not create Linear issues without explicit user instruction
- Do not post multiple comments on a Linear issue during implementation

