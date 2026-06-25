# Agent Development Rules

This repo is for real-robot deployment. Keep hardware integration conservative, explicit, and easy to audit.

## Submodule Policy

`external/ur5e-ws` is a vendor submodule for the UR5e ROS2 hardware workspace.

Do not edit files inside `external/ur5e-ws` during normal development. Treat it as read-only upstream code. If behavior must change, prefer one of these approaches:

1. Add wrapper scripts in this repo's `scripts/` directory.
2. Add runtime config in this repo's `configs/` directory.
3. Add a new ROS2 package or adapter outside `external/`.
4. Document required upstream changes in `docs/` before patching the submodule.

Only update the submodule pointer intentionally:

```bash
git submodule update --remote external/ur5e-ws
git add external/ur5e-ws
git commit -m "Update ur5e-ws submodule"
```

If a local experiment requires editing upstream code, create a separate branch or fork of `ur5e-ws`; do not leave uncommitted edits inside the submodule.

## Where New Code Goes

Use this repo as the integration layer:

- `scripts/`: executable bridge, preflight, tunnel, and demo orchestration scripts.
- `configs/`: robot topics, frames, action scaling, workspace limits, policy endpoint settings.
- `docs/`: deployment notes, safety checklists, experiment logs, troubleshooting.
- `packages/`: optional new ROS2/Python packages if scripts become too large.

Do not put new LEAH/OpenPI/CORA bridge logic into `external/ur5e-ws` unless there is no clean adapter alternative.

## Reuse Existing Hardware Interfaces

Prefer using the interfaces already provided by `ur5e-ws`:

- `/target_pose` for pose targets consumed by `ur_pose_tracking`.
- `/servo_node/delta_twist_cmds` for low-level servo commands through the upstream stack.
- `/gripper/command`, `/gripper/width`, and `/gripper/grip_detected` for OnRobot 2FG control/state.
- RealSense topics under `/camera/camera/...`.
- TF frames such as `world` and `rg2_base_link`.

If a topic/frame differs on the real laptop, update `configs/ur5e_demo.yaml` or create a local config file ignored by Git.

## Safety Requirements

All execution code must preserve these invariants:

- Start with `--dry-run`; never make physical execution the default.
- Require an explicit `--execute` flag for robot motion.
- Clamp translation, rotation, and workspace bounds before publishing commands.
- Stop or hold on missing observation, policy timeout, TF failure, or unexpected action shape.
- Keep ROS2 hardware control local to the robot laptop; do not publish ROS2 robot commands from the remote GPU server.

The remote GPU server should only run the policy server. The robot laptop owns sensing, action validation, and command publication.

## Development Workflow

Before committing:

```bash
python3 -m py_compile scripts/*.py
bash -n scripts/*.sh
git status --short
git submodule status
```

Do not commit generated data, checkpoints, videos, rosbags, or local credentials. Use ignored directories such as `data/`, `logs/`, `videos/`, `recordings/`, `rosbags/`, `checkpoints/`, and `test/` for local artifacts.

## Commit Hygiene

Keep commits focused:

- One commit for documentation/process changes.
- One commit for bridge behavior changes.
- One commit for config/safety limit changes.
- One commit for intentional submodule pointer updates.

When changing anything that can move the robot, mention the safety effect in the commit message or PR description.
