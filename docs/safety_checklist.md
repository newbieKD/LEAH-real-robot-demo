# Real-Robot Safety Checklist

Run this checklist before every physical execution.

## Human/Workspace

- E-stop is reachable.
- Operator is beside the robot.
- Workspace is clear of unnecessary objects.
- First run uses an empty or low-risk scene.

## Network/ROS

- Laptop can ping `192.168.56.101`.
- `scripts/run_preflight.sh` passes.
- RealSense image topic is live.
- TF from `world` to `rg2_base_link` is live.
- Gripper command topic works.

## Policy

- Policy server is reachable through SSH tunnel.
- `--dry-run` action magnitudes are inspected.
- Translation and rotation clamps are conservative.
- Gripper sign convention is verified.

## Execution

- Start with fixed-K.
- Stop immediately if action direction is wrong.
- Do not run adaptive comparison until fixed-K can complete a basic task safely.
