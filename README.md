# LEAH Real Robot Demo

Standalone deployment repo for running LEAH/OpenPI/CORA real-robot demos on a UR5e setup.

This repository intentionally separates real-robot deployment from the main paper codebase. The borrowed-lab laptop should clone this repo, run the UR5e ROS2 hardware workspace locally, and connect to our lab GPU server only for policy inference.

## Repository Layout

```text
LEAH-real-robot-demo/
  external/ur5e-ws/       # submodule: hardware ROS2 workspace
  configs/ur5e_demo.yaml  # robot topics, policy endpoint, safety clamps
  scripts/                # bridge, preflight, tunnel helpers
  docs/                   # deployment and safety notes
```

`external/ur5e-ws` is a git submodule pointing to `https://github.com/tars3017/ur5e-ws`.

## Development Rules

Before adding bridge code or changing hardware behavior, read [AGENTS.md](AGENTS.md). In short: keep `external/ur5e-ws` read-only, add our integration code outside `external/`, and reuse upstream ROS2 interfaces through wrappers/configs.

## Laptop Deployment

Clone with submodules on the physical robot laptop:

```bash
git clone --recurse-submodules https://github.com/newbieKD/LEAH-real-robot-demo.git
cd LEAH-real-robot-demo
```

If the repo was cloned without submodules:

```bash
git submodule update --init --recursive
```

## Network Architecture

```text
UR5e + RealSense + gripper
        |
  wired Ethernet 192.168.56.0/24
        |
borrowed-lab laptop
  - external/ur5e-ws ROS2 hardware stack
  - scripts/openpi_real_bridge.py
        |
  Wi-Fi / VPN / SSH tunnel
        |
our lab GPU server
  - OpenPI policy server
  - trained checkpoint
  - LEAH/CORA adaptive method
```

Do not expose ROS2 directly over the lab network. Keep ROS2 and low-level robot control local to the laptop. The GPU server should only receive observations and return action chunks through the policy websocket endpoint.

## Basic Workflow

1. Configure the laptop wired Ethernet for UR5e:

```text
Laptop IP: 192.168.56.1/24
UR5e IP:  192.168.56.101
Gateway:  empty
```

2. Start the UR5e hardware stack inside `external/ur5e-ws` according to its README.

3. On the GPU server, start the OpenPI policy server and bind it to localhost if possible.

4. On the laptop, create an SSH tunnel:

```bash
scripts/start_policy_tunnel.sh user@our-gpu-server
```

5. Run preflight checks:

```bash
scripts/run_preflight.sh
```

6. Start with policy dry-run:

```bash
scripts/run_fixed_k_demo.sh --dry-run --prompt "pick up the block and place it in the bowl"
```

7. Only after action scale/frame checks pass, run physical execution:

```bash
scripts/run_fixed_k_demo.sh --execute --prompt "pick up the block and place it in the bowl"
```

## Safety Policy

The first hardware milestone is fixed-K model validation, not adaptive-method comparison. Run the demo gates in order:

1. Hardware bringup and E-stop check.
2. Scripted small-motion coordinate-frame check.
3. Policy dry-run without robot motion.
4. Conservative fixed-K closed-loop execution.
5. LEAH/CORA adaptive execution comparison.

If the trained checkpoint was not trained with this exact real-robot observation/action convention, do not run closed-loop execution until the action conversion is verified.
