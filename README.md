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

2. Build the upstream Docker image:

```bash
cd external/ur5e-ws/docker
docker compose build
cd ../../..
```

3. Build the ROS2 workspace. On a robot laptop where the upstream compose stack works:

```bash
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash
cd /home/user/ur5e-ws
colcon build --symlink-install
source install/setup.bash
```

On a camera-only laptop without NVIDIA runtime, use the host-side wrapper shell instead. This opens the same image without requiring the compose file's NVIDIA device reservation:

```bash
scripts/run_realsense_rgbd.sh --shell
colcon build --symlink-install
source install/setup.bash
exit
```

4. Start the RGBD camera from the repo root on the host laptop:

```bash
scripts/run_realsense_rgbd.sh --enumerate
scripts/run_realsense_rgbd.sh
```

To launch the same camera-only setup with the upstream RViz view:

```bash
xhost +local:docker
scripts/run_realsense_rgbd.sh --rviz
```

To match the upstream script's calibration branch without starting easy-handeye:

```bash
scripts/run_realsense_rgbd.sh --calibration
```

Use this command whenever the goal is camera-only RGBD bringup, including on a full robot laptop with GPU and NVIDIA Container Toolkit installed. Do not run `external/ur5e-ws/scripts/realsense_bringup.sh` for this camera-only path, because that upstream script launches `realsense_launch/realsense.launch.py`, which also starts the easy-handeye publisher and expects `eye_on_base_calibration.calib`.

`scripts/run_realsense_rgbd.sh` directly launches `realsense2_camera rs_launch.py`. It publishes RGBD topics such as `/camera/camera/rgbd`, disables pointcloud output, and does not start `easy_handeye2`; therefore `eye_on_base_calibration.calib` is not required.

If you are already inside a prepared `ur5e-ws` container and want the same camera-only behavior manually, run:

```bash
cd /home/user/ur5e-ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  config_file:="''" \
  initial_reset:=false \
  enable_rgbd:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true \
  enable_gyro:=false \
  enable_accel:=false \
  align_depth.enable:=true \
  unite_imu_method:=0 \
  pointcloud.enable:=false \
  depth_module.depth_profile:=480x270x60 \
  depth_module.infra_profile:=480x270x60 \
  depth_module.depth_format:=Z16 \
  rgb_camera.color_profile:=424x240x60 \
  clip_distance:=8.0 \
  hole_filling_filter.enable:=true
```

For RViz from inside the container, keep the camera launch running and open another container shell:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ur5e-ws/install/setup.bash
rviz2 -d "$(ros2 pkg prefix realsense_launch)/share/realsense_launch/rviz/realsense.rviz"
```

This uses the same RViz config as upstream. The only removed startup component is the easy-handeye publisher.

5. Start the UR5e driver in a separate container shell:

```bash
cd external/ur5e-ws/docker
docker compose exec ur5e-ws bash
cd /home/user/ur5e-ws
source install/setup.bash
./scripts/ur_driver_bringup.sh
```

If compose cannot run on a no-GPU laptop, open an equivalent shell with `scripts/run_realsense_rgbd.sh --shell` from the repo root and run the same commands from `/home/user/ur5e-ws`.

6. For the upstream record/replay pipeline, keep the RGBD camera and UR driver terminals running, then open another container shell:

```bash
cd /home/user/ur5e-ws
source install/setup.bash
./scripts/record_replay.sh /home/user/ur5e-ws/data 180.0
```

The record/replay script still starts servo and pose-tracking processes internally with `tmux`. The changed part is only camera bringup: use `scripts/run_realsense_rgbd.sh` instead of `./scripts/realsense_bringup.sh`.

7. On the GPU server, start the OpenPI policy server and bind it to localhost if possible.

8. On the laptop, create an SSH tunnel:

```bash
scripts/start_policy_tunnel.sh user@our-gpu-server
```

9. Run preflight checks:

```bash
scripts/run_preflight.sh
```

10. Start with policy dry-run:

```bash
scripts/run_fixed_k_demo.sh --dry-run --prompt "pick up the block and place it in the bowl"
```

11. Only after action scale/frame checks pass, run physical execution:

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
