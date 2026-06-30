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
(can skip this)
```bash
cd external/ur5e-ws/docker
docker compose build
cd ../../..
```

3. Build the ROS2 workspace. On a robot laptop where the upstream compose stack works:


```bash
xhost +local:docker
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash
```


```bash
cd /home/user/ur5e-ws
colcon build --symlink-install
source install/setup.bash
```


On a camera-only laptop without NVIDIA runtime, use the host-side wrapper shell instead. This opens the same image without requiring the compose file's NVIDIA device reservation:
(can skip this)
```bash
scripts/run_realsense_rgbd.sh --shell
colcon build --symlink-install
source install/setup.bash
exit
```

4. Start the RGBD camera from the repo root on the host laptop:
(can skip this)
```bash
scripts/run_realsense_rgbd.sh --enumerate
scripts/run_realsense_rgbd.sh
```

To launch the same camera-only setup with the upstream RViz view:
(can skip this)
```bash
xhost +local:docker
scripts/run_realsense_rgbd.sh --rviz
```

To match the upstream script's calibration branch without starting easy-handeye:
(can skip this)
```bash
scripts/run_realsense_rgbd.sh --calibration
```

Use this command whenever the goal is camera-only RGBD bringup, including on a full robot laptop with GPU and NVIDIA Container Toolkit installed. Do not run `external/ur5e-ws/scripts/realsense_bringup.sh` for this camera-only path, because that upstream script launches `realsense_launch/realsense.launch.py`, which also starts the easy-handeye publisher and expects `eye_on_base_calibration.calib`.

`scripts/run_realsense_rgbd.sh` directly launches `realsense2_camera rs_launch.py`. It publishes RGBD topics such as `/camera/camera/rgbd`, disables pointcloud output, and does not start `easy_handeye2`; therefore `eye_on_base_calibration.calib` is not required.

* If you are already inside a prepared `ur5e-ws` container and want the same camera-only behavior manually, run: (notice: plug in the realsense camera first and start the container, or "cannot find camera" error will occur)

```bash
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

```
# for pi05
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  serial_no:=_317622074945 \
  enable_rgbd:=true \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true \
  enable_gyro:=false \
  enable_accel:=false \
  align_depth.enable:=true \
  pointcloud.enable:=false \
  depth_module.depth_profile:=480x270x60 \
  rgb_camera.color_profile:=424x240x60

# 手腕相機（wrist）— D435i，Serial 348122071811

# 新開一個 container shell：

docker compose exec ur5e-ws bash

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=wrist_camera \
  serial_no:=_348122071811 \
  enable_color:=true \
  enable_depth:=false \
  enable_sync:=false \
  enable_gyro:=false \
  enable_accel:=false \
  pointcloud.enable:=false \
  rgb_camera.color_profile:=424x240x60
```

For RViz from inside the container, keep the camera launch running and open another container shell:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
rviz2 -d "$(ros2 pkg prefix realsense_launch)/share/realsense_launch/rviz/realsense.rviz"
```

This uses the same RViz config as upstream. The only removed startup component is the easy-handeye publisher.

5. Start the UR5e/grip driver & moveit in a separate container shell:

```bash

# Terminal 1

export LIBGL_ALWAYS_SOFTWARE=1
# ur5 driver
./scripts/ur_driver_bringup.sh

# Terminal 2
export LIBGL_ALWAYS_SOFTWARE=1
# grip driver
ros2 launch onrobot_2fg_driver onrobot_2fg_driver.launch.py

# Terminal 3
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ur_moveit_config ur_moveit.launch.py launch_rviz:=true ur_type:=ur5e
```

If compose cannot run on a no-GPU laptop, open an equivalent shell with `scripts/run_realsense_rgbd.sh --shell` from the repo root and run the same commands from `/home/user/ur5e-ws`.

5.5. To record the data. First run the docker container and record the data to hdf5 file (notice: the data should be transformed to Lerobot format for training)

```bash
xhost +local:docker
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash

python3 /home/user/LEAH-real-robot-demo/scripts/collect_hdf5.py \
  --output-dir /home/user/LEAH-real-robot-demo/data/hdf5 \
  --hz 10 \
  --seconds 30 \
  --prompt "test2"
```
  To see wether the data is usable, you can run this command to show the actions

```bash
python3 - << 'EOF'
import h5py, numpy as np

path = "/home/user/LEAH-real-robot-demo/data/hdf5/episode_20260630_175931.hdf5"

with h5py.File(path, "r") as f:
    prompt=f.attrs["prompt"]; n=int(f.attrs["n_frames"]); hz=float(f.attrs["record_hz"])
    top=f["observations/images/top"][:]; wrist=f["observations/images/wrist"][:]
    states=f["observations/state"][:]; acts=f["actions"][:]; ts=f["timestamps"][:]

print(f"prompt : {prompt}")
print(f"frames : {n}  ({n/hz:.1f}s @ {hz}Hz)")
print(f"\nShapes:")
print(f"  top    {top.shape} {top.dtype}")
print(f"  wrist  {wrist.shape} {wrist.dtype}")
print(f"  state  {states.shape}  [x,y,z,qx,qy,qz,qw,grip]")
print(f"  action {acts.shape}  [dx,dy,dz,dRx,dRy,dRz,grip]")
print(f"\nState stats:")
for i,lb in enumerate(["x","y","z","qx","qy","qz","qw","grip"]):
    c=states[:,i]; print(f"  {lb:4s}: [{c.min():.4f}, {c.max():.4f}]  mean={c.mean():.4f}  std={c.std():.5f}")
print(f"\nAction stats:")
for i,lb in enumerate(["dx","dy","dz","dRx","dRy","dRz","grip"]):
    c=acts[:,i]; print(f"  {lb:4s}: [{c.min():.6f}, {c.max():.6f}]  std={c.std():.6f}")
dts=np.diff(ts)
print(f"\nTiming: avg={dts.mean()*1000:.1f}ms  std={dts.std()*1000:.1f}ms  (target={1000/hz:.0f}ms)")
grip=states[:,7]
print(f"\nGripper: range=[{grip.min():.3f}, {grip.max():.3f}]  steps with |Δ|>0.01: {(np.abs(np.diff(grip))>0.01).sum()}")
print(f"Images:  top mean={top.mean():.1f} std={top.std():.1f} | wrist mean={wrist.mean():.1f} std={wrist.std():.1f}")
EOF
```

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

## Manual Container Workflow

Complete step-by-step commands for a person sitting at the laptop. Everything below assumes the repo is already cloned with submodules and the Docker image `tars3017cbc/ur5e-ws:latest` is present locally.

### Enter the container

**Path A — full robot laptop (NVIDIA GPU + NVIDIA Container Toolkit):**

```bash
xhost +local:docker
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash
# lands at /home/user/ur5e-ws inside the container
```

**Path B — camera-only or no NVIDIA Container Toolkit:**

```bash
# run from repo root
scripts/run_realsense_rgbd.sh --shell
# lands at /home/user/ur5e-ws inside the container
```

Both paths mount `external/ur5e-ws` as `/home/user/ur5e-ws` and share the host network (`--network host`). The container `.bashrc` auto-sources ROS2 and `install/setup.bash` on every interactive shell open, so you do not need to source them manually after the first build.

### Build the workspace (first time only)

Inside the container:

```bash
cd /home/user/ur5e-ws
colcon build --symlink-install
```

Full build of all 24 packages takes about 40 seconds on this hardware. Deprecation warnings from `ur_controllers` and `ur_robot_driver` are expected and harmless. After the build, source the workspace for the current shell (subsequent shells source it automatically via `.bashrc`):

```bash
source install/setup.bash
```

### Open additional container shells

Each component below runs in its own terminal tab. Open more shells into the same container:

```bash
# Path A:
docker compose exec ur5e-ws bash

# Path B:
scripts/run_realsense_rgbd.sh --shell
```

### Terminal 1 — RealSense RGBD camera (run from HOST, repo root)

Verify device is visible before starting:

```bash
scripts/run_realsense_rgbd.sh --enumerate
```

Start the RGBD node:

```bash
scripts/run_realsense_rgbd.sh
```

This directly launches `realsense2_camera rs_launch.py` (depth 480×270 Z16 @ 60 fps, color 424×240 RGB8 @ 60 fps) and does **not** start the easy-handeye publisher. No `eye_on_base_calibration.calib` file is required. Published topics include:

```
/camera/camera/rgbd
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/depth/image_rect_raw
```

To view camera output in RViz (requires `xhost +local:docker` first):

```bash
scripts/run_realsense_rgbd.sh --rviz
```

### Terminal 2 — UR5e robot driver (inside container)

Prerequisite: on the UR5e pendant, go to **Program → URCaps → External Control** and press Play.

```bash
cd /home/user/ur5e-ws
./scripts/ur_driver_bringup.sh
```

Expands to:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.56.101 \
    enable_rg2_gripper:=false \
    launch_rviz:=true
```

### Terminal 3 — OnRobot 2FG gripper driver (inside container)

```bash
ros2 launch onrobot_2fg_driver onrobot_2fg_driver.launch.py
```

### Terminal 4 — MoveIt2 Servo (inside container)

```bash
ros2 launch ur_servo_control servo.launch.py
```

### Terminal 5 — Pose tracking node (inside container)

```bash
ros2 launch ur_pose_tracking ur_pose_tracking.launch.py
```

### Verify all required topics

From any container shell, once all five terminals are running:

```bash
ros2 topic list
```

Required topics for the bridge to function:

```
/target_pose                   # consumed by ur_pose_tracking
/servo_node/delta_twist_cmds   # servo twist command to UR driver
/camera/camera/color/image_raw # RealSense RGB image
/gripper/command               # OnRobot 2FG width command
```

Optional feedback topics:

```
/gripper/width
/gripper/grip_detected
```

### Record/Replay pipeline (optional, inside container)

Keep the RGBD camera (Terminal 1) and UR driver (Terminal 2) terminals running, then:

```bash
cd /home/user/ur5e-ws
./scripts/record_replay.sh /home/user/ur5e-ws/data 180.0
```

The script internally starts `ur_servo_control` and `ur_pose_tracking` via `tmux`. Do not start Terminals 4 and 5 separately if you use this script.

### One-time UR calibration extraction (new robot only)

Run once when first connecting to a robot whose kinematics have not been extracted before:

```bash
cd /home/user/ur5e-ws
./scripts/extract_ur_calibration.sh
```

Writes to `src/Universal_Robots_ROS2_Description/config/ur5/default_kinematics.yaml`.

## Safety Policy

The first hardware milestone is fixed-K model validation, not adaptive-method comparison. Run the demo gates in order:

1. Hardware bringup and E-stop check.
2. Scripted small-motion coordinate-frame check.
3. Policy dry-run without robot motion.
4. Conservative fixed-K closed-loop execution.
5. LEAH/CORA adaptive execution comparison.

If the trained checkpoint was not trained with this exact real-robot observation/action convention, do not run closed-loop execution until the action conversion is verified.



