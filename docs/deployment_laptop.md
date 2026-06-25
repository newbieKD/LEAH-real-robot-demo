# Laptop Deployment Notes

The physical robot laptop is the robot-side controller. It should run ROS2, RealSense, UR driver, gripper driver, and the LEAH bridge locally.

## Required Local Setup

- Ubuntu 22.04 laptop with Docker and NVIDIA Container Toolkit if using the upstream workspace as-is.
- Wired Ethernet to UR5e on `192.168.56.0/24`.
- Wi-Fi/VPN route to the lab GPU server.
- `git clone --recurse-submodules` of this repo.

## UR5e Workspace

Use the submodule:

```bash
cd external/ur5e-ws/docker
xhost +local:docker
docker compose up -d
docker compose exec ur5e-ws bash
```

Inside the container:

```bash
cd /home/user/ur5e-ws
colcon build --symlink-install
source install/setup.bash
./scripts/ur_driver_bringup.sh
ros2 launch onrobot_2fg_driver onrobot_2fg_driver.launch.py
ros2 launch ur_servo_control servo.launch.py
ros2 launch ur_pose_tracking ur_pose_tracking.launch.py
```

Start the RealSense RGBD camera from the repo root on the host laptop:

```bash
scripts/run_realsense_rgbd.sh
```

This wrapper launches only the RealSense RGBD node and does not start the easy-handeye publisher, so it does not require an `eye_on_base_calibration.calib` file. To check device visibility before launching ROS:

```bash
scripts/run_realsense_rgbd.sh --enumerate
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

The RViz config and camera launch parameters match the upstream path. The only removed startup component is the easy-handeye publisher.

## Policy Server Tunnel

Start the policy server on the lab GPU server. On the laptop:

```bash
scripts/start_policy_tunnel.sh user@our-gpu-server
```

The bridge should connect to `127.0.0.1:8000`; SSH forwards that to the remote server.

## Route Pitfall

The UR5e wired profile should not set a gateway. If it sets a gateway, it can steal the laptop default route and break the SSH tunnel to the GPU server.
