#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="${ROOT_DIR}/external/ur5e-ws"
IMAGE="${LEAH_UR5E_IMAGE:-tars3017cbc/ur5e-ws:latest}"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker was not found on PATH." >&2
    exit 2
fi

if [ ! -d "${WORKSPACE_DIR}" ]; then
    echo "ERROR: missing workspace directory: ${WORKSPACE_DIR}" >&2
    exit 2
fi

TTY_ARGS=()
if [ -t 0 ] && [ -t 1 ]; then
    TTY_ARGS=(-it)
fi

X11_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
    X11_ARGS+=(-e "DISPLAY=${DISPLAY}")
    if [ -d /tmp/.X11-unix ]; then
        X11_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
    fi
    XAUTH_FILE="${XAUTHORITY:-${HOME}/.Xauthority}"
    if [ -f "${XAUTH_FILE}" ]; then
        X11_ARGS+=(-e XAUTHORITY=/root/.Xauthority -v "${XAUTH_FILE}":/root/.Xauthority:ro)
    fi
fi

docker run --rm "${TTY_ARGS[@]}" \
    --user root \
    --privileged \
    --network host \
    -e ROS_LOCALHOST_ONLY=1 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RCUTILS_COLORIZED_OUTPUT=1 \
    -e ROS2_WS=/home/user/ur5e-ws \
    "${X11_ARGS[@]}" \
    -v /dev:/dev \
    -v "${WORKSPACE_DIR}":/home/user/ur5e-ws \
    -w /home/user/ur5e-ws \
    "${IMAGE}" \
    bash -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
if [ "${1:-}" = "--shell" ]; then
    [ -r install/setup.bash ] && source install/setup.bash
    exec bash
fi

if [ ! -r install/setup.bash ]; then
    echo "ERROR: /home/user/ur5e-ws/install/setup.bash is missing." >&2
    echo "Run this first inside the container: colcon build --symlink-install" >&2
    exit 2
fi
source install/setup.bash

run_camera() {
    ros2 launch realsense2_camera rs_launch.py \
        config_file:="'"'"''"'"'" \
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
        hole_filling_filter.enable:=true \
        "$@"
}

run_camera_calibration() {
    ros2 launch realsense2_camera rs_launch.py \
        config_file:="'"'"''"'"'" \
        initial_reset:=false \
        enable_rgbd:=false \
        enable_color:=true \
        enable_depth:=false \
        enable_sync:=false \
        enable_gyro:=false \
        enable_accel:=false \
        align_depth.enable:=false \
        unite_imu_method:=0 \
        pointcloud.enable:=false \
        depth_module.depth_profile:=480x270x60 \
        depth_module.infra_profile:=480x270x60 \
        depth_module.depth_format:=Z16 \
        rgb_camera.color_profile:=1920x1080x8 \
        clip_distance:=8.0 \
        hole_filling_filter.enable:=true \
        "$@"
}

case "${1:-}" in
    --enumerate)
        rs-enumerate-devices
        exit 0
        ;;
    --calibration)
        shift
        run_camera_calibration "$@"
        exit $?
        ;;
    --rviz)
        shift
        rviz_config="$(ros2 pkg prefix realsense_launch)/share/realsense_launch/rviz/realsense.rviz"
        run_camera "$@" &
        camera_pid=$!
        trap '"'"'kill "${camera_pid}" 2>/dev/null || true'"'"' EXIT
        sleep 2
        rviz2 -d "${rviz_config}"
        exit $?
        ;;
esac

run_camera "$@"
' bash "$@"
