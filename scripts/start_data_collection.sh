#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="${ROOT_DIR}/external/ur5e-ws/docker"
SESSION="ur5e_collect"
PROMPT="test2"
RECORD_SECONDS="30"
RECORD_HZ="10"
OUTPUT_DIR="/home/user/LEAH-real-robot-demo/data/hdf5"
ATTACH=1
FORCE=0
RECORD=1

usage() {
  cat <<'EOF'
Usage:
  scripts/start_data_collection.sh [options]

Options:
  --prompt TEXT       Prompt stored in the HDF5 attrs. Default: test2
  --seconds NUM      Recording duration in seconds. Default: 30
  --hz NUM           Recording frequency in Hz. Default: 10
  --output-dir PATH  HDF5 output directory inside the container.
                     Default: /home/user/LEAH-real-robot-demo/data/hdf5
  --session NAME     tmux session name. Default: ur5e_collect
  --bringup-only     Start drivers/cameras only; do not start the recorder.
  --force            Kill an existing tmux session with the same name first.
  --no-attach        Start the session but do not attach to tmux.
  -h, --help         Show this help.

Example:
  scripts/start_data_collection.sh --prompt "pick up the block" --seconds 30 --hz 10
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prompt)
      PROMPT="${2:-}"
      shift 2
      ;;
    --seconds)
      RECORD_SECONDS="${2:-}"
      shift 2
      ;;
    --hz)
      RECORD_HZ="${2:-}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    --session)
      SESSION="${2:-}"
      shift 2
      ;;
    --bringup-only)
      RECORD=0
      shift
      ;;
    --force)
      FORCE=1
      shift
      ;;
    --no-attach)
      ATTACH=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "ERROR: required command not found: $1" >&2
    exit 2
  fi
}

shell_quote() {
  printf "%q" "$1"
}

container_shell() {
  local body="$1"
  cat <<EOF
source /opt/ros/humble/setup.bash
cd /home/user/ur5e-ws
if [ -r install/setup.bash ]; then
  source install/setup.bash
else
  echo "WARN: /home/user/ur5e-ws/install/setup.bash is missing. Run colcon build first."
fi
export LIBGL_ALWAYS_SOFTWARE=1
${body}
status=\$?
echo
echo "[command exited with status \${status}]"
exec bash
EOF
}

tmux_window() {
  local name="$1"
  local body="$2"
  local container_cmd host_cmd

  container_cmd="$(container_shell "$body")"
  host_cmd="cd $(shell_quote "${COMPOSE_DIR}") && docker compose exec ur5e-ws bash -lc $(shell_quote "${container_cmd}")"
  tmux new-window -t "${SESSION}:" -n "${name}" "${host_cmd}"
}

require_cmd docker
require_cmd tmux

if [[ ! -d "${COMPOSE_DIR}" ]]; then
  echo "ERROR: missing compose directory: ${COMPOSE_DIR}" >&2
  echo "Make sure the git submodule is initialized: git submodule update --init --recursive" >&2
  exit 2
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  if [[ "${FORCE}" -eq 1 ]]; then
    tmux kill-session -t "${SESSION}"
  else
    echo "ERROR: tmux session already exists: ${SESSION}" >&2
    echo "Attach with: tmux attach -t ${SESSION}" >&2
    echo "Or restart it with: $0 --force" >&2
    exit 2
  fi
fi

if command -v xhost >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  xhost +local:docker >/dev/null || true
fi

echo "Starting docker compose stack..."
cd "${COMPOSE_DIR}"
docker compose up -d

tmux new-session -d -s "${SESSION}" -n "help" "cat <<'EOF'
UR5e data collection session is starting.

Windows:
  1 ur_driver
  2 gripper
  3 moveit_rviz
  4 top_camera
  5 wrist_camera
  6 recorder

Before recording:
  1. Load the External Control program on the UR5e panel.
  2. Press Play on the UR5e panel.
  3. Check RViz/MoveIt and camera images if needed.

tmux keys:
  Ctrl-b n      next window
  Ctrl-b p      previous window
  Ctrl-b d      detach session

Attach again:
  tmux attach -t ${SESSION}
EOF
exec bash"

tmux_window "ur_driver" "./scripts/ur_driver_bringup.sh"

tmux_window "gripper" "ros2 launch onrobot_2fg_driver onrobot_2fg_driver.launch.py"

tmux_window "moveit_rviz" "ros2 launch ur_moveit_config ur_moveit.launch.py launch_rviz:=true ur_type:=ur5e"

tmux_window "top_camera" "ros2 launch realsense2_camera rs_launch.py \\
  camera_name:=camera \\
  serial_no:=_317622074945 \\
  enable_rgbd:=true \\
  enable_color:=true \\
  enable_depth:=true \\
  enable_sync:=true \\
  enable_gyro:=false \\
  enable_accel:=false \\
  align_depth.enable:=true \\
  pointcloud.enable:=false \\
  depth_module.depth_profile:=480x270x60 \\
  rgb_camera.color_profile:=424x240x60"

tmux_window "wrist_camera" "ros2 launch realsense2_camera rs_launch.py \\
  camera_name:=wrist_camera \\
  serial_no:=_348122071811 \\
  enable_color:=true \\
  enable_depth:=false \\
  enable_sync:=false \\
  enable_gyro:=false \\
  enable_accel:=false \\
  pointcloud.enable:=false \\
  rgb_camera.color_profile:=424x240x60"

if [[ "${RECORD}" -eq 1 ]]; then
  prompt_q="$(shell_quote "${PROMPT}")"
  output_q="$(shell_quote "${OUTPUT_DIR}")"
  hz_q="$(shell_quote "${RECORD_HZ}")"
  seconds_q="$(shell_quote "${RECORD_SECONDS}")"

  tmux_window "recorder" "required_topics=(
  /camera/camera/color/image_raw
  /camera/wrist_camera/color/image_raw
  /joint_states
  /gripper/width
)

echo 'Recorder is waiting for required ROS topics before starting HDF5 collection.'
echo 'Make sure the UR5e External Control program is loaded and Play has been pressed.'

while true; do
  topic_list=\"\$(ros2 topic list 2>/dev/null || true)\"
  missing=()
  for topic in \"\${required_topics[@]}\"; do
    if ! grep -Fxq \"\${topic}\" <<<\"\${topic_list}\"; then
      missing+=(\"\${topic}\")
    fi
  done
  if [ \"\${#missing[@]}\" -eq 0 ]; then
    break
  fi
  echo \"Waiting for topics: \${missing[*]}\"
  sleep 2
done

echo 'All required topics are visible. Starting HDF5 recording...'
python3 /home/user/LEAH-real-robot-demo/scripts/collect_hdf5.py \\
  --output-dir ${output_q} \\
  --hz ${hz_q} \\
  --seconds ${seconds_q} \\
  --prompt ${prompt_q}"
fi

tmux select-window -t "${SESSION}:help"

echo "Started tmux session: ${SESSION}"
echo "Attach with: tmux attach -t ${SESSION}"

if [[ "${ATTACH}" -eq 1 ]]; then
  tmux attach -t "${SESSION}"
fi
