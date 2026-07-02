# Collecting Data Guideline

這份文件整理 UR5e 搭配兩台 RealSense 相機錄製資料的標準流程。除非特別註明，以下指令都在 robot laptop 上執行。

## 0. 相關檔案說明

這份 guideline 裡的指令會用到以下 repo 檔案與 upstream workspace：

- `Collecting_Data_GUIDELINE.md`: 本文件，整理 UR5e 連線、雙相機啟動、RViz 檢查、HDF5 錄製與檢查流程。
- `external/ur5e-ws/`: upstream ROS2 hardware workspace。進入 Docker container 後，這個 submodule 會被 mount 成 `/home/user/ur5e-ws`。UR5e driver、gripper driver、MoveIt、RealSense launch file 和 RViz config 都主要來自這裡。
- `external/ur5e-ws/docker/docker-compose.yml`: 啟動 `ur5e-ws` Docker container 的 compose 設定。`docker compose up -d` 和 `docker compose exec ur5e-ws bash` 會使用這個檔案。
- `external/ur5e-ws/scripts/ur_driver_bringup.sh`: 啟動 UR5e ROS2 driver 的 upstream helper script，連到 robot IP `192.168.56.101`。
- `scripts/run_realsense_rgbd.sh`: repo 內的 camera-only wrapper。可用來 enumerate RealSense、開 no-NVIDIA shell、跑 calibration mode，或啟動 upstream RealSense RViz view。這份 guideline 的雙相機流程主要是在 container 內手動 launch `realsense2_camera rs_launch.py`。
- `scripts/start_data_collection.sh`: 一鍵啟動 data collection 的主控腳本。它會用 `tmux` 開多個 window，自動進 Docker container，啟動 UR5e driver、gripper driver、MoveIt/RViz、兩台 RealSense camera，並在 recorder window 等必要 ROS topics 出現後開始執行 `collect_hdf5.py`。
- `scripts/collect_hdf5.py`: 主要錄製程式。它會訂閱 `/camera/camera/color/image_raw`、`/camera/wrist_camera/color/image_raw`、`/joint_states` 和 `/gripper/width`，並輸出一個 `episode_*.hdf5`，內容包含 top/wrist images、UR5e joints、normalized gripper、actions、timestamps 和 prompt。
- `scripts/view_hdf5.py`: HDF5 檢查工具。可以只印 statistics，也可以輸出 top/wrist side-by-side preview video 或 individual frames。
- `scripts/convert_to_lerobot.py`: 將錄好的 `episode_*.hdf5` 轉成 LeRobot v2.0 dataset，給 OpenPI/pi0.5 fine-tuning 使用。
- `configs/ur5e_demo.yaml`: policy/demo execution 用的 robot topics、policy endpoint 和 safety limits 設定。單純錄 raw HDF5 data 時不一定會用到，但 bridge/demo scripts 會讀這個 config。

## Quick Start: 一鍵啟動 Data Collection

如果環境已經 build 好、UR5e 網路和兩台相機都接好，而且 robot laptop 有安裝 `tmux`，可以直接在 repo root 執行：

```bash
scripts/start_data_collection.sh \
  --prompt "test2" \
  --seconds 30 \
  --hz 10
```

這個 script 會自動：

1. 執行 `xhost +local:docker`。
2. 進到 `external/ur5e-ws/docker` 並執行 `docker compose up -d`。
3. 建立 `tmux` session：`ur5e_collect`。
4. 分別啟動 UR5e driver、gripper driver、MoveIt/RViz、top camera、wrist camera。
5. 在 recorder window 等待 `/camera/camera/color/image_raw`、`/camera/wrist_camera/color/image_raw`、`/joint_states`、`/gripper/width` 都出現。
6. topics 都出現後，自動執行 `scripts/collect_hdf5.py` 開始錄製。

啟動後仍需要在 UR5e panel 上 load External Control program 並按 Play。建議在開始正式錄製前確認 RViz/MoveIt 可以控制機器人小幅移動，且兩台 camera image 都正常。

常用參數：

```bash
# 指定 prompt、錄製秒數與頻率
scripts/start_data_collection.sh --prompt "pick up the block" --seconds 30 --hz 10

# 如果同名 tmux session 已存在，先重開一個乾淨 session
scripts/start_data_collection.sh --force --prompt "test2"

# 只啟動 UR5e、gripper、MoveIt 和 cameras，不啟動 recorder
scripts/start_data_collection.sh --bringup-only

# 啟動後不自動 attach tmux
scripts/start_data_collection.sh --no-attach
```

`tmux` 常用按鍵：

```text
Ctrl-b n    切到下一個 window
Ctrl-b p    切到上一個 window
Ctrl-b d    detach session
```

detach 後可以用下面指令回到同一個 session：

```bash
tmux attach -t ur5e_collect
```

## 1. 設定 UR5e 連線

先設定 laptop 的有線網路，讓 laptop 可以連到 UR5e：

```text
Laptop IP: 192.168.56.1/24
UR5e IP:  192.168.56.101
Gateway:  empty
```

如果是第一次在這台 laptop 上設定環境，可以先 build upstream Docker image：

```bash
cd external/ur5e-ws/docker
docker compose build
cd ../../..
```

接著啟動 Docker container：

```bash
xhost +local:docker
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash
```

進到 container 後，build ROS2 workspace：

```bash
cd /home/user/ur5e-ws
colcon build --symlink-install
source install/setup.bash
```

之後每次需要開新的 terminal 進 container 時，可以使用：

```bash
xhost +local:docker
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash
```

## 2. 啟動 UR5e、夾爪和 MoveIt

這一段需要三個不同的 host terminal。每個 terminal 都先進入 Docker container：

```bash
xhost +local:docker
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash
```

### Terminal 1: UR5e Driver

在 container 內執行：

```bash
cd /home/user/ur5e-ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
./scripts/ur_driver_bringup.sh
```

### Terminal 2: OnRobot 2FG Gripper Driver

在 container 內執行：

```bash
cd /home/user/ur5e-ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch onrobot_2fg_driver onrobot_2fg_driver.launch.py
```

### Terminal 3: MoveIt + RViz

在 container 內執行：

```bash
cd /home/user/ur5e-ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ur_moveit_config ur_moveit.launch.py launch_rviz:=true ur_type:=ur5e
```

三個 terminal 都啟動後：

1. 到 UR5e teach pendant 上 load External Control program。
2. 在 UR5e panel 上按 Play。
3. 從 UR5e driver terminal 和 RViz 確認連線成功。
4. 在 RViz/MoveIt 中調整機器人目標姿態，執行小幅度移動，確認 UR5e 可以移動到指定位置。

## 3. 啟動兩台 RealSense 相機

UR5e 連線確認成功後，就可以開始架設並連接相機。因為有兩台相機，所以分成兩個不同 terminal 啟動。每個 terminal 一樣先進入 Docker container：

```bash
xhost +local:docker
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash
```

### Terminal 4: Top Camera pi05

在 container 內執行：

```bash
cd /home/user/ur5e-ws
source install/setup.bash

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
```

### Terminal 5: Wrist Camera D435i

在 container 內執行：

```bash
cd /home/user/ur5e-ws
source install/setup.bash

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

確認兩個 camera node 都有起來：

```bash
ros2 node list
```

接著在 RViz 裡檢查影像：

1. 在 RViz 點 Add。
2. 加入 `camera` 的 image display。
3. 加入 `wrist_camera` 的 image display。
4. 確認兩個 image stream 都有正常畫面。

如果兩台相機都能在 RViz 看到即時影像，代表相機設置成功。

## 4. 錄製 HDF5 Data

相機和 UR5e 都確認正常後，開另一個 terminal 進入 Docker container：

```bash
xhost +local:docker
cd external/ur5e-ws/docker
docker compose up -d
docker compose exec ur5e-ws bash
```

在 container 內開始錄製：

```bash
python3 /home/user/LEAH-real-robot-demo/scripts/collect_hdf5.py \
  --output-dir /home/user/LEAH-real-robot-demo/data/hdf5 \
  --hz 10 \
  --seconds 30 \
  --prompt "test2"
```

可以依照實驗需求調整 `--hz`、`--seconds` 和 `--prompt`。錄製完成後，data 會存成 `.hdf5` file，位置在：

```text
/home/user/LEAH-real-robot-demo/data/hdf5
```

## 5. 檢查錄製結果

錄製完成後，可以在 container 內執行下面指令，印出最新 HDF5 file 的內容與統計資訊：

```bash
python3 - << 'EOF'
from pathlib import Path
import h5py
import numpy as np

data_dir = Path("/home/user/LEAH-real-robot-demo/data/hdf5")
files = sorted(data_dir.glob("*.hdf5"), key=lambda p: p.stat().st_mtime)
if not files:
    raise FileNotFoundError(f"No .hdf5 files found in {data_dir}")

path = files[-1]
print(f"Inspecting: {path}")

with h5py.File(path, "r") as f:
    prompt = f.attrs["prompt"]
    n = int(f.attrs["n_frames"])
    hz = float(f.attrs["record_hz"])
    joint_names = list(f.attrs.get("joint_names", ["q1", "q2", "q3", "q4", "q5", "q6"]))
    top = f["observations/images/top"][:]
    wrist = f["observations/images/wrist"][:]
    joints = f["observations/joints"][:]
    gripper = f["observations/gripper"][:]
    acts = f["actions"][:]
    ts = f["timestamps"][:]

print(f"prompt : {prompt}")
print(f"frames : {n}  ({n / hz:.1f}s @ {hz}Hz)")
print("\nShapes:")
print(f"  top    {top.shape} {top.dtype}")
print(f"  wrist  {wrist.shape} {wrist.dtype}")
print(f"  joints {joints.shape}  [q1..q6] rad")
print(f"  grip   {gripper.shape}  normalized [0,1]")
print(f"  action {acts.shape}  [q1..q6,gripper] absolute")

print("\nJoint stats:")
for i, lb in enumerate(joint_names):
    c = joints[:, i]
    print(f"  {lb:30s}: [{c.min():.4f}, {c.max():.4f}]  mean={c.mean():.4f}  std={c.std():.5f}")

print("\nAction stats:")
for i, lb in enumerate(joint_names + ["gripper"]):
    c = acts[:, i]
    print(f"  {lb:30s}: [{c.min():.6f}, {c.max():.6f}]  std={c.std():.6f}")

dts = np.diff(ts)
print(f"\nTiming: avg={dts.mean() * 1000:.1f}ms  std={dts.std() * 1000:.1f}ms  (target={1000 / hz:.0f}ms)")
print(f"\nGripper: range=[{gripper.min():.3f}, {gripper.max():.3f}]  steps with |delta|>0.01: {(np.abs(np.diff(gripper[:, 0])) > 0.01).sum()}")
print(f"Images:  top mean={top.mean():.1f} std={top.std():.1f} | wrist mean={wrist.mean():.1f} std={wrist.std():.1f}")
EOF
```

如果 shapes、joint/action statistics、timing、gripper 數值和 image statistics 都看起來正常，代表這次 data collection 完成。
