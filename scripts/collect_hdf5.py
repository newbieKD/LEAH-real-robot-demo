#!/usr/bin/env python3
"""
HDF5 training data collector for pi0.5/OpenPI — joint-space version.

Records synchronized observations from two cameras + robot joint states,
then saves one HDF5 episode file per run.

Usage (inside container, from /home/user/ur5e-ws):
  python3 /home/user/LEAH-real-robot-demo/scripts/collect_hdf5.py \
      --output-dir /home/user/ur5e-ws/data/hdf5 \
      --hz 10 \
      --seconds 30 \
      --prompt "pick up the block and place it in the bowl"

HDF5 layout:
  observations/images/top    (T, 224, 224, 3) uint8
  observations/images/wrist  (T, 224, 224, 3) uint8
  observations/joints        (T, 6)  float32  [q1..q6] absolute rad
  observations/gripper       (T, 1)  float32  gripper_norm [0, 1]
  actions                    (T, 7)  float32  [q1..q6, gripper] absolute
                                              action[t] = joints at t+1
  timestamps                 (T,)    float64
  attrs: prompt, n_frames, record_hz, image_size, joint_names
"""

import argparse
import os
import time
from datetime import datetime

import cv2
import h5py
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32


TOP_CAMERA_TOPIC   = "/camera/camera/color/image_raw"
WRIST_CAMERA_TOPIC = "/camera/wrist_camera/color/image_raw"
GRIPPER_TOPIC      = "/gripper/width"
JOINT_STATE_TOPIC  = "/joint_states"

# Canonical UR5e joint order
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

GRIPPER_OPEN_MM  = 80.0   # OnRobot 2FG fully open
GRIPPER_CLOSE_MM = 0.0    # OnRobot 2FG fully closed


class HDF5Collector(Node):
    def __init__(self, output_dir: str, hz: float, seconds: float,
                 image_size: int, prompt: str):
        super().__init__("hdf5_collector")

        self.output_dir  = output_dir
        self.record_hz   = hz
        self.max_seconds = seconds
        self.image_size  = image_size
        self.prompt      = prompt
        os.makedirs(output_dir, exist_ok=True)

        self.bridge = CvBridge()

        # Latest sensor data
        self.latest_top    = None
        self.latest_wrist  = None
        self.gripper_width = None
        self.joint_map     = None  # dict: joint_name -> position

        # Episode buffers
        self.top_images   = []
        self.wrist_images = []
        self.joints_buf   = []  # list of (6,) float32
        self.gripper_buf  = []  # list of (1,) float32
        self.timestamps   = []

        self.create_subscription(Image,      TOP_CAMERA_TOPIC,   self._cb_top,    10)
        self.create_subscription(Image,      WRIST_CAMERA_TOPIC, self._cb_wrist,  10)
        self.create_subscription(Float32,    GRIPPER_TOPIC,      self._cb_gripper, 10)
        self.create_subscription(JointState, JOINT_STATE_TOPIC,  self._cb_joints, 10)

        self.start_time = time.time()
        self.create_timer(1.0 / self.record_hz, self._record_step)
        self.get_logger().info(
            f"Recording {seconds}s @ {hz} Hz → {output_dir}"
        )

    # ── callbacks ──────────────────────────────────────────────────────

    def _cb_top(self, msg):     self.latest_top    = msg
    def _cb_wrist(self, msg):   self.latest_wrist  = msg
    def _cb_gripper(self, msg): self.gripper_width = msg.data

    def _cb_joints(self, msg):
        self.joint_map = {n: p for n, p in zip(msg.name, msg.position)}

    # ── helpers ─────────────────────────────────────────────────────────

    def _resize(self, img_rgb):
        return cv2.resize(img_rgb, (self.image_size, self.image_size))

    def _get_joints(self):
        """Return (6,) float32 in canonical JOINT_NAMES order, or None."""
        if self.joint_map is None:
            return None
        if not all(j in self.joint_map for j in JOINT_NAMES):
            return None
        return np.array([self.joint_map[j] for j in JOINT_NAMES],
                        dtype=np.float32)

    # ── recording step ───────────────────────────────────────────────────

    def _record_step(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.max_seconds:
            self._save()
            raise SystemExit

        missing = []
        if self.latest_top    is None: missing.append("top camera")
        if self.latest_wrist  is None: missing.append("wrist camera")
        if self.gripper_width is None: missing.append("gripper")
        if self.joint_map     is None: missing.append("joint states")
        if missing:
            self.get_logger().warn(
                "Waiting for: " + ", ".join(missing),
                throttle_duration_sec=2.0,
            )
            return

        joints = self._get_joints()
        if joints is None:
            self.get_logger().warn(
                "Waiting for all UR5e joint names in /joint_states…",
                throttle_duration_sec=2.0,
            )
            return

        top   = self._resize(self.bridge.imgmsg_to_cv2(self.latest_top,   "rgb8"))
        wrist = self._resize(self.bridge.imgmsg_to_cv2(self.latest_wrist, "rgb8"))

        gripper_norm = float(np.clip(
            (self.gripper_width - GRIPPER_CLOSE_MM) / (GRIPPER_OPEN_MM - GRIPPER_CLOSE_MM),
            0.0, 1.0))

        self.top_images.append(top)
        self.wrist_images.append(wrist)
        self.joints_buf.append(joints)
        self.gripper_buf.append(np.array([gripper_norm], dtype=np.float32))
        self.timestamps.append(time.time())

        n = len(self.top_images)
        if n % int(self.record_hz) == 0:
            self.get_logger().info(f"  {n} frames  ({elapsed:.1f}s)")

    # ── save ─────────────────────────────────────────────────────────────

    def _save(self):
        T = len(self.top_images)
        if T < 2:
            self.get_logger().error("Too few frames recorded — aborting.")
            return

        top_arr     = np.stack(self.top_images)            # (T, H, W, 3) uint8
        wrist_arr   = np.stack(self.wrist_images)          # (T, H, W, 3) uint8
        joints_arr  = np.stack(self.joints_buf)            # (T, 6)  float32
        gripper_arr = np.stack(self.gripper_buf)           # (T, 1)  float32
        ts_arr      = np.array(self.timestamps, dtype=np.float64)

        # action[t] = target state at t+1 (for DeltaActions transform in openpi)
        # Last frame: duplicate final state so shapes stay (T, 7)
        joints_next  = np.concatenate([joints_arr[1:],  joints_arr[-1:]], axis=0)
        gripper_next = np.concatenate([gripper_arr[1:], gripper_arr[-1:]], axis=0)
        act_arr = np.concatenate([joints_next, gripper_next],
                                 axis=1).astype(np.float32)  # (T, 7)

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        path   = os.path.join(self.output_dir, f"episode_{ts_str}.hdf5")

        with h5py.File(path, "w") as f:
            obs  = f.create_group("observations")
            imgs = obs.create_group("images")
            imgs.create_dataset("top",   data=top_arr,   compression="gzip")
            imgs.create_dataset("wrist", data=wrist_arr, compression="gzip")
            obs.create_dataset("joints",  data=joints_arr)
            obs.create_dataset("gripper", data=gripper_arr)
            f.create_dataset("actions",    data=act_arr)
            f.create_dataset("timestamps", data=ts_arr)
            f.attrs["prompt"]      = self.prompt
            f.attrs["n_frames"]    = T
            f.attrs["record_hz"]   = self.record_hz
            f.attrs["image_size"]  = self.image_size
            f.attrs["joint_names"] = JOINT_NAMES

        self.get_logger().info(f"\n{'='*56}")
        self.get_logger().info(f"Saved → {path}")
        self.get_logger().info(
            f"  observations/images/top   {top_arr.shape}  uint8")
        self.get_logger().info(
            f"  observations/images/wrist {wrist_arr.shape}  uint8")
        self.get_logger().info(
            f"  observations/joints       {joints_arr.shape}  [q1..q6] rad")
        self.get_logger().info(
            f"  observations/gripper      {gripper_arr.shape}  [norm 0-1]")
        self.get_logger().info(
            f"  actions                   {act_arr.shape}  [q1..q6,grip] absolute")
        self.get_logger().info(f"{'='*56}")


# ── entrypoint ────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="/home/user/ur5e-ws/data/hdf5")
    parser.add_argument("--hz",         type=float, default=10.0,
                        help="Recording frequency in Hz")
    parser.add_argument("--seconds",    type=float, default=30.0,
                        help="Episode duration in seconds")
    parser.add_argument("--image-size", type=int,   default=224,
                        help="Square crop size (pixels)")
    parser.add_argument("--prompt",     default="pick up the block",
                        help="Task instruction string stored in HDF5 attrs")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = HDF5Collector(
        output_dir=args.output_dir,
        hz=args.hz,
        seconds=args.seconds,
        image_size=args.image_size,
        prompt=args.prompt,
    )
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
