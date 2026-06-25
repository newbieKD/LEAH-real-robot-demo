#!/usr/bin/env python3
"""Conservative OpenPI action-chunk bridge for UR5e real-robot demos.

The bridge uses the external ur5e-ws public ROS2 interface:

* publish target poses to /target_pose for ur_pose_tracking;
* publish gripper widths to /gripper/command;
* keep all motion deltas clamped before publishing.

Validate checkpoint action convention in --dry-run mode before using --execute.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import ast

import numpy as np


@dataclass(frozen=True)
class BridgeConfig:
    base_frame_id: str
    ee_frame_id: str
    target_pose_topic: str
    gripper_command_topic: str
    gripper_width_topic: str
    rgb_topic: str
    wrist_rgb_topic: str
    policy_host: str
    policy_port: int
    resize_size: int
    default_k: int
    max_queries: int
    query_period_s: float
    max_translation_step_m: float
    max_rotation_step_rad: float
    gripper_open_width_mm: float
    gripper_close_width_mm: float
    gripper_close_threshold: float
    workspace_min_xyz_m: tuple[float, float, float]
    workspace_max_xyz_m: tuple[float, float, float]


def _parse_scalar(value: str) -> Any:
    value = value.strip()
    if value == "":
        return ""
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    if value.startswith("[") and value.endswith("]"):
        return ast.literal_eval(value)
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _load_config_dict(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError:
        data: dict[str, Any] = {}
        current_section: str | None = None
        for raw_line in path.read_text().splitlines():
            content = raw_line.split("#", 1)[0].rstrip()
            if not content.strip():
                continue
            indent = len(raw_line) - len(raw_line.lstrip(" "))
            stripped = content.strip()
            if indent == 0 and stripped.endswith(":"):
                current_section = stripped[:-1]
                data[current_section] = {}
                continue
            if current_section is None or ":" not in stripped:
                continue
            key, value = stripped.split(":", 1)
            data[current_section][key.strip()] = _parse_scalar(value)
        return data
    return yaml.safe_load(path.read_text()) or {}


def _load_config(path: Path) -> BridgeConfig:
    raw = _load_config_dict(path)
    ros = raw.get("ros", {})
    policy = raw.get("policy", {})
    action = raw.get("action", {})
    safety = raw.get("safety", {})
    return BridgeConfig(
        base_frame_id=str(ros.get("base_frame_id", "world")),
        ee_frame_id=str(ros.get("ee_frame_id", "rg2_base_link")),
        target_pose_topic=str(ros.get("target_pose_topic", "/target_pose")),
        gripper_command_topic=str(ros.get("gripper_command_topic", "/gripper/command")),
        gripper_width_topic=str(ros.get("gripper_width_topic", "/gripper/width")),
        rgb_topic=str(ros.get("rgb_topic", "/camera/camera/color/image_raw")),
        wrist_rgb_topic=str(ros.get("wrist_rgb_topic", "")),
        policy_host=str(policy.get("host", "127.0.0.1")),
        policy_port=int(policy.get("port", 8000)),
        resize_size=int(policy.get("resize_size", 224)),
        default_k=int(policy.get("default_k", 5)),
        max_queries=int(policy.get("max_queries", 30)),
        query_period_s=float(policy.get("query_period_s", 0.2)),
        max_translation_step_m=float(action.get("max_translation_step_m", 0.015)),
        max_rotation_step_rad=float(action.get("max_rotation_step_rad", 0.08)),
        gripper_open_width_mm=float(action.get("gripper_open_width_mm", 80.0)),
        gripper_close_width_mm=float(action.get("gripper_close_width_mm", 0.0)),
        gripper_close_threshold=float(action.get("gripper_close_threshold", 0.0)),
        workspace_min_xyz_m=tuple(float(v) for v in safety.get("workspace_min_xyz_m", [0.15, -0.55, 0.05])),
        workspace_max_xyz_m=tuple(float(v) for v in safety.get("workspace_max_xyz_m", [0.85, 0.35, 0.65])),
    )


def _axis_angle_to_quat(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    axis = rotvec / angle
    half = 0.5 * angle
    return np.array(
        [axis[0] * math.sin(half), axis[1] * math.sin(half), axis[2] * math.sin(half), math.cos(half)],
        dtype=np.float64,
    )


def _quat_to_axis_angle(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float64).copy()
    q = q / max(np.linalg.norm(q), 1e-9)
    if q[3] < 0:
        q = -q
    angle = 2.0 * math.atan2(float(np.linalg.norm(q[:3])), float(q[3]))
    if angle < 1e-9:
        return np.zeros(3, dtype=np.float64)
    axis = q[:3] / max(math.sin(angle / 2.0), 1e-9)
    return axis * angle


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float64,
    )


def _clip_delta(action: np.ndarray, cfg: BridgeConfig) -> np.ndarray:
    clipped = np.asarray(action, dtype=np.float64).copy()
    if clipped.shape[0] < 7:
        raise ValueError(f"Expected at least 7 action dimensions, got shape={clipped.shape}")
    clipped[:3] = np.clip(clipped[:3], -cfg.max_translation_step_m, cfg.max_translation_step_m)
    clipped[3:6] = np.clip(clipped[3:6], -cfg.max_rotation_step_rad, cfg.max_rotation_step_rad)
    return clipped[:7]


def _clip_workspace(position: np.ndarray, cfg: BridgeConfig) -> np.ndarray:
    return np.minimum(np.maximum(position, np.asarray(cfg.workspace_min_xyz_m)), np.asarray(cfg.workspace_max_xyz_m))


def _gripper_width(action_value: float, cfg: BridgeConfig) -> float:
    if action_value > cfg.gripper_close_threshold:
        return cfg.gripper_open_width_mm
    return cfg.gripper_close_width_mm


def _make_policy_client(host: str, port: int) -> Any:
    try:
        from openpi_client import websocket_client_policy
    except ImportError as exc:
        raise RuntimeError(
            "openpi_client is unavailable. Run inside the OpenPI environment or add the OpenPI client package to PYTHONPATH."
        ) from exc
    return websocket_client_policy.WebsocketClientPolicy(host, port)


def _build_dummy_observation(prompt: str, cfg: BridgeConfig) -> dict[str, Any]:
    image = np.zeros((cfg.resize_size, cfg.resize_size, 3), dtype=np.uint8)
    return {
        "observation/image": image,
        "observation/wrist_image": image,
        "observation/state": np.zeros(8, dtype=np.float32),
        "prompt": prompt,
    }


def _summarize_chunk(chunk: np.ndarray) -> str:
    if chunk.ndim != 2:
        return f"shape={chunk.shape}"
    translation = np.linalg.norm(chunk[:, :3], axis=1)
    rotation = np.linalg.norm(chunk[:, 3:6], axis=1)
    return (
        f"shape={chunk.shape}, "
        f"max_translation={translation.max(initial=0.0):.5f}, "
        f"max_rotation={rotation.max(initial=0.0):.5f}, "
        f"gripper_minmax=({chunk[:, 6].min(initial=0.0):.5f}, {chunk[:, 6].max(initial=0.0):.5f})"
    )


def run_dry_run(prompt: str, cfg: BridgeConfig) -> int:
    client = _make_policy_client(cfg.policy_host, cfg.policy_port)
    response = client.infer(_build_dummy_observation(prompt, cfg))
    chunk = np.asarray(response["actions"], dtype=np.float32)
    print("Policy dry-run response:", _summarize_chunk(chunk))
    print("First clipped actions:")
    for idx, action in enumerate(chunk[: cfg.default_k]):
        print(f"  {idx:02d}: {_clip_delta(action, cfg).tolist()}")
    return 0


def run_execute(prompt: str, cfg: BridgeConfig) -> int:
    try:
        import rclpy
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32
        from tf2_ros import Buffer, TransformListener
    except ImportError as exc:
        raise RuntimeError("ROS2 Python dependencies are unavailable. Run this inside the ur5e-ws container.") from exc

    class BridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("openpi_real_bridge")
            self.bridge = CvBridge()
            self.rgb = None
            self.wrist_rgb = None
            self.gripper_width_mm = cfg.gripper_open_width_mm
            self.policy = _make_policy_client(cfg.policy_host, cfg.policy_port)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
            self.target_pub = self.create_publisher(PoseStamped, cfg.target_pose_topic, 10)
            self.gripper_pub = self.create_publisher(Float32, cfg.gripper_command_topic, 10)
            self.create_subscription(Image, cfg.rgb_topic, self._rgb_cb, 10)
            self.create_subscription(Float32, cfg.gripper_width_topic, self._gripper_width_cb, 10)
            if cfg.wrist_rgb_topic:
                self.create_subscription(Image, cfg.wrist_rgb_topic, self._wrist_rgb_cb, 10)

        def _rgb_cb(self, msg: Image) -> None:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        def _wrist_rgb_cb(self, msg: Image) -> None:
            self.wrist_rgb = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        def _gripper_width_cb(self, msg: Float32) -> None:
            self.gripper_width_mm = float(msg.data)

        def _current_pose(self) -> tuple[np.ndarray, np.ndarray]:
            tf = self.tf_buffer.lookup_transform(cfg.base_frame_id, cfg.ee_frame_id, rclpy.time.Time())
            pos = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z], dtype=np.float64)
            quat = np.array([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w], dtype=np.float64)
            return pos, quat

        def _observation(self) -> dict[str, Any]:
            if self.rgb is None:
                raise RuntimeError(f"No RGB image received on {cfg.rgb_topic}")
            wrist = self.wrist_rgb if self.wrist_rgb is not None else np.zeros_like(self.rgb)
            pos, quat = self._current_pose()
            rotvec = _quat_to_axis_angle(quat)
            width_range = max(cfg.gripper_open_width_mm - cfg.gripper_close_width_mm, 1e-6)
            grip_norm = (float(self.gripper_width_mm) - cfg.gripper_close_width_mm) / width_range
            grip_norm = float(np.clip(grip_norm, 0.0, 1.0))
            state = np.concatenate([pos, rotvec, [grip_norm, grip_norm]]).astype(np.float32)
            return {
                "observation/image": np.asarray(self.rgb, dtype=np.uint8),
                "observation/wrist_image": np.asarray(wrist, dtype=np.uint8),
                "observation/state": state,
                "prompt": prompt,
            }

        def publish_action(self, action: np.ndarray) -> None:
            clipped = _clip_delta(action, cfg)
            pos, quat = self._current_pose()
            target_pos = _clip_workspace(pos + clipped[:3], cfg)
            target_quat = _quat_multiply(_axis_angle_to_quat(clipped[3:6]), quat)
            target_quat = target_quat / max(np.linalg.norm(target_quat), 1e-9)

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = cfg.base_frame_id
            pose.pose.position.x = float(target_pos[0])
            pose.pose.position.y = float(target_pos[1])
            pose.pose.position.z = float(target_pos[2])
            pose.pose.orientation.x = float(target_quat[0])
            pose.pose.orientation.y = float(target_quat[1])
            pose.pose.orientation.z = float(target_quat[2])
            pose.pose.orientation.w = float(target_quat[3])
            self.target_pub.publish(pose)
            self.gripper_pub.publish(Float32(data=float(_gripper_width(clipped[6], cfg))))

        def query_policy(self) -> np.ndarray:
            response = self.policy.infer(self._observation())
            return np.asarray(response["actions"], dtype=np.float32)

    rclpy.init()
    node = BridgeNode()
    try:
        for query_idx in range(cfg.max_queries):
            rclpy.spin_once(node, timeout_sec=1.0)
            chunk = node.query_policy()
            node.get_logger().info(f"Query {query_idx}: {_summarize_chunk(chunk)}")
            for action in chunk[: cfg.default_k]:
                node.publish_action(action)
                deadline = time.time() + cfg.query_period_s
                while time.time() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("real_robot_control/configs/ur5e_demo.yaml"))
    parser.add_argument("--prompt", required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    cfg = _load_config(args.config)
    if args.dry_run:
        return run_dry_run(args.prompt, cfg)
    return run_execute(args.prompt, cfg)


if __name__ == "__main__":
    raise SystemExit(main())
