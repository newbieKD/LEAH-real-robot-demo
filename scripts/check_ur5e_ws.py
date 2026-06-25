#!/usr/bin/env python3
"""Check whether the UR5e ROS2 workspace exposes topics needed by the bridge."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class TopicCheck:
    name: str
    required: bool
    description: str


DEFAULT_TOPICS = (
    TopicCheck("/target_pose", True, "Pose target consumed by ur_pose_tracking"),
    TopicCheck("/servo_node/delta_twist_cmds", True, "Servo twist command to UR driver"),
    TopicCheck("/camera/camera/color/image_raw", True, "RealSense RGB image"),
    TopicCheck("/gripper/command", True, "OnRobot 2FG width command"),
    TopicCheck("/gripper/width", False, "OnRobot 2FG width feedback"),
    TopicCheck("/gripper/grip_detected", False, "OnRobot 2FG object-detected feedback"),
)


def _run_ros2(args: list[str], timeout_s: float | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["ros2", *args],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout_s,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-frame", default="world")
    parser.add_argument("--ee-frame", default="rg2_base_link")
    parser.add_argument("--skip-tf", action="store_true")
    args = parser.parse_args()

    if shutil.which("ros2") is None:
        print("ERROR: ros2 was not found. Run this inside the ur5e-ws ROS2 container.", file=sys.stderr)
        return 2

    topic_proc = _run_ros2(["topic", "list"])
    if topic_proc.returncode != 0:
        print(topic_proc.stderr.strip(), file=sys.stderr)
        return topic_proc.returncode

    available_topics = set(topic_proc.stdout.splitlines())
    failed = False

    print("ROS2 topic checks:")
    for check in DEFAULT_TOPICS:
        ok = check.name in available_topics
        status = "OK" if ok else ("MISSING" if check.required else "optional-missing")
        print(f"  {status:16s} {check.name:36s} {check.description}")
        failed = failed or (check.required and not ok)

    node_proc = _run_ros2(["node", "list"])
    if node_proc.returncode == 0:
        print("\nROS2 nodes:")
        for node in node_proc.stdout.splitlines():
            print(f"  {node}")
    else:
        print("\nWARNING: failed to list nodes:")
        print(node_proc.stderr.strip())

    if not args.skip_tf:
        print("\nTF check:")
        try:
            tf_proc = _run_ros2(["run", "tf2_ros", "tf2_echo", args.base_frame, args.ee_frame], timeout_s=3.0)
            tf_text = f"{tf_proc.stdout}\n{tf_proc.stderr}"
            tf_ok = "Translation:" in tf_text or "transform" in tf_text.lower()
        except subprocess.TimeoutExpired as exc:
            tf_text = f"{exc.stdout or ''}\n{exc.stderr or ''}"
            tf_ok = "Translation:" in tf_text or "transform" in tf_text.lower()
        if tf_ok:
            print(f"  OK {args.base_frame} -> {args.ee_frame}")
        else:
            failed = True
            print(f"  MISSING {args.base_frame} -> {args.ee_frame}")
            print("  Run ur_driver_bringup, servo.launch.py, and ur_pose_tracking first.")

    if failed:
        print("\nResult: hardware workspace is not ready for policy execution.")
        return 1

    print("\nResult: minimum topics are available.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
