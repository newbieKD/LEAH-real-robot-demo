#!/usr/bin/env python3
"""
View or export frames from a recorded HDF5 episode (joint-space format).

Usage:
  # Save side-by-side video (top + wrist)
  python3 view_hdf5.py episode.hdf5

  # Save individual frames as images
  python3 view_hdf5.py episode.hdf5 --frames

  # Print stats only (no images)
  python3 view_hdf5.py episode.hdf5 --stats-only
"""

import argparse
import os
import h5py
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("hdf5_path", help="Path to episode HDF5 file")
    parser.add_argument("--frames",     action="store_true",
                        help="Save individual frames instead of video")
    parser.add_argument("--fps",        type=float, default=10.0)
    parser.add_argument("--stats-only", action="store_true")
    args = parser.parse_args()

    with h5py.File(args.hdf5_path, "r") as f:
        prompt      = f.attrs.get("prompt", "")
        n_frames    = int(f.attrs.get("n_frames", 0))
        hz          = float(f.attrs.get("record_hz", args.fps))
        joint_names = list(f.attrs.get("joint_names",
                                       ["q1","q2","q3","q4","q5","q6"]))

        top_imgs   = f["observations/images/top"][:]    # (T, H, W, 3) RGB
        wrist_imgs = f["observations/images/wrist"][:]  # (T, H, W, 3) RGB
        joints     = f["observations/joints"][:]        # (T, 6)
        gripper    = f["observations/gripper"][:]       # (T, 1)
        actions    = f["actions"][:]                    # (T, 7)

    print(f"prompt  : {prompt}")
    print(f"frames  : {n_frames}  ({n_frames/hz:.1f}s @ {hz}Hz)")
    print(f"top     : {top_imgs.shape}  uint8")
    print(f"wrist   : {wrist_imgs.shape}  uint8")
    print(f"joints  : {joints.shape}   [rad]  names={joint_names}")
    print(f"gripper : {gripper.shape}   [norm 0-1]")
    print(f"actions : {actions.shape}   [q1..q6, gripper] absolute")

    print(f"\nJoint range (rad):")
    for i, name in enumerate(joint_names):
        print(f"  {name:30s}  min={joints[:,i].min():.4f}  "
              f"max={joints[:,i].max():.4f}")

    print(f"\nGripper state : min={gripper.min():.3f}  max={gripper.max():.3f}")

    print(f"\nAction range (absolute joint rad + gripper):")
    labels = joint_names + ["gripper"]
    for i, label in enumerate(labels):
        print(f"  {label:30s}  min={actions[:,i].min():.4f}  "
              f"max={actions[:,i].max():.4f}")

    if args.stats_only:
        return

    base = os.path.splitext(args.hdf5_path)[0]
    H, W = top_imgs.shape[1], top_imgs.shape[2]

    if args.frames:
        out_dir = base + "_frames"
        os.makedirs(out_dir, exist_ok=True)
        for i in range(n_frames):
            top_bgr   = cv2.cvtColor(top_imgs[i],   cv2.COLOR_RGB2BGR)
            wrist_bgr = cv2.cvtColor(wrist_imgs[i], cv2.COLOR_RGB2BGR)
            side_by_side = np.concatenate([top_bgr, wrist_bgr], axis=1)
            cv2.imwrite(os.path.join(out_dir, f"frame_{i:04d}.jpg"),
                        side_by_side)
        print(f"\nSaved {n_frames} frames → {out_dir}/")
    else:
        out_path = base + "_preview.mp4"
        fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
        writer   = cv2.VideoWriter(out_path, fourcc, hz, (W * 2, H))
        for i in range(n_frames):
            top_bgr   = cv2.cvtColor(top_imgs[i],   cv2.COLOR_RGB2BGR)
            wrist_bgr = cv2.cvtColor(wrist_imgs[i], cv2.COLOR_RGB2BGR)
            frame = np.concatenate([top_bgr, wrist_bgr], axis=1)
            writer.write(frame)
        writer.release()
        print(f"\nSaved video → {out_path}")


if __name__ == "__main__":
    main()
