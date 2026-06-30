#!/usr/bin/env python3
"""
Convert UR5e HDF5 episodes → LeRobot v2.0 dataset for openpi/pi0.5 fine-tuning.

Follows the openpi UR5 example conventions so the dataset is directly loadable
by LeRobotUR5DataConfig (RepackTransform expects: image, wrist_image, joints,
gripper, actions, task).

Usage:
  python3 convert_to_lerobot.py \
      --input-dir  /path/to/data/hdf5 \
      --repo-id    your_hf_username/ur5e-demo \
      [--push-to-hub]

Dependencies:
  pip install lerobot

Output (local):
  ~/.cache/huggingface/lerobot/<repo-id>/
    meta/info.json
    meta/episodes.jsonl
    meta/tasks.jsonl
    data/chunk-000/episode_XXXXXX.parquet
    videos/chunk-000/observation.images.image/episode_XXXXXX.mp4
    videos/chunk-000/observation.images.wrist_image/episode_XXXXXX.mp4
"""

import argparse
import os
import shutil
import glob

import h5py
import numpy as np

from lerobot.common.datasets.lerobot_dataset import HF_LEROBOT_HOME, LeRobotDataset


def convert(input_dir: str, repo_id: str, push_to_hub: bool = False):
    hdf5_files = sorted(glob.glob(os.path.join(input_dir, "episode_*.hdf5")))
    if not hdf5_files:
        raise FileNotFoundError(f"No episode_*.hdf5 found in {input_dir}")

    print(f"Found {len(hdf5_files)} episodes in {input_dir}")

    # Detect image size from first file
    with h5py.File(hdf5_files[0], "r") as f:
        H, W = f["observations/images/top"].shape[1:3]
        fps  = float(f.attrs.get("record_hz", 10.0))

    output_path = HF_LEROBOT_HOME / repo_id
    if output_path.exists():
        print(f"Removing existing dataset at {output_path}")
        shutil.rmtree(output_path)

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        robot_type="ur5e",
        fps=fps,
        features={
            # openpi UR5 RepackTransform expects these exact keys
            "image": {
                "dtype": "image",
                "shape": (H, W, 3),
                "names": ["height", "width", "channel"],
            },
            "wrist_image": {
                "dtype": "image",
                "shape": (H, W, 3),
                "names": ["height", "width", "channel"],
            },
            "joints": {
                "dtype": "float32",
                "shape": (6,),
                "names": ["joints"],
            },
            "gripper": {
                "dtype": "float32",
                "shape": (1,),
                "names": ["gripper"],
            },
            "actions": {
                "dtype": "float32",
                "shape": (7,),
                "names": ["actions"],
            },
        },
        image_writer_threads=4,
        image_writer_processes=2,
    )

    for ep_idx, hdf5_path in enumerate(hdf5_files):
        print(f"  [{ep_idx+1}/{len(hdf5_files)}] {os.path.basename(hdf5_path)}")

        with h5py.File(hdf5_path, "r") as f:
            top_imgs   = f["observations/images/top"][:]    # (T, H, W, 3) uint8
            wrist_imgs = f["observations/images/wrist"][:]  # (T, H, W, 3) uint8
            joints     = f["observations/joints"][:]        # (T, 6)  float32
            gripper    = f["observations/gripper"][:]       # (T, 1)  float32
            actions    = f["actions"][:]                    # (T, 7)  float32
            prompt     = str(f.attrs.get("prompt", ""))
            T          = int(f.attrs.get("n_frames", len(actions)))

        for t in range(T):
            dataset.add_frame({
                "image":       top_imgs[t],      # (H, W, 3) uint8
                "wrist_image": wrist_imgs[t],    # (H, W, 3) uint8
                "joints":      joints[t],        # (6,) float32
                "gripper":     gripper[t],       # (1,) float32
                "actions":     actions[t],       # (7,) float32 absolute joint
                "task":        prompt,
            })

        dataset.save_episode()
        print(f"      → saved {T} frames, prompt='{prompt}'")

    dataset.consolidate(run_compute_stats=True)
    print(f"\nDataset saved to: {output_path}")
    print(f"  Total episodes : {len(hdf5_files)}")
    print(f"  Total frames   : {sum(1 for _ in dataset)}")

    if push_to_hub:
        dataset.push_to_hub(
            tags=["ur5e", "pi0.5", "real-robot"],
            private=True,
            push_videos=True,
            license="apache-2.0",
        )
        print(f"Pushed to HuggingFace Hub: {repo_id}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert UR5e HDF5 episodes to LeRobot dataset for pi0.5"
    )
    parser.add_argument(
        "--input-dir", required=True,
        help="Directory containing episode_*.hdf5 files"
    )
    parser.add_argument(
        "--repo-id", default="local/ur5e-demo",
        help="HuggingFace repo id, e.g. your_username/ur5e-demo"
    )
    parser.add_argument(
        "--push-to-hub", action="store_true",
        help="Push the converted dataset to HuggingFace Hub"
    )
    args = parser.parse_args()

    convert(
        input_dir=args.input_dir,
        repo_id=args.repo_id,
        push_to_hub=args.push_to_hub,
    )


if __name__ == "__main__":
    main()
