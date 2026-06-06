#!/usr/bin/env python3
"""Create an IR-SIM world YAML from a GS-derived occupancy map.

The bridge uses IR-SIM's built-in `world.obstacle_map` image path. This avoids
decomposing concave or fragmented GS obstacles into convex polygons.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml
from scipy import ndimage


OCCUPIED = 0
UNKNOWN = 127
FREE = 255


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def world_from_pixel(row: int, col: int, height: int, resolution: float) -> Tuple[float, float]:
    x = (col + 0.5) * resolution
    y = (height - row - 0.5) * resolution
    return float(x), float(y)


def choose_free_pose(free_mask: np.ndarray, resolution: float, prefer: str) -> Tuple[float, float, float]:
    height, width = free_mask.shape
    ys, xs = np.where(free_mask)
    if len(xs) == 0:
        raise ValueError("No free cells found in occupancy map")

    if prefer == "start":
        target = np.array([height - 1, 0], dtype=np.float32)
    elif prefer == "goal":
        target = np.array([0, width - 1], dtype=np.float32)
    else:
        target = np.array([height * 0.5, width * 0.5], dtype=np.float32)

    pts = np.stack([ys, xs], axis=1).astype(np.float32)
    idx = int(np.argmin(np.linalg.norm(pts - target[None, :], axis=1)))
    row, col = int(ys[idx]), int(xs[idx])
    x, y = world_from_pixel(row, col, height, resolution)
    return x, y, 0.0


def farthest_free_pose(free_mask: np.ndarray, resolution: float, start_xy: Tuple[float, float]) -> Tuple[float, float, float]:
    height, _width = free_mask.shape
    ys, xs = np.where(free_mask)
    if len(xs) == 0:
        raise ValueError("No free cells found in occupancy map")
    coords = np.asarray([world_from_pixel(int(r), int(c), height, resolution) for r, c in zip(ys, xs)], dtype=np.float32)
    start = np.asarray(start_xy, dtype=np.float32)
    idx = int(np.argmax(np.linalg.norm(coords - start[None, :], axis=1)))
    return float(coords[idx, 0]), float(coords[idx, 1]), 0.0


def safe_pose_mask(free_mask: np.ndarray, resolution: float, robot_radius: float, mdownsample: int) -> np.ndarray:
    # IR-SIM buffers obstacle-map points by roughly the downsampled map cell size.
    # Keep additional clearance so auto-selected poses do not start on the line boundary.
    clearance_m = float(robot_radius) + float(mdownsample) * float(resolution)
    clearance_px = max(1, int(math.ceil(clearance_m / float(resolution))))
    bounded_free = free_mask.copy()
    bounded_free[:clearance_px, :] = False
    bounded_free[-clearance_px:, :] = False
    bounded_free[:, :clearance_px] = False
    bounded_free[:, -clearance_px:] = False
    distance = ndimage.distance_transform_edt(bounded_free)
    safe = distance >= clearance_px
    labels, num = ndimage.label(safe)
    if num <= 1:
        return safe if np.any(safe) else free_mask
    counts = np.bincount(labels.ravel())
    counts[0] = 0
    largest = int(np.argmax(counts))
    return labels == largest


def parse_pose(value: Optional[str]) -> Optional[Tuple[float, float, float]]:
    if value is None:
        return None
    parts = [float(x) for x in value.split(",")]
    if len(parts) == 2:
        parts.append(0.0)
    if len(parts) != 3:
        raise ValueError("Pose must be 'x,y' or 'x,y,theta'")
    return float(parts[0]), float(parts[1]), float(parts[2])


def make_irsim_image(grid: np.ndarray, unknown_as: str) -> np.ndarray:
    free = grid == FREE
    if unknown_as == "free":
        free |= grid == UNKNOWN
    out = np.zeros_like(grid, dtype=np.uint8)
    out[free] = 255
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Export an IR-SIM YAML from GS occupancy map outputs.")
    parser.add_argument("--occupancy_dir", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--world_name", default="scene01_gs_irsim")
    parser.add_argument("--unknown_as", choices=["occupied", "free"], default="occupied")
    parser.add_argument("--mdownsample", type=int, default=4)
    parser.add_argument("--robot_radius", type=float, default=0.25)
    parser.add_argument("--start", default=None, help="Optional 'x,y,theta' in IR-SIM map coordinates")
    parser.add_argument("--goal", default=None, help="Optional 'x,y,theta' in IR-SIM map coordinates")
    parser.add_argument("--control_mode", default="auto", choices=["auto", "keyboard"])
    args = parser.parse_args()

    occupancy_dir = args.occupancy_dir
    meta = load_yaml(occupancy_dir / "map.yaml")
    map_img = cv2.imread(str(occupancy_dir / "map.png"), cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        raise FileNotFoundError(occupancy_dir / "map.png")

    resolution = float(meta["resolution"])
    world_width = float(map_img.shape[1] * resolution)
    world_height = float(map_img.shape[0] * resolution)

    irsim_img = make_irsim_image(map_img, args.unknown_as)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    map_path = (args.output_dir / f"{args.world_name}_obstacle_map.png").resolve()
    cv2.imwrite(str(map_path), irsim_img)

    sampled_free_mask = irsim_img[:: int(args.mdownsample), :: int(args.mdownsample)] == 255
    sampled_resolution = resolution * int(args.mdownsample)
    free_for_pose = safe_pose_mask(sampled_free_mask, sampled_resolution, args.robot_radius, 1)

    start = parse_pose(args.start)
    goal = parse_pose(args.goal)
    if start is None:
        start = choose_free_pose(free_for_pose, sampled_resolution, "start")
    if goal is None:
        goal = farthest_free_pose(free_for_pose, sampled_resolution, (start[0], start[1]))

    world = {
        "height": world_height,
        "width": world_width,
        "step_time": 0.1,
        "sample_time": 0.1,
        "offset": [0, 0],
        "collision_mode": "stop",
        "control_mode": args.control_mode,
        "obstacle_map": str(map_path),
        "mdownsample": int(args.mdownsample),
        "plot": {"show_title": True},
    }
    robot = [
        {
            "kinematics": {"name": "diff"},
            "shape": {"name": "circle", "radius": float(args.robot_radius)},
            "state": [float(start[0]), float(start[1]), float(start[2])],
            "goal": [float(goal[0]), float(goal[1]), float(goal[2])],
            "behavior": {"name": "dash"},
            "vel_max": [1.0, 1.0],
            "plot": {"show_trail": True, "show_trajectory": True, "show_goal": True},
            "sensors": [
                {
                    "type": "lidar2d",
                    "range_min": 0,
                    "range_max": 10,
                    "angle_range": 3.14,
                    "number": 120,
                    "noise": False,
                    "alpha": 0.35,
                }
            ],
        }
    ]
    config = {
        "world": world,
        "robot": robot,
        "obstacle": [],
    }
    metadata = {
        "source_occupancy_dir": str(occupancy_dir.resolve()),
        "source_map_yaml": str((occupancy_dir / "map.yaml").resolve()),
        "unknown_as": args.unknown_as,
        "source_resolution": resolution,
        "image_shape": [int(map_img.shape[0]), int(map_img.shape[1])],
        "note": "IR-SIM ignores the original GS world origin; this YAML uses local image coordinates.",
    }

    yaml_path = args.output_dir / f"{args.world_name}.yaml"
    with yaml_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False)
    with (args.output_dir / f"{args.world_name}_metadata.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(metadata, f, sort_keys=False)

    print(f"Wrote IR-SIM map image: {map_path}")
    print(f"Wrote IR-SIM world YAML: {yaml_path}")
    print(f"world: width={world_width:.3f} height={world_height:.3f} m, mdownsample={args.mdownsample}")
    print(f"robot start={robot[0]['state']} goal={robot[0]['goal']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
