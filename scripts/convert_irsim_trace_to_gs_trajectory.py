#!/usr/bin/env python3
"""Convert an IR-SIM follow trace into Habitat-GS agent trajectory poses."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import yaml
import cv2


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def iter_trace(path: Path):
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                yield json.loads(line)


class IrsimToGsMapper:
    """Map IR-SIM image-local coordinates back to GS scene coordinates."""

    def __init__(self, map_meta: dict[str, Any], world_meta: dict[str, Any] | None = None) -> None:
        self.resolution = float(map_meta["resolution"])
        self.origin_a = float(map_meta["origin"][0])
        self.origin_b = float(map_meta["origin"][1])
        self.out_width = int(map_meta["width"])
        self.out_height = int(map_meta["height"])
        self.transform = map_meta.get("image_transform", "identity")
        stats = map_meta.get("stats", {})
        self.raw_width = int(stats.get("width", self.out_width))
        self.raw_height = int(stats.get("height", self.out_height))
        axes = map_meta.get("axes", {})
        self.plane = list(axes.get("plane", ["x", "z"]))
        self.up = str(axes.get("up", "y"))
        self.floor = float(map_meta.get("floor_z", 0.0))
        self.world_meta = world_meta
        self.world_width = None
        self.world_height = None
        self.mdownsample = 1
        self.sampled_width = None
        self.sampled_height = None
        if world_meta is not None:
            world = world_meta["world"]
            self.world_width = float(world["width"])
            self.world_height = float(world["height"])
            self.mdownsample = int(world.get("mdownsample", 1))
            self.sampled_width = len(range(0, self.out_width, self.mdownsample))
            self.sampled_height = len(range(0, self.out_height, self.mdownsample))

    def _output_pixel_from_irsim(self, x: float, y: float) -> tuple[float, float]:
        if self.world_width is not None and self.world_height is not None:
            # Match IR-SIM world.py:
            #   grid_map = image[::mdownsample, ::mdownsample]
            #   grid_map = np.fliplr(grid_map.T)
            #   obstacle_positions = np.where(grid_map > 50) * reso
            x_reso = self.world_width / float(self.sampled_width)
            y_reso = self.world_height / float(self.sampled_height)
            sampled_col = x / x_reso - 0.5
            sampled_row = self.sampled_height - 0.5 - y / y_reso
            return sampled_row * self.mdownsample, sampled_col * self.mdownsample
        col = x / self.resolution - 0.5
        row = self.out_height - 0.5 - y / self.resolution
        return row, col

    def _raw_pixel_from_output(self, row: float, col: float) -> tuple[float, float]:
        t = self.transform
        if t == "identity":
            return row, col
        if t == "flip_x":
            return row, self.out_width - 1.0 - col
        if t == "flip_y":
            return self.out_height - 1.0 - row, col
        if t == "rotate_180":
            return self.out_height - 1.0 - row, self.out_width - 1.0 - col
        if t == "transpose":
            return col, row
        if t == "transpose_rotate_180":
            return self.out_width - 1.0 - col, self.out_height - 1.0 - row
        if t == "transpose_flip_x":
            return self.out_width - 1.0 - col, row
        if t == "transpose_flip_y":
            return col, self.out_height - 1.0 - row
        raise ValueError(f"Unsupported image_transform: {t}")

    def irsim_xy_to_gs_plane(self, x: float, y: float) -> tuple[float, float]:
        out_row, out_col = self._output_pixel_from_irsim(x, y)
        raw_row, raw_col = self._raw_pixel_from_output(out_row, out_col)
        plane_a = self.origin_a + (raw_col + 0.5) * self.resolution
        plane_b = self.origin_b + (self.raw_height - raw_row - 0.5) * self.resolution
        return float(plane_a), float(plane_b)

    def irsim_pose_to_gs(self, x: float, y: float, theta: float, camera_height: float) -> tuple[list[float], float]:
        a0, b0 = self.irsim_xy_to_gs_plane(x, y)
        a1, b1 = self.irsim_xy_to_gs_plane(x + math.cos(theta), y + math.sin(theta))
        da = a1 - a0
        db = b1 - b0

        xyz = {"x": 0.0, "y": 0.0, "z": 0.0}
        xyz[self.plane[0]] = a0
        xyz[self.plane[1]] = b0
        xyz[self.up] = self.floor + camera_height

        # Habitat camera convention: local -Z is forward.
        forward = {"x": 0.0, "y": 0.0, "z": 0.0}
        forward[self.plane[0]] = da
        forward[self.plane[1]] = db
        yaw = math.atan2(-forward["x"], -forward["z"])
        return [float(xyz["x"]), float(xyz["y"]), float(xyz["z"])], float(yaw)


def convert(args: argparse.Namespace) -> list[dict[str, Any]]:
    map_meta = load_yaml(args.map_yaml)
    world_meta = load_yaml(args.world) if args.world is not None else None
    mapper = IrsimToGsMapper(map_meta, world_meta)
    poses = []
    for idx, row in enumerate(iter_trace(args.trace)):
        state = row["state"]
        position, yaw = mapper.irsim_pose_to_gs(
            float(state[0]),
            float(state[1]),
            float(state[2]),
            float(args.camera_height),
        )
        poses.append(
            {
                "frame": idx,
                "t": float(row.get("step", idx)) * float(args.dt),
                "position": position,
                "yaw": yaw,
                "source_irsim_state": [float(state[0]), float(state[1]), float(state[2])],
            }
        )
    return poses


def save_outputs(poses: list[dict[str, Any]], args: argparse.Namespace) -> None:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    traj_path = args.output_dir / "gs_agent_trajectory.jsonl"
    with traj_path.open("w", encoding="utf-8") as f:
        for pose in poses:
            f.write(json.dumps(pose) + "\n")

    meta = {
        "source_trace": str(args.trace.resolve()),
        "source_map_yaml": str(args.map_yaml.resolve()),
        "source_world": str(args.world.resolve()) if args.world is not None else None,
        "dt": float(args.dt),
        "camera_height": float(args.camera_height),
        "num_frames": len(poses),
        "trajectory": str(traj_path.resolve()),
    }
    with (args.output_dir / "gs_agent_trajectory_metadata.json").open("w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    xs = [p["position"][0] for p in poses]
    zs = [p["position"][2] for p in poses]
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xs, zs, "r-", linewidth=1.5)
    if poses:
        ax.scatter([xs[0]], [zs[0]], c="green", s=60, label="start")
        ax.scatter([xs[-1]], [zs[-1]], c="blue", s=60, label="end")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("GS x")
    ax.set_ylabel("GS z")
    ax.set_title("IR-SIM trace mapped to GS x-z")
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / "gs_trajectory_debug.png", dpi=160)
    plt.close(fig)
    save_occupancy_overlay(poses, args)
    print(f"Wrote trajectory: {traj_path}")
    print(f"Wrote debug plot: {args.output_dir / 'gs_trajectory_debug.png'}")


def save_occupancy_overlay(poses: list[dict[str, Any]], args: argparse.Namespace) -> None:
    map_meta = load_yaml(args.map_yaml)
    world_meta = load_yaml(args.world) if args.world is not None else None
    mapper = IrsimToGsMapper(map_meta, world_meta)
    map_path = args.map_yaml.parent / map_meta.get("image", "map.png")
    image = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        return
    rows = []
    cols = []
    for pose in poses:
        x, y, _theta = pose["source_irsim_state"]
        row, col = mapper._output_pixel_from_irsim(float(x), float(y))
        rows.append(row)
        cols.append(col)
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(image, cmap="gray", origin="upper", interpolation="nearest")
    ax.plot(cols, rows, "r-", linewidth=1.2, label="IR-SIM trace projected to occupancy")
    if rows:
        ax.scatter([cols[0]], [rows[0]], c="green", s=40, label="start")
        ax.scatter([cols[-1]], [rows[-1]], c="blue", s=40, label="end")
    ax.set_title("IR-SIM trace projected back onto GS occupancy map")
    ax.set_xlabel("output map col")
    ax.set_ylabel("output map row")
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / "irsim_trace_on_occupancy.png", dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True, help="irsim_follow_trace.jsonl")
    parser.add_argument("--map_yaml", type=Path, required=True, help="GS occupancy map.yaml")
    parser.add_argument("--world", type=Path, default=None, help="IR-SIM world YAML used to generate the trace; improves mdownsample coordinate alignment.")
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--camera_height", type=float, default=1.5)
    args = parser.parse_args()
    poses = convert(args)
    if not poses:
        raise RuntimeError(f"No poses converted from {args.trace}")
    save_outputs(poses, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
