#!/usr/bin/env python3
"""Convert 3D Gaussian Splatting PLY files to 2D occupancy maps.

This is a minimal center-projection pipeline:
  GS/point PLY -> height slicing -> x-y occupancy grid -> obstacle inflation.

Pixel encoding:
  0   occupied
  127 unknown
  255 free
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy import ndimage


OCCUPIED = 0
UNKNOWN = 127
FREE = 255

PLY_NUMPY_TYPES = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}

PLY_STRUCT_TYPES = {
    "char": "b",
    "int8": "b",
    "uchar": "B",
    "uint8": "B",
    "short": "h",
    "int16": "h",
    "ushort": "H",
    "uint16": "H",
    "int": "i",
    "int32": "i",
    "uint": "I",
    "uint32": "I",
    "float": "f",
    "float32": "f",
    "double": "d",
    "float64": "d",
}


@dataclass
class GaussianPoints:
    xyz: np.ndarray
    opacity: Optional[np.ndarray]
    scales: Optional[np.ndarray]
    properties: List[str]


@dataclass
class Bounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def width_m(self) -> float:
        return self.x_max - self.x_min

    @property
    def height_m(self) -> float:
        return self.y_max - self.y_min


@dataclass
class AxisConfig:
    plane_a: int
    plane_b: int
    up: int
    plane_a_name: str
    plane_b_name: str
    up_name: str


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50.0, 50.0)))


def parse_ply_header(path: Path) -> Tuple[str, int, List[Tuple[str, str]], int]:
    with path.open("rb") as f:
        first = f.readline().decode("ascii", errors="replace").strip()
        if first != "ply":
            raise ValueError(f"{path} is not a PLY file")
        fmt = ""
        vertex_count = 0
        vertex_props: List[Tuple[str, str]] = []
        in_vertex = False
        while True:
            line_b = f.readline()
            if not line_b:
                raise ValueError("PLY header ended unexpectedly")
            line = line_b.decode("ascii", errors="replace").strip()
            if line.startswith("format "):
                fmt = line.split()[1]
            elif line.startswith("element "):
                parts = line.split()
                in_vertex = parts[1] == "vertex"
                if in_vertex:
                    vertex_count = int(parts[2])
            elif in_vertex and line.startswith("property "):
                parts = line.split()
                if parts[1] == "list":
                    raise ValueError("List properties in vertex elements are not supported")
                vertex_props.append((parts[2], parts[1]))
            elif line == "end_header":
                return fmt, vertex_count, vertex_props, f.tell()


def load_gaussian_ply(path: str | Path) -> GaussianPoints:
    path = Path(path)
    fmt, vertex_count, vertex_props, data_offset = parse_ply_header(path)
    names = [name for name, _ in vertex_props]
    if not {"x", "y", "z"}.issubset(names):
        raise ValueError("PLY must contain x, y, z vertex properties")

    endian = "<" if fmt == "binary_little_endian" else ">" if fmt == "binary_big_endian" else None
    if endian is None:
        if fmt != "ascii":
            raise ValueError(f"Unsupported PLY format: {fmt}")
        rows = []
        with path.open("rb") as f:
            f.seek(data_offset)
            for _ in range(vertex_count):
                rows.append([float(v) for v in f.readline().decode("ascii", errors="replace").split()[: len(vertex_props)]])
        arr = np.asarray(rows, dtype=np.float32)
        columns = {name: arr[:, i] for i, (name, _) in enumerate(vertex_props)}
    else:
        dtype = np.dtype([(name, endian + PLY_NUMPY_TYPES[prop_type]) for name, prop_type in vertex_props])
        with path.open("rb") as f:
            f.seek(data_offset)
            data = np.fromfile(f, dtype=dtype, count=vertex_count)
        columns = {name: np.asarray(data[name]) for name in names}

    xyz = np.stack([columns["x"], columns["y"], columns["z"]], axis=1).astype(np.float32)
    opacity = columns.get("opacity")
    if opacity is not None:
        opacity = np.asarray(opacity, dtype=np.float32)
        if np.nanmin(opacity) < 0.0 or np.nanmax(opacity) > 1.0:
            opacity = sigmoid(opacity)

    scale_names = ["scale_0", "scale_1", "scale_2"]
    scales = None
    if all(name in columns for name in scale_names):
        scales = np.stack([columns[name] for name in scale_names], axis=1).astype(np.float32)

    valid = np.isfinite(xyz).all(axis=1)
    if opacity is not None:
        valid &= np.isfinite(opacity)
    xyz = xyz[valid]
    opacity = opacity[valid] if opacity is not None else None
    scales = scales[valid] if scales is not None else None
    return GaussianPoints(xyz=xyz, opacity=opacity, scales=scales, properties=names)


def infer_axes(points: np.ndarray, up_axis: str) -> AxisConfig:
    names = ["x", "y", "z"]
    if up_axis == "auto":
        spans = [np.percentile(points[:, i], 98.0) - np.percentile(points[:, i], 2.0) for i in range(3)]
        up = int(np.argmin(spans))
    else:
        up = names.index(up_axis)
    plane = [i for i in range(3) if i != up]
    return AxisConfig(
        plane_a=plane[0],
        plane_b=plane[1],
        up=up,
        plane_a_name=names[plane[0]],
        plane_b_name=names[plane[1]],
        up_name=names[up],
    )


def estimate_floor_z(points: np.ndarray, up_index: int, percentile: float = 2.0) -> float:
    z = points[:, up_index]
    low = np.percentile(z, percentile)
    high = np.percentile(z, min(percentile + 20.0, 100.0))
    band = z[(z >= low) & (z <= high)]
    if len(band) < 100:
        return float(low)
    hist, edges = np.histogram(band, bins=80)
    idx = int(np.argmax(hist))
    return float((edges[idx] + edges[idx + 1]) * 0.5)


def compute_bounds(points: np.ndarray, axes: AxisConfig, args: argparse.Namespace) -> Bounds:
    plane_a = points[:, axes.plane_a]
    plane_b = points[:, axes.plane_b]
    x_min = float(args.x_min) if args.x_min is not None else float(np.percentile(plane_a, args.bounds_percentile))
    x_max = float(args.x_max) if args.x_max is not None else float(np.percentile(plane_a, 100.0 - args.bounds_percentile))
    y_min = float(args.y_min) if args.y_min is not None else float(np.percentile(plane_b, args.bounds_percentile))
    y_max = float(args.y_max) if args.y_max is not None else float(np.percentile(plane_b, 100.0 - args.bounds_percentile))
    pad = float(args.bounds_padding)
    return Bounds(x_min=x_min - pad, x_max=x_max + pad, y_min=y_min - pad, y_max=y_max + pad)


def xy_to_grid(x: np.ndarray, y: np.ndarray, bounds: Bounds, resolution: float, height: int) -> Tuple[np.ndarray, np.ndarray]:
    col = np.floor((x - bounds.x_min) / resolution).astype(np.int32)
    row_from_bottom = np.floor((y - bounds.y_min) / resolution).astype(np.int32)
    row = height - 1 - row_from_bottom
    return row, col


def build_occupancy_grid(points: GaussianPoints, axes: AxisConfig, bounds: Bounds, floor_z: float, args: argparse.Namespace) -> Tuple[np.ndarray, Dict[str, Any]]:
    res = float(args.resolution)
    width = int(math.ceil(bounds.width_m / res))
    height = int(math.ceil(bounds.height_m / res))
    grid = np.full((height, width), UNKNOWN, dtype=np.uint8)

    xyz = points.xyz
    plane_a = xyz[:, axes.plane_a]
    plane_b = xyz[:, axes.plane_b]
    up = xyz[:, axes.up]
    mask = (
        (plane_a >= bounds.x_min)
        & (plane_a <= bounds.x_max)
        & (plane_b >= bounds.y_min)
        & (plane_b <= bounds.y_max)
    )
    if points.opacity is not None:
        mask &= points.opacity >= float(args.opacity_threshold)
    xyz = xyz[mask]
    plane_a = xyz[:, axes.plane_a]
    plane_b = xyz[:, axes.plane_b]
    z_rel = xyz[:, axes.up] - float(floor_z)
    rows, cols = xy_to_grid(plane_a, plane_b, bounds, res, height)
    in_grid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    rows, cols, z_rel = rows[in_grid], cols[in_grid], z_rel[in_grid]

    ground = (z_rel >= float(args.ground_min_height)) & (z_rel <= float(args.ground_max_height))
    obstacle = (z_rel >= float(args.obstacle_min_height)) & (z_rel <= float(args.obstacle_max_height))

    grid[rows[ground], cols[ground]] = FREE
    grid[rows[obstacle], cols[obstacle]] = OCCUPIED

    stats = {
        "input_points": int(len(points.xyz)),
        "points_in_bounds_after_opacity": int(len(xyz)),
        "ground_points": int(np.count_nonzero(ground)),
        "obstacle_points": int(np.count_nonzero(obstacle)),
        "width": int(width),
        "height": int(height),
    }
    return grid, stats


def inflate_obstacles(grid: np.ndarray, robot_radius: float, resolution: float) -> np.ndarray:
    radius_px = int(math.ceil(float(robot_radius) / float(resolution)))
    if radius_px <= 0:
        return grid.copy()
    yy, xx = np.ogrid[-radius_px : radius_px + 1, -radius_px : radius_px + 1]
    kernel = (xx * xx + yy * yy) <= radius_px * radius_px
    occupied = grid == OCCUPIED
    inflated = ndimage.binary_dilation(occupied, structure=kernel)
    out = grid.copy()
    out[inflated] = OCCUPIED
    return out


def remove_small_components(mask: np.ndarray, min_area: int) -> np.ndarray:
    labels, num = ndimage.label(mask)
    if num == 0:
        return mask
    counts = np.bincount(labels.ravel())
    remove = counts < int(min_area)
    remove[0] = False
    out = mask.copy()
    out[remove[labels]] = False
    return out


def keep_largest_component(mask: np.ndarray) -> np.ndarray:
    labels, num = ndimage.label(mask)
    if num == 0:
        return mask
    counts = np.bincount(labels.ravel())
    counts[0] = 0
    largest = int(np.argmax(counts))
    return labels == largest


def postprocess_grid(grid: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    out = grid.copy()
    if args.remove_small_obstacles:
        occupied = remove_small_components(out == OCCUPIED, int(args.min_obstacle_area))
        out[out == OCCUPIED] = UNKNOWN
        out[occupied] = OCCUPIED
    if args.fill_small_holes:
        occupied = ndimage.binary_closing(out == OCCUPIED, iterations=int(args.fill_small_holes_iterations))
        out[occupied] = OCCUPIED
    if args.keep_largest_free_component:
        free = keep_largest_component(out == FREE)
        out[out == FREE] = UNKNOWN
        out[free] = FREE
    return out


def save_png(path: Path, grid: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), grid)


def apply_image_transform(grid: np.ndarray, transform: str) -> np.ndarray:
    if transform == "identity":
        return grid.copy()
    if transform == "flip_x":
        return np.fliplr(grid)
    if transform == "flip_y":
        return np.flipud(grid)
    if transform == "rotate_180":
        return np.flipud(np.fliplr(grid))
    if transform == "transpose":
        return grid.T
    if transform == "transpose_rotate_180":
        return np.flipud(np.fliplr(grid.T))
    if transform == "transpose_flip_x":
        return np.fliplr(grid.T)
    if transform == "transpose_flip_y":
        return np.flipud(grid.T)
    raise ValueError(f"Unsupported image transform: {transform}")


def save_orientation_variants(output_dir: Path, grid: np.ndarray) -> None:
    variants = [
        "identity",
        "flip_x",
        "flip_y",
        "rotate_180",
        "transpose",
        "transpose_rotate_180",
        "transpose_flip_x",
        "transpose_flip_y",
    ]
    fig, plot_axes = plt.subplots(2, 4, figsize=(16, 8), dpi=140)
    for ax, name in zip(plot_axes.ravel(), variants):
        transformed = apply_image_transform(grid, name)
        ax.imshow(transformed, cmap="gray", vmin=0, vmax=255)
        ax.set_title(name)
        ax.axis("off")
        cv2.imwrite(str(output_dir / f"map_{name}.png"), transformed)
    fig.tight_layout()
    fig.savefig(output_dir / "orientation_variants.png")
    plt.close(fig)


def save_debug_topdown(
    path: Path,
    grid: np.ndarray,
    raw_grid: np.ndarray,
    points: GaussianPoints,
    floor_z: float,
    bounds: Bounds,
    args: argparse.Namespace,
    axis_config: AxisConfig,
) -> None:
    xyz = points.xyz
    z_rel = xyz[:, axis_config.up] - floor_z
    obstacle = (z_rel >= args.obstacle_min_height) & (z_rel <= args.obstacle_max_height)
    ground = (z_rel >= args.ground_min_height) & (z_rel <= args.ground_max_height)

    fig, plot_axes = plt.subplots(1, 3, figsize=(15, 5), dpi=160)
    plot_axes[0].imshow(raw_grid, cmap="gray", vmin=0, vmax=255)
    plot_axes[0].set_title("before inflation")
    plot_axes[1].imshow(grid, cmap="gray", vmin=0, vmax=255)
    plot_axes[1].set_title("after inflation")
    plot_axes[2].scatter(xyz[ground, axis_config.plane_a], xyz[ground, axis_config.plane_b], s=0.2, c="#2ca02c", label="ground")
    plot_axes[2].scatter(xyz[obstacle, axis_config.plane_a], xyz[obstacle, axis_config.plane_b], s=0.2, c="#d62728", label="obstacle")
    plot_axes[2].set_xlim(bounds.x_min, bounds.x_max)
    plot_axes[2].set_ylim(bounds.y_min, bounds.y_max)
    plot_axes[2].set_xlabel(axis_config.plane_a_name)
    plot_axes[2].set_ylabel(axis_config.plane_b_name)
    plot_axes[2].set_aspect("equal", adjustable="box")
    plot_axes[2].legend(markerscale=20, fontsize=7)
    plot_axes[2].set_title("raw projected points")
    for ax in plot_axes:
        ax.grid(False)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save_aux_debug(output_dir: Path, raw_grid: np.ndarray, inflated_grid: np.ndarray, points: GaussianPoints, floor_z: float, args: argparse.Namespace, axes: AxisConfig) -> None:
    save_png(output_dir / "occupancy_before_inflation.png", raw_grid)
    save_png(output_dir / "occupancy_after_inflation.png", inflated_grid)

    z = points.xyz[:, axes.up]
    fig, ax = plt.subplots(figsize=(8, 4), dpi=140)
    ax.hist(z, bins=120, color="#333333")
    ax.axvline(floor_z, color="red", label=f"floor_z={floor_z:.3f}")
    ax.set_xlabel(axes.up_name)
    ax.set_ylabel("count")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "height_histogram.png")
    plt.close(fig)


def save_outputs(grid: np.ndarray, raw_grid: np.ndarray, metadata: Dict[str, Any], output_dir: Path, points: GaussianPoints, floor_z: float, bounds: Bounds, args: argparse.Namespace, axes: AxisConfig) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    output_grid = apply_image_transform(grid, args.image_transform)
    output_raw_grid = apply_image_transform(raw_grid, args.image_transform)
    save_png(output_dir / "map.png", output_grid)
    with (output_dir / "map.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(metadata, f, sort_keys=False)
    with (output_dir / "map.json").open("w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)
    save_debug_topdown(output_dir / "debug_topdown.png", output_grid, output_raw_grid, points, floor_z, bounds, args, axes)
    save_aux_debug(output_dir, output_raw_grid, output_grid, points, floor_z, args, axes)
    if args.save_orientation_variants:
        save_orientation_variants(output_dir, grid)


def parse_floor_z(value: str, points: np.ndarray, axes: AxisConfig) -> float:
    if str(value).lower() == "auto":
        return estimate_floor_z(points, axes.up)
    return float(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert GS/point PLY to a 2D IR-SIM-style occupancy map.")
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output_dir", required=True, type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--robot_radius", type=float, default=0.25)
    parser.add_argument("--floor_z", default="auto")
    parser.add_argument("--up_axis", choices=["auto", "x", "y", "z"], default="auto")
    parser.add_argument(
        "--image_transform",
        choices=[
            "identity",
            "flip_x",
            "flip_y",
            "rotate_180",
            "transpose",
            "transpose_rotate_180",
            "transpose_flip_x",
            "transpose_flip_y",
        ],
        default="identity",
        help="Post-transform map image orientation to match viewer/top-down conventions.",
    )
    parser.add_argument("--save_orientation_variants", action="store_true")
    parser.add_argument("--ground_min_height", type=float, default=-0.05)
    parser.add_argument("--ground_max_height", type=float, default=0.15)
    parser.add_argument("--obstacle_min_height", type=float, default=0.20)
    parser.add_argument("--obstacle_max_height", type=float, default=1.50)
    parser.add_argument("--opacity_threshold", type=float, default=0.1)
    parser.add_argument("--x_min", type=float, default=None)
    parser.add_argument("--x_max", type=float, default=None)
    parser.add_argument("--y_min", type=float, default=None)
    parser.add_argument("--y_max", type=float, default=None)
    parser.add_argument("--bounds_padding", type=float, default=0.25)
    parser.add_argument("--bounds_percentile", type=float, default=0.5)
    parser.add_argument("--remove_small_obstacles", action="store_true")
    parser.add_argument("--min_obstacle_area", type=int, default=5)
    parser.add_argument("--fill_small_holes", action="store_true")
    parser.add_argument("--fill_small_holes_iterations", type=int, default=1)
    parser.add_argument("--keep_largest_free_component", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    points = load_gaussian_ply(args.input)
    axes = infer_axes(points.xyz, args.up_axis)
    floor_z = parse_floor_z(args.floor_z, points.xyz, axes)
    bounds = compute_bounds(points.xyz, axes, args)
    raw_grid, stats = build_occupancy_grid(points, axes, bounds, floor_z, args)
    inflated = inflate_obstacles(raw_grid, args.robot_radius, args.resolution)
    grid = postprocess_grid(inflated, args)
    output_grid = apply_image_transform(grid, args.image_transform)

    metadata = {
        "image": "map.png",
        "resolution": float(args.resolution),
        "origin": [float(bounds.x_min), float(bounds.y_min)],
        "width": int(output_grid.shape[1]),
        "height": int(output_grid.shape[0]),
        "occupied_value": OCCUPIED,
        "unknown_value": UNKNOWN,
        "free_value": FREE,
        "robot_radius": float(args.robot_radius),
        "floor_z": float(floor_z),
        "coordinate_frame": f"{axes.plane_a_name}-{axes.plane_b_name} topdown, {axes.up_name}-up",
        "image_transform": args.image_transform,
        "axes": {
            "plane": [axes.plane_a_name, axes.plane_b_name],
            "up": axes.up_name,
            "requested_up_axis": args.up_axis,
        },
        "bounds": {
            "x_min": float(bounds.x_min),
            "x_max": float(bounds.x_max),
            "y_min": float(bounds.y_min),
            "y_max": float(bounds.y_max),
        },
        "thresholds": {
            "ground_min_height": float(args.ground_min_height),
            "ground_max_height": float(args.ground_max_height),
            "obstacle_min_height": float(args.obstacle_min_height),
            "obstacle_max_height": float(args.obstacle_max_height),
            "opacity_threshold": float(args.opacity_threshold),
        },
        "stats": stats,
        "ply_properties": points.properties,
    }
    save_outputs(grid, raw_grid, metadata, args.output_dir, points, floor_z, bounds, args, axes)
    print(f"Wrote occupancy map to {args.output_dir}")
    print(f"up_axis={axes.up_name} plane={axes.plane_a_name}-{axes.plane_b_name}")
    print(f"floor_z={floor_z:.4f} size={grid.shape[1]}x{grid.shape[0]} resolution={args.resolution}")
    print(f"points: input={stats['input_points']} in_bounds={stats['points_in_bounds_after_opacity']} ground={stats['ground_points']} obstacle={stats['obstacle_points']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
