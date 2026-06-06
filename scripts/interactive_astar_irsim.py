#!/usr/bin/env python3
"""Click start/goal on an IR-SIM obstacle map, plan A*, and optionally follow it."""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import os
import sys
from collections import deque
from pathlib import Path
from typing import Iterable

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy import ndimage


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def save_yaml(data: dict, path: Path) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class GridMap:
    def __init__(
        self,
        image: np.ndarray,
        world_width: float,
        world_height: float,
        planning_inflation_radius: float = 0.0,
    ) -> None:
        self.image = image
        raw_free = image > 127
        self.height, self.width = raw_free.shape
        self.world_width = float(world_width)
        self.world_height = float(world_height)
        self.res_x = self.world_width / float(self.width)
        self.res_y = self.world_height / float(self.height)
        if abs(self.res_x - self.res_y) > max(self.res_x, self.res_y) * 0.05:
            print(f"[WARN] non-square display cells: res_x={self.res_x:.4f}, res_y={self.res_y:.4f}")
        self.resolution = 0.5 * (self.res_x + self.res_y)
        self.free = self._inflate_obstacles(raw_free, float(planning_inflation_radius))

    def _inflate_obstacles(self, raw_free: np.ndarray, radius_m: float) -> np.ndarray:
        if radius_m <= 0.0:
            return raw_free
        radius_px = max(1, int(math.ceil(radius_m / self.resolution)))
        obstacle = ~raw_free
        structure = ndimage.generate_binary_structure(2, 2)
        inflated_obstacle = ndimage.binary_dilation(obstacle, structure=structure, iterations=radius_px)
        free = raw_free & ~inflated_obstacle
        print(
            f"[INFO] Planning obstacle inflation: radius={radius_m:.3f}m "
            f"({radius_px}px at {self.resolution:.3f}m/cell), "
            f"free_ratio {raw_free.mean():.3f}->{free.mean():.3f}"
        )
        return free

    def xy_to_rc(self, x: float, y: float) -> tuple[int, int]:
        col = int(round(x / self.res_x))
        row = int(round(self.height - 1 - y / self.res_y))
        return row, col

    def rc_to_xy(self, row: int, col: int) -> tuple[float, float]:
        x = (float(col) + 0.5) * self.res_x
        y = (float(self.height - row) - 0.5) * self.res_y
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width


def nearest_free(grid: GridMap, rc: tuple[int, int], max_radius: int = 250) -> tuple[int, int]:
    row0, col0 = rc
    if grid.in_bounds(row0, col0) and grid.free[row0, col0]:
        return rc
    queue: deque[tuple[int, int]] = deque([rc])
    seen = {rc}
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while queue:
        row, col = queue.popleft()
        if abs(row - row0) > max_radius or abs(col - col0) > max_radius:
            continue
        if grid.in_bounds(row, col) and grid.free[row, col]:
            return row, col
        for drow, dcol in dirs:
            nxt = (row + drow, col + dcol)
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    raise RuntimeError(f"Could not find a free cell near {rc}")


def astar(grid: GridMap, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    motions = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    ]
    open_heap: list[tuple[float, float, tuple[int, int], tuple[int, int] | None]] = []
    heapq.heappush(open_heap, (0.0, 0.0, start, None))
    cost = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int] | None] = {}
    closed: set[tuple[int, int]] = set()
    while open_heap:
        _priority, g_cost, current, prev = heapq.heappop(open_heap)
        if current in closed:
            continue
        parent[current] = prev
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]
        closed.add(current)
        row, col = current
        for drow, dcol, step_cost in motions:
            nxt = (row + drow, col + dcol)
            if not grid.in_bounds(*nxt) or not grid.free[nxt]:
                continue
            new_cost = g_cost + step_cost
            if new_cost >= cost.get(nxt, float("inf")):
                continue
            cost[nxt] = new_cost
            heuristic = math.hypot(nxt[0] - goal[0], nxt[1] - goal[1])
            heapq.heappush(open_heap, (new_cost + heuristic, new_cost, nxt, current))
    raise RuntimeError("A* failed: no path found")


def sparsify_path(points: list[tuple[float, float]], min_spacing: float) -> list[tuple[float, float]]:
    if not points:
        return []
    sparse = [points[0]]
    for point in points[1:-1]:
        if math.hypot(point[0] - sparse[-1][0], point[1] - sparse[-1][1]) >= min_spacing:
            sparse.append(point)
    if sparse[-1] != points[-1]:
        sparse.append(points[-1])
    return sparse


def parse_pose(text: str | None) -> tuple[float, float] | None:
    if text is None:
        return None
    vals = [float(x.strip()) for x in text.split(",")]
    if len(vals) < 2:
        raise ValueError("Pose must be x,y or x,y,theta")
    return vals[0], vals[1]


def click_start_goal(grid: GridMap, default_start: tuple[float, float] | None, default_goal: tuple[float, float] | None) -> tuple[tuple[float, float], tuple[float, float]]:
    if default_start is not None and default_goal is not None:
        return default_start, default_goal

    selected: list[tuple[float, float]] = []
    markers = []
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(
        np.flipud(grid.image),
        cmap="gray",
        origin="lower",
        extent=(0.0, grid.world_width, 0.0, grid.world_height),
        interpolation="nearest",
    )
    ax.set_title("Left click: start, then goal. Key c: clear. Close window when done.")
    ax.set_xlabel("IR-SIM x [m]")
    ax.set_ylabel("IR-SIM y [m]")

    def redraw() -> None:
        nonlocal markers
        for artist in markers:
            artist.remove()
        markers = []
        if len(selected) >= 1:
            markers.extend(ax.plot(selected[0][0], selected[0][1], "go", markersize=8, label="start"))
        if len(selected) >= 2:
            markers.extend(ax.plot(selected[1][0], selected[1][1], "bo", markersize=8, label="goal"))
        fig.canvas.draw_idle()

    def on_click(event) -> None:
        if event.inaxes != ax or event.button != 1 or event.xdata is None or event.ydata is None:
            return
        if len(selected) >= 2:
            selected.clear()
        selected.append((float(event.xdata), float(event.ydata)))
        redraw()
        if len(selected) == 2:
            plt.close(fig)

    def on_key(event) -> None:
        if event.key == "c":
            selected.clear()
            redraw()

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.show()

    if default_start is not None:
        selected.insert(0, default_start)
    if default_goal is not None:
        selected.append(default_goal)
    if len(selected) < 2:
        raise RuntimeError("Need start and goal. Re-run and click two free-space points.")
    return selected[0], selected[1]


def write_path(path_xy: list[tuple[float, float]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "astar_path.json").open("w", encoding="utf-8") as f:
        json.dump({"path_xy": path_xy}, f, indent=2)
    with (output_dir / "astar_path.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        writer.writerows(path_xy)


def save_debug_plot(grid: GridMap, path_xy: list[tuple[float, float]], start_xy: tuple[float, float], goal_xy: tuple[float, float], output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(
        np.flipud(grid.image),
        cmap="gray",
        origin="lower",
        extent=(0.0, grid.world_width, 0.0, grid.world_height),
        interpolation="nearest",
    )
    xs = [p[0] for p in path_xy]
    ys = [p[1] for p in path_xy]
    ax.plot(xs, ys, "r-", linewidth=1.5)
    ax.plot(start_xy[0], start_xy[1], "go", markersize=8)
    ax.plot(goal_xy[0], goal_xy[1], "bo", markersize=8)
    ax.set_title("A* path on IR-SIM obstacle_map")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(output_dir / "astar_debug.png", dpi=160)
    plt.close(fig)


def run_irsim_follow(args: argparse.Namespace, path_xy: list[tuple[float, float]], start_xy: tuple[float, float], goal_xy: tuple[float, float], output_dir: Path) -> None:
    sys.path.insert(0, str(args.irsim_root))
    import irsim  # type: ignore

    cfg = load_yaml(args.world)
    cfg["robot"][0]["state"] = [float(start_xy[0]), float(start_xy[1]), 0.0]
    cfg["robot"][0]["goal"] = [float(goal_xy[0]), float(goal_xy[1]), 0.0]
    tmp_world = output_dir / "clicked_start_goal_world.yaml"
    save_yaml(cfg, tmp_world)

    env = irsim.make(str(tmp_world), display=args.display, save_ani=False, full=False)
    trajectory = np.asarray(path_xy, dtype=np.float32).T
    if trajectory.size:
        env.draw_trajectory(trajectory, traj_type="r-")

    waypoint_id = 0
    trace_path = output_dir / "irsim_follow_trace.jsonl"
    with trace_path.open("w", encoding="utf-8") as trace_file:
        for step in range(args.max_steps):
            state = env.robot.state.flatten()
            x, y, theta = float(state[0]), float(state[1]), float(state[2])
            while waypoint_id < len(path_xy) - 1 and math.hypot(path_xy[waypoint_id][0] - x, path_xy[waypoint_id][1] - y) < args.waypoint_threshold:
                waypoint_id += 1
            tx, ty = path_xy[waypoint_id]
            target_heading = math.atan2(ty - y, tx - x)
            heading_error = wrap_to_pi(target_heading - theta)
            distance = math.hypot(tx - x, ty - y)
            angular = float(np.clip(args.k_angular * heading_error, -args.max_angular, args.max_angular))
            linear = args.max_linear * max(0.0, math.cos(heading_error))
            linear = min(linear, args.k_linear * distance)
            if abs(heading_error) > args.turn_in_place_angle:
                linear = 0.0
            if waypoint_id >= len(path_xy) - 1 and distance < args.goal_threshold:
                linear = 0.0
                angular = 0.0
                env.step(np.array([[linear], [angular]], dtype=float))
                break
            env.step(np.array([[linear], [angular]], dtype=float))
            if args.display:
                env.render()
            trace_file.write(
                json.dumps(
                    {
                        "step": step,
                        "state": [x, y, theta],
                        "target": [tx, ty],
                        "waypoint_id": waypoint_id,
                        "action": [linear, angular],
                    }
                )
                + "\n"
            )
            trace_file.flush()
            if env.done():
                break
    env.end(rm_fig_path=False)
    print(f"Wrote IR-SIM follow trace: {trace_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True, help="IR-SIM YAML generated from occupancy map")
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--start", default=None, help="Optional x,y. If omitted, click on the map.")
    parser.add_argument("--goal", default=None, help="Optional x,y. If omitted, click on the map.")
    parser.add_argument("--path_spacing", type=float, default=0.8)
    parser.add_argument(
        "--planning-inflation-radius",
        type=float,
        default=0.0,
        help="Extra obstacle inflation for A* only, in meters. Try 0.4-0.8 if the follower clips obstacles.",
    )
    parser.add_argument("--follow", action="store_true", help="Run IR-SIM and follow the planned waypoints")
    parser.add_argument("--display", action="store_true", help="Show IR-SIM while following")
    parser.add_argument(
        "--irsim_root",
        type=Path,
        default=Path(os.environ.get("IRSIM_ROOT", "")) if os.environ.get("IRSIM_ROOT") else None,
        help="Path to the IR-SIM repository root. Defaults to $IRSIM_ROOT when set.",
    )
    parser.add_argument("--max_steps", type=int, default=5000)
    parser.add_argument("--waypoint_threshold", type=float, default=0.45)
    parser.add_argument("--goal_threshold", type=float, default=0.5)
    parser.add_argument("--max_linear", type=float, default=0.8)
    parser.add_argument("--max_angular", type=float, default=1.2)
    parser.add_argument("--k_linear", type=float, default=1.2)
    parser.add_argument("--k_angular", type=float, default=2.0)
    parser.add_argument("--turn_in_place_angle", type=float, default=0.75)
    args = parser.parse_args()
    if args.follow and args.irsim_root is None:
        parser.error("--follow requires --irsim_root or the IRSIM_ROOT environment variable")

    cfg = load_yaml(args.world)
    world = cfg["world"]
    map_path = Path(world["obstacle_map"])
    image = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(map_path)
    mdownsample = int(world.get("mdownsample", 1))
    image = image[::mdownsample, ::mdownsample]
    grid = GridMap(
        image,
        float(world["width"]),
        float(world["height"]),
        planning_inflation_radius=float(args.planning_inflation_radius),
    )

    start_xy, goal_xy = click_start_goal(grid, parse_pose(args.start), parse_pose(args.goal))
    start_rc = nearest_free(grid, grid.xy_to_rc(*start_xy))
    goal_rc = nearest_free(grid, grid.xy_to_rc(*goal_xy))
    path_rc = astar(grid, start_rc, goal_rc)
    dense_path_xy = [grid.rc_to_xy(row, col) for row, col in path_rc]
    path_xy = sparsify_path(dense_path_xy, args.path_spacing)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_path(path_xy, args.output_dir)
    save_debug_plot(grid, path_xy, grid.rc_to_xy(*start_rc), grid.rc_to_xy(*goal_rc), args.output_dir)
    path_len = sum(math.hypot(path_xy[i][0] - path_xy[i - 1][0], path_xy[i][1] - path_xy[i - 1][1]) for i in range(1, len(path_xy)))
    print(f"A*: success dense_nodes={len(path_rc)} waypoints={len(path_xy)} length={path_len:.3f}m")
    print(f"Wrote: {args.output_dir / 'astar_debug.png'}")
    print(f"Wrote: {args.output_dir / 'astar_path.json'}")

    if args.follow:
        run_irsim_follow(args, path_xy, grid.rc_to_xy(*start_rc), grid.rc_to_xy(*goal_rc), args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
