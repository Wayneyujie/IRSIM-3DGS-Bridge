#!/usr/bin/env python3
"""Continuously convert a growing IR-SIM trace JSONL into a GS trajectory JSONL."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from convert_irsim_trace_to_gs_trajectory import IrsimToGsMapper, load_yaml


def read_existing_count(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", encoding="utf-8") as f:
        return sum(1 for line in f if line.strip())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True, help="Growing irsim_follow_trace.jsonl")
    parser.add_argument("--map_yaml", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Output GS trajectory JSONL")
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--camera_height", type=float, default=1.5)
    parser.add_argument("--poll", type=float, default=0.05)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.overwrite and args.output.exists():
        args.output.unlink()

    converted = 0 if args.overwrite else read_existing_count(args.output)
    print(f"[watch] trace={args.trace}")
    print(f"[watch] world={args.world}")
    print(f"[watch] output={args.output}")
    print(f"[watch] starting at converted={converted}")

    mapper = None
    world_mtime = None
    trace_offset = 0
    trace_size = 0
    frame = converted
    waiting_world_logged = False
    waiting_trace_logged = False

    while True:
        if not args.world.exists():
            if not waiting_world_logged:
                print(f"[watch] waiting for world file: {args.world}")
                waiting_world_logged = True
            time.sleep(args.poll)
            continue
        current_world_mtime = args.world.stat().st_mtime
        if mapper is None or current_world_mtime != world_mtime:
            mapper = IrsimToGsMapper(load_yaml(args.map_yaml), load_yaml(args.world))
            world_mtime = current_world_mtime
            print(f"[watch] loaded world mapping: {args.world}")

        if not args.trace.exists():
            if not waiting_trace_logged:
                print(f"[watch] waiting for trace file: {args.trace}")
                waiting_trace_logged = True
            time.sleep(args.poll)
            continue

        current_size = args.trace.stat().st_size
        if current_size < trace_offset:
            print("[watch] trace was truncated/restarted; resetting conversion")
            trace_offset = 0
            trace_size = 0
            frame = 0
            args.output.write_text("", encoding="utf-8")
        if current_size == trace_offset:
            time.sleep(args.poll)
            continue

        with args.trace.open("r", encoding="utf-8") as src, args.output.open("a", encoding="utf-8") as dst:
            src.seek(trace_offset)
            while True:
                line = src.readline()
                if not line:
                    trace_offset = src.tell()
                    break
                line = line.strip()
                if not line:
                    continue
                row = json.loads(line)
                state = row["state"]
                position, yaw = mapper.irsim_pose_to_gs(
                    float(state[0]),
                    float(state[1]),
                    float(state[2]),
                    float(args.camera_height),
                )
                pose = {
                    "frame": frame,
                    "t": float(row.get("step", frame)) * float(args.dt),
                    "position": position,
                    "yaw": yaw,
                    "source_irsim_state": [float(state[0]), float(state[1]), float(state[2])],
                }
                dst.write(json.dumps(pose) + "\n")
                frame += 1
            dst.flush()
        trace_size = current_size


if __name__ == "__main__":
    raise SystemExit(main())
