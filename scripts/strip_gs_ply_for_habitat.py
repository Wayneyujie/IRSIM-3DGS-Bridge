#!/usr/bin/env python3
"""Strip unsupported SH-rest fields from a Gaussian Splatting PLY for Habitat-GS."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from plyfile import PlyData, PlyElement


KEEP_FIELDS = [
    "x",
    "y",
    "z",
    "nx",
    "ny",
    "nz",
    "f_dc_0",
    "f_dc_1",
    "f_dc_2",
    "opacity",
    "scale_0",
    "scale_1",
    "scale_2",
    "rot_0",
    "rot_1",
    "rot_2",
    "rot_3",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    ply = PlyData.read(str(args.input), mmap=True)
    vertex = ply["vertex"].data
    names = set(vertex.dtype.names or [])
    missing = [name for name in KEEP_FIELDS if name not in names]
    if missing:
        raise ValueError(f"Input PLY is missing required fields: {missing}")

    dtype = [(name, np.float32) for name in KEEP_FIELDS]
    out = np.empty(vertex.shape[0], dtype=dtype)
    for name in KEEP_FIELDS:
        out[name] = vertex[name].astype(np.float32, copy=False)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    PlyData([PlyElement.describe(out, "vertex")], text=False).write(str(args.output))
    print(f"Wrote Habitat-GS-compatible PLY: {args.output}")
    print(f"vertices={len(out)} fields={','.join(KEEP_FIELDS)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
