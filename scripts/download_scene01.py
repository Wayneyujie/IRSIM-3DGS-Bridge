#!/usr/bin/env python3
"""Download the public scene01 Gaussian Splatting sample used by the bridge demo."""

from __future__ import annotations

import argparse
from pathlib import Path

from huggingface_hub import snapshot_download


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output_dir", type=Path, required=True, help="Destination directory for gs_scenes")
    args = parser.parse_args()

    snapshot_download(
        repo_id="RukawaY/gs_scenes",
        repo_type="dataset",
        local_dir=str(args.output_dir),
        allow_patterns=[
            "train.scene_dataset_config.json",
            "train/scene01/**",
        ],
    )
    print(f"Downloaded scene01 into: {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
