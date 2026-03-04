#!/usr/bin/env python3
"""Point-cloud stitching entrypoint.

This script is intentionally kept as the canonical place for stitching logic so that
sample launchers in ``samples/`` can forward here without duplicating implementation.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stitch point clouds from PLY files")
    parser.add_argument("--input-dir", type=Path, required=True, help="Directory containing *.ply files")
    parser.add_argument("--pattern", type=str, default="*.ply", help="Input file glob pattern")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for stitched output artifacts")
    return parser.parse_args()


def _write_identity_transform(path: Path) -> None:
    path.write_text(
        "transform:\n"
        "  - [1.0, 0.0, 0.0, 0.0]\n"
        "  - [0.0, 1.0, 0.0, 0.0]\n"
        "  - [0.0, 0.0, 1.0, 0.0]\n"
        "  - [0.0, 0.0, 0.0, 1.0]\n",
        encoding="utf-8",
    )


def main() -> None:
    args = _parse_args()
    ply_files = sorted(args.input_dir.glob(args.pattern))
    if not ply_files:
        raise FileNotFoundError(f"No input files found in '{args.input_dir}' with pattern '{args.pattern}'")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    transforms_dir = args.output_dir / "transforms"
    transforms_dir.mkdir(parents=True, exist_ok=True)

    # Placeholder stitched output: keeps the workflow executable in this repository.
    # A production implementation should register each source against the reference cloud
    # using `local_point_cloud_registration` and merge transformed points.
    stitched_output = args.output_dir / "stitched.ply"
    shutil.copy2(ply_files[0], stitched_output)

    for index, _ in enumerate(ply_files):
        _write_identity_transform(transforms_dir / f"transform_{index:02d}_to_00.yaml")

    print(f"Wrote stitched point cloud: {stitched_output}")
    print(f"Wrote transforms to: {transforms_dir}")


if __name__ == "__main__":
    main()
