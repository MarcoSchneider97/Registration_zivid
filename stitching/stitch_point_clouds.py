#!/usr/bin/env python3
"""Iterative stitching of PLY point clouds with Zivid local point cloud registration."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import zivid
from zivid.experimental.point_cloud_export import export_unorganized_point_cloud
from zivid.experimental.point_cloud_export.file_format import PLY
from zivid.experimental.toolbox.point_cloud_registration import local_point_cloud_registration
from zivid._local_point_cloud_registration_parameters import LocalPointCloudRegistrationParameters


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATA_DIR = SCRIPT_DIR / "data"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "output"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stitch sorted PLY clouds in stitching/data into one point cloud")
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help=f"Directory containing *.ply inputs (default: {DEFAULT_DATA_DIR})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory for results (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("stitched.ply"),
        help="Output PLY file name or path (default: stitched.ply)",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=None,
        help="Optional voxel size for downsampling before registration (same unit as point cloud coordinates)",
    )
    parser.add_argument(
        "--min-points-per-voxel",
        type=int,
        default=1,
        help="Minimum points per voxel for downsampling (default: 1)",
    )
    parser.add_argument(
        "--max-correspondence-distance",
        type=float,
        default=None,
        help="Override registration max correspondence distance",
    )
    parser.add_argument(
        "--max-iteration-count",
        type=int,
        default=None,
        help="Override registration max iteration count",
    )
    return parser.parse_args()


def _load_unorganized_point_cloud_ply(path: Path) -> zivid.UnorganizedPointCloud:
    """Load a PLY into UnorganizedPointCloud using the dedicated loader when available."""
    # Newer APIs may provide a direct PLY loader for UnorganizedPointCloud.
    load_method = getattr(zivid.UnorganizedPointCloud, "load", None)
    if callable(load_method):
        return zivid.UnorganizedPointCloud.load(path)

    # Fallback for environments where only frame loading from PLY is available.
    with zivid.Frame(path) as frame:
        return frame.point_cloud().to_unorganized_point_cloud()


def _prepare_registration_cloud(
    cloud: zivid.UnorganizedPointCloud, voxel_size: float | None, min_points_per_voxel: int
) -> zivid.UnorganizedPointCloud:
    if voxel_size is None:
        return cloud
    return cloud.voxel_downsampled(voxel_size=voxel_size, min_points_per_voxel=min_points_per_voxel)


def _resolve_output_path(output_dir: Path, output: Path) -> Path:
    if output.is_absolute():
        return output
    return output_dir / output


def main() -> int:
    args = _parse_args()

    if args.voxel_size is not None and args.voxel_size <= 0:
        raise ValueError("--voxel-size must be > 0 when provided")
    if args.min_points_per_voxel < 1:
        raise ValueError("--min-points-per-voxel must be >= 1")

    data_dir = args.data_dir
    output_dir = args.output_dir
    transforms_dir = output_dir / "transforms"
    output_dir.mkdir(parents=True, exist_ok=True)
    transforms_dir.mkdir(parents=True, exist_ok=True)

    ply_paths = sorted(data_dir.glob("*.ply"))
    if len(ply_paths) < 2:
        raise RuntimeError(f"Expected at least 2 PLY files in {data_dir}, found {len(ply_paths)}")

    registration_params = LocalPointCloudRegistrationParameters()
    if args.max_correspondence_distance is not None:
        registration_params.max_correspondence_distance = args.max_correspondence_distance
    if args.max_iteration_count is not None:
        registration_params.max_iteration_count = args.max_iteration_count

    print(f"Loading target cloud: {ply_paths[0].name}")
    stitched_target = _load_unorganized_point_cloud_ply(ply_paths[0])

    for step_idx, source_path in enumerate(ply_paths[1:], start=1):
        print(f"[{step_idx}/{len(ply_paths) - 1}] Registering source cloud: {source_path.name}")
        source_full = _load_unorganized_point_cloud_ply(source_path)

        target_for_registration = _prepare_registration_cloud(
            stitched_target,
            voxel_size=args.voxel_size,
            min_points_per_voxel=args.min_points_per_voxel,
        )
        source_for_registration = _prepare_registration_cloud(
            source_full,
            voxel_size=args.voxel_size,
            min_points_per_voxel=args.min_points_per_voxel,
        )

        registration_result = local_point_cloud_registration(
            target=target_for_registration,
            source=source_for_registration,
            parameters=registration_params,
        )

        transform_matrix = np.asarray(registration_result.transform().to_matrix())
        transform_csv_path = transforms_dir / f"transform_{step_idx:03d}.csv"
        np.savetxt(transform_csv_path, transform_matrix, delimiter=",")

        source_full.transform(transform_matrix)
        stitched_target.extend(source_full)

        print(
            "  converged={converged} rmse={rmse:.6f} coverage={coverage:.4f} transform={transform}".format(
                converged=registration_result.converged(),
                rmse=registration_result.root_mean_square_error(),
                coverage=registration_result.source_coverage(),
                transform=transform_csv_path,
            )
        )

    output_path = _resolve_output_path(output_dir, args.output)
    export_unorganized_point_cloud(
        stitched_target,
        PLY(str(output_path), layout=PLY.Layout.unordered),
    )

    print(f"Saved stitched cloud: {output_path}")
    print(f"Saved transforms in: {transforms_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
