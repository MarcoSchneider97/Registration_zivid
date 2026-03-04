"""Utility for stitching multiple PLY point clouds into one cloud."""

from __future__ import annotations

import argparse
from pathlib import Path

import zivid
from zivid.experimental.point_cloud_export import export_unorganized_point_cloud
from zivid.experimental.point_cloud_export.file_format import PLY
from zivid.experimental.toolbox.point_cloud_registration import local_point_cloud_registration


def load_unorganized_point_cloud_from_ply(ply_path: Path) -> zivid.UnorganizedPointCloud:
    """Load a PLY file and convert it to an unorganized point cloud."""
    with zivid.Frame(ply_path) as frame:
        return frame.point_cloud().to_unorganized_point_cloud().clone()


def stitch_point_clouds(
    input_dir: Path,
    output_ply: Path,
    *,
    min_source_coverage: float = 0.5,
    max_root_mean_square_error: float = 5.0,
) -> tuple[zivid.UnorganizedPointCloud, list[dict[str, float]]]:
    """Stitch all PLY files in ``input_dir`` and save merged cloud to ``output_ply``."""
    ply_files = sorted(input_dir.glob("*.ply"))
    if not ply_files:
        raise RuntimeError(f"No PLY files found in directory: {input_dir}")

    merged = load_unorganized_point_cloud_from_ply(ply_files[0])
    metrics: list[dict[str, float]] = []

    for ply_file in ply_files[1:]:
        source = load_unorganized_point_cloud_from_ply(ply_file)
        result = local_point_cloud_registration(target=merged, source=source)

        if result.source_coverage() < min_source_coverage:
            raise RuntimeError(
                f"Insufficient overlap for {ply_file.name}: source_coverage={result.source_coverage():.6f}"
            )
        if result.root_mean_square_error() > max_root_mean_square_error:
            raise RuntimeError(
                f"Registration RMSE too high for {ply_file.name}: rmse={result.root_mean_square_error():.6f}"
            )

        merged.extend(source.transformed(result.transform().to_matrix()))
        metrics.append(
            {
                "source_coverage": result.source_coverage(),
                "root_mean_square_error": result.root_mean_square_error(),
            }
        )

    output_ply.parent.mkdir(parents=True, exist_ok=True)
    export_unorganized_point_cloud(merged, PLY(str(output_ply), layout=PLY.Layout.unordered))
    return merged, metrics


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stitch PLY point clouds")
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("output_ply", type=Path)
    parser.add_argument("--min-source-coverage", type=float, default=0.5)
    parser.add_argument("--max-rmse", type=float, default=5.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    stitch_point_clouds(
        input_dir=args.input_dir,
        output_ply=args.output_ply,
        min_source_coverage=args.min_source_coverage,
        max_root_mean_square_error=args.max_rmse,
    )


if __name__ == "__main__":
    main()
