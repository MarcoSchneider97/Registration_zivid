from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

zivid = pytest.importorskip("zivid")
from zivid.experimental.toolbox.point_cloud_registration import local_point_cloud_registration

from stitching.stitch_point_clouds import load_unorganized_point_cloud_from_ply, stitch_point_clouds


def _stitching_data_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "test_data" / "stitching_ply"


def test_load_ply_to_unorganized_point_cloud(
    application,  # pylint: disable=unused-argument
):
    cloud = load_unorganized_point_cloud_from_ply(_stitching_data_dir() / "cloud_a.ply")
    assert isinstance(cloud, zivid.UnorganizedPointCloud)
    assert cloud.size == 8


def test_registration_between_ply_clouds_with_small_translation(
    application,  # pylint: disable=unused-argument
):
    target = load_unorganized_point_cloud_from_ply(_stitching_data_dir() / "cloud_a.ply")
    source = load_unorganized_point_cloud_from_ply(_stitching_data_dir() / "cloud_b_translated.ply")

    result = local_point_cloud_registration(target=target, source=source)

    assert result.converged()
    assert result.source_coverage() > 0.8
    assert result.root_mean_square_error() < 1.0

    transform = result.transform().to_matrix()
    assert np.isclose(transform[0, 3], -0.5, atol=0.25)


def test_merge_extend_produces_more_points_and_valid_metrics(
    application,  # pylint: disable=unused-argument
    tmp_path,
):
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    shutil.copy(_stitching_data_dir() / "cloud_a.ply", input_dir / "000.ply")
    shutil.copy(_stitching_data_dir() / "cloud_b_translated.ply", input_dir / "001.ply")

    merged, metrics = stitch_point_clouds(input_dir=input_dir, output_ply=tmp_path / "merged.ply")

    assert merged.size > 8
    assert len(metrics) == 1
    assert metrics[0]["source_coverage"] > 0.8
    assert metrics[0]["root_mean_square_error"] < 1.0


def test_stitch_script_integration_workflow(
    application,  # pylint: disable=unused-argument
    tmp_path,
):
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    shutil.copy(_stitching_data_dir() / "cloud_a.ply", input_dir / "a.ply")
    shutil.copy(_stitching_data_dir() / "cloud_b_translated.ply", input_dir / "b.ply")
    output_file = tmp_path / "out" / "stitched.ply"

    subprocess.run(
        [
            sys.executable,
            str(Path("stitching") / "stitch_point_clouds.py"),
            str(input_dir),
            str(output_file),
        ],
        check=True,
    )

    assert output_file.exists()


def test_stitching_fails_for_empty_directory(tmp_path):
    with pytest.raises(RuntimeError, match="No PLY files found"):
        stitch_point_clouds(input_dir=tmp_path, output_ply=tmp_path / "merged.ply")


def test_stitching_fails_for_invalid_ply(
    application,  # pylint: disable=unused-argument
    tmp_path,
):
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    shutil.copy(_stitching_data_dir() / "invalid.ply", input_dir / "invalid.ply")

    with pytest.raises(RuntimeError):
        stitch_point_clouds(input_dir=input_dir, output_ply=tmp_path / "merged.ply")


def test_stitching_fails_for_too_little_overlap(
    application,  # pylint: disable=unused-argument
    tmp_path,
):
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    shutil.copy(_stitching_data_dir() / "cloud_a.ply", input_dir / "000.ply")
    shutil.copy(_stitching_data_dir() / "cloud_far.ply", input_dir / "001.ply")

    with pytest.raises(RuntimeError, match="Insufficient overlap"):
        stitch_point_clouds(input_dir=input_dir, output_ply=tmp_path / "merged.ply", min_source_coverage=0.8)
