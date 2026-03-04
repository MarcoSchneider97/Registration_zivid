"""Helpers for loading and saving unorganized point clouds as PLY files."""

from pathlib import Path

import numpy
from zivid.experimental.point_cloud_export import export_unorganized_point_cloud
from zivid.experimental.point_cloud_export.file_format import PLY
from zivid.unorganized_point_cloud import UnorganizedPointCloud, _from_numpy_arrays


def _validate_path(path):
    if not isinstance(path, (str, Path)):
        raise TypeError(
            "Unsupported type for argument path. Got {}, expected {} or {}.".format(
                type(path), str.__name__, Path.__name__
            )
        )

    path = Path(path)
    if not path.exists():
        raise ValueError("The file does not exist: {}".format(path))
    if not path.is_file():
        raise ValueError("The path does not refer to a file: {}".format(path))
    return path


def _ensure_xyz(xyz):
    xyz = numpy.asarray(xyz, dtype=numpy.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("Expected xyz to have shape (N, 3), got {}".format(xyz.shape))
    if xyz.shape[0] == 0:
        raise ValueError("Point cloud must contain at least one point")
    if not numpy.isfinite(xyz).all():
        raise ValueError("Point cloud xyz values must be finite")
    return xyz


def _ensure_rgba(rgba, point_count):
    if rgba is None:
        return None

    rgba = numpy.asarray(rgba)
    if rgba.ndim != 2 or rgba.shape[1] != 4:
        raise ValueError("Expected rgba to have shape (N, 4), got {}".format(rgba.shape))
    if rgba.shape[0] != point_count:
        raise ValueError(
            "Expected rgba point count to match xyz. Got {}, expected {}".format(rgba.shape[0], point_count)
        )

    if numpy.issubdtype(rgba.dtype, numpy.floating):
        if numpy.any((rgba < 0.0) | (rgba > 1.0)):
            raise ValueError("Floating-point color values must be in range [0.0, 1.0]")
        rgba = numpy.rint(rgba * 255.0).astype(numpy.uint8)
    else:
        if numpy.any((rgba < 0) | (rgba > 255)):
            raise ValueError("Integer color values must be in range [0, 255]")
        rgba = rgba.astype(numpy.uint8)

    return rgba


def _load_with_open3d(path):
    try:
        import open3d
    except ImportError:
        return None, None

    point_cloud = open3d.io.read_point_cloud(str(path))
    xyz = numpy.asarray(point_cloud.points, dtype=numpy.float32)

    rgba = None
    if point_cloud.has_colors():
        rgb = numpy.asarray(point_cloud.colors)
        alpha = numpy.full((rgb.shape[0], 1), 255.0, dtype=rgb.dtype)
        rgba = numpy.concatenate((rgb, alpha), axis=1)

    return xyz, rgba


def _load_with_ascii_ply_parser(path):
    with path.open("r", encoding="utf-8") as file:
        line = file.readline().strip()
        if line != "ply":
            raise ValueError("Unsupported PLY file, expected magic header 'ply'")

        if file.readline().strip() != "format ascii 1.0":
            raise ValueError("Only ASCII PLY files are supported without Open3D")

        vertex_count = None
        properties = []

        while True:
            line = file.readline()
            if not line:
                raise ValueError("Unexpected end-of-file while reading PLY header")
            line = line.strip()
            if line.startswith("element vertex "):
                vertex_count = int(line.split()[-1])
            elif line.startswith("property ") and vertex_count is not None:
                properties.append(line.split()[-1])
            elif line == "end_header":
                break

        if vertex_count is None:
            raise ValueError("Could not find vertex element in PLY header")

        for property_name in ("x", "y", "z"):
            if property_name not in properties:
                raise ValueError("Missing required '{}' property in PLY vertex element".format(property_name))

        xyz = numpy.empty((vertex_count, 3), dtype=numpy.float32)
        rgba = numpy.empty((vertex_count, 4), dtype=numpy.uint8)

        color_keys = ("red", "green", "blue", "alpha")
        has_rgb = all(color in properties for color in color_keys[:3])
        has_alpha = color_keys[3] in properties
        property_index = {name: index for index, name in enumerate(properties)}

        for row in range(vertex_count):
            values = file.readline().split()
            if len(values) < len(properties):
                raise ValueError("Invalid PLY vertex row at index {}".format(row))

            xyz[row, 0] = float(values[property_index["x"]])
            xyz[row, 1] = float(values[property_index["y"]])
            xyz[row, 2] = float(values[property_index["z"]])

            if has_rgb:
                rgba[row, 0] = int(values[property_index["red"]])
                rgba[row, 1] = int(values[property_index["green"]])
                rgba[row, 2] = int(values[property_index["blue"]])
                rgba[row, 3] = int(values[property_index["alpha"]]) if has_alpha else 255

    return xyz, rgba if has_rgb else None


def load_unorganized_point_cloud_from_ply(path):
    """Load a PLY file into an UnorganizedPointCloud.

    The loader first attempts Open3D, then falls back to an ASCII PLY parser.
    """

    path = _validate_path(path)

    xyz, rgba = _load_with_open3d(path)
    if xyz is None:
        xyz, rgba = _load_with_ascii_ply_parser(path)

    xyz = _ensure_xyz(xyz)
    rgba = _ensure_rgba(rgba, xyz.shape[0])

    return _from_numpy_arrays(xyz=xyz, rgba=rgba)


def export_stitched_output_to_ply(unorganized_point_cloud, path):
    """Export stitched/unorganized point cloud output to PLY (unordered layout)."""
    if not isinstance(unorganized_point_cloud, UnorganizedPointCloud):
        raise TypeError(
            "Unsupported type for argument unorganized_point_cloud. Got {}, expected {}".format(
                type(unorganized_point_cloud), UnorganizedPointCloud
            )
        )

    if not isinstance(path, (str, Path)):
        raise TypeError(
            "Unsupported type for argument path. Got {}, expected {} or {}.".format(
                type(path), str.__name__, Path.__name__
            )
        )

    export_unorganized_point_cloud(
        unorganized_point_cloud=unorganized_point_cloud,
        file_format=PLY(file_name=str(path), layout=PLY.Layout.unordered),
    )
