#!/usr/bin/env python3
"""Thin launcher for stitching workflow.

Delegates to stitching/stitch_point_clouds.py to keep logic in one place.
"""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    script = Path(__file__).resolve().parent.parent / "stitching" / "stitch_point_clouds.py"
    runpy.run_path(str(script), run_name="__main__")
