"""Compatibility wrapper for V3.5 geometry helpers."""

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
if _SRC.exists():
    sys.path.insert(0, str(_SRC))

from hjmb_pathgen.legacy.v35.path_geometry import *  # noqa: F401,F403
