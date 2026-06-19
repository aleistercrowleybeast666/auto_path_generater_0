"""Compatibility wrapper for V3.5 batch models."""

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
if _SRC.exists():
    sys.path.insert(0, str(_SRC))

from hjmb_pathgen.legacy.v35.batch_models import *  # noqa: F401,F403
