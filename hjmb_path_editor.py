"""Thin GUI launcher for HJMB Path Generator V4.0."""

from __future__ import annotations

import multiprocessing
import sys
from pathlib import Path


_SRC = Path(__file__).resolve().parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from hjmb_pathgen.py_app.gui_main import main


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
