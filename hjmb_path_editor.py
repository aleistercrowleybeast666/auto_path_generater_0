"""Compatibility launcher for legacy imports and the V4 workflow UI."""

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
if _SRC.exists():
    sys.path.insert(0, str(_SRC))

from hjmb_pathgen.legacy.v35.editor import *  # noqa: F401,F403


def main() -> int:
    from hjmb_pathgen.ui.main_window import main as v4_main

    return v4_main()


if __name__ == "__main__":
    import multiprocessing

    multiprocessing.freeze_support()
    main()
