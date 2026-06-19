"""Compatibility launcher for the current V3.5 CLI."""

import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parent / "src"
if _SRC.exists():
    sys.path.insert(0, str(_SRC))

from hjmb_pathgen.legacy.v35.path_codec_cli import *  # noqa: F401,F403
from hjmb_pathgen.legacy.v35.path_codec_cli import main


if __name__ == "__main__":
    main()
