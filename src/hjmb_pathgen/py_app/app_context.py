"""Application composition state shared by the two GUI pages."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_ui.ui_state import LoadedProjectState


@dataclass
class AppContext:
    project_root: Path
    generation_mode: GenerationMode = GenerationMode.MANUAL
    traj_id: int = 0
    state: LoadedProjectState | None = None

    def load(self, root: str | Path | None = None) -> LoadedProjectState:
        if root is not None:
            self.project_root = Path(root).resolve(strict=False)
        self.state = LoadedProjectState.load(self.project_root)
        return self.state
