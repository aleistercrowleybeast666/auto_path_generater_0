"""Small undo commands used by GUI editing tabs."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtGui import QUndoCommand


class CallbackCommand(QUndoCommand):
    def __init__(self, text: str, undo: Callable[[], None], redo: Callable[[], None]) -> None:
        super().__init__(text)
        self._undo = undo
        self._redo = redo

    def undo(self) -> None:
        self._undo()

    def redo(self) -> None:
        self._redo()
