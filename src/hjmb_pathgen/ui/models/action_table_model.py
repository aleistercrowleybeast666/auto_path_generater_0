"""Read/write-ish action table models for source and compiled actions."""

from __future__ import annotations

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt

from hjmb_pathgen.models.route_case import CaseManifestV40


class ActionTableModel(QAbstractTableModel):
    HEADERS = ("seq", "action", "mode", "arrival/target", "timeout", "post_wait", "check_start", "diagnostics")

    def __init__(self, *, source_key: str) -> None:
        super().__init__()
        self.source_key = source_key
        self.actions: list[dict] = []

    def set_case(self, case: CaseManifestV40 | None) -> None:
        self.beginResetModel()
        self.actions = list((case.actions if case is not None else {}).get(self.source_key, []))
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.actions)

    def columnCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.HEADERS)

    def data(self, index: QModelIndex, role=Qt.DisplayRole):  # noqa: ANN001
        if not index.isValid():
            return None
        action = self.actions[index.row()]
        col = index.column()
        values = (
            index.row(),
            action.get("action", ""),
            action.get("mode", ""),
            action.get("arrival_state_id", action.get("target", "")),
            action.get("timeout_ms", ""),
            action.get("post_wait_ms", ""),
            action.get("check_start_s_mm", ""),
            "final" if action.get("final_binding") else "",
        )
        if role in (Qt.DisplayRole, Qt.EditRole):
            return values[col]
        if role == Qt.TextAlignmentRole:
            return Qt.AlignCenter
        return None

    def headerData(self, section: int, orientation: Qt.Orientation, role=Qt.DisplayRole):  # noqa: ANN001, N802
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.HEADERS[section]
        return None
