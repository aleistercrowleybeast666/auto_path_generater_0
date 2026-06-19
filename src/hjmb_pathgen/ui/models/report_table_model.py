"""Report/final output table model."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt


class ReportTableModel(QAbstractTableModel):
    HEADERS = ("类型", "名称", "路径", "大小")

    def __init__(self) -> None:
        super().__init__()
        self.paths: list[Path] = []
        self.final_bins: dict[int, Path] = {}

    def set_data(self, reports: list[Path], final_bins: dict[int, Path]) -> None:
        self.beginResetModel()
        self.paths = list(reports)
        self.final_bins = dict(final_bins)
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.paths) + len(self.final_bins)

    def columnCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.HEADERS)

    def data(self, index: QModelIndex, role=Qt.DisplayRole):  # noqa: ANN001
        if not index.isValid():
            return None
        entries: list[tuple[str, str, Path]] = [(f"FINAL P{tid:04d}", path.name, path) for tid, path in sorted(self.final_bins.items())]
        entries.extend(("REPORT", path.name, path) for path in self.paths)
        kind, name, path = entries[index.row()]
        values = (kind, name, str(path), path.stat().st_size if path.exists() else "")
        if role in (Qt.DisplayRole, Qt.EditRole):
            return values[index.column()]
        if role == Qt.TextAlignmentRole:
            return Qt.AlignCenter
        return None

    def headerData(self, section: int, orientation: Qt.Orientation, role=Qt.DisplayRole):  # noqa: ANN001, N802
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.HEADERS[section]
        return None
