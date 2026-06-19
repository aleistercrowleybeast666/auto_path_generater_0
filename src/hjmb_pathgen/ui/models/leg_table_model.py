"""Table model for leg_library.json."""

from __future__ import annotations

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt

from hjmb_pathgen.models.leg import LegLibraryV40, LegV40


class LegTableModel(QAbstractTableModel):
    HEADERS = ("leg_id", "from", "to", "route family", "state", "quality", "time", "min clearance", "max rpm", "refs", "approved", "locked", "stale reason", "updated")

    def __init__(self) -> None:
        super().__init__()
        self.legs: list[LegV40] = []

    def set_library(self, library: LegLibraryV40 | None) -> None:
        self.beginResetModel()
        self.legs = list(library.legs) if library is not None else []
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.legs)

    def columnCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.HEADERS)

    def data(self, index: QModelIndex, role=Qt.DisplayRole):  # noqa: ANN001
        if not index.isValid():
            return None
        leg = self.legs[index.row()]
        key = leg.key
        analysis = leg.analysis
        review = leg.review
        col = index.column()
        values = (
            leg.leg_id,
            key.get("from_state_key", key.get("from_state_id", "")),
            key.get("to_state_key", key.get("to_state_id", "")),
            key.get("route_family", ""),
            leg.state.value,
            review.get("state", ""),
            analysis.get("planned_time_ms", ""),
            analysis.get("min_clearance_mm", analysis.get("clearance_mm", "")),
            analysis.get("max_metrics", {}).get("max_wheel_rpm", ""),
            review.get("ref_count", ""),
            bool(review.get("approved", False)),
            bool(review.get("locked", False)),
            review.get("stale_reason", ""),
            review.get("updated_at", ""),
        )
        if role in (Qt.DisplayRole, Qt.EditRole):
            return values[col]
        if role == Qt.ToolTipRole:
            return leg.leg_id
        if role == Qt.TextAlignmentRole:
            return Qt.AlignCenter
        return None

    def headerData(self, section: int, orientation: Qt.Orientation, role=Qt.DisplayRole):  # noqa: ANN001, N802
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.HEADERS[section]
        return None

    def leg_at(self, row: int) -> LegV40 | None:
        return self.legs[row] if 0 <= row < len(self.legs) else None

    def row_for_leg(self, leg_id: str) -> int:
        for index, leg in enumerate(self.legs):
            if leg.leg_id == leg_id:
                return index
        return -1
