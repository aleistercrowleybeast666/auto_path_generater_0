"""Table model for route_case_table.json and generated case status."""

from __future__ import annotations

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt

from hjmb_pathgen.models.route_case import CaseManifestV40, RouteCaseTableV40


class TaskCaseTableModel(QAbstractTableModel):
    HEADERS = ("traj_id", "bean", "drop", "任务语义", "候选/selected", "locked", "total", "motion", "mechanism/post", "legs", "task JSON", "task BIN")

    def __init__(self) -> None:
        super().__init__()
        self.rows = []
        self.task_cases: dict[int, CaseManifestV40] = {}

    def set_data(self, table: RouteCaseTableV40 | None, task_cases: dict[int, CaseManifestV40]) -> None:
        self.beginResetModel()
        self.rows = list(table.cases) if table is not None else []
        self.task_cases = dict(task_cases)
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.rows)

    def columnCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.HEADERS)

    def data(self, index: QModelIndex, role=Qt.DisplayRole):  # noqa: ANN001
        if not index.isValid():
            return None
        row = self.rows[index.row()]
        case = self.task_cases.get(row.traj_id)
        col = index.column()
        estimates = case.estimates if case is not None else {}
        selected = case.selected_plan if case is not None else {}
        values = (
            row.traj_id,
            row.bean_code,
            row.drop_code,
            f"{row.pick_assignment}",
            selected.get("route_family", "未生成"),
            bool(selected.get("locked_by_user", False)),
            estimates.get("planned_total_estimate_ms", ""),
            estimates.get("planned_motion_time_ms", ""),
            estimates.get("planned_mechanism_time_ms", estimates.get("planned_post_wait_ms", "")),
            len(case.leg_refs) if case is not None else "",
            "有" if case is not None else "无",
            "按输出目录检查",
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

    def traj_id_at(self, row: int) -> int | None:
        return self.rows[row].traj_id if 0 <= row < len(self.rows) else None
