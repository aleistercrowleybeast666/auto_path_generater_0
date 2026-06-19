"""Route/Leg visualization tab."""

from __future__ import annotations

from PySide6.QtCore import QModelIndex, Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTableView,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.models.leg import LegV40
from hjmb_pathgen.services.leg_clear_service import clear_optimized_leg_result
from hjmb_pathgen.services.leg_optimization_service import validate_leg

from ..field_view import V4FieldView
from ..models.leg_table_model import LegTableModel
from ..ui_state import LoadedProjectState


class RouteLegTab(QWidget):
    statusMessage = Signal(str)
    dirtyChanged = Signal(bool, str)
    workerRequested = Signal(str, dict)
    reloadRequested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.state: LoadedProjectState | None = None
        self.model = LegTableModel()
        self.table = QTableView()
        self.table.setModel(self.model)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.verticalHeader().hide()
        self.field_view = V4FieldView(mode="route")
        self.detail = QLabel("未选择路段")
        self.detail.setWordWrap(True)
        self._build_ui()
        self._connect()

    def _build_ui(self) -> None:
        left = QWidget()
        left_layout = QVBoxLayout(left)
        form = QFormLayout()
        form.addRow("当前路段", self.detail)
        left_layout.addLayout(form)
        left_layout.addWidget(self.table, 1)
        buttons = QHBoxLayout()
        validate = QPushButton("validate")
        validate.clicked.connect(self.validate_selected)
        retime = QPushButton("retime only")
        retime.clicked.connect(lambda: self.statusMessage.emit("retime 当前路段需要通过显式服务命令启动"))
        quick = QPushButton("QUICK")
        quick.clicked.connect(lambda: self.workerRequested.emit("optimize-missing-legs", {"profile": "QUICK", "max_count": 1}))
        standard = QPushButton("STANDARD")
        standard.clicked.connect(lambda: self.workerRequested.emit("optimize-missing-legs", {"profile": "STANDARD", "max_count": 1}))
        final = QPushButton("FINAL")
        final.clicked.connect(lambda: self.workerRequested.emit("optimize-missing-legs", {"profile": "FINAL", "max_count": 1}))
        clear = QPushButton("清除当前路段优化结果")
        clear.clicked.connect(self.clear_selected)
        for button in (validate, retime, quick, standard, final, clear):
            buttons.addWidget(button)
        left_layout.addLayout(buttons)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(self.field_view)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 7)
        splitter.setSizes([460, 1090])
        self.field_view.setMinimumWidth(720)
        layout = QVBoxLayout(self)
        layout.addWidget(splitter, 1)

    def _connect(self) -> None:
        self.table.selectionModel().currentRowChanged.connect(self._row_changed)
        self.table.doubleClicked.connect(lambda index: self.open_leg_at(index.row()))

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.state = state
        self.model.set_library(state.leg_library if state is not None else None)
        self.field_view.set_project(state.project if state is not None else None)
        if self.model.legs:
            self.table.selectRow(0)
            self.open_leg_at(0)
        else:
            self.field_view.set_leg(None)
            self.detail.setText("未加载 leg_library 或没有路段")
        self.field_view.fit_to_field()

    def open_leg(self, leg_id: str) -> None:
        row = self.model.row_for_leg(leg_id)
        if row >= 0:
            self.table.selectRow(row)
            self.open_leg_at(row)

    def selected_leg(self) -> LegV40 | None:
        return self.model.leg_at(self.table.currentIndex().row())

    def open_leg_at(self, row: int) -> None:
        leg = self.model.leg_at(row)
        if leg is None:
            return
        self.field_view.set_leg(leg)
        analysis = leg.analysis
        self.detail.setText(
            "\n".join(
                (
                    f"leg_id: {leg.leg_id}",
                    f"from: {leg.key.get('from_state_key', leg.key.get('from_state_id', ''))}",
                    f"to: {leg.key.get('to_state_key', leg.key.get('to_state_id', ''))}",
                    f"route_family: {leg.key.get('route_family', '')}",
                    f"topology: {leg.topology_profile}",
                    f"state: {leg.state.value}",
                    f"time: {analysis.get('planned_time_ms', '')}",
                    f"max rpm: {analysis.get('max_metrics', {}).get('max_wheel_rpm', '')}",
                )
            )
        )

    def validate_selected(self) -> None:
        if self.state is None:
            return
        leg = self.selected_leg()
        if leg is None:
            return
        report = validate_leg(self.state.project, leg)
        self.statusMessage.emit(f"{leg.leg_id} validate: {report.get('valid')} {report.get('errors', '')}")

    def clear_selected(self) -> None:
        if self.state is None:
            return
        leg = self.selected_leg()
        if leg is None:
            return
        confirm = QMessageBox.question(self, "清除路段", f"确认清除 {leg.leg_id} 的优化结果？\n不会修改 project.json，也不会自动重算。")
        if confirm != QMessageBox.Yes:
            self.statusMessage.emit("已取消清除路段")
            return
        needs_token = bool(leg.review.get("approved", False) or leg.review.get("locked", False))
        result = clear_optimized_leg_result(
            self.state.layout,
            leg.leg_id,
            confirm_leg_id=leg.leg_id if needs_token else None,
        )
        self.statusMessage.emit(f"已清除 {leg.leg_id}: {result.previous_state} -> {result.new_state}")
        self.dirtyChanged.emit(True, "leg_library 已修改；相关 Case 标记 STALE_MISSING_LEG")
        self.reloadRequested.emit()

    def _row_changed(self, current: QModelIndex, _previous: QModelIndex) -> None:
        if current.isValid():
            self.open_leg_at(current.row())
