"""Unified MANUAL, SEMI_AUTO, and FULL_AUTO path editor page."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QUndoStack
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.route_case import CaseManifestV40

from ..field_view import V4FieldView
from ..graphics_items import DragCommit, YawCommit
from ..ui_state import LoadedProjectState, ManualPointDraft


class PathEditorPage(QWidget):
    dirtyChanged = Signal(bool, str)
    statusMessage = Signal(str)
    conversionRequested = Signal(int)
    generationRequested = Signal(str, int)

    def __init__(self) -> None:
        super().__init__()
        self.state: LoadedProjectState | None = None
        self.mode = GenerationMode.MANUAL
        self.traj_id = 0
        self.case: CaseManifestV40 | None = None
        self.points: list[ManualPointDraft] = []
        self.logical_templates: list[dict[str, Any]] = []
        self.actions: list[dict[str, Any]] = []
        self.auxiliary_points: list[dict[str, Any]] = []
        self.loading = False
        self.dirty = False
        self.undo_stack = QUndoStack(self)

        self.mode_label = QLabel()
        self.read_only_label = QLabel()
        self.read_only_label.setStyleSheet("color:#b45309")
        self.field_view = V4FieldView(mode="manual")
        self.point_table = QTableWidget(0, 5)
        self.point_table.setHorizontalHeaderLabels(("逻辑点/序号", "类型", "x_mm", "y_mm", "yaw_ddeg"))
        self.point_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.action_table = QTableWidget(0, 5)
        self.action_table.setHorizontalHeaderLabels(("seq", "action", "mode", "arrival", "post_wait_ms"))
        self.aux_policy = QComboBox()
        self.aux_policy.addItems(("LOCKED_PASS", "INITIAL_GUESS", "OPTIMIZABLE"))
        self._build_ui()
        self._connect()
        self._refresh_mode_text()

    def _build_ui(self) -> None:
        header = QHBoxLayout()
        header.addWidget(self.mode_label)
        header.addWidget(self.read_only_label)
        header.addStretch(1)
        fit = QPushButton("适应场地")
        fit.clicked.connect(self.field_view.fit_to_field)
        header.addWidget(fit)

        point_buttons = QHBoxLayout()
        for text, point_type in (("添加 START", "START"), ("添加 WAYPOINT", "WAYPOINT"), ("添加 ARRIVAL", "ARRIVAL")):
            button = QPushButton(text)
            button.clicked.connect(lambda _checked=False, value=point_type: self.add_point(value, 0, 0))
            point_buttons.addWidget(button)
        delete = QPushButton("删除")
        delete.clicked.connect(self.delete_selected_point)
        up = QPushButton("上移")
        up.clicked.connect(lambda: self.move_selected_point(-1))
        down = QPushButton("下移")
        down.clicked.connect(lambda: self.move_selected_point(1))
        point_buttons.addWidget(delete)
        point_buttons.addWidget(up)
        point_buttons.addWidget(down)

        aux_row = QHBoxLayout()
        aux_row.addWidget(QLabel("半自动辅助点策略"))
        aux_row.addWidget(self.aux_policy)
        add_aux = QPushButton("添加辅助点")
        add_aux.clicked.connect(self.add_auxiliary_point)
        aux_row.addWidget(add_aux)

        add_action = QPushButton("添加动作")
        add_action.clicked.connect(self.add_action)
        delete_action = QPushButton("删除动作")
        delete_action.clicked.connect(self.delete_selected_action)
        action_buttons = QHBoxLayout()
        action_buttons.addWidget(add_action)
        action_buttons.addWidget(delete_action)
        action_buttons.addStretch(1)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addLayout(point_buttons)
        right_layout.addLayout(aux_row)
        right_layout.addWidget(self.point_table, 3)
        right_layout.addWidget(QLabel("机械动作（V4 FIFO）"))
        right_layout.addLayout(action_buttons)
        right_layout.addWidget(self.action_table, 2)

        splitter = QSplitter()
        splitter.addWidget(self.field_view)
        splitter.addWidget(right)
        splitter.setSizes((1050, 520))
        layout = QVBoxLayout(self)
        layout.addLayout(header)
        layout.addWidget(splitter, 1)

    def _connect(self) -> None:
        self.point_table.cellChanged.connect(self._point_cell_changed)
        self.point_table.itemSelectionChanged.connect(self._selection_changed)
        self.action_table.cellChanged.connect(self._action_cell_changed)
        self.field_view.backgroundDoubleClicked.connect(self._background_double_clicked)
        self.field_view.manualPointPositionPreview.connect(self._position_preview)
        self.field_view.manualPointPositionCommitted.connect(self._position_committed)
        self.field_view.manualPointYawPreview.connect(self._yaw_preview)
        self.field_view.manualPointYawCommitted.connect(self._yaw_committed)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.state = state
        self.field_view.set_project(state.project if state else None)
        self.load_case(self.mode, self.traj_id)

    def load_case(self, mode: GenerationMode, traj_id: int) -> None:
        self.mode = mode
        self.traj_id = traj_id
        self.case = self.state.current_case(traj_id, mode) if self.state else None
        self.points = []
        self.logical_templates = []
        self.actions = []
        self.auxiliary_points = []
        if self.case is not None:
            if mode == GenerationMode.MANUAL and self.case.manual_path is not None:
                self.points = [_manual_draft(item, index) for index, item in enumerate(self.case.manual_path.get("points", []))]
            else:
                self.logical_templates = [dict(item) for item in self.case.logical_points]
                self.points = [
                    ManualPointDraft(
                        point_type="TASK_ANCHOR",
                        point_id=str(item["point_id"]),
                        x_mm=int(item["pose"]["x_mm"]),
                        y_mm=int(item["pose"]["y_mm"]),
                        yaw_ddeg=int(item["pose"]["yaw_ddeg"]),
                    )
                    for item in self.logical_templates
                ]
            self.actions = [dict(item) for item in self.case.actions.get("source", [])]
            self.auxiliary_points = [dict(item) for item in self.case.auxiliary_points]
        self.dirty = False
        self._refresh_mode_text()
        self._refresh_tables()
        self._refresh_field()

    def set_mode_and_traj(self, mode: GenerationMode, traj_id: int) -> None:
        self.load_case(mode, traj_id)

    def case_for_save(self) -> CaseManifestV40:
        if self.mode == GenerationMode.FULL_AUTO:
            if self.case is None:
                raise ValueError("FULL_AUTO Case 尚未生成")
            return self.case
        if self.mode == GenerationMode.MANUAL:
            data = self.case.to_dict() if self.case is not None else _empty_manual_case(self.traj_id)
            data["manual_path"] = {"points": [_manual_point_dict(point) for point in self.points]}
            data["logical_points"] = []
        else:
            if self.case is None:
                raise ValueError("请先从 FULL_AUTO 转为 SEMI_AUTO，或打开已有半自动 Case")
            data = self.case.to_dict()
            logical_points = []
            for template, point in zip(self.logical_templates, self.points, strict=True):
                item = dict(template)
                item["pose"] = {"x_mm": point.x_mm, "y_mm": point.y_mm, "yaw_ddeg": int(point.yaw_ddeg or 0)}
                logical_points.append(item)
            data["logical_points"] = logical_points
            data["auxiliary_points"] = list(self.auxiliary_points)
        data["generation_mode"] = self.mode.value
        data["actions"] = {"source": list(self.actions), "compiled": [] if self.dirty else list(data.get("actions", {}).get("compiled", []))}
        review = dict(data.get("review", {}))
        if self.dirty:
            review.update({"state": "STALE", "approved": False, "stale_reason": "GUI edit"})
        data["review"] = review
        return CaseManifestV40.from_dict(data)

    def mark_saved(self, case: CaseManifestV40) -> None:
        self.case = case
        self.dirty = False
        self.dirtyChanged.emit(False, "当前 Case JSON 已保存；未自动规划")

    def add_point(self, point_type: str, x_mm: int, y_mm: int) -> None:
        if self.mode != GenerationMode.MANUAL:
            self.statusMessage.emit("只有 MANUAL 可以增删任意路径点")
            return
        if point_type == "START" and any(point.point_type == "START" for point in self.points):
            self.statusMessage.emit("START 必须唯一")
            return
        yaw = 0 if point_type in {"START", "ARRIVAL"} else None
        self.points.append(ManualPointDraft(point_type, x_mm, y_mm, yaw))
        self._edited("手动点已修改，结果标记 STALE")

    def delete_selected_point(self) -> None:
        row = self.point_table.currentRow()
        if self.mode != GenerationMode.MANUAL or not 0 <= row < len(self.points):
            return
        self.points.pop(row)
        self._edited("手动点已删除，结果标记 STALE")

    def move_selected_point(self, offset: int) -> None:
        row = self.point_table.currentRow()
        target = row + offset
        if self.mode != GenerationMode.MANUAL or not 0 <= row < len(self.points) or not 0 <= target < len(self.points):
            return
        self.points[row], self.points[target] = self.points[target], self.points[row]
        self._edited("手动点顺序已修改，结果标记 STALE")
        self.point_table.selectRow(target)

    def add_auxiliary_point(self) -> None:
        if self.mode != GenerationMode.SEMI_AUTO:
            self.statusMessage.emit("辅助点只用于 SEMI_AUTO")
            return
        self.auxiliary_points.append({"x_mm": 0, "y_mm": 0, "policy": self.aux_policy.currentText()})
        self._edited("半自动辅助点已添加，结果标记 STALE")

    def add_action(self) -> None:
        if self.mode == GenerationMode.FULL_AUTO:
            self.conversionRequested.emit(self.traj_id)
            return
        self.actions.append({"action": "NONE", "mode": "ASYNC", "timeout_ms": 1000, "post_wait_ms": 0})
        self._edited("机械动作已修改，结果标记 STALE")

    def delete_selected_action(self) -> None:
        row = self.action_table.currentRow()
        if self.mode == GenerationMode.FULL_AUTO:
            self.conversionRequested.emit(self.traj_id)
        elif 0 <= row < len(self.actions):
            self.actions.pop(row)
            self._edited("机械动作已删除，结果标记 STALE")

    def _edited(self, reason: str) -> None:
        self.dirty = True
        self._refresh_tables()
        self._refresh_field()
        self.dirtyChanged.emit(True, reason)

    def _refresh_mode_text(self) -> None:
        names = {GenerationMode.MANUAL: "手动模式", GenerationMode.SEMI_AUTO: "半自动模式", GenerationMode.FULL_AUTO: "全自动模式"}
        self.mode_label.setText(f"{names[self.mode]} / P{self.traj_id:04d}")
        self.read_only_label.setText("只读；编辑前请转为半自动副本" if self.mode == GenerationMode.FULL_AUTO else "编辑只标记 STALE，不会自动规划")
        self.field_view.set_editable(self.mode != GenerationMode.FULL_AUTO)

    def _refresh_tables(self) -> None:
        self.loading = True
        self.point_table.setRowCount(len(self.points))
        for row, point in enumerate(self.points):
            values = (point.point_id or str(row), point.point_type, point.x_mm, point.y_mm, "" if point.yaw_ddeg is None else point.yaw_ddeg)
            for column, value in enumerate(values):
                self.point_table.setItem(row, column, QTableWidgetItem(str(value)))
        self.point_table.setEditTriggers(QAbstractItemView.NoEditTriggers if self.mode == GenerationMode.FULL_AUTO else QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self.action_table.setRowCount(len(self.actions))
        for row, action in enumerate(self.actions):
            values = (row, action.get("action", ""), action.get("mode", ""), action.get("arrival_state_id", action.get("arrival_point_id", "")), action.get("post_wait_ms", 0))
            for column, value in enumerate(values):
                self.action_table.setItem(row, column, QTableWidgetItem(str(value)))
        self.action_table.setEditTriggers(QAbstractItemView.NoEditTriggers if self.mode == GenerationMode.FULL_AUTO else QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self.loading = False

    def _refresh_field(self) -> None:
        self.field_view.set_manual_points(self.points)

    def _point_cell_changed(self, row: int, column: int) -> None:
        if self.loading or self.mode == GenerationMode.FULL_AUTO or not 0 <= row < len(self.points):
            return
        point = self.points[row]
        text = self.point_table.item(row, column).text()
        try:
            if column == 2:
                point.x_mm = int(round(float(text)))
            elif column == 3:
                point.y_mm = int(round(float(text)))
            elif column == 4 and point.has_yaw():
                point.yaw_ddeg = int(text)
            else:
                return
        except ValueError:
            self._refresh_tables()
            return
        self._edited("点表已修改，结果标记 STALE")

    def _action_cell_changed(self, row: int, column: int) -> None:
        if self.loading or self.mode == GenerationMode.FULL_AUTO or not 0 <= row < len(self.actions):
            return
        text = self.action_table.item(row, column).text()
        if column == 1:
            self.actions[row]["action"] = text
        elif column == 2:
            self.actions[row]["mode"] = text
        elif column == 3:
            self.actions[row]["arrival_state_id"] = text
        elif column == 4:
            try:
                self.actions[row]["post_wait_ms"] = int(text)
            except ValueError:
                self._refresh_tables()
                return
        else:
            return
        self._edited("机械动作已修改，结果标记 STALE")

    def _selection_changed(self) -> None:
        row = self.point_table.currentRow()
        self.field_view.set_selected_manual_index(row if 0 <= row < len(self.points) else None)

    def _background_double_clicked(self, x_mm: float, y_mm: float) -> None:
        if self.mode == GenerationMode.FULL_AUTO:
            self.conversionRequested.emit(self.traj_id)
        elif self.mode == GenerationMode.MANUAL:
            self.add_point("WAYPOINT", round(x_mm), round(y_mm))
        else:
            row = self.point_table.currentRow()
            if 0 <= row < len(self.points):
                self.points[row].x_mm = round(x_mm)
                self.points[row].y_mm = round(y_mm)
                self._edited("半自动锚点已设置，结果标记 STALE")

    def _position_preview(self, index: int, x_mm: int, y_mm: int) -> None:
        if self.mode == GenerationMode.FULL_AUTO:
            return
        if 0 <= index < len(self.points):
            self.points[index].x_mm = x_mm
            self.points[index].y_mm = y_mm

    def _position_committed(self, commit: DragCommit) -> None:
        if self.mode == GenerationMode.FULL_AUTO:
            self.conversionRequested.emit(self.traj_id)
            self._refresh_field()
            return
        index = int(commit.key)
        if 0 <= index < len(self.points):
            self.points[index].x_mm = commit.new_x_mm
            self.points[index].y_mm = commit.new_y_mm
            self._edited("拖动点位完成，结果标记 STALE")

    def _yaw_preview(self, index: int, yaw_ddeg: int) -> None:
        if self.mode != GenerationMode.FULL_AUTO and 0 <= index < len(self.points):
            self.points[index].yaw_ddeg = yaw_ddeg

    def _yaw_committed(self, commit: YawCommit) -> None:
        if self.mode == GenerationMode.FULL_AUTO:
            self.conversionRequested.emit(self.traj_id)
            self._refresh_field()
            return
        index = int(commit.key)
        if 0 <= index < len(self.points):
            self.points[index].yaw_ddeg = commit.new_yaw_ddeg
            self._edited("yaw 已修改，结果标记 STALE")


def _manual_draft(item: dict[str, Any], index: int) -> ManualPointDraft:
    return ManualPointDraft(
        point_type=str(item["type"]),
        point_id=str(item.get("point_id", index)),
        x_mm=int(item["x_mm"]),
        y_mm=int(item["y_mm"]),
        yaw_ddeg=int(item["yaw_ddeg"]) if item.get("yaw_ddeg") is not None else None,
        exact_pass=bool(item.get("exact_pass", True)),
    )


def _manual_point_dict(point: ManualPointDraft) -> dict[str, Any]:
    result: dict[str, Any] = {"type": point.point_type, "x_mm": point.x_mm, "y_mm": point.y_mm}
    if point.has_yaw():
        result["yaw_ddeg"] = int(point.yaw_ddeg or 0)
    elif point.point_type == "WAYPOINT":
        result["exact_pass"] = point.exact_pass
    return result


def _empty_manual_case(traj_id: int) -> dict[str, Any]:
    return {
        "format": "HJMB_ROUTE_CASE_JSON_V40",
        "storage_mode": "REFERENCED",
        "generation_mode": "MANUAL",
        "traj_id": traj_id,
        "bean_code": traj_id // 60,
        "drop_code": traj_id % 60,
        "source_mapping": {"manual": True},
        "selected_plan": {"route_family": "MANUAL", "vehicle_bin_assignment": {}, "drop_targets": [], "unload_sequence": [], "yaw_direction": "SHORTEST", "locked_by_user": True},
        "manual_path": {"points": []},
        "logical_points": [],
        "arrival_states": [],
        "leg_refs": [],
        "actions": {"source": [], "compiled": []},
        "finish": {"mode": "AT_FINAL_DROP"},
        "estimates": {},
        "hashes": {},
        "review": {"state": "STALE", "detached_from_library": True, "manual_override": True, "approved": False, "override_reason": "manual GUI Case"},
    }
