"""Manual free-path visual editor."""

from __future__ import annotations

from PySide6.QtCore import QModelIndex, Qt, Signal
from PySide6.QtGui import QUndoStack
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSplitter,
    QTableView,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.models.enums import PathSource

from ..commands import CallbackCommand
from ..field_view import V4FieldView
from ..graphics_items import DragCommit, YawCommit
from ..models.manual_point_table_model import ManualPointTableModel
from ..ui_state import LoadedProjectState, ManualPointDraft


class ManualFreeTab(QWidget):
    dirtyChanged = Signal(bool, str)
    statusMessage = Signal(str)
    workerRequested = Signal(str, dict)

    def __init__(self) -> None:
        super().__init__()
        self.state: LoadedProjectState | None = None
        self.points: list[ManualPointDraft] = []
        self.add_tool = "SELECT"
        self.model = ManualPointTableModel()
        self.table = QTableView()
        self.table.setModel(self.model)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.verticalHeader().hide()
        self.field_view = V4FieldView(mode="manual")
        self.undo_stack = QUndoStack(self)
        self._syncing = False
        self._build_ui()
        self._connect()

    def _build_ui(self) -> None:
        left = QWidget()
        left_layout = QVBoxLayout(left)
        toolbar = QHBoxLayout()
        for label, mode in (("选择/拖动", "SELECT"), ("添加START", "START"), ("添加WAYPOINT", "WAYPOINT"), ("添加ARRIVAL", "ARRIVAL")):
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, value=mode: self._set_tool(value))
            toolbar.addWidget(button)
        delete = QPushButton("删除")
        delete.clicked.connect(self.delete_selected)
        up = QPushButton("上移")
        up.clicked.connect(lambda: self.move_selected(-1))
        down = QPushButton("下移")
        down.clicked.connect(lambda: self.move_selected(1))
        fit = QPushButton("适配视图")
        fit.clicked.connect(self.field_view.fit_to_field)
        undo = QToolButton()
        undo.setText("撤销")
        undo.clicked.connect(self.undo_stack.undo)
        redo = QToolButton()
        redo.setText("重做")
        redo.clicked.connect(self.undo_stack.redo)
        for button in (delete, up, down, fit, undo, redo):
            toolbar.addWidget(button)
        toolbar.addStretch(1)
        left_layout.addLayout(toolbar)
        left_layout.addWidget(QLabel("自由路径点（MANUAL_FREE，会写入 cases/manual_free，不覆盖 TASK_COMPILED）"))
        left_layout.addWidget(self.table, 1)
        actions = QHBoxLayout()
        validate = QPushButton("验证当前")
        validate.clicked.connect(lambda: self.statusMessage.emit("验证需要先生成 MANUAL_FREE Case；编辑本身不会自动规划"))
        generate = QPushButton("生成 manual JSON/BIN")
        generate.clicked.connect(lambda: self.statusMessage.emit("当前源码 UI 已接好编辑模型；生成需要已有 MANUAL_FREE Case 作为服务输入"))
        final = QPushButton("设为 final")
        final.clicked.connect(lambda: self.workerRequested.emit("export-final", {"traj_id": 0, "path_source": PathSource.MANUAL_FREE.value}))
        for button in (validate, generate, final):
            actions.addWidget(button)
        left_layout.addLayout(actions)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(self.field_view)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 7)
        splitter.setSizes([430, 1120])
        self.field_view.setMinimumWidth(720)
        layout = QVBoxLayout(self)
        layout.addWidget(splitter, 1)

    def _connect(self) -> None:
        self.table.selectionModel().currentRowChanged.connect(self._table_row_changed)
        self.model.pointEdited.connect(self._point_table_edited)
        self.field_view.backgroundDoubleClicked.connect(self._double_click)
        self.field_view.manualPointSelected.connect(self._select_from_view)
        self.field_view.manualPointPositionPreview.connect(self._position_preview)
        self.field_view.manualPointPositionCommitted.connect(self._position_committed)
        self.field_view.manualPointYawPreview.connect(self._yaw_preview)
        self.field_view.manualPointYawCommitted.connect(self._yaw_committed)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.state = state
        self.field_view.set_project(state.project if state is not None else None)
        self.points = self._points_from_state(state)
        self.model.set_points(self.points)
        self.field_view.set_manual_points(self.points)
        if self.points:
            self.select_index(0)
        self.field_view.fit_to_field()

    def _points_from_state(self, state: LoadedProjectState | None) -> list[ManualPointDraft]:
        if state is None:
            return []
        case = state.current_case(source=PathSource.MANUAL_FREE)
        if case is None or case.manual_path is None:
            return []
        result: list[ManualPointDraft] = []
        for item in case.manual_path.get("points", []):
            result.append(
                ManualPointDraft(
                    point_type=str(item["type"]),
                    x_mm=int(item["x_mm"]),
                    y_mm=int(item["y_mm"]),
                    yaw_ddeg=item.get("yaw_ddeg"),
                    exact_pass=bool(item.get("exact_pass", str(item["type"]) != "WAYPOINT")),
                )
            )
        return result

    def _set_tool(self, tool: str) -> None:
        self.add_tool = tool
        self.statusMessage.emit(f"自由路径工具: {tool}")

    def selected_index(self) -> int | None:
        row = self.table.currentIndex().row()
        return row if 0 <= row < len(self.points) else None

    def select_index(self, index: int) -> None:
        if not 0 <= index < len(self.points):
            return
        self._syncing = True
        self.table.selectRow(index)
        self.field_view.set_selected_manual_index(index, center=True)
        self._syncing = False

    def add_point(self, point_type: str, x_mm: int, y_mm: int) -> None:
        if point_type == "START":
            if any(point.point_type == "START" for point in self.points):
                self.statusMessage.emit("已存在 START；请先删除或移动现有 START")
                return
            insert_at = 0
            point = ManualPointDraft("START", x_mm, y_mm, 0, True)
        elif point_type == "ARRIVAL":
            insert_at = len(self.points)
            point = ManualPointDraft("ARRIVAL", x_mm, y_mm, 0, True)
        else:
            insert_at = max(1, len(self.points))
            point = ManualPointDraft("WAYPOINT", x_mm, y_mm, None, False)

        def undo() -> None:
            del self.points[insert_at]
            self._reset_points()

        def redo() -> None:
            self.points.insert(insert_at, point)
            self._reset_points()
            self.select_index(insert_at)

        self.undo_stack.push(CallbackCommand(f"添加 {point_type}", undo, redo))
        self._mark_dirty(f"已添加 {point_type}，结果标记 STALE")

    def delete_selected(self) -> None:
        index = self.selected_index()
        if index is None:
            return
        point = self.points[index]

        def undo() -> None:
            self.points.insert(index, point)
            self._reset_points()
            self.select_index(index)

        def redo() -> None:
            del self.points[index]
            self._reset_points()

        self.undo_stack.push(CallbackCommand("删除自由点", undo, redo))
        self._mark_dirty("已删除自由点，结果标记 STALE")

    def move_selected(self, offset: int) -> None:
        index = self.selected_index()
        if index is None:
            return
        target = index + offset
        if target <= 0 and self.points[index].point_type != "START":
            target = 1
        if not 0 <= target < len(self.points):
            return
        if self.points[index].point_type == "START" and target != 0:
            self.statusMessage.emit("START 必须保持第一项")
            return

        def swap(a: int, b: int) -> None:
            self.points[a], self.points[b] = self.points[b], self.points[a]
            self._reset_points()
            self.select_index(b)

        self.undo_stack.push(CallbackCommand("调整自由点顺序", lambda: swap(target, index), lambda: swap(index, target)))
        self._mark_dirty("自由点顺序已调整，结果标记 STALE")

    def _double_click(self, x_mm: float, y_mm: float) -> None:
        if self.add_tool == "SELECT":
            self.statusMessage.emit("当前为选择/拖动工具；请选择添加工具后再双击新增点")
            return
        self.add_point(self.add_tool, int(round(x_mm)), int(round(y_mm)))

    def _table_row_changed(self, current: QModelIndex, _previous: QModelIndex) -> None:
        if self._syncing or not current.isValid():
            return
        self.field_view.set_selected_manual_index(current.row(), center=True)

    def _select_from_view(self, index: int) -> None:
        if self._syncing:
            return
        self.select_index(index)

    def _point_table_edited(self, index: int) -> None:
        self.field_view.refresh()
        self.field_view.set_selected_manual_index(index)
        self._mark_dirty("自由点表格已编辑，结果标记 STALE")

    def _position_preview(self, index: int, x_mm: int, y_mm: int) -> None:
        if not 0 <= index < len(self.points):
            return
        self.points[index].x_mm = x_mm
        self.points[index].y_mm = y_mm
        self.model.refresh_row(index)

    def _position_committed(self, commit: DragCommit) -> None:
        index = int(commit.key)
        if commit.old_x_mm == commit.new_x_mm and commit.old_y_mm == commit.new_y_mm:
            return

        def apply(x_mm: int, y_mm: int) -> None:
            self.points[index].x_mm = x_mm
            self.points[index].y_mm = y_mm
            self._reset_points()
            self.select_index(index)

        self.undo_stack.push(
            CallbackCommand(
                "移动自由点",
                lambda: apply(commit.old_x_mm, commit.old_y_mm),
                lambda: apply(commit.new_x_mm, commit.new_y_mm),
            )
        )
        self._mark_dirty("自由点已移动，结果标记 STALE")

    def _yaw_preview(self, index: int, yaw_ddeg: int) -> None:
        if 0 <= index < len(self.points) and self.points[index].has_yaw():
            self.points[index].yaw_ddeg = yaw_ddeg
            self.model.refresh_row(index)

    def _yaw_committed(self, commit: YawCommit) -> None:
        index = int(commit.key)
        if commit.old_yaw_ddeg == commit.new_yaw_ddeg:
            return

        def apply(yaw_ddeg: int) -> None:
            self.points[index].yaw_ddeg = yaw_ddeg
            self._reset_points()
            self.select_index(index)

        self.undo_stack.push(
            CallbackCommand(
                "调整自由点 yaw",
                lambda: apply(commit.old_yaw_ddeg),
                lambda: apply(commit.new_yaw_ddeg),
            )
        )
        self._mark_dirty("自由点 yaw 已调整，结果标记 STALE")

    def _reset_points(self) -> None:
        self.model.set_points(self.points)
        self.field_view.set_manual_points(self.points)

    def _mark_dirty(self, message: str) -> None:
        self.dirtyChanged.emit(True, message)
        self.statusMessage.emit(message)
