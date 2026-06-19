"""Project/Sites tab with a real V4 field canvas."""

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

from hjmb_pathgen.models.project import ProjectV40

from ..commands import CallbackCommand
from ..field_view import V4FieldView
from ..graphics_items import DragCommit, YawCommit
from ..models.site_table_model import SiteTableModel
from ..ui_state import LoadedProjectState, SITE_KEYS, site_has_yaw


class ProjectSitesTab(QWidget):
    dirtyChanged = Signal(bool, str)
    statusMessage = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.state: LoadedProjectState | None = None
        self.model = SiteTableModel()
        self.table = QTableView()
        self.table.setModel(self.model)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.verticalHeader().hide()
        self.table.setAlternatingRowColors(True)
        self.field_view = V4FieldView(mode="sites")
        self.undo_stack = QUndoStack(self)
        self._syncing = False
        self._build_ui()
        self._connect()

    def _build_ui(self) -> None:
        left = QWidget()
        left_layout = QVBoxLayout(left)
        toolbar = QHBoxLayout()
        fit = QPushButton("适配场地")
        fit.clicked.connect(self.field_view.fit_to_field)
        undo = QToolButton()
        undo.setText("撤销")
        undo.clicked.connect(self.undo_stack.undo)
        redo = QToolButton()
        redo.setText("重做")
        redo.clicked.connect(self.undo_stack.redo)
        toolbar.addWidget(fit)
        toolbar.addWidget(undo)
        toolbar.addWidget(redo)
        toolbar.addStretch(1)
        left_layout.addLayout(toolbar)
        left_layout.addWidget(QLabel("固定点位（project.json / sites）"))
        left_layout.addWidget(self.table, 1)
        hint = QLabel("双击画布会设置当前选中 site；拖动点或 yaw 只标记 STALE，不自动规划。")
        hint.setWordWrap(True)
        left_layout.addWidget(hint)
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
        self.model.siteEdited.connect(self._site_table_edited)
        self.field_view.siteSelected.connect(self._select_site_from_view)
        self.field_view.backgroundDoubleClicked.connect(self._double_click_set_site)
        self.field_view.sitePositionPreview.connect(self._site_position_preview)
        self.field_view.sitePositionCommitted.connect(self._site_position_committed)
        self.field_view.siteYawPreview.connect(self._site_yaw_preview)
        self.field_view.siteYawCommitted.connect(self._site_yaw_committed)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.state = state
        project = state.project if state is not None else None
        self.model.set_project(project)
        self.field_view.set_project(project)
        if project is not None:
            self.select_site("P_START")
            self.field_view.fit_to_field()

    def select_site(self, site_key: str) -> None:
        row = self.model.row_for_site(site_key)
        self._syncing = True
        self.table.selectRow(row)
        self.field_view.set_selected_site(site_key, center=True)
        self._syncing = False

    def selected_site(self) -> str | None:
        row = self.table.currentIndex().row()
        if 0 <= row < len(SITE_KEYS):
            return SITE_KEYS[row]
        return None

    def set_site_from_world(self, site_key: str, x_mm: int, y_mm: int) -> None:
        project = self._project()
        site = project.sites[site_key]
        old = (int(site["x_mm"]), int(site["y_mm"]), bool(site["configured"]))

        def apply(value: tuple[int, int, bool]) -> None:
            site["x_mm"] = value[0]
            site["y_mm"] = value[1]
            site["configured"] = value[2]
            if site_has_yaw(site_key) and "yaw_ddeg" not in site:
                site["yaw_ddeg"] = 0
            self._refresh_site(site_key)

        new = (int(x_mm), int(y_mm), True)
        self.undo_stack.push(CallbackCommand(f"设置 {site_key}", lambda: apply(old), lambda: apply(new)))
        self._mark_dirty(f"{site_key} 已更新，相关结果标记 STALE")

    def _project(self) -> ProjectV40:
        if self.state is None:
            raise RuntimeError("project is not loaded")
        return self.state.project

    def _table_row_changed(self, current: QModelIndex, _previous: QModelIndex) -> None:
        if self._syncing or not current.isValid():
            return
        self.field_view.set_selected_site(SITE_KEYS[current.row()], center=True)

    def _select_site_from_view(self, site_key: str) -> None:
        if self._syncing:
            return
        self.select_site(site_key)

    def _double_click_set_site(self, x_mm: float, y_mm: float) -> None:
        site_key = self.selected_site()
        if site_key is None:
            self.statusMessage.emit("请先选择要设置的固定点")
            return
        self.set_site_from_world(site_key, int(round(x_mm)), int(round(y_mm)))

    def _site_table_edited(self, site_key: str) -> None:
        self._refresh_site(site_key)
        self._mark_dirty(f"{site_key} 表格编辑后标记 STALE")

    def _site_position_preview(self, site_key: str, x_mm: int, y_mm: int) -> None:
        project = self._project()
        site = project.sites[site_key]
        site["x_mm"] = x_mm
        site["y_mm"] = y_mm
        site["configured"] = True
        self.model.refresh_site(site_key)

    def _site_position_committed(self, commit: DragCommit) -> None:
        site_key = str(commit.key)
        if commit.old_x_mm == commit.new_x_mm and commit.old_y_mm == commit.new_y_mm:
            return
        project = self._project()
        site = project.sites[site_key]

        def apply(x_mm: int, y_mm: int) -> None:
            site["x_mm"] = x_mm
            site["y_mm"] = y_mm
            site["configured"] = True
            self._refresh_site(site_key)

        self.undo_stack.push(
            CallbackCommand(
                f"移动 {site_key}",
                lambda: apply(commit.old_x_mm, commit.old_y_mm),
                lambda: apply(commit.new_x_mm, commit.new_y_mm),
            )
        )
        self._mark_dirty(f"{site_key} 已移动，相关结果标记 STALE")

    def _site_yaw_preview(self, site_key: str, yaw_ddeg: int) -> None:
        if not site_has_yaw(site_key):
            return
        project = self._project()
        project.sites[site_key]["yaw_ddeg"] = yaw_ddeg
        self.model.refresh_site(site_key)

    def _site_yaw_committed(self, commit: YawCommit) -> None:
        site_key = str(commit.key)
        if not site_has_yaw(site_key) or commit.old_yaw_ddeg == commit.new_yaw_ddeg:
            return
        project = self._project()
        site = project.sites[site_key]

        def apply(yaw_ddeg: int) -> None:
            site["yaw_ddeg"] = yaw_ddeg
            site["configured"] = True
            self._refresh_site(site_key)

        self.undo_stack.push(
            CallbackCommand(
                f"调整 {site_key} yaw",
                lambda: apply(commit.old_yaw_ddeg),
                lambda: apply(commit.new_yaw_ddeg),
            )
        )
        self._mark_dirty(f"{site_key} yaw 已调整，相关结果标记 STALE")

    def _refresh_site(self, site_key: str) -> None:
        self.model.refresh_site(site_key)
        self.field_view.refresh()
        self.field_view.set_selected_site(site_key)

    def _mark_dirty(self, message: str) -> None:
        self.dirtyChanged.emit(True, message)
        self.statusMessage.emit(message)
