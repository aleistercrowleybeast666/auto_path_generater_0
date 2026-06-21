"""Fourth main-window page for operator-authored XY leg templates."""

from __future__ import annotations

import uuid
from dataclasses import replace
from pathlib import Path
from typing import Any

from PySide6.QtCore import QSignalBlocker, QTimer, Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.py_domain.leg_template import (
    LegTemplateInstancesV40,
    LegTemplateState,
    LegTemplateValidationReportV40,
    LegTemplateWaypointV40,
    LegTemplatesV40,
)
from hjmb_pathgen.py_io.codecs.json_codec import (
    load_leg_template_instances,
    load_leg_templates,
    load_leg_template_validation_report,
    load_project,
    save_leg_templates,
)
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_planning.geometry.bezier import BezierPath, Point2D
from hjmb_pathgen.py_services.competition_task_config_service import load_competition_task_config
from hjmb_pathgen.py_services.leg_template_service import (
    expected_leg_template_slots,
    export_all_leg_template_documents,
    export_leg_template_document,
    leg_template_id,
    leg_template_topology_gates,
    sync_leg_templates_for_layout,
)
from hjmb_pathgen.py_ui.field_view import V4FieldView
from hjmb_pathgen.py_ui.ui_state import ManualPointDraft


STATUS_COLORS = {
    "DISABLED": "#64748b",
    "DRAFT": "#64748b",
    "STALE": "#d97706",
    "CHECKING": "#7c3aed",
    "PASSED": "#15803d",
    "PARTIAL": "#d97706",
    "FAILED": "#dc2626",
}


class LegTemplatePage(QWidget):
    validationRequested = Signal(str, object, str, int)
    cancelRequested = Signal()
    statusMessage = Signal(str)
    timeSummaryChanged = Signal(str)

    def __init__(self, parent=None, *, field_view: V4FieldView | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("leg_template_page")
        self.layout_ref: ProjectLayout | None = None
        self.project = None
        self.task_config = None
        self.templates = LegTemplatesV40.empty()
        self.instances = LegTemplateInstancesV40.empty()
        self.report = LegTemplateValidationReportV40.empty()
        self.selected_template_id = ""
        self.selected_instance_id = ""
        self.selected_waypoint_index: int | None = None
        self.waypoint_draft: list[LegTemplateWaypointV40] = []
        self.dirty = False
        self.revision = 0
        self.active_job_token = ""
        self.checking_template_ids: set[str] = set()
        self._refreshing = False
        # Runtime uses the main window's large shared field canvas.  A private
        # view is retained only for isolated tests or standalone embedding.
        self.field_view = field_view or V4FieldView(mode="template")
        self._owns_field_view = field_view is None
        self._bind_field_view()
        self._build_ui()

    def _bind_field_view(self) -> None:
        self.field_view.backgroundClicked.connect(self.add_waypoint)
        self.field_view.manualPointSelected.connect(self._waypoint_selected)
        self.field_view.manualPointPositionPreview.connect(self._waypoint_drag_preview)
        self.field_view.manualPointPositionCommitted.connect(self._waypoint_drag_committed)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        self.scroll = QScrollArea()
        self.scroll.setObjectName("leg_template_page_scroll")
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        content = QWidget()
        content.setMinimumWidth(720)
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(4, 4, 4, 4)
        content_layout.setSpacing(6)

        toolbar_top = QHBoxLayout()
        for text, callback in (
            ("从固定点重新同步", self.sync_from_project),
            ("重新加载", self.reload_from_project),
            ("保存草稿", self.save_current_draft),
            ("验证当前", self.validate_current),
            ("验证全部已启用", self.validate_all_enabled),
            ("取消验证", lambda: self.cancelRequested.emit()),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            toolbar_top.addWidget(button)
        toolbar_top.addStretch(1)
        content_layout.addLayout(toolbar_top)

        toolbar_export = QHBoxLayout()
        for text, callback in (
            ("输出模板 JSON", lambda: self._choose_export("templates")),
            ("输出实例 JSON", lambda: self._choose_export("instances")),
            ("输出验证报告", lambda: self._choose_export("report")),
            ("全部输出", self._choose_export_all),
            ("适配主场地", self.field_view.fit_to_field),
            ("在选中点后插入", self.insert_waypoint_after_selected),
            ("删除选中 waypoint", self.delete_selected_waypoint),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            toolbar_export.addWidget(button)
        toolbar_export.addStretch(1)
        content_layout.addLayout(toolbar_export)

        self.stale_label = QLabel("尚未加载项目")
        self.stale_label.setWordWrap(True)
        content_layout.addWidget(self.stale_label)
        field_hint = QLabel(
            "在主窗口原有的大场地图区域中编辑当前模板：左键空白处添加 waypoint，拖动修改位置，"
            "Delete 删除选中点。此页不再创建第二块小场地。"
        )
        field_hint.setWordWrap(True)
        content_layout.addWidget(field_hint)

        self.template_table = QTableWidget(0, 11)
        self.template_table.setObjectName("leg_template_table")
        self.template_table.setHorizontalHeaderLabels(
            ["启用", "template_id", "路线", "起点", "终点", "waypoint", "实例", "状态", "最佳时间(ms)", "最小净空(mm)", "操作"]
        )
        self.template_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.template_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.template_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.template_table.setMinimumHeight(300)
        self.template_table.setColumnWidth(0, 58)
        self.template_table.setColumnWidth(1, 330)
        self.template_table.setColumnWidth(10, 210)
        self.template_table.itemSelectionChanged.connect(self._template_selection_changed)
        self.template_table.itemChanged.connect(self._template_item_changed)
        content_layout.addWidget(self.template_table)

        detail_group = QGroupBox("当前模板 waypoint（在主场地图编辑）")
        detail_layout = QVBoxLayout(detail_group)
        self.detail_label = QLabel("请选择模板")
        self.detail_label.setWordWrap(True)
        detail_layout.addWidget(self.detail_label)
        self.waypoint_table = QTableWidget(0, 6)
        self.waypoint_table.setObjectName("leg_template_waypoint_table")
        self.waypoint_table.setHorizontalHeaderLabels(["序号", "x_mm", "y_mm", "上移", "下移", "删除"])
        self.waypoint_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.waypoint_table.setMinimumHeight(210)
        detail_layout.addWidget(self.waypoint_table)
        content_layout.addWidget(detail_group)

        instance_group = QGroupBox("精确实例与验证结果")
        instance_layout = QVBoxLayout(instance_group)
        self.instance_table = QTableWidget(0, 9)
        self.instance_table.setObjectName("leg_template_instance_table")
        self.instance_table.setHorizontalHeaderLabels(
            ["instance_id", "精确起点", "精确终点", "起点profile", "终点profile", "状态", "时间(ms)", "净空(mm)", "失败原因"]
        )
        self.instance_table.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.instance_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.instance_table.setMinimumHeight(230)
        self.instance_table.setColumnWidth(0, 210)
        self.instance_table.setColumnWidth(1, 210)
        self.instance_table.setColumnWidth(2, 210)
        self.instance_table.setColumnWidth(8, 420)
        self.instance_table.itemSelectionChanged.connect(self._instance_selection_changed)
        instance_layout.addWidget(self.instance_table)
        self.error_text = QPlainTextEdit()
        self.error_text.setReadOnly(True)
        self.error_text.setPlaceholderText("结构化失败原因与指标")
        self.error_text.setMaximumHeight(150)
        instance_layout.addWidget(self.error_text)
        content_layout.addWidget(instance_group)

        self.preview_timer = QTimer(self)
        self.preview_timer.setSingleShot(True)
        self.preview_timer.setInterval(200)
        self.preview_timer.timeout.connect(self._refresh_preview)
        self.scroll.setWidget(content)
        root.addWidget(self.scroll)

    def load_layout(self, layout: ProjectLayout, *, synchronize: bool = True) -> None:
        if self.dirty:
            self.save_current_draft()
        self.layout_ref = layout
        self.active_job_token = ""
        self.checking_template_ids.clear()
        self.selected_template_id = ""
        self.selected_instance_id = ""
        self.revision += 1
        if synchronize:
            sync_leg_templates_for_layout(layout)
        self._load_documents()

    def clear_project(self) -> None:
        self.layout_ref = None
        self.project = None
        self.task_config = None
        self.templates = LegTemplatesV40.empty()
        self.instances = LegTemplateInstancesV40.empty()
        self.report = LegTemplateValidationReportV40.empty()
        self.selected_template_id = ""
        self.selected_instance_id = ""
        self.waypoint_draft = []
        self.dirty = False
        self.revision += 1
        self._refresh_all()

    def sync_from_project(self) -> None:
        if self.layout_ref is None:
            return
        self.save_current_draft()
        sync_leg_templates_for_layout(self.layout_ref)
        self.revision += 1
        self._load_documents(preserve_selection=True)
        self.statusMessage.emit("Leg 模板已从固定点与规划配置同步；未运行严格验证")

    def reload_from_project(self) -> None:
        if self.layout_ref is None:
            return
        self.save_current_draft()
        self.revision += 1
        self._load_documents(preserve_selection=True)
        self.statusMessage.emit("已重新加载 Leg 模板 JSON")

    def _load_documents(self, *, preserve_selection: bool = False) -> None:
        if self.layout_ref is None:
            self.clear_project()
            return
        selected = self.selected_template_id if preserve_selection else ""
        self.project = load_project(self.layout_ref.project_json)
        self.task_config = load_competition_task_config(self.layout_ref.competition_task_config_json)
        self.templates = load_leg_templates(self.layout_ref.leg_templates_json)
        self.instances = (
            load_leg_template_instances(self.layout_ref.leg_template_instances_json)
            if self.layout_ref.leg_template_instances_json.exists()
            else LegTemplateInstancesV40.empty(self.project.project_id)
        )
        self.report = (
            load_leg_template_validation_report(self.layout_ref.leg_template_validation_report_json)
            if self.layout_ref.leg_template_validation_report_json.exists()
            else LegTemplateValidationReportV40.empty(self.project.project_id)
        )
        self.dirty = False
        self.selected_template_id = selected if any(
            item.template_id == selected and not item.orphaned
            for item in self.templates.templates
        ) else ""
        self._refresh_all()

    def _ordered_templates(self):
        active = tuple(item for item in self.templates.templates if not item.orphaned)
        if self.task_config is None:
            return active
        order = {
            leg_template_id(*slot): index
            for index, slot in enumerate(expected_leg_template_slots(self.task_config))
        }
        return tuple(sorted(active, key=lambda item: (order.get(item.template_id, 9999), item.template_id)))

    def _refresh_all(self) -> None:
        self._refresh_template_table()
        if self.selected_template_id:
            self._load_selected_template_draft()
        else:
            self.waypoint_draft = []
            self._refresh_waypoint_table()
            self._refresh_instance_table()
            self.field_view.set_project(self.project)
            self.field_view.set_manual_points([])
            self.field_view.set_leg(None)
            self.field_view.set_preview_xy(())
        stale_count = sum(
            item.state == LegTemplateState.STALE and not item.orphaned
            for item in self.templates.templates
        )
        self.stale_label.setText(f"固定点/规划配置变化后不会自动规划；当前有 {stale_count} 条模板需要重新验证。")
        self._emit_time_summary()

    def _refresh_template_table(self) -> None:
        ordered = self._ordered_templates()
        by_template: dict[str, list[Any]] = {}
        for instance in self.instances.instances:
            by_template.setdefault(instance.template_id, []).append(instance)
        self._refreshing = True
        self.template_table.setRowCount(len(ordered))
        for row, template in enumerate(ordered):
            enable = QTableWidgetItem()
            enable.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsUserCheckable)
            enable.setCheckState(Qt.Checked if template.enabled else Qt.Unchecked)
            enable.setData(Qt.UserRole, template.template_id)
            self.template_table.setItem(row, 0, enable)
            route = "左绕" if template.route_family.value == "PICK_1_TO_3" else "右绕"
            current_instances = by_template.get(template.template_id, [])
            passed = sum(item.state.value == "PASSED" for item in current_instances)
            status = "CHECKING" if template.template_id in self.checking_template_ids else template.state.value
            if not template.enabled:
                status = "DISABLED"
            times = [item.planned_time_ms for item in current_instances if item.state.value == "PASSED"]
            clearances = [item.min_clearance_mm for item in current_instances if item.min_clearance_mm is not None]
            values = (
                template.template_id, route, template.from_site, template.to_site,
                str(len(template.waypoints)), f"{passed}/{len(current_instances)}", status,
                str(min(times)) if times else "—", f"{min(clearances):.1f}" if clearances else "—",
            )
            for column, value in enumerate(values, start=1):
                item = QTableWidgetItem(value)
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                item.setData(Qt.UserRole, template.template_id)
                if column == 7:
                    item.setForeground(QColor(STATUS_COLORS.get(status, "#334155")))
                self.template_table.setItem(row, column, item)
            operations = QWidget()
            row_layout = QHBoxLayout(operations)
            row_layout.setContentsMargins(0, 0, 0, 0)
            for text, callback in (
                ("编辑", lambda _checked=False, value=template.template_id: self.select_template(value)),
                ("确定", lambda _checked=False, value=template.template_id: self.validate_template(value)),
                ("清除", lambda _checked=False, value=template.template_id: self.clear_template_waypoints(value)),
            ):
                button = QPushButton(text)
                button.clicked.connect(callback)
                row_layout.addWidget(button)
            self.template_table.setCellWidget(row, 10, operations)
        self._refreshing = False
        if self.selected_template_id:
            for row in range(self.template_table.rowCount()):
                item = self.template_table.item(row, 1)
                if item is not None and item.data(Qt.UserRole) == self.selected_template_id:
                    self.template_table.selectRow(row)
                    break

    def _template_item_changed(self, item: QTableWidgetItem) -> None:
        if self._refreshing or item.column() != 0:
            return
        template_id = str(item.data(Qt.UserRole) or "")
        if not template_id:
            return
        self._persist_template_change(template_id, enabled=item.checkState() == Qt.Checked)

    def _template_selection_changed(self) -> None:
        rows = self.template_table.selectionModel().selectedRows()
        if not rows:
            return
        item = self.template_table.item(rows[0].row(), 1)
        if item is not None:
            self.select_template(str(item.data(Qt.UserRole) or ""))

    def select_template(self, template_id: str) -> None:
        if not template_id or template_id == self.selected_template_id:
            return
        self.save_current_draft()
        self.selected_template_id = template_id
        self.selected_instance_id = ""
        self.selected_waypoint_index = None
        self._load_selected_template_draft()
        self._refresh_template_table()

    def _selected_template(self):
        return next((item for item in self.templates.templates if item.template_id == self.selected_template_id), None)

    def _load_selected_template_draft(self) -> None:
        template = self._selected_template()
        if template is None:
            return
        if not self.dirty:
            self.waypoint_draft = list(template.waypoints)
        self.detail_label.setText(
            f"{template.template_id}\n{template.from_site} → {template.to_site} | {template.route_family.value} | "
            f"状态={template.state.value}{' | 草稿未保存' if self.dirty else ''}"
        )
        self.field_view.set_project(self.project)
        self.field_view.set_topology_gates_override(leg_template_topology_gates(self.project, template) if self.project else ())
        self._refresh_waypoint_table()
        self._refresh_preview()
        self._refresh_instance_table()

    def _refresh_waypoint_table(self, *, update_field: bool = True) -> None:
        self._refreshing = True
        self.waypoint_table.setRowCount(len(self.waypoint_draft))
        for row, waypoint in enumerate(self.waypoint_draft):
            self.waypoint_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            for column, value in ((1, waypoint.x_mm), (2, waypoint.y_mm)):
                spin = QDoubleSpinBox()
                spin.setRange(-2500.0, 2500.0)
                spin.setDecimals(1)
                spin.setSingleStep(10.0)
                spin.setValue(value)
                spin.valueChanged.connect(lambda new_value, r=row, c=column: self._waypoint_spin_changed(r, c, new_value))
                self.waypoint_table.setCellWidget(row, column, spin)
            for column, text, callback in (
                (3, "↑", lambda _checked=False, r=row: self.move_waypoint(r, -1)),
                (4, "↓", lambda _checked=False, r=row: self.move_waypoint(r, 1)),
                (5, "删除", lambda _checked=False, r=row: self.delete_waypoint(r)),
            ):
                button = QPushButton(text)
                button.clicked.connect(callback)
                self.waypoint_table.setCellWidget(row, column, button)
        self._refreshing = False
        if update_field:
            self._sync_field_waypoints()

    def _sync_field_waypoints(self) -> None:
        drafts = [ManualPointDraft("WAYPOINT", round(item.x_mm), round(item.y_mm), point_id=f"W{index + 1}") for index, item in enumerate(self.waypoint_draft)]
        self.field_view.set_manual_points(drafts)
        self.field_view.set_selected_manual_index(self.selected_waypoint_index)

    def add_waypoint(self, x_mm: float, y_mm: float) -> None:
        if self._selected_template() is None:
            return
        self.waypoint_draft.append(LegTemplateWaypointV40(round(x_mm, 1), round(y_mm, 1)))
        self.selected_waypoint_index = len(self.waypoint_draft) - 1
        self._mark_dirty()

    def insert_waypoint_after_selected(self) -> None:
        template = self._selected_template()
        if template is None or self.project is None:
            return
        index = self.selected_waypoint_index
        if index is None:
            index = len(self.waypoint_draft) - 1
        left = self.waypoint_draft[index] if self.waypoint_draft and index >= 0 else LegTemplateWaypointV40(
            float(self.project.sites[template.from_site]["x_mm"]), float(self.project.sites[template.from_site]["y_mm"])
        )
        if index + 1 < len(self.waypoint_draft):
            right = self.waypoint_draft[index + 1]
        else:
            right = LegTemplateWaypointV40(float(self.project.sites[template.to_site]["x_mm"]), float(self.project.sites[template.to_site]["y_mm"]))
        insert_at = index + 1
        self.waypoint_draft.insert(insert_at, LegTemplateWaypointV40((left.x_mm + right.x_mm) / 2.0, (left.y_mm + right.y_mm) / 2.0))
        self.selected_waypoint_index = insert_at
        self._mark_dirty()

    def delete_selected_waypoint(self) -> None:
        if self.selected_waypoint_index is not None:
            self.delete_waypoint(self.selected_waypoint_index)

    def delete_waypoint(self, row: int) -> None:
        if 0 <= row < len(self.waypoint_draft):
            del self.waypoint_draft[row]
            self.selected_waypoint_index = min(row, len(self.waypoint_draft) - 1) if self.waypoint_draft else None
            self._mark_dirty()

    def move_waypoint(self, row: int, delta: int) -> None:
        target = row + delta
        if 0 <= row < len(self.waypoint_draft) and 0 <= target < len(self.waypoint_draft):
            self.waypoint_draft[row], self.waypoint_draft[target] = self.waypoint_draft[target], self.waypoint_draft[row]
            self.selected_waypoint_index = target
            self._mark_dirty()

    def _waypoint_spin_changed(self, row: int, column: int, value: float) -> None:
        if self._refreshing or not 0 <= row < len(self.waypoint_draft):
            return
        old = self.waypoint_draft[row]
        self.waypoint_draft[row] = LegTemplateWaypointV40(value if column == 1 else old.x_mm, value if column == 2 else old.y_mm)
        self._mark_dirty(refresh_table=False)
        self._sync_field_waypoints()

    def _waypoint_selected(self, index: int) -> None:
        self.selected_waypoint_index = index
        self.field_view.set_selected_manual_index(index)

    def _waypoint_drag_preview(self, index: int, x_mm: int, y_mm: int) -> None:
        """Update the draft without rebuilding the graphics scene mid-drag.

        Rebuilding ``V4FieldView`` while Qt is still dispatching a drag event
        destroys the active graphics item.  The next event then uses a stale
        grab position and the point appears to fly to the upper-right corner.
        Only the numeric table is updated during movement; the curve/scene is
        refreshed after mouse release.
        """
        if not 0 <= index < len(self.waypoint_draft):
            return
        self.waypoint_draft[index] = LegTemplateWaypointV40(float(x_mm), float(y_mm))
        self._update_field_waypoint_cache(index, x_mm, y_mm)
        self.dirty = True
        self._update_waypoint_coordinate_cells(index)
        template = self._selected_template()
        if template is not None:
            self.detail_label.setText(f"{template.template_id} | 草稿未保存 | 未运行严格验证")

    def _waypoint_drag_committed(self, commit: Any) -> None:
        index = int(commit.key)
        if not 0 <= index < len(self.waypoint_draft):
            return
        self.waypoint_draft[index] = LegTemplateWaypointV40(
            float(commit.new_x_mm), float(commit.new_y_mm)
        )
        self._update_field_waypoint_cache(index, commit.new_x_mm, commit.new_y_mm)
        self.dirty = True
        self.revision += 1
        self._update_waypoint_coordinate_cells(index)
        # Defer the scene rebuild until after QGraphicsItem.mouseReleaseEvent
        # returns; the 200 ms preview timer also coalesces rapid edits.
        self.preview_timer.start()

    def _update_field_waypoint_cache(self, index: int, x_mm: float, y_mm: float) -> None:
        """Keep the shared field model aligned with the dragged graphics item.

        ``set_preview_xy()`` rebuilds the scene after mouse release.  The
        graphics item itself has already moved, but a stale ``manual_points``
        cache would recreate it at the old coordinate while drawing the curve
        from the new draft.  Updating the cache in place preserves the active
        item during dragging and makes the deferred rebuild reproduce the same
        point position.
        """
        if not 0 <= index < len(self.field_view.manual_points):
            return
        point = self.field_view.manual_points[index]
        point.x_mm = int(round(float(x_mm)))
        point.y_mm = int(round(float(y_mm)))

    def _update_waypoint_coordinate_cells(self, row: int) -> None:
        if not 0 <= row < len(self.waypoint_draft):
            return
        waypoint = self.waypoint_draft[row]
        for column, value in ((1, waypoint.x_mm), (2, waypoint.y_mm)):
            widget = self.waypoint_table.cellWidget(row, column)
            if isinstance(widget, QDoubleSpinBox):
                blocker = QSignalBlocker(widget)
                widget.setValue(float(value))
                del blocker

    def _mark_dirty(self, *, refresh_table: bool = True) -> None:
        self.dirty = True
        self.revision += 1
        if refresh_table:
            self._refresh_waypoint_table()
        self.preview_timer.start()
        template = self._selected_template()
        if template is not None:
            self.detail_label.setText(f"{template.template_id} | 草稿未保存 | 未运行严格验证")

    def _refresh_preview(self) -> None:
        template = self._selected_template()
        if template is None or self.project is None:
            self.field_view.set_preview_xy(())
            return
        start = self.project.sites[template.from_site]
        finish = self.project.sites[template.to_site]
        points = [Point2D(float(start["x_mm"]), float(start["y_mm"]))]
        points.extend(Point2D(item.x_mm, item.y_mm) for item in self.waypoint_draft)
        points.append(Point2D(float(finish["x_mm"]), float(finish["y_mm"])))
        try:
            samples = BezierPath.from_waypoints(points).sample_arclength(max_spacing_mm=35.0, oversample_per_segment=24)
            preview = tuple((item.x_mm, item.y_mm) for item in samples)
        except ValueError:
            preview = tuple((item.x_mm, item.y_mm) for item in points)
        self.field_view.set_preview_xy(preview)

    def save_current_draft(self) -> None:
        if not self.dirty or self.layout_ref is None or not self.selected_template_id:
            return
        document = load_leg_templates(self.layout_ref.leg_templates_json)
        items = []
        for item in document.templates:
            if item.template_id == self.selected_template_id:
                state = LegTemplateState.STALE if item.enabled else LegTemplateState.DISABLED
                item = replace(item, waypoints=tuple(self.waypoint_draft), state=state, last_validated_hash="")
            items.append(item)
        save_leg_templates(self.layout_ref.leg_templates_json, replace(document, templates=tuple(items)))
        sync_leg_templates_for_layout(self.layout_ref)
        self.dirty = False
        self._load_documents(preserve_selection=True)
        self.statusMessage.emit(f"已保存模板草稿：{self.selected_template_id}")

    def _persist_template_change(self, template_id: str, *, enabled: bool) -> None:
        if self.layout_ref is None:
            return
        if template_id == self.selected_template_id and self.dirty:
            self.save_current_draft()
        document = load_leg_templates(self.layout_ref.leg_templates_json)
        items = tuple(
            replace(item, enabled=enabled, state=LegTemplateState.STALE if enabled else LegTemplateState.DISABLED, last_validated_hash="")
            if item.template_id == template_id else item
            for item in document.templates
        )
        save_leg_templates(self.layout_ref.leg_templates_json, replace(document, templates=items))
        sync_leg_templates_for_layout(self.layout_ref)
        self.revision += 1
        self._load_documents(preserve_selection=True)

    def clear_template_waypoints(self, template_id: str) -> None:
        if QMessageBox.question(self, "清除 waypoint", f"确定清空 {template_id} 的全部 waypoint？") != QMessageBox.Yes:
            return
        self.select_template(template_id)
        self.waypoint_draft = []
        self.dirty = True
        self.revision += 1
        self.save_current_draft()

    def validate_current(self) -> None:
        if self.selected_template_id:
            self.validate_template(self.selected_template_id)

    def validate_template(self, template_id: str) -> None:
        if self.layout_ref is None:
            return
        self.select_template(template_id)
        self.save_current_draft()
        template = self._selected_template()
        if template is None or not template.enabled:
            QMessageBox.information(self, "验证模板", "请先启用该模板。")
            return
        self._start_validation("validate-leg-template", {
            "template_id": template.template_id,
            "template_hash": template.template_hash,
            "dependency_hashes": template.dependency_hashes,
        }, {template.template_id})

    def validate_all_enabled(self) -> None:
        if self.layout_ref is None:
            return
        self.save_current_draft()
        enabled = [item for item in self.templates.templates if item.enabled and not item.orphaned]
        if not enabled:
            QMessageBox.information(self, "批量验证", "当前没有已启用模板。")
            return
        if QMessageBox.question(self, "批量验证", f"确定严格验证 {len(enabled)} 条已启用模板？") != QMessageBox.Yes:
            return
        self._start_validation("validate-all-leg-templates", {
            "template_hashes": {item.template_id: item.template_hash for item in enabled},
            "dependency_hashes": self.templates.dependency_hashes,
        }, {item.template_id for item in enabled})

    def _start_validation(self, job: str, params: dict[str, Any], checking: set[str]) -> None:
        token = uuid.uuid4().hex
        self.active_job_token = token
        self.checking_template_ids = set(checking)
        params = {**params, "job_token": token, "revision": self.revision}
        self._refresh_template_table()
        self._emit_time_summary()
        self.validationRequested.emit(job, params, token, self.revision)

    def accept_worker_result(self, payload: dict[str, Any], token: str, revision: int, project_root: str) -> bool:
        if self.layout_ref is None:
            return False
        if token != self.active_job_token or revision != self.revision or Path(project_root).resolve() != self.layout_ref.root:
            return False
        self.active_job_token = ""
        self.checking_template_ids.clear()
        self._load_documents(preserve_selection=True)
        self.statusMessage.emit("Leg 模板严格验证完成")
        self._emit_time_summary()
        return True

    def accept_worker_failure(self, token: str, message: str) -> None:
        if token != self.active_job_token:
            return
        self.active_job_token = ""
        self.checking_template_ids.clear()
        self._refresh_template_table()
        self.statusMessage.emit(f"Leg 模板验证未完成：{message}")
        self._emit_time_summary()

    def _refresh_instance_table(self) -> None:
        items = [item for item in self.instances.instances if item.template_id == self.selected_template_id]
        self.instance_table.setRowCount(len(items))
        for row, instance in enumerate(items):
            failures = "; ".join(item.message for item in instance.failures)
            values = (
                instance.instance_id, instance.from_state_key, instance.to_state_key,
                instance.from_unload_pose_profile_id or "—", instance.to_unload_pose_profile_id or "—",
                instance.state.value, str(instance.planned_time_ms),
                f"{instance.min_clearance_mm:.1f}" if instance.min_clearance_mm is not None else "—", failures or "—",
            )
            for column, value in enumerate(values):
                cell = QTableWidgetItem(value)
                cell.setData(Qt.UserRole, instance.instance_id)
                cell.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                self.instance_table.setItem(row, column, cell)
        self.error_text.clear()
        self.field_view.set_leg(None)
        if self.selected_instance_id and not any(
            item.instance_id == self.selected_instance_id for item in items
        ):
            self.selected_instance_id = ""
        self._emit_time_summary()

    def _instance_selection_changed(self) -> None:
        rows = self.instance_table.selectionModel().selectedRows()
        if not rows:
            return
        item = self.instance_table.item(rows[0].row(), 0)
        instance_id = str(item.data(Qt.UserRole) if item else "")
        instance = next((value for value in self.instances.instances if value.instance_id == instance_id), None)
        if instance is None:
            return
        self.selected_instance_id = instance.instance_id
        self.field_view.set_leg(instance.compiled_leg)
        lines = [f"{failure.code}: {failure.message}\n{failure.details}" for failure in instance.failures]
        if instance.analysis_metrics:
            lines.append(f"指标：{instance.analysis_metrics}")
        self.error_text.setPlainText("\n\n".join(lines))
        self._emit_time_summary()

    def current_time_summary(self) -> str:
        if self.active_job_token:
            return "模板最佳时间：正在验证 | 当前实例：正在验证"
        items = [
            item for item in self.instances.instances
            if item.template_id == self.selected_template_id and item.state.value == "PASSED"
        ]
        best = min((item.planned_time_ms for item in items), default=None)
        current = next(
            (item.planned_time_ms for item in items if item.instance_id == self.selected_instance_id),
            None,
        )
        best_text = "—" if best is None else f"{best / 1000.0:.2f} s"
        current_text = "—" if current is None else f"{current / 1000.0:.2f} s"
        return f"模板最佳时间：{best_text} | 当前实例：{current_text}"

    def _emit_time_summary(self) -> None:
        self.timeSummaryChanged.emit(self.current_time_summary())

    def export_document(self, document: str, target: str | Path) -> Path:
        if self.layout_ref is None:
            raise RuntimeError("尚未加载项目")
        self.save_current_draft()
        if document == "instances" and not self.instances.instances:
            raise RuntimeError("当前没有模板实例结果；请先严格验证模板。")
        if document == "report" and not self.report.template_reports:
            raise RuntimeError("当前没有模板验证报告；请先严格验证模板。")
        path = export_leg_template_document(self.layout_ref, document, target)
        self.statusMessage.emit(f"已输出：{path}")
        return path

    def export_all(self, target_dir: str | Path) -> tuple[Path, Path, Path]:
        if self.layout_ref is None:
            raise RuntimeError("尚未加载项目")
        self.save_current_draft()
        if not self.instances.instances or not self.report.template_reports:
            raise RuntimeError("当前没有完整的实例与验证报告；请先严格验证模板。")
        paths = export_all_leg_template_documents(self.layout_ref, target_dir)
        self.statusMessage.emit(f"已输出三个 Leg 模板 JSON：{target_dir}")
        return paths

    def _choose_export(self, document: str) -> None:
        names = {"templates": "leg_templates.json", "instances": "leg_template_instances.json", "report": "leg_template_validation_report.json"}
        path, _filter = QFileDialog.getSaveFileName(self, "输出 JSON", names[document], "JSON (*.json)")
        if path:
            try:
                self.export_document(document, path)
            except Exception as exc:  # noqa: BLE001
                QMessageBox.warning(self, "输出失败", str(exc))

    def _choose_export_all(self) -> None:
        directory = QFileDialog.getExistingDirectory(self, "选择全部输出目录")
        if directory:
            try:
                self.export_all(directory)
            except Exception as exc:  # noqa: BLE001
                QMessageBox.warning(self, "输出失败", str(exc))

    def keyPressEvent(self, event):  # type: ignore[override]
        if event.key() == Qt.Key_Delete:
            self.delete_selected_waypoint()
            event.accept()
            return
        super().keyPressEvent(event)


__all__ = ["LegTemplatePage"]
