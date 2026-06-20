"""V4 application shell built directly on the proven V3.5 editor widget tree.

The visual hierarchy, field view, tables, splitters, tabs and editing gestures come
from :mod:`hjmb_pathgen.py_ui.v35_base.editor`.  This module only replaces the
business callbacks with V4 MANUAL / SEMI_AUTO / FULL_AUTO services.
"""

from __future__ import annotations

import copy
import math
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

from PySide6.QtCore import QTimer
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_domain.semi_path import (
    ROUTE_A_SITE_SEQUENCE,
    ROUTE_B_SITE_SEQUENCE,
    route_family_from_site_sequence,
)
from hjmb_pathgen.py_io.codecs.bin_codec import load_bin
from hjmb_pathgen.py_io.codecs.json_codec import load_case, save_case, save_project
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.leg_clear_service import clear_optimized_leg_result
from hjmb_pathgen.py_services.case_draft_service import generate_case_draft
from hjmb_pathgen.py_services.project_bootstrap_service import bootstrap_v4_workspace
from hjmb_pathgen.py_services.mode_case_service import convert_full_auto_to_semi_auto
from hjmb_pathgen.py_ui.ui_state import LoadedProjectState
from hjmb_pathgen.py_workers.worker_process import WorkerJobHandle, start_worker_job

from .v35_base import editor as legacy
from .v35_base.path_models import (
    ACTIONS,
    ACTION_CODES,
    ACTION_MODE_ASYNC,
    ACTION_MODE_KINEMATIC,
    ACTION_MODE_STOP_AND_WAIT,
    EditPoint,
    FixedSite,
    MechanicalAction,
    PATH_MODE_FIXED_8,
    PATH_MODE_FREE,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    SITE_ID_FREE,
    TrajectoryNode,
    PlanResult,
    PlanSummary,
    ResolvedMechanicalAction,
    YAW_UNSPECIFIED_DDEG,
)

FULL_AUTO_SENTINEL = "FULL_AUTO"
MODE_NAMES = {
    GenerationMode.MANUAL: "手动模式",
    GenerationMode.SEMI_AUTO: "半自动模式",
    GenerationMode.FULL_AUTO: "全自动模式",
}
LOGICAL_SITE_KEYS = (
    "P_START",
    "P_PICK_1",
    "P_PICK_2L",
    "P_PICK_2R",
    "P_PICK_3",
    "P_DROP_1",
    "P_DROP_2",
    "P_DROP_3",
)


class V35ExactV4MainWindow(legacy.MainWindow):
    """The literal V3.5 GUI with V4 service callbacks."""

    def __init__(self, project_root: str | Path | None = None) -> None:
        self._v4_booting = True
        self._generation_mode = GenerationMode.MANUAL
        self._v4_state: LoadedProjectState | None = None
        self._v4_project_root = (
            Path(project_root).resolve(strict=False) if project_root else Path.cwd()
        )
        self._v4_worker: WorkerJobHandle | None = None
        self._v4_followup: tuple[str, dict[str, Any]] | None = None
        self._v4_current_job = ""
        self._v4_dirty = False
        self._v4_loading_case = False
        super().__init__()
        self._v4_booting = False

        # Keep the V3.5 visual hierarchy exactly: one field, one right panel,
        # the original four tabs and the original toolbar placement.
        self.setWindowTitle("HJMB V4.0 空间轨迹编辑器（三模式，V3.5界面基准）")
        self.resize(1680, 940)
        self.plan_timer.stop()

        self._v4_poll_timer = QTimer(self)
        self._v4_poll_timer.setInterval(200)
        self._v4_poll_timer.timeout.connect(self._poll_worker)

        # The legacy spin box remains as a hidden compatibility control.
        # The V4 GUI adds an explicit 0..359 drop-down so selecting a multi-
        # digit traj_id cannot be reset by a refresh of the legacy widgets.
        self.traj_id_spin.setRange(0, 359)
        self.traj_id_spin.setKeyboardTracking(False)
        self.traj_id_spin.setAccelerated(True)
        self._pending_traj_id = max(0, min(359, int(self.project.traj_id)))
        self.traj_id_spin.editingFinished.connect(self._commit_traj_id_selection)

        self._replace_mode_combo()
        self._rewire_toolbar()
        self._extend_fixed_site_tab()
        self._update_v4_mode_ui()
        self.update_status("请先打开一个 V4 项目目录；编辑不会自动规划")

        if project_root is not None:
            self.load_v4_project(project_root)

    # ------------------------------------------------------------------
    # Keep V3.5 layout, replace only toolbar semantics.
    # ------------------------------------------------------------------
    def _rewire_toolbar(self) -> None:
        mapping = {
            "新建": ("新建V4项目", self.new_project),
            "清空": ("清空当前路径", self.clear_project),
            "导入配置 JSON": ("打开V4项目", self.open_json),
            "保存配置 JSON": ("保存当前Case JSON", self.save_json),
            "导出配置 JSON": ("另存当前Case JSON", self.save_json_as),
            "导出 BIN": ("导出当前BIN", self.export_bin),
            "打开 BIN": ("打开V4 Case", self.open_bin),
            "适配场地": ("适配场地", self.field.fit_to_field),
            "重新规划": ("生成/更新当前路径", self.plan_now),
            "校验": ("验证当前路径", self.validate_current_project),
        }
        for action in self.findChildren(QAction):
            item = mapping.get(action.text())
            if item is None:
                continue
            text, callback = item
            try:
                action.triggered.disconnect()
            except (RuntimeError, TypeError):
                pass
            action.setText(text)
            action.triggered.connect(callback)

    def _replace_mode_combo(self) -> None:
        self.updating_ui = True
        self.path_mode_combo.clear()
        self.path_mode_combo.addItem("手动模式 MANUAL", PATH_MODE_FREE)
        self.path_mode_combo.addItem("半自动模式 SEMI_AUTO", PATH_MODE_FIXED_8)
        self.path_mode_combo.addItem("全自动模式 FULL_AUTO", FULL_AUTO_SENTINEL)
        self.path_mode_combo.setToolTip(
            "MANUAL：人工设置全部点和动作，不优化人工几何；"
            "SEMI_AUTO：人工设置8个逻辑锚点和动作，显式生成时优化路段；"
            "FULL_AUTO：按traj_id自动生成，结果只读，修改前转为半自动副本。"
        )
        self.updating_ui = False

    def _extend_fixed_site_tab(self) -> None:
        # Global task strip: it belongs to the right panel rather than one tab,
        # so progress and total time remain visible on every page.
        panel = self.right_tabs.parentWidget()
        panel_layout = panel.layout() if panel is not None else None
        task_strip = QWidget()
        task_layout = QHBoxLayout(task_strip)
        task_layout.setContentsMargins(0, 0, 0, 0)
        self.v4_task_label = QLabel("任务：空闲")
        self.v4_task_label.setMinimumWidth(120)
        self.v4_progress = QProgressBar()
        self.v4_progress.setRange(0, 100)
        self.v4_progress.setValue(0)
        self.v4_progress.setTextVisible(True)
        self.v4_total_time_label = QLabel("底盘运动时间：— | 总时间：—")
        self.v4_total_time_label.setMinimumWidth(300)
        stop_button = QPushButton("立即停止")
        stop_button.clicked.connect(self._cancel_worker)
        task_layout.addWidget(self.v4_task_label)
        task_layout.addWidget(self.v4_progress, 1)
        task_layout.addWidget(self.v4_total_time_label)
        task_layout.addWidget(stop_button)
        if panel_layout is not None:
            # The first row contains traj_id.  An explicit apply button makes
            # typed multi-digit IDs reliable even when the user immediately
            # clicks Generate without first leaving the spin box.
            top_item = panel_layout.itemAt(0)
            top_layout = top_item.layout() if top_item is not None else None
            if top_layout is not None:
                self.traj_id_combo = QComboBox()
                self.traj_id_combo.setMinimumWidth(115)
                self.traj_id_combo.setMaxVisibleItems(24)
                for traj_id in range(360):
                    self.traj_id_combo.addItem(f"P{traj_id:04d}", traj_id)
                self.traj_id_combo.setToolTip("选择 traj_id，范围 P0000~P0359")
                self.traj_id_combo.activated.connect(self._traj_id_combo_activated)
                self.traj_id_spin.hide()
                top_layout.insertWidget(1, self.traj_id_combo)
                apply_id = QPushButton("载入ID")
                apply_id.setToolTip("载入下拉框中选择的 traj_id 对应模式Case")
                apply_id.clicked.connect(self._commit_traj_id_selection)
                top_layout.insertWidget(2, apply_id)
                self._set_traj_id_controls(self._pending_traj_id)
            tab_index = panel_layout.indexOf(self.right_tabs)
            panel_layout.insertWidget(max(0, tab_index), task_strip)

        self.right_tabs.setTabText(
            self.right_tabs.indexOf(self.fixed_site_tab),
            "固定8点 / 最优路段 / 批量",
        )
        root_layout = self.fixed_site_tab.layout()
        for button in self.fixed_site_tab.findChildren(QPushButton):
            if button.text() == "导入固定点 JSON":
                button.setText("从V4项目重新加载")
                button.setToolTip("重新读取project.json和当前模式Case，不再导入V3.5固定点文件")
            elif button.text() == "导出固定点 JSON":
                button.setText("保存固定点到V4项目")
                button.setToolTip("保存公共姿态到project.json；半自动模式同时保存8个逻辑点到Case JSON")

        project_group = QGroupBox("V4 项目")
        project_layout = QVBoxLayout(project_group)
        row = QHBoxLayout()
        self.v4_project_edit = QLineEdit(str(self._v4_project_root))
        choose = QPushButton("打开项目")
        choose.clicked.connect(self._choose_project)
        save_project_button = QPushButton("保存公共配置")
        save_project_button.clicked.connect(lambda: self._save_project_config())
        row.addWidget(QLabel("项目目录"))
        row.addWidget(self.v4_project_edit, 1)
        row.addWidget(choose)
        row.addWidget(save_project_button)
        project_layout.addLayout(row)
        self.v4_project_status = QLabel("未加载")
        self.v4_project_status.setWordWrap(True)
        project_layout.addWidget(self.v4_project_status)
        root_layout.addWidget(project_group)

        batch_group = QGroupBox("最优路段 / 批量生成")
        batch_layout = QVBoxLayout(batch_group)
        row1 = QHBoxLayout()
        self.v4_leg_combo = QComboBox()
        self.v4_leg_combo.setMinimumWidth(280)
        for text, callback in (
            ("优化当前/缺失路段", self._optimize_missing),
            ("强制重算当前路段", self._reoptimize_selected_leg),
            ("清除当前最优路段", self._clear_selected_leg),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            row1.addWidget(button)
        row1.addWidget(QLabel("leg"))
        row1.addWidget(self.v4_leg_combo, 1)
        batch_layout.addLayout(row1)

        row2 = QHBoxLayout()
        for text, callback in (
            ("生成当前ID", self.plan_now),
            ("生成全部360", self._generate_all),
            ("验证全部", self._validate_all),
            ("转为半自动编辑", self._convert_to_semi_auto),
            ("设为最终版本", self._export_final),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            row2.addWidget(button)
        batch_layout.addLayout(row2)

        self.v4_log = QPlainTextEdit()
        self.v4_log.setReadOnly(True)
        self.v4_log.setMaximumBlockCount(800)
        self.v4_log.setMaximumHeight(140)
        batch_layout.addWidget(self.v4_log)
        root_layout.addWidget(batch_group)

    # ------------------------------------------------------------------
    # Disable V3.5 automatic planner.  Editing only marks V4 state stale.
    # ------------------------------------------------------------------
    def schedule_plan(self) -> None:
        if self._v4_booting or self._v4_loading_case:
            return
        self.plan_timer.stop()
        self._v4_dirty = True
        self.plan_result = None
        self.plan_error = "当前编辑尚未生成V4轨迹"
        if hasattr(self, "v4_total_time_label"):
            self.v4_total_time_label.setText("底盘运动时间：STALE | 总时间：STALE")
        self.update_status("已修改：仅标记 STALE，不会自动规划")

    def plan_now(self) -> None:
        if self._v4_booting:
            return
        if not self._ensure_v4_workspace("生成路径"):
            return
        try:
            self._save_project_config(show_message=False)
            if self._generation_mode != GenerationMode.FULL_AUTO:
                self._save_current_case_to_project()
        except Exception as exc:  # noqa: BLE001
            self._warn("保存Case失败", str(exc))
            return
        job = {
            GenerationMode.MANUAL: "generate-manual",
            GenerationMode.SEMI_AUTO: "generate-semi-auto",
            GenerationMode.FULL_AUTO: "generate-full-auto-one",
        }[self._generation_mode]
        params: dict[str, Any] = {
            "traj_id": self._current_traj_id(),
        }
        self._start_worker(job, params)

    # ------------------------------------------------------------------
    # Original V3.5 callbacks are preserved visually but made V4-aware.
    # ------------------------------------------------------------------
    def refresh_all(self, *args, **kwargs):  # type: ignore[override]
        super().refresh_all(*args, **kwargs)
        if hasattr(self, "traj_id_combo"):
            self._set_traj_id_controls(self.project.traj_id)
        if hasattr(self, "path_mode_combo"):
            self._set_mode_combo(self._generation_mode)
            self._update_v4_mode_ui()
        self._update_total_time_display()

    def path_mode_changed(self, _index: int) -> None:
        if self._v4_booting or self.updating_ui:
            return
        data = self.path_mode_combo.currentData()
        mode = {
            PATH_MODE_FREE: GenerationMode.MANUAL,
            PATH_MODE_FIXED_8: GenerationMode.SEMI_AUTO,
            FULL_AUTO_SENTINEL: GenerationMode.FULL_AUTO,
        }.get(data, GenerationMode.MANUAL)
        if self._v4_dirty and mode != self._generation_mode:
            answer = QMessageBox.question(
                self,
                "切换模式",
                "当前模式有未保存修改。仍然切换并丢弃这些界面修改？",
            )
            if answer != QMessageBox.Yes:
                self._set_mode_combo(self._generation_mode)
                return
        self._generation_mode = mode
        self.project.path_mode = (
            PATH_MODE_FREE if mode == GenerationMode.MANUAL else PATH_MODE_FIXED_8
        )
        self._load_current_mode_case()

    def _set_traj_id_controls(self, value: int) -> int:
        value = max(0, min(359, int(value)))
        self._pending_traj_id = value
        self.project.traj_id = value
        self.traj_id_spin.setValue(value)
        combo = getattr(self, "traj_id_combo", None)
        if combo is not None:
            index = combo.findData(value)
            if index >= 0:
                combo.setCurrentIndex(index)
        return value

    def _traj_id_changed(self, value: int) -> None:
        # Kept for the hidden legacy spin box.  No Case load is performed here.
        if self.updating_ui or self._v4_loading_case:
            return
        self._set_traj_id_controls(value)

    def _traj_id_combo_activated(self, _index: int) -> None:
        if self.updating_ui or self._v4_loading_case:
            return
        self._commit_traj_id_selection()

    def _current_traj_id(self) -> int:
        combo = getattr(self, "traj_id_combo", None)
        if combo is not None and combo.currentData() is not None:
            value = int(combo.currentData())
        else:
            self.traj_id_spin.interpretText()
            value = int(self.traj_id_spin.value())
        return self._set_traj_id_controls(value)

    def _commit_traj_id_selection(self) -> None:
        if self.updating_ui or self._v4_loading_case:
            return
        value = self._current_traj_id()
        if self._v4_state is not None:
            self._load_current_mode_case()
        else:
            self.update_status(f"traj_id 已设为 {value}")

    def parameter_changed(self, *_args) -> None:
        if self.updating_ui:
            return
        self.apply_parameter_widgets()
        self.schedule_plan()

    def on_point_moved(self, index: int, x_mm: int, y_mm: int) -> None:
        if self._generation_mode == GenerationMode.FULL_AUTO:
            self._offer_convert_from_full_auto()
            self._load_current_mode_case()
            return
        super().on_point_moved(index, x_mm, y_mm)

    def on_yaw_changed(self, index: int, yaw_ddeg: int) -> None:
        if self._generation_mode == GenerationMode.FULL_AUTO:
            self._offer_convert_from_full_auto()
            self._load_current_mode_case()
            return
        super().on_yaw_changed(index, yaw_ddeg)

    def on_point_item_changed(self, item) -> None:
        if self._generation_mode == GenerationMode.FULL_AUTO and not self.updating_ui:
            self._offer_convert_from_full_auto()
            self._load_current_mode_case()
            return
        super().on_point_item_changed(item)

    def on_action_item_changed(self, item) -> None:
        if self._generation_mode == GenerationMode.FULL_AUTO and not self.updating_ui:
            self._offer_convert_from_full_auto()
            self._load_current_mode_case()
            return
        super().on_action_item_changed(item)

    def on_fixed_site_item_changed(self, item) -> None:
        if self._generation_mode == GenerationMode.FULL_AUTO and not self.updating_ui:
            self._offer_convert_from_full_auto()
            self._load_current_mode_case()
            return
        super().on_fixed_site_item_changed(item)

    # ------------------------------------------------------------------
    # Toolbar commands, now all V4.
    # ------------------------------------------------------------------
    def new_project(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "新建V4项目目录", str(self._v4_project_root))
        if not path:
            return
        target = Path(path).resolve(strict=False)
        if (target / "project.json").exists():
            if QMessageBox.question(
                self,
                "项目已存在",
                "目标目录已经包含project.json。是否直接打开该V4项目？",
            ) == QMessageBox.Yes:
                self.load_v4_project(target, create_if_missing=False)
            return
        self._v4_project_root = target
        if hasattr(self, "v4_project_edit"):
            self.v4_project_edit.setText(str(target))
        if not self._ensure_v4_workspace("新建项目", root=target):
            return
        self._save_project_config(show_message=False)
        self.update_status(f"已创建V4项目：{target}")

    def clear_project(self) -> None:
        if QMessageBox.question(
            self,
            "清空当前路径",
            "只清空当前界面中的路径点和动作，不删除project.json、Case、leg_library或BIN。继续？",
        ) != QMessageBox.Yes:
            return
        self.project.points.clear()
        self.project.actions.clear()
        self.plan_result = None
        self._v4_dirty = True
        self.refresh_all()
        self.update_status("当前路径已清空；尚未保存")

    def open_json(self) -> None:
        self._choose_project()

    def save_json(self) -> None:
        if not self._ensure_v4_workspace("保存Case"):
            return
        try:
            self._save_project_config(show_message=False)
            path = self._save_current_case_to_project()
        except Exception as exc:  # noqa: BLE001
            self._warn("保存失败", str(exc))
            return
        self.update_status(f"已保存V4 Case：{path}")

    def save_json_as(self) -> None:
        # V4 mode directories and Pxxxx naming are authoritative; arbitrary paths
        # would break mode isolation.  Keep the button but explain the rule.
        self.save_json()
        if self._v4_state is not None:
            path = self._v4_state.layout.case_json_path_for_mode(
                self._current_traj_id(), self._generation_mode
            )
            QMessageBox.information(
                self,
                "V4 Case路径",
                f"V4不允许任意另存并绕过模式目录。当前Case已保存到：\n{path}",
            )

    def export_bin(self) -> None:
        self.plan_now()

    def open_bin(self) -> None:
        if self._v4_state is None:
            self._warn("打开Case", "请先打开V4项目。")
            return
        path, _ = QFileDialog.getOpenFileName(
            self,
            "打开V4 Case JSON",
            str(self._v4_state.layout.cases_dir),
            "JSON (*.json)",
        )
        if not path:
            return
        try:
            case = load_case(path, enforce_filename=False)
        except Exception as exc:  # noqa: BLE001
            self._warn("打开失败", str(exc))
            return
        self._generation_mode = case.generation_mode
        self._set_traj_id_controls(case.traj_id)
        self._put_case_in_state(case)
        self._load_case_into_legacy_view(case)

    def validate_current_project(self) -> None:
        if not self._ensure_v4_workspace("验证路径"):
            return
        try:
            self._save_project_config(show_message=False)
            if self._generation_mode != GenerationMode.FULL_AUTO:
                self._save_current_case_to_project()
        except Exception as exc:  # noqa: BLE001
            self._warn("验证失败", str(exc))
            return
        self._start_worker(
            "validate-current",
            {
                "traj_id": self._current_traj_id(),
                "generation_mode": self._generation_mode.value,
            },
        )

    # ------------------------------------------------------------------
    # Project/case conversion helpers.
    # ------------------------------------------------------------------
    def _choose_project(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "打开或创建V4项目", str(self._v4_project_root))
        if path:
            self.load_v4_project(path, create_if_missing=True)

    def _candidate_traj_csv(self, target_root: Path) -> Path | None:
        candidates = (
            target_root / "traj_id.csv",
            Path(__file__).resolve().parents[3] / "traj_id.csv",
            Path.cwd() / "traj_id.csv",
        )
        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _bootstrap_arguments(self) -> dict[str, Any]:
        common_sites = {}
        for index, key in enumerate(LOGICAL_SITE_KEYS):
            site = self.project.fixed_sites[index]
            common_sites[key] = {
                "configured": True,
                "x_mm": int(round(site.x_mm)),
                "y_mm": int(round(site.y_mm)),
                "yaw_ddeg": int(site.yaw_ddeg),
            }
        planner = self.project.planner
        vehicle_profile = self.project.vehicle_profile
        return {
            "project_id": self._v4_project_root.name or "HJMB_V40_PROJECT",
            "common_sites": common_sites,
            "vehicle": {
                "wheel": {
                    "radius_mm": float(vehicle_profile.wheel_radius_mm),
                    "rotation_radius_mm": float(vehicle_profile.rotation_radius_mm),
                    "plan_limit_rpm": int(vehicle_profile.wheel_plan_limit_rpm),
                    "hard_limit_rpm": int(vehicle_profile.wheel_hard_limit_rpm),
                },
                "footprint": {
                    "r_large_mm": float(vehicle_profile.r_large_mm),
                    "r_small_mm": float(vehicle_profile.r_small_mm),
                    "collision_resolution_mm": float(vehicle_profile.collision_resolution_mm),
                    "strict_validation_resolution_mm": float(vehicle_profile.strict_validation_resolution_mm),
                    "pickup_arc_segments": int(vehicle_profile.pickup_arc_segments),
                },
            },
            "dynamics": {
                "max_speed_mmps": int(planner.max_speed_mmps),
                "linear_accel_mmps2": int(planner.linear_accel_mmps2),
                "braking_accel_mmps2": int(planner.linear_accel_mmps2),
                "lateral_accel_mmps2": int(planner.lateral_accel_mmps2),
                "max_wz_ddegps": int(round(math.degrees(planner.max_wz_radps) * 10.0)),
                "angular_accel_moving_ddegps2": int(round(math.degrees(planner.angular_accel_moving_radps2) * 10.0)),
                "angular_accel_rotate_ddegps2": int(round(math.degrees(planner.angular_accel_rotate_radps2) * 10.0)),
                "dynamic_margin_ratio": 0.1,
            },
            "start_check": {
                "position_tolerance_mm": int(self.project.start_check.position_tolerance_mm),
                "yaw_tolerance_ddeg": int(self.project.start_check.yaw_tolerance_ddeg),
                "stable_time_ms": int(self.project.start_check.stable_time_ms),
            },
            "arrival_check": {
                "position_tolerance_mm": int(self.project.arrival_check.position_tolerance_mm),
                "yaw_tolerance_ddeg": int(self.project.arrival_check.yaw_tolerance_ddeg),
                "speed_tolerance_mmps": int(self.project.arrival_check.speed_tolerance_mmps),
                "wz_tolerance_ddegps": int(self.project.arrival_check.wz_tolerance_ddegps),
                "stable_time_ms": int(self.project.arrival_check.stable_time_ms),
            },
            "action_durations_ms": dict(self.project.mechanism_profile.action_duration_ms),
        }

    def _ensure_v4_workspace(self, reason: str, *, root: Path | None = None) -> bool:
        if self._v4_state is not None:
            return True
        draft_project = copy.deepcopy(self.project)
        target = (root or Path(self.v4_project_edit.text().strip() or self._v4_project_root)).resolve(strict=False)
        self._v4_project_root = target
        try:
            result = bootstrap_v4_workspace(
                target,
                **self._bootstrap_arguments(),
                source_traj_csv=self._candidate_traj_csv(target),
            )
            state = LoadedProjectState.load(result.layout.root)
        except Exception as exc:  # noqa: BLE001
            self._warn(f"{reason}失败", f"无法创建或加载V4工作区：\n{exc}")
            return False
        self._v4_state = state
        self._v4_project_root = state.layout.root
        self.v4_project_edit.setText(str(state.layout.root))
        for warning in result.warnings:
            self._append_log(f"提示：{warning}")
        self._refresh_leg_combo()
        if result.created_project:
            # Creating a workspace must not discard points/actions already edited before
            # project.json existed.  Keep the editor draft and let the following save or
            # planning command persist it as V4.
            self.project = draft_project
            self._v4_dirty = True
            self.refresh_all(selected_point=0 if self.project.points else None)
        else:
            self._apply_project_common_sites()
            self._load_current_mode_case()
        self.v4_project_status.setText(
            f"{state.layout.status().status.value} | project.json已就绪"
        )
        if result.created_project:
            self._append_log(f"已自动创建 {state.layout.project_json}")
        if result.created_route_table:
            self._append_log(f"已自动生成 {state.layout.route_case_table_json}")
        return True

    def load_v4_project(self, root: str | Path, *, create_if_missing: bool = False) -> bool:
        target = Path(root).resolve(strict=False)
        if not (target / "project.json").exists() and create_if_missing:
            self._v4_state = None
            self._v4_project_root = target
            self.v4_project_edit.setText(str(target))
            return self._ensure_v4_workspace("打开项目", root=target)
        try:
            state = LoadedProjectState.load(target)
        except Exception as exc:  # noqa: BLE001
            self._warn("打开项目失败", str(exc))
            return False
        self._v4_state = state
        self._v4_project_root = state.layout.root
        self.v4_project_edit.setText(str(state.layout.root))
        self._v4_dirty = False
        self._apply_project_common_sites()
        self._refresh_leg_combo()
        self._load_current_mode_case()
        reasons = list(state.layout.status().reasons)
        self.v4_project_status.setText(
            f"{state.layout.status().status.value} | " + ("；".join(reasons) if reasons else "项目已加载")
        )
        for warning in state.warnings:
            self._append_log(f"警告：{warning}")
        return True

    def _save_project_config(self, *, show_message: bool = True) -> Path | None:
        if not self._ensure_v4_workspace("保存公共配置"):
            return None
        assert self._v4_state is not None
        base = self._v4_state.project
        sites = {key: dict(value) for key, value in base.sites.items()}
        for index, key in enumerate(LOGICAL_SITE_KEYS):
            site = self.project.fixed_sites[index]
            sites[key] = {
                "configured": True,
                "x_mm": int(round(site.x_mm)),
                "y_mm": int(round(site.y_mm)),
                "yaw_ddeg": int(site.yaw_ddeg),
            }
        wheel = {
            **dict(base.vehicle.get("wheel", {})),
            "radius_mm": float(self.project.vehicle_profile.wheel_radius_mm),
            "rotation_radius_mm": float(self.project.vehicle_profile.rotation_radius_mm),
            "plan_limit_rpm": int(self.project.vehicle_profile.wheel_plan_limit_rpm),
            "hard_limit_rpm": int(self.project.vehicle_profile.wheel_hard_limit_rpm),
        }
        footprint = {
            **dict(base.vehicle.get("footprint", {})),
            "r_large_mm": float(self.project.vehicle_profile.r_large_mm),
            "r_small_mm": float(self.project.vehicle_profile.r_small_mm),
            "collision_resolution_mm": float(self.project.vehicle_profile.collision_resolution_mm),
            "strict_validation_resolution_mm": float(self.project.vehicle_profile.strict_validation_resolution_mm),
            "pickup_arc_segments": int(self.project.vehicle_profile.pickup_arc_segments),
            "field_boundary_footprint_profile": "LARGE_CIRCLE",
        }
        vehicle = {**dict(base.vehicle), "wheel": wheel, "footprint": footprint}
        planner = self.project.planner
        dynamics = {
            **dict(base.dynamics),
            "max_speed_mmps": int(planner.max_speed_mmps),
            "linear_accel_mmps2": int(planner.linear_accel_mmps2),
            "braking_accel_mmps2": int(planner.linear_accel_mmps2),
            "lateral_accel_mmps2": int(planner.lateral_accel_mmps2),
            "max_wz_ddegps": int(round(math.degrees(planner.max_wz_radps) * 10.0)),
            "angular_accel_moving_ddegps2": int(round(math.degrees(planner.angular_accel_moving_radps2) * 10.0)),
            "angular_accel_rotate_ddegps2": int(round(math.degrees(planner.angular_accel_rotate_radps2) * 10.0)),
        }
        start_check = {
            "position_tolerance_mm": int(self.project.start_check.position_tolerance_mm),
            "yaw_tolerance_ddeg": int(self.project.start_check.yaw_tolerance_ddeg),
            "stable_time_ms": int(self.project.start_check.stable_time_ms),
        }
        arrival_check = {
            "position_tolerance_mm": int(self.project.arrival_check.position_tolerance_mm),
            "yaw_tolerance_ddeg": int(self.project.arrival_check.yaw_tolerance_ddeg),
            "speed_tolerance_mmps": int(self.project.arrival_check.speed_tolerance_mmps),
            "wz_tolerance_ddegps": int(self.project.arrival_check.wz_tolerance_ddegps),
            "stable_time_ms": int(self.project.arrival_check.stable_time_ms),
        }
        action_profiles = {key: dict(value) for key, value in base.action_profiles.items()}
        for key, duration in self.project.mechanism_profile.action_duration_ms.items():
            if key in action_profiles:
                action_profiles[key]["estimated_time_ms"] = int(duration)
                action_profiles[key]["timeout_ms"] = max(
                    int(action_profiles[key].get("timeout_ms", 1000)),
                    int(duration) + 2000,
                )
        project = replace(
            base,
            sites=sites,
            vehicle=vehicle,
            dynamics=dynamics,
            start_check=start_check,
            arrival_check=arrival_check,
            action_profiles=action_profiles,
        )
        save_project(self._v4_state.layout.project_json, project)
        self._v4_state.project = project
        self._append_log("project.json公共姿态与规划参数已保存；没有自动规划")
        if show_message:
            self.update_status(f"已保存V4项目配置：{self._v4_state.layout.project_json}")
        return self._v4_state.layout.project_json

    def import_fixed_sites(self) -> None:
        if self._v4_state is None:
            self._choose_project()
            return
        self.load_v4_project(self._v4_state.layout.root, create_if_missing=False)
        self.update_status("已从project.json和当前模式Case重新加载")

    def export_fixed_sites(self) -> None:
        if not self._ensure_v4_workspace("保存固定点"):
            return
        project_path = self._save_project_config(show_message=False)
        case_path: Path | None = None
        if self._generation_mode == GenerationMode.SEMI_AUTO:
            try:
                case_path = self._save_current_case_to_project()
            except Exception as exc:  # noqa: BLE001
                self._warn("保存8点失败", str(exc))
                return
        message = f"公共配置已保存到：\n{project_path}"
        if case_path is not None:
            message += f"\n\n当前8个逻辑点已保存到：\n{case_path}"
        else:
            message += "\n\n8个固定点均已写入project.json；yaw=0xFFFF会原样保存，表示不约束到点方向。"
        QMessageBox.information(self, "V4固定点已保存", message)

    def _load_current_mode_case(self) -> None:
        if self._v4_state is None:
            if self._generation_mode == GenerationMode.SEMI_AUTO:
                self._prepare_empty_mode_view()
                return
            self._update_v4_mode_ui()
            return
        case = self._v4_state.current_case(self._current_traj_id(), self._generation_mode)
        if case is None:
            self._prepare_empty_mode_view()
        else:
            self._load_case_into_legacy_view(case)

    def _prepare_empty_mode_view(self) -> None:
        self._v4_loading_case = True
        try:
            if self._generation_mode == GenerationMode.MANUAL:
                self.project.path_mode = PATH_MODE_FREE
                self.project.points = [
                    EditPoint(point_id=0, type=POINT_TYPE_START, x_mm=0, y_mm=0, yaw_ddeg=0, exact_pass=True),
                    EditPoint(point_id=1, type=POINT_TYPE_ARRIVAL, x_mm=0, y_mm=0, yaw_ddeg=0, exact_pass=True),
                ]
            elif self._generation_mode == GenerationMode.SEMI_AUTO:
                # Semi-auto is user-authored: start with a genuinely empty
                # execution list. The eight configured anchors remain in the
                # fixed-site table and are added explicitly in the desired
                # order, including P_START.
                self.project.path_mode = PATH_MODE_FIXED_8
                self.project.points = []
            else:
                # FULL_AUTO is read-only and is shown only after an actual Case
                # has been generated.  An empty full-auto view must not resemble
                # a bogus route through all eight fixed points.
                self.project.path_mode = PATH_MODE_FIXED_8
                self.project.points = []
            self.project.actions = []
            self.plan_result = None
            self._v4_dirty = False
            self.refresh_all(selected_point=0 if self.project.points else None)
            self.update_status(f"{MODE_NAMES[self._generation_mode]}：当前ID尚无Case")
        finally:
            self._v4_loading_case = False

    def _canonical_fixed_points(self) -> list[EditPoint]:
        points: list[EditPoint] = []
        for index in range(8):
            site = self.project.fixed_sites[index]
            points.append(
                EditPoint(
                    point_id=index,
                    type=POINT_TYPE_START if index == 0 else POINT_TYPE_ARRIVAL,
                    site_id=index,
                    x_mm=site.x_mm,
                    y_mm=site.y_mm,
                    yaw_ddeg=site.yaw_ddeg,
                    exact_pass=True,
                )
            )
        return points

    def _load_case_into_legacy_view(
        self,
        case: CaseManifestV40,
        *,
        display_mode: GenerationMode | None = None,
    ) -> None:
        mode = display_mode or case.generation_mode
        self._v4_loading_case = True
        try:
            self._generation_mode = mode
            self.project.traj_id = case.traj_id
            self._set_traj_id_controls(case.traj_id)
            if mode == GenerationMode.MANUAL:
                self.project.path_mode = PATH_MODE_FREE
                self.project.points = []
                for index, item in enumerate((case.manual_path or {}).get("points", [])):
                    ptype = str(item["type"])
                    self.project.points.append(
                        EditPoint(
                            point_id=index,
                            type=ptype,
                            site_id=SITE_ID_FREE,
                            x_mm=float(item["x_mm"]),
                            y_mm=float(item["y_mm"]),
                            yaw_ddeg=(
                                YAW_UNSPECIFIED_DDEG
                                if ptype == POINT_TYPE_WAYPOINT
                                else int(item.get("yaw_ddeg", 0))
                            ),
                            max_speed_mmps=int(item.get("max_speed_mmps", 0) or 0),
                            corner_trim_mm=float(item.get("corner_trim_mm", 200.0 if ptype == POINT_TYPE_WAYPOINT else 0.0)),
                            exact_pass=bool(item.get("exact_pass", ptype != POINT_TYPE_WAYPOINT)),
                        )
                    )
            else:
                self.project.path_mode = PATH_MODE_FIXED_8
                self.project.points = self._route_points_from_case(case)
            self.project.actions = self._legacy_actions_from_case(case)
            self.plan_result = self._load_plan_result(case)
            self._v4_dirty = False
            self.refresh_all(selected_point=0 if self.project.points else None)
            self.update_status(f"已加载 {mode.value} P{case.traj_id:04d}")
        finally:
            self._v4_loading_case = False

    def _route_points_from_case(self, case: CaseManifestV40) -> list[EditPoint]:
        key_to_site = {key: index for index, key in enumerate(LOGICAL_SITE_KEYS)}
        points: list[EditPoint] = []

        if case.generation_mode == GenerationMode.SEMI_AUTO and case.semi_path is not None:
            for item in case.semi_path.get("points", []):
                ptype = str(item["type"])
                if ptype in (POINT_TYPE_START, POINT_TYPE_ARRIVAL):
                    key = str(item["site_key"])
                    site_id = key_to_site[key]
                    site = self.project.fixed_sites[site_id]
                    point = EditPoint(
                        point_id=len(points),
                        type=ptype,
                        site_id=site_id,
                        x_mm=site.x_mm,
                        y_mm=site.y_mm,
                        yaw_ddeg=site.yaw_ddeg,
                        exact_pass=True,
                    )
                else:
                    point = EditPoint(
                        point_id=len(points),
                        type=POINT_TYPE_WAYPOINT,
                        site_id=SITE_ID_FREE,
                        x_mm=float(item["x_mm"]),
                        y_mm=float(item["y_mm"]),
                        yaw_ddeg=YAW_UNSPECIFIED_DDEG,
                        max_speed_mmps=int(item.get("max_speed_mmps", 0) or 0),
                        corner_trim_mm=float(item.get("corner_trim_mm", 200.0)),
                        exact_pass=bool(item.get("exact_pass", False)),
                    )
                points.append(point)
            return points

        # FULL_AUTO: show only the selected legal route.  Never append the
        # unused 2L/2R branch or the remaining fixed anchors.
        route_family = str(case.selected_plan.get("route_family", ""))
        if route_family == "PICK_1_TO_3":
            order = ROUTE_A_SITE_SEQUENCE
        elif route_family == "PICK_3_TO_1":
            order = ROUTE_B_SITE_SEQUENCE
        else:
            order = ("P_START",)
        pose_by_id = {
            str(item.get("point_id")): dict(item.get("pose", {}))
            for item in case.logical_points
        }
        for key in order:
            site_id = key_to_site[key]
            pose = pose_by_id.get(key) or {
                "x_mm": self.project.fixed_sites[site_id].x_mm,
                "y_mm": self.project.fixed_sites[site_id].y_mm,
                "yaw_ddeg": self.project.fixed_sites[site_id].yaw_ddeg,
            }
            points.append(
                EditPoint(
                    point_id=len(points),
                    type=POINT_TYPE_START if not points else POINT_TYPE_ARRIVAL,
                    site_id=site_id,
                    x_mm=float(pose["x_mm"]),
                    y_mm=float(pose["y_mm"]),
                    yaw_ddeg=int(pose.get("yaw_ddeg", YAW_UNSPECIFIED_DDEG)),
                    exact_pass=True,
                )
            )
        return points

    def _legacy_actions_from_case(self, case: CaseManifestV40) -> list[MechanicalAction]:
        state_to_point: dict[str, int] = {}
        if case.generation_mode == GenerationMode.SEMI_AUTO and case.semi_path is not None:
            for index, item in enumerate(case.semi_path.get("points", [])):
                if str(item.get("type")) == POINT_TYPE_ARRIVAL:
                    state_to_point[str(item.get("state_id") or item.get("site_key"))] = index
        else:
            for index, point in enumerate(self.project.points):
                if point.type != POINT_TYPE_ARRIVAL:
                    continue
                if 0 <= point.site_id < len(LOGICAL_SITE_KEYS):
                    state_to_point[LOGICAL_SITE_KEYS[point.site_id]] = index
            for step in case.selected_plan.get("unload_sequence", []):
                state_id = f"DROP_STEP_{int(step.get('step_index', 0))}"
                ranks = [int(value) for value in step.get("target_ranks", [])]
                if ranks:
                    key = f"P_DROP_{ranks[0]}"
                    for index, point in enumerate(self.project.points):
                        if point.type == POINT_TYPE_ARRIVAL and 0 <= point.site_id < len(LOGICAL_SITE_KEYS) and LOGICAL_SITE_KEYS[point.site_id] == key:
                            state_to_point[state_id] = index
                            break

        result: list[MechanicalAction] = []
        for index, item in enumerate(case.actions.get("source", [])):
            action_raw = item.get("action", "NONE")
            if isinstance(action_raw, str):
                action_code = ACTION_CODES.get(action_raw.removeprefix("PATH_ACT_"), 0)
            else:
                action_code = int(action_raw)
            arrival_point_id = item.get("arrival_point_index", item.get("arrival_point_id"))
            if arrival_point_id is None and item.get("arrival_state_id") is not None:
                arrival_point_id = state_to_point.get(str(item["arrival_state_id"]))
            result.append(
                MechanicalAction(
                    action_seq=index,
                    action=action_code,
                    mode=str(item.get("mode", ACTION_MODE_ASYNC)).removeprefix("ACTION_MODE_"),
                    timeout_ms=int(item.get("timeout_ms", 3000)),
                    post_wait_ms=int(item.get("post_wait_ms", 0)),
                    arrival_point_id=(int(arrival_point_id) if arrival_point_id is not None else None),
                    accel_limit_mmps2=int(item.get("accel_limit_mmps2", 0)),
                    beta_limit_ddegps2=int(item.get("beta_limit_ddegps2", 0)),
                    wz_limit_ddegps=int(item.get("wz_limit_ddegps", 0)),
                    speed_limit_mmps=int(item.get("speed_limit_mmps", 0)),
                    stable_time_ms=int(item.get("stable_time_ms", 0)),
                )
            )
        return result

    def _save_current_case_to_project(self) -> Path:
        if self._v4_state is None:
            raise RuntimeError("未打开V4项目")
        traj_id = self._current_traj_id()
        if self._generation_mode == GenerationMode.MANUAL:
            case = self._manual_case_from_view(traj_id)
        elif self._generation_mode == GenerationMode.SEMI_AUTO:
            case = self._semi_case_from_view(traj_id)
        else:
            case = self._v4_state.current_case(traj_id, GenerationMode.FULL_AUTO)
            if case is None:
                raise RuntimeError("FULL_AUTO Case尚未生成")
        path = self._v4_state.layout.case_json_path_for_mode(traj_id, self._generation_mode)
        save_case(path, case)
        self._put_case_in_state(case)
        self._v4_dirty = False
        self._append_log(f"已保存 {path}")
        return path

    def _manual_case_from_view(self, traj_id: int) -> CaseManifestV40:
        points = []
        for point in self.project.points:
            item: dict[str, Any] = {
                "type": point.type,
                "x_mm": int(round(point.x_mm)),
                "y_mm": int(round(point.y_mm)),
            }
            if point.type in (POINT_TYPE_START, POINT_TYPE_ARRIVAL):
                item["yaw_ddeg"] = int(point.yaw_ddeg)
                item["exact_pass"] = True
            else:
                item["exact_pass"] = bool(point.exact_pass)
                item["corner_trim_mm"] = float(point.corner_trim_mm)
                if point.max_speed_mmps > 0:
                    item["max_speed_mmps"] = int(point.max_speed_mmps)
            points.append(item)
        existing = self._v4_state.current_case(traj_id, GenerationMode.MANUAL) if self._v4_state else None
        data = existing.to_dict() if existing is not None else self._empty_manual_case(traj_id)
        data["manual_path"] = {"points": points}
        data["semi_path"] = None
        data["logical_points"] = []
        data["auxiliary_points"] = []
        data["leg_refs"] = []
        data["actions"] = {"source": self._action_source_from_view(), "compiled": []}
        data["review"] = {
            **dict(data.get("review", {})),
            "state": "STALE",
            "approved": False,
            "detached_from_library": True,
            "manual_override": True,
            "stale_reason": "V3.5基准GUI编辑",
        }
        return CaseManifestV40.from_dict(data)

    def _semi_case_from_view(self, traj_id: int) -> CaseManifestV40:
        assert self._v4_state is not None
        semi_points: list[dict[str, Any]] = []
        fixed_sequence: list[str] = []
        for point in self.project.points:
            if point.type in (POINT_TYPE_START, POINT_TYPE_ARRIVAL):
                if not 0 <= point.site_id < len(LOGICAL_SITE_KEYS):
                    raise ValueError("半自动模式的START/ARRIVAL必须从8个固定点中选择")
                site_key = LOGICAL_SITE_KEYS[point.site_id]
                fixed_sequence.append(site_key)
                semi_points.append(
                    {
                        "type": point.type,
                        "site_key": site_key,
                        "state_id": site_key,
                    }
                )
            elif point.type == POINT_TYPE_WAYPOINT:
                item: dict[str, Any] = {
                    "type": POINT_TYPE_WAYPOINT,
                    "x_mm": int(round(point.x_mm)),
                    "y_mm": int(round(point.y_mm)),
                    "exact_pass": bool(point.exact_pass),
                    "corner_trim_mm": float(point.corner_trim_mm),
                }
                if point.max_speed_mmps > 0:
                    item["max_speed_mmps"] = int(point.max_speed_mmps)
                semi_points.append(item)
            else:
                raise ValueError(f"半自动模式不支持点类型：{point.type}")
        route_family = route_family_from_site_sequence(tuple(fixed_sequence))

        # Drop STOP actions from a generated full-auto Case use DROP_STEP_n.
        # Give matching fixed drop rows those stable IDs while user-created
        # point-index actions continue to work unchanged.
        route_name = route_family.name
        full = self._v4_state.current_case(traj_id, GenerationMode.FULL_AUTO)
        if full is None:
            full = generate_case_draft(self._v4_state.layout, traj_id).case
            self._v4_state.full_auto_cases[traj_id] = full
        candidate = next(
            (
                dict(item)
                for item in full.selected_plan.get("candidates", [])
                if str(item.get("route_family")) == route_name
            ),
            None,
        )
        if candidate is None:
            candidate = dict(full.selected_plan)
        candidate["route_family"] = route_name
        candidate["yaw_direction"] = "SHORTEST"
        candidate["locked_by_user"] = True
        candidate["selection_state"] = "USER_SEMI_AUTO"
        candidate["drop_targets"] = list(full.selected_plan.get("drop_targets", []))
        drop_state_by_rank: dict[int, str] = {}
        for step in candidate.get("unload_sequence", []):
            state_id = f"DROP_STEP_{int(step.get('step_index', 0))}"
            for rank in step.get("target_ranks", []):
                drop_state_by_rank[int(rank)] = state_id
        for item in semi_points:
            key = str(item.get("site_key", ""))
            if key.startswith("P_DROP_"):
                item["state_id"] = drop_state_by_rank.get(int(key.rsplit("_", 1)[1]), key)

        existing = self._v4_state.current_case(traj_id, GenerationMode.SEMI_AUTO)
        base = existing.to_dict() if existing is not None else full.to_dict()
        base.update(
            {
                "generation_mode": GenerationMode.SEMI_AUTO.value,
                "selected_plan": candidate,
                "manual_path": None,
                "semi_path": {"points": semi_points},
                "logical_points": [],
                "auxiliary_points": [],
                "arrival_states": [],
                "leg_refs": [],
                "actions": {"source": self._action_source_from_view(), "compiled": []},
                "derived_from": {
                    "generation_mode": GenerationMode.FULL_AUTO.value,
                    "traj_id": traj_id,
                    "case_hash": str(full.hashes.get("case_hash", "")),
                },
                "review": {
                    **dict(base.get("review", {})),
                    "state": "STALE",
                    "approved": False,
                    "detached_from_library": True,
                    "manual_override": True,
                    "override_reason": "user-authored ordered semi-auto path",
                    "stale_reason": "半自动路径已编辑，等待显式生成",
                },
            }
        )
        return CaseManifestV40.from_dict(base)

    def _action_source_from_view(self) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        for action in self.project.actions:
            item: dict[str, Any] = {
                "action": ACTIONS.get(action.action, "NONE"),
                "mode": action.mode,
                "timeout_ms": int(action.timeout_ms),
                "post_wait_ms": int(action.post_wait_ms),
            }
            if action.mode == ACTION_MODE_STOP_AND_WAIT:
                if action.arrival_point_id is None:
                    raise ValueError(f"STOP_AND_WAIT动作{action.action_seq}未绑定ARRIVAL点")
                item["arrival_point_index"] = int(action.arrival_point_id)
            elif action.mode == ACTION_MODE_KINEMATIC:
                item.update(
                    {
                        "accel_limit_mmps2": int(action.accel_limit_mmps2),
                        "beta_limit_ddegps2": int(action.beta_limit_ddegps2),
                        "wz_limit_ddegps": int(action.wz_limit_ddegps),
                        "speed_limit_mmps": int(action.speed_limit_mmps),
                        "stable_time_ms": int(action.stable_time_ms),
                    }
                )
            result.append(item)
        return result

    @staticmethod
    def _empty_manual_case(traj_id: int) -> dict[str, Any]:
        return {
            "format": "HJMB_ROUTE_CASE_JSON_V40",
            "storage_mode": "REFERENCED",
            "generation_mode": "MANUAL",
            "traj_id": traj_id,
            "bean_code": traj_id // 60,
            "drop_code": traj_id % 60,
            "source_mapping": {"manual": True},
            "selected_plan": {
                "route_family": "MANUAL",
                "vehicle_bin_assignment": {},
                "drop_targets": [],
                "unload_sequence": [],
                "yaw_direction": "SHORTEST",
                "locked_by_user": True,
            },
            "manual_path": {"points": []},
            "logical_points": [],
            "auxiliary_points": [],
            "arrival_states": [],
            "leg_refs": [],
            "actions": {"source": [], "compiled": []},
            "finish": {"mode": "AT_FINAL_DROP"},
            "estimates": {},
            "hashes": {},
            "review": {
                "state": "STALE",
                "detached_from_library": True,
                "manual_override": True,
                "approved": False,
                "override_reason": "manual V3.5-base GUI Case",
            },
        }

    def _put_case_in_state(self, case: CaseManifestV40) -> None:
        if self._v4_state is None:
            return
        mapping = {
            GenerationMode.MANUAL: self._v4_state.manual_cases,
            GenerationMode.SEMI_AUTO: self._v4_state.semi_auto_cases,
            GenerationMode.FULL_AUTO: self._v4_state.full_auto_cases,
        }[case.generation_mode]
        mapping[case.traj_id] = case

    def _apply_project_common_sites(self) -> None:
        if self._v4_state is None:
            return
        for index, key in enumerate(LOGICAL_SITE_KEYS):
            raw = self._v4_state.project.sites[key]
            site = self.project.fixed_sites[index]
            site.x_mm = float(raw["x_mm"])
            site.y_mm = float(raw["y_mm"])
            site.yaw_ddeg = int(raw["yaw_ddeg"])
        footprint = dict(self._v4_state.project.vehicle.get("footprint", {}))
        profile = self.project.vehicle_profile
        profile.r_large_mm = float(footprint.get("r_large_mm", profile.r_large_mm))
        profile.r_small_mm = float(footprint.get("r_small_mm", profile.r_small_mm))
        profile.collision_resolution_mm = float(
            footprint.get("collision_resolution_mm", profile.collision_resolution_mm)
        )
        profile.strict_validation_resolution_mm = float(
            footprint.get(
                "strict_validation_resolution_mm",
                profile.strict_validation_resolution_mm,
            )
        )
        profile.pickup_arc_segments = int(
            footprint.get("pickup_arc_segments", profile.pickup_arc_segments)
        )

    # ------------------------------------------------------------------
    # Worker and batch controls.
    # ------------------------------------------------------------------
    def _start_worker(
        self,
        job: str,
        params: dict[str, Any],
        *,
        continuation: bool = False,
    ) -> None:
        if not self._ensure_v4_workspace("启动任务"):
            return
        if self._v4_worker is not None and self._v4_worker.is_alive():
            self._warn("任务繁忙", "已有worker正在运行。")
            return
        try:
            self._v4_worker = start_worker_job(self._v4_state.layout.root, job, params)
        except Exception as exc:  # noqa: BLE001
            self._warn("启动失败", str(exc))
            return
        self._v4_current_job = job
        if not continuation:
            self._v4_followup = None
            self.v4_progress.setValue(0)
        self.v4_task_label.setText(f"任务：{job}")
        self._v4_poll_timer.start()
        prefix = "继续任务" if continuation else "启动任务"
        self._append_log(f"{prefix} {job}: {params}")

    def _poll_worker(self) -> None:
        if self._v4_worker is None:
            return
        for message in self._v4_worker.poll():
            payload = message.payload
            if message.kind == "progress":
                if "percent" in payload:
                    self.v4_progress.setValue(max(0, min(100, int(payload["percent"]))))
                stage = str(payload.get("stage", "PROGRESS"))
                self.v4_task_label.setText(f"任务：{stage}")
                self._append_log(f"[{payload.get('stage', 'PROGRESS')}] {payload.get('message', '')}")
            elif message.kind == "result":
                followup = payload.get("followup")
                if isinstance(followup, dict) and followup.get("job"):
                    self._v4_followup = (
                        str(followup["job"]),
                        dict(followup.get("params", {})),
                    )
                    self.v4_progress.setValue(max(78, self.v4_progress.value()))
                    self.v4_task_label.setText("任务：准备在新进程中装配")
                    prepared = payload.get("prepared_candidate_id")
                    if prepared:
                        self._append_log(f"候选 {prepared} 已完成优化；即将启动干净进程装配")
                    else:
                        self._append_log("依赖路段已就绪；即将启动干净进程装配")
                else:
                    self.v4_progress.setValue(100)
                    self.v4_task_label.setText("任务：完成")
                    generation = payload.get("generation")
                    if isinstance(generation, dict):
                        timing = generation.get("timing", {})
                        motion = timing.get("motion_time_ms")
                        total = timing.get("total_time_ms")
                        self._append_log(
                            f"任务完成: P{int(generation.get('traj_id', 0)):04d}; "
                            f"底盘运动时间={motion} ms; 总时间={total} ms"
                        )
                    else:
                        self._append_log(f"任务完成: {payload}")
            elif message.kind == "error":
                self._v4_followup = None
                self.v4_task_label.setText("任务：失败")
                self._append_log(f"任务失败: {payload.get('error', payload)}")
            elif message.kind == "cancelled":
                self._v4_followup = None
                self.v4_task_label.setText("任务：已停止")
                self._append_log("任务已取消")
        if self._v4_worker.is_alive():
            return

        self._v4_worker.join(0.1)
        self._v4_worker.close()
        self._v4_worker = None
        self._reload_v4_state()

        followup = self._v4_followup
        self._v4_followup = None
        if followup is not None:
            job, params = followup
            self._start_worker(job, params, continuation=True)
            self.v4_progress.setValue(max(80, self.v4_progress.value()))
            return

        self._v4_poll_timer.stop()
        self._v4_current_job = ""

    def _cancel_worker(self) -> None:
        if self._v4_worker is not None and self._v4_worker.is_alive():
            worker = self._v4_worker
            worker.cancel()
            worker.join(0.05)
            worker.close()
            self._v4_worker = None
            self._v4_followup = None
            self._v4_current_job = ""
            self._v4_poll_timer.stop()
            self.v4_task_label.setText("任务：已停止")
            self._append_log("已立即终止worker")
            self._reload_v4_state()
        else:
            self.v4_task_label.setText("任务：空闲")

    def _reload_v4_state(self) -> None:
        if self._v4_state is None:
            return
        root = self._v4_state.layout.root
        try:
            self._v4_state = LoadedProjectState.load(root)
            self._apply_project_common_sites()
            self._refresh_leg_combo()
            self._load_current_mode_case()
            self._update_total_time_display()
        except Exception as exc:  # noqa: BLE001
            self._append_log(f"任务后重新加载失败: {exc}")

    def _optimize_missing(self) -> None:
        self._start_worker(
            "optimize-missing-legs",
            {},
        )

    def _reoptimize_selected_leg(self) -> None:
        leg_id = str(self.v4_leg_combo.currentData() or "")
        if not leg_id:
            self._warn("重算失败", "请先选择leg。")
            return
        self._start_worker(
            "reoptimize-current-leg",
            {"leg_id": leg_id},
        )

    def _clear_selected_leg(self) -> None:
        if self._v4_state is None:
            return
        leg_id = str(self.v4_leg_combo.currentData() or "")
        if not leg_id:
            self._warn("清除失败", "请先选择leg。")
            return
        if QMessageBox.question(
            self,
            "清除当前最优路段",
            f"确定从leg_library.json清除 {leg_id} 的优化结果？\n"
            "不会修改project.json；依赖Case将标记STALE；不会自动重算。",
        ) != QMessageBox.Yes:
            return
        try:
            clear_optimized_leg_result(self._v4_state.layout, leg_id, confirm_leg_id=leg_id)
        except Exception as exc:  # noqa: BLE001
            self._warn("清除失败", str(exc))
            return
        self._append_log(f"已清除 {leg_id}")
        self._reload_v4_state()

    def _generate_all(self) -> None:
        if self._generation_mode != GenerationMode.FULL_AUTO:
            self._warn("批量生成", "生成全部360只适用于全自动模式。")
            return
        self._start_worker(
            "generate-full-auto-all",
            {},
        )

    def _validate_all(self) -> None:
        self._start_worker("validate-all", {})

    def _convert_to_semi_auto(self) -> None:
        if self._v4_state is None:
            return
        traj_id = self._current_traj_id()
        try:
            converted = convert_full_auto_to_semi_auto(self._v4_state.layout, traj_id)
        except Exception as exc:  # noqa: BLE001
            self._warn("转换失败", str(exc))
            return
        self._v4_state.semi_auto_cases[traj_id] = converted
        self._generation_mode = GenerationMode.SEMI_AUTO
        self._load_case_into_legacy_view(converted)

    def _offer_convert_from_full_auto(self) -> None:
        if QMessageBox.question(
            self,
            "全自动结果只读",
            "全自动Case不能直接修改。是否复制为半自动Case后继续编辑？",
        ) == QMessageBox.Yes:
            self._convert_to_semi_auto()

    def _export_final(self) -> None:
        if self._v4_state is None:
            return
        self._start_worker(
            "export-final",
            {
                "traj_id": self._current_traj_id(),
                "generation_mode": self._generation_mode.value,
                "profile": "default",
                "approve": True,
            },
        )

    def _refresh_leg_combo(self) -> None:
        if not hasattr(self, "v4_leg_combo"):
            return
        self.v4_leg_combo.clear()
        if self._v4_state is None or self._v4_state.leg_library is None:
            return
        for leg in self._v4_state.leg_library.legs:
            self.v4_leg_combo.addItem(
                f"{leg.leg_id} | {leg.state.value}",
                leg.leg_id,
            )

    # ------------------------------------------------------------------
    # Display generated V4 trajectory in the exact old field widget.
    # ------------------------------------------------------------------
    def _load_plan_result(self, case: CaseManifestV40) -> PlanResult | None:
        if self._v4_state is None:
            return None
        path = self._v4_state.layout.bin_path_for_mode(case.traj_id, case.generation_mode)
        if not path.exists():
            return None
        try:
            compiled = load_bin(path)
        except Exception as exc:  # noqa: BLE001
            self._append_log(f"BIN显示加载失败: {exc}")
            return None
        nodes: list[TrajectoryNode] = []
        max_rpm = 0.0
        for item in compiled.nodes:
            speed = math.hypot(item.vx_mmps, item.vy_mmps)
            nodes.append(
                TrajectoryNode(
                    s_mm=float(item.s_mm),
                    x_mm=float(item.x_mm),
                    y_mm=float(item.y_mm),
                    yaw_rad=math.radians(float(item.yaw_ddeg) / 10.0),
                    vx_mmps=float(item.vx_mmps),
                    vy_mmps=float(item.vy_mmps),
                    wz_radps=math.radians(float(item.wz_ddegps) / 10.0),
                    arrival_id=int(item.arrival_id),
                    flags=int(item.flags),
                    speed_mmps=speed,
                )
            )
        resolved_actions = [
            ResolvedMechanicalAction(
                action_seq=int(item.action_seq),
                action=int(item.action),
                mode={0: ACTION_MODE_STOP_AND_WAIT, 1: ACTION_MODE_ASYNC, 2: ACTION_MODE_KINEMATIC}.get(int(item.mode), ACTION_MODE_ASYNC),
                arrival_id=int(item.arrival_id),
                timeout_ms=int(item.timeout_ms),
                post_wait_ms=int(item.post_wait_ms),
                check_start_s_mm=int(item.check_start_s_mm),
                accel_limit_mmps2=int(item.accel_limit_mmps2),
                beta_limit_ddegps2=int(item.beta_limit_ddegps2),
                wz_limit_ddegps=int(item.wz_limit_ddegps),
                speed_limit_mmps=int(item.speed_limit_mmps),
                stable_time_ms=int(item.stable_time_ms),
            )
            for item in compiled.actions
        ]
        total_time = int(compiled.header.planned_motion_time_ms)
        return PlanResult(
            nodes=nodes,
            actions=resolved_actions,
            summary=PlanSummary(
                total_length_mm=float(compiled.header.total_length_mm),
                formal_time_ms=total_time,
                estimated_total_time_ms=int(case.estimates.get("planned_total_estimate_ms", total_time)),
                max_speed_mmps=max((node.speed_mmps for node in nodes), default=0.0),
                max_wheel_rpm=max_rpm,
            ),
        )

    # ------------------------------------------------------------------
    # Small UI helpers.
    # ------------------------------------------------------------------
    def _update_total_time_display(self) -> None:
        if not hasattr(self, "v4_total_time_label"):
            return
        motion_ms: int | None = None
        total_ms: int | None = None
        if self.plan_result is not None:
            motion_ms = int(self.plan_result.summary.formal_time_ms)
            total_ms = int(self.plan_result.summary.estimated_total_time_ms)
        elif self._v4_state is not None:
            case = self._v4_state.current_case(
                self._current_traj_id(), self._generation_mode
            )
            if case is not None:
                raw_motion = case.estimates.get("planned_motion_time_ms")
                raw_total = case.estimates.get("planned_total_estimate_ms")
                if isinstance(raw_motion, (int, float)):
                    motion_ms = int(raw_motion)
                if isinstance(raw_total, (int, float)):
                    total_ms = int(raw_total)
        motion_text = "—" if motion_ms is None else f"{motion_ms / 1000.0:.2f} s"
        total_text = "—" if total_ms is None else f"{total_ms / 1000.0:.2f} s"
        self.v4_total_time_label.setText(
            f"底盘运动时间：{motion_text} | 总时间：{total_text}"
        )

    def _set_mode_combo(self, mode: GenerationMode) -> None:
        if not hasattr(self, "path_mode_combo"):
            return
        data = {
            GenerationMode.MANUAL: PATH_MODE_FREE,
            GenerationMode.SEMI_AUTO: PATH_MODE_FIXED_8,
            GenerationMode.FULL_AUTO: FULL_AUTO_SENTINEL,
        }[mode]
        self.updating_ui = True
        index = self.path_mode_combo.findData(data)
        if index >= 0:
            self.path_mode_combo.setCurrentIndex(index)
        self.updating_ui = False

    def _update_v4_mode_ui(self) -> None:
        if not hasattr(self, "point_table"):
            return
        readonly = self._generation_mode == GenerationMode.FULL_AUTO
        self.point_table.setEnabled(not readonly)
        self.action_table.setEnabled(not readonly)
        self.fixed_site_table.setEnabled(not readonly)
        mode_text = MODE_NAMES[self._generation_mode]
        suffix = "（只读，修改请转半自动）" if readonly else "（编辑后仅STALE，不自动规划）"
        if hasattr(self, "status_label"):
            self.status_label.setToolTip(f"当前模式：{mode_text}{suffix}")

    def _append_log(self, text: str) -> None:
        if hasattr(self, "v4_log"):
            self.v4_log.appendPlainText(text)

    def _warn(self, title: str, text: str) -> None:
        QMessageBox.warning(self, title, text)
        self._append_log(f"{title}: {text}")


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    window = V35ExactV4MainWindow()
    window.show()
    return app.exec()


__all__ = ["V35ExactV4MainWindow", "main"]
