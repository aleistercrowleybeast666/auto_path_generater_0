"""V4 application shell built directly on the proven V3.5 editor widget tree.

The visual hierarchy, field view, tables, splitters, tabs and editing gestures come
from :mod:`hjmb_pathgen.py_ui.v35_base.editor`.  This module only replaces the
business callbacks with V4 MANUAL / SEMI_AUTO / FULL_AUTO services.
"""

from __future__ import annotations

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
from hjmb_pathgen.py_io.codecs.bin_codec import load_bin
from hjmb_pathgen.py_io.codecs.json_codec import load_case, save_case, save_project
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.leg_clear_service import clear_optimized_leg_result
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
        self.right_tabs.setTabText(
            self.right_tabs.indexOf(self.fixed_site_tab),
            "固定8点 / 最优路段 / 批量",
        )
        root_layout = self.fixed_site_tab.layout()

        project_group = QGroupBox("V4 项目")
        project_layout = QVBoxLayout(project_group)
        row = QHBoxLayout()
        self.v4_project_edit = QLineEdit(str(self._v4_project_root))
        choose = QPushButton("打开项目")
        choose.clicked.connect(self._choose_project)
        save_project_button = QPushButton("保存公共配置")
        save_project_button.clicked.connect(self._save_project_config)
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
        self.v4_profile_combo = QComboBox()
        for value in ("QUICK", "STANDARD", "FINAL"):
            self.v4_profile_combo.addItem(value, value)
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
        row1.addWidget(QLabel("profile"))
        row1.addWidget(self.v4_profile_combo)
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
            ("停止", self._cancel_worker),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            row2.addWidget(button)
        batch_layout.addLayout(row2)

        self.v4_progress = QProgressBar()
        self.v4_progress.setRange(0, 100)
        self.v4_progress.setValue(0)
        self.v4_progress.hide()
        batch_layout.addWidget(self.v4_progress)
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
        self.update_status("已修改：仅标记 STALE，不会自动规划")

    def plan_now(self) -> None:
        if self._v4_booting:
            return
        if self._v4_state is None:
            self._warn("生成失败", "请先打开包含 project.json 的 V4 项目目录。")
            return
        try:
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
            "traj_id": int(self.traj_id_spin.value()),
            "profile": str(self.v4_profile_combo.currentData() or "STANDARD"),
        }
        self._start_worker(job, params)

    # ------------------------------------------------------------------
    # Original V3.5 callbacks are preserved visually but made V4-aware.
    # ------------------------------------------------------------------
    def refresh_all(self, *args, **kwargs):  # type: ignore[override]
        super().refresh_all(*args, **kwargs)
        if hasattr(self, "path_mode_combo"):
            self._set_mode_combo(self._generation_mode)
            self._update_v4_mode_ui()

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

    def _traj_id_changed(self, value: int) -> None:
        if self.updating_ui:
            return
        self.project.traj_id = value
        if self._v4_state is not None:
            self._load_current_mode_case()
        else:
            self.update_status()

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
        target = Path(path)
        if any(target.iterdir()):
            self._warn("新建失败", "目标目录必须为空。")
            return
        if self._v4_state is None:
            self._warn("新建失败", "请先打开一个V4项目作为公共配置模板。")
            return
        try:
            layout = ProjectLayout.create(target, self._v4_state.project)
            source_csv = self._v4_state.layout.traj_id_csv
            if source_csv.exists():
                import shutil

                shutil.copy2(source_csv, layout.traj_id_csv)
            self.load_v4_project(target)
        except Exception as exc:  # noqa: BLE001
            self._warn("新建失败", str(exc))

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
        if self._v4_state is None:
            self._warn("保存失败", "请先打开V4项目。")
            return
        try:
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
                int(self.traj_id_spin.value()), self._generation_mode
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
        self.traj_id_spin.setValue(case.traj_id)
        self._put_case_in_state(case)
        self._load_case_into_legacy_view(case)

    def validate_current_project(self) -> None:
        if self._v4_state is None:
            self._warn("验证失败", "请先打开V4项目。")
            return
        try:
            if self._generation_mode != GenerationMode.FULL_AUTO:
                self._save_current_case_to_project()
        except Exception as exc:  # noqa: BLE001
            self._warn("验证失败", str(exc))
            return
        self._start_worker(
            "validate-current",
            {
                "traj_id": int(self.traj_id_spin.value()),
                "generation_mode": self._generation_mode.value,
            },
        )

    # ------------------------------------------------------------------
    # Project/case conversion helpers.
    # ------------------------------------------------------------------
    def _choose_project(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "打开V4项目", str(self._v4_project_root))
        if path:
            self.load_v4_project(path)

    def load_v4_project(self, root: str | Path) -> bool:
        try:
            state = LoadedProjectState.load(root)
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

    def _save_project_config(self) -> None:
        if self._v4_state is None:
            self._warn("保存失败", "请先打开V4项目。")
            return
        sites = {key: dict(value) for key, value in self._v4_state.project.sites.items()}
        for index, key in enumerate(LOGICAL_SITE_KEYS[:5]):
            site = self.project.fixed_sites[index]
            sites[key] = {
                "configured": True,
                "x_mm": int(round(site.x_mm)),
                "y_mm": int(round(site.y_mm)),
                "yaw_ddeg": int(site.yaw_ddeg),
            }
        project = replace(self._v4_state.project, sites=sites)
        save_project(self._v4_state.layout.project_json, project)
        self._v4_state.project = project
        self._append_log("project.json公共姿态已保存；没有自动规划")

    def _load_current_mode_case(self) -> None:
        if self._v4_state is None:
            self._update_v4_mode_ui()
            return
        case = self._v4_state.current_case(int(self.traj_id_spin.value()), self._generation_mode)
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
                self.project.actions = []
            elif self._generation_mode == GenerationMode.SEMI_AUTO:
                full = self._v4_state.current_case(int(self.traj_id_spin.value()), GenerationMode.FULL_AUTO)
                if full is not None:
                    self._load_case_into_legacy_view(full, display_mode=GenerationMode.SEMI_AUTO)
                    self._v4_dirty = True
                    return
                self.project.path_mode = PATH_MODE_FIXED_8
                self.project.points = self._canonical_fixed_points()
                self.project.actions = []
            else:
                self.project.path_mode = PATH_MODE_FIXED_8
                self.project.points = self._canonical_fixed_points()
                self.project.actions = []
            self.plan_result = None
            self._v4_dirty = False
            self.refresh_all(selected_point=0)
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
            self.traj_id_spin.setValue(case.traj_id)
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
                            exact_pass=bool(item.get("exact_pass", ptype != POINT_TYPE_WAYPOINT)),
                        )
                    )
            else:
                self.project.path_mode = PATH_MODE_FIXED_8
                pose_by_id = {
                    str(item["point_id"]): dict(item["pose"])
                    for item in case.logical_points
                }
                for index, key in enumerate(LOGICAL_SITE_KEYS):
                    pose = pose_by_id.get(key)
                    if pose is None:
                        continue
                    site = self.project.fixed_sites[index]
                    site.x_mm = float(pose["x_mm"])
                    site.y_mm = float(pose["y_mm"])
                    site.yaw_ddeg = int(pose["yaw_ddeg"])
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
        order: list[str] = ["P_START"]
        for key in case.selected_plan.get("pickup_arrival_state_order", []):
            key = str(key)
            if key in key_to_site and key not in order:
                order.append(key)
        for step in case.selected_plan.get("unload_sequence", []):
            ranks = [int(value) for value in step.get("target_ranks", [])]
            if ranks:
                key = f"P_DROP_{ranks[0]}"
                if key in key_to_site and key not in order:
                    order.append(key)
        for key in LOGICAL_SITE_KEYS:
            if key not in order:
                order.append(key)
        points: list[EditPoint] = []
        for key in order:
            index = key_to_site[key]
            site = self.project.fixed_sites[index]
            points.append(
                EditPoint(
                    point_id=len(points),
                    type=POINT_TYPE_START if not points else POINT_TYPE_ARRIVAL,
                    site_id=index,
                    x_mm=site.x_mm,
                    y_mm=site.y_mm,
                    yaw_ddeg=site.yaw_ddeg,
                    exact_pass=True,
                )
            )
        for item in case.auxiliary_points:
            points.insert(
                max(1, len(points) - 1),
                EditPoint(
                    point_id=0,
                    type=POINT_TYPE_WAYPOINT,
                    site_id=SITE_ID_FREE,
                    x_mm=float(item["x_mm"]),
                    y_mm=float(item["y_mm"]),
                    yaw_ddeg=YAW_UNSPECIFIED_DDEG,
                    exact_pass=str(item.get("policy", "LOCKED_PASS")) == "LOCKED_PASS",
                ),
            )
        for index, point in enumerate(points):
            point.point_id = index
        return points

    def _legacy_actions_from_case(self, case: CaseManifestV40) -> list[MechanicalAction]:
        result: list[MechanicalAction] = []
        for index, item in enumerate(case.actions.get("source", [])):
            action_raw = item.get("action", "NONE")
            if isinstance(action_raw, str):
                action_code = ACTION_CODES.get(action_raw.removeprefix("PATH_ACT_"), 0)
            else:
                action_code = int(action_raw)
            arrival_point_id = item.get("arrival_point_index", item.get("arrival_point_id"))
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
        traj_id = int(self.traj_id_spin.value())
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
                if point.max_speed_mmps > 0:
                    item["max_speed_mmps"] = int(point.max_speed_mmps)
            points.append(item)
        existing = self._v4_state.current_case(traj_id, GenerationMode.MANUAL) if self._v4_state else None
        data = existing.to_dict() if existing is not None else self._empty_manual_case(traj_id)
        data["manual_path"] = {"points": points}
        data["logical_points"] = []
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
        case = self._v4_state.current_case(traj_id, GenerationMode.SEMI_AUTO)
        if case is None:
            full = self._v4_state.current_case(traj_id, GenerationMode.FULL_AUTO)
            if full is None:
                raise RuntimeError("半自动需要任务语义模板。请先生成该ID的FULL_AUTO，再转为半自动。")
            case = convert_full_auto_to_semi_auto(self._v4_state.layout, traj_id)
            self._v4_state.semi_auto_cases[traj_id] = case
        pose_by_key = {
            key: {
                "x_mm": int(round(self.project.fixed_sites[index].x_mm)),
                "y_mm": int(round(self.project.fixed_sites[index].y_mm)),
                "yaw_ddeg": int(self.project.fixed_sites[index].yaw_ddeg),
            }
            for index, key in enumerate(LOGICAL_SITE_KEYS)
        }
        logical_points = []
        for item in case.logical_points:
            updated = dict(item)
            key = str(item["point_id"])
            if key in pose_by_key:
                updated["pose"] = pose_by_key[key]
            logical_points.append(updated)
        auxiliary = [
            {
                "x_mm": int(round(point.x_mm)),
                "y_mm": int(round(point.y_mm)),
                "policy": "LOCKED_PASS" if point.exact_pass else "INITIAL_GUESS",
            }
            for point in self.project.points
            if point.type == POINT_TYPE_WAYPOINT and point.site_id == SITE_ID_FREE
        ]
        data = case.to_dict()
        data["logical_points"] = logical_points
        data["auxiliary_points"] = auxiliary
        data["actions"] = {"source": self._action_source_from_view(), "compiled": []}
        data["review"] = {
            **dict(data.get("review", {})),
            "state": "STALE",
            "approved": False,
            "manual_override": True,
            "stale_reason": "V3.5基准GUI半自动编辑",
        }
        return CaseManifestV40.from_dict(data)

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
        for index, key in enumerate(LOGICAL_SITE_KEYS[:5]):
            raw = self._v4_state.project.sites[key]
            site = self.project.fixed_sites[index]
            site.x_mm = float(raw["x_mm"])
            site.y_mm = float(raw["y_mm"])
            site.yaw_ddeg = int(raw["yaw_ddeg"])

    # ------------------------------------------------------------------
    # Worker and batch controls.
    # ------------------------------------------------------------------
    def _start_worker(self, job: str, params: dict[str, Any]) -> None:
        if self._v4_state is None:
            self._warn("任务失败", "请先打开V4项目。")
            return
        if self._v4_worker is not None and self._v4_worker.is_alive():
            self._warn("任务繁忙", "已有worker正在运行。")
            return
        try:
            self._v4_worker = start_worker_job(self._v4_state.layout.root, job, params)
        except Exception as exc:  # noqa: BLE001
            self._warn("启动失败", str(exc))
            return
        self.v4_progress.setValue(0)
        self.v4_progress.show()
        self._v4_poll_timer.start()
        self._append_log(f"启动任务 {job}: {params}")

    def _poll_worker(self) -> None:
        if self._v4_worker is None:
            return
        for message in self._v4_worker.poll():
            payload = message.payload
            if message.kind == "progress":
                if "percent" in payload:
                    self.v4_progress.setValue(max(0, min(100, int(payload["percent"]))))
                self._append_log(f"[{payload.get('stage', 'PROGRESS')}] {payload.get('message', '')}")
            elif message.kind == "result":
                self._append_log(f"任务完成: {payload}")
            elif message.kind == "error":
                self._append_log(f"任务失败: {payload.get('error', payload)}")
            elif message.kind == "cancelled":
                self._append_log("任务已取消")
        if self._v4_worker.is_alive():
            return
        self._v4_worker.join(0.1)
        self._v4_worker = None
        self._v4_poll_timer.stop()
        self.v4_progress.hide()
        self._reload_v4_state()

    def _cancel_worker(self) -> None:
        if self._v4_worker is not None and self._v4_worker.is_alive():
            self._v4_worker.cancel()
            self._append_log("已请求停止worker")

    def _reload_v4_state(self) -> None:
        if self._v4_state is None:
            return
        root = self._v4_state.layout.root
        try:
            self._v4_state = LoadedProjectState.load(root)
            self._apply_project_common_sites()
            self._refresh_leg_combo()
            self._load_current_mode_case()
        except Exception as exc:  # noqa: BLE001
            self._append_log(f"任务后重新加载失败: {exc}")

    def _optimize_missing(self) -> None:
        self._start_worker(
            "optimize-missing-legs",
            {"profile": str(self.v4_profile_combo.currentData() or "STANDARD")},
        )

    def _reoptimize_selected_leg(self) -> None:
        leg_id = str(self.v4_leg_combo.currentData() or "")
        if not leg_id:
            self._warn("重算失败", "请先选择leg。")
            return
        self._start_worker(
            "reoptimize-current-leg",
            {"leg_id": leg_id, "profile": str(self.v4_profile_combo.currentData() or "STANDARD")},
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
        self._start_worker("generate-full-auto-all", {})

    def _validate_all(self) -> None:
        self._start_worker("validate-all", {})

    def _convert_to_semi_auto(self) -> None:
        if self._v4_state is None:
            return
        traj_id = int(self.traj_id_spin.value())
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
                "traj_id": int(self.traj_id_spin.value()),
                "generation_mode": self._generation_mode.value,
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
