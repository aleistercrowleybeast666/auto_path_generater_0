"""Vehicle and collision configuration tab."""

from __future__ import annotations

import math

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QDoubleSpinBox, QFormLayout, QHBoxLayout, QLabel, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

from ..field_view import V4FieldView
from ..ui_state import LoadedProjectState


class VehicleCollisionTab(QWidget):
    dirtyChanged = Signal(bool, str)
    statusMessage = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.state: LoadedProjectState | None = None
        self.field_view = V4FieldView(mode="route")
        self.r_large = _spin(1, 1000)
        self.r_small = _spin(1, 1000)
        self.collision_resolution = _spin(0.1, 200)
        self.epsilon = _spin(0, 10, decimals=6)
        self.wheel_radius = _spin(1, 500)
        self.plan_rpm = _spin(1, 5000)
        self.hard_rpm = _spin(1, 5000)
        self.unload_table = QTableWidget(0, 5)
        self.unload_table.setHorizontalHeaderLabels(("mask", "configured", "yaw", "dx", "dy"))
        self.metrics = QLabel("")
        self._build_ui()
        self._connect()

    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        left = QWidget()
        form = QFormLayout(left)
        form.addRow("R_large", self.r_large)
        form.addRow("R_small", self.r_small)
        form.addRow("collision resolution", self.collision_resolution)
        form.addRow("epsilon", self.epsilon)
        form.addRow("wheel radius", self.wheel_radius)
        form.addRow("plan rpm", self.plan_rpm)
        form.addRow("hard rpm", self.hard_rpm)
        form.addRow("理论速度", self.metrics)
        left_layout = QVBoxLayout()
        left_layout.addWidget(left)
        left_layout.addWidget(QLabel("Unload profiles"))
        left_layout.addWidget(self.unload_table)
        left_box = QWidget()
        left_box.setLayout(left_layout)
        root.addWidget(left_box, 3)
        self.field_view.setMinimumWidth(720)
        root.addWidget(self.field_view, 7)

    def _connect(self) -> None:
        for widget in (self.r_large, self.r_small, self.collision_resolution, self.epsilon, self.wheel_radius, self.plan_rpm, self.hard_rpm):
            widget.valueChanged.connect(self._spin_changed)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.state = state
        project = state.project if state is not None else None
        self.field_view.set_project(project)
        if project is None:
            return
        footprint = project.vehicle["footprint"]
        wheel = project.vehicle["wheel"]
        self._set_spin_values(
            (
                (self.r_large, footprint["r_large_mm"]),
                (self.r_small, footprint["r_small_mm"]),
                (self.collision_resolution, footprint["collision_resolution_mm"]),
                (self.epsilon, footprint["numerical_epsilon_mm"]),
                (self.wheel_radius, wheel["radius_mm"]),
                (self.plan_rpm, wheel["plan_limit_rpm"]),
                (self.hard_rpm, wheel["hard_limit_rpm"]),
            )
        )
        self._fill_unload_table()
        self._refresh_metrics()
        self.field_view.fit_to_field()

    def _set_spin_values(self, pairs) -> None:  # noqa: ANN001
        for spin, value in pairs:
            spin.blockSignals(True)
            spin.setValue(float(value))
            spin.blockSignals(False)

    def _fill_unload_table(self) -> None:
        if self.state is None:
            return
        profiles = self.state.project.unload_profiles
        self.unload_table.setRowCount(len(profiles))
        for row, key in enumerate(sorted(profiles)):
            profile = profiles[key]
            values = (key, profile["configured"], profile["yaw_ddeg"], profile["dx_mm"], profile["dy_mm"])
            for col, value in enumerate(values):
                self.unload_table.setItem(row, col, QTableWidgetItem(str(value)))

    def _spin_changed(self) -> None:
        if self.state is None:
            return
        project = self.state.project
        footprint = project.vehicle["footprint"]
        wheel = project.vehicle["wheel"]
        footprint["r_large_mm"] = self.r_large.value()
        footprint["r_small_mm"] = self.r_small.value()
        footprint["collision_resolution_mm"] = self.collision_resolution.value()
        footprint["numerical_epsilon_mm"] = self.epsilon.value()
        wheel["radius_mm"] = self.wheel_radius.value()
        wheel["plan_limit_rpm"] = self.plan_rpm.value()
        wheel["hard_limit_rpm"] = self.hard_rpm.value()
        self._refresh_metrics()
        self.field_view.refresh()
        self.dirtyChanged.emit(True, "vehicle/collision 配置已修改；相关 leg/case 标记 STALE")

    def _refresh_metrics(self) -> None:
        radius = self.wheel_radius.value()
        plan_rpm = self.plan_rpm.value()
        hard_rpm = self.hard_rpm.value()
        plan_mmps = 2.0 * math.pi * radius * plan_rpm / 60.0
        hard_mmps = 2.0 * math.pi * radius * hard_rpm / 60.0
        self.metrics.setText(f"前后/横向约 {plan_mmps:.0f} mm/s；硬上限 {hard_mmps:.0f} mm/s")


def _spin(minimum: float, maximum: float, *, decimals: int = 2) -> QDoubleSpinBox:
    spin = QDoubleSpinBox()
    spin.setRange(minimum, maximum)
    spin.setDecimals(decimals)
    spin.setSingleStep(1.0)
    return spin
