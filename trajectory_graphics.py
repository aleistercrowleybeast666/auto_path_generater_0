# -*- coding: utf-8 -*-
"""Shared V3.3 trajectory overlay bands and hover text."""
from __future__ import annotations

from typing import List, Tuple

from path_models import TrajectoryNode

ANALYSIS_MODE_NORMAL = "normal"
ANALYSIS_MODE_SPEED = "speed"
ANALYSIS_MODE_ACCEL = "accel"
ANALYSIS_MODE_WZ = "angular_speed"
ANALYSIS_MODE_BETA = "beta"
ANALYSIS_MODES = (
    ANALYSIS_MODE_NORMAL,
    ANALYSIS_MODE_SPEED,
    ANALYSIS_MODE_ACCEL,
    ANALYSIS_MODE_WZ,
    ANALYSIS_MODE_BETA,
)

OVER_LIMIT_TOLERANCE_RATIO = 0.005

NORMAL_COLOR = "#e05028"
OVER_LIMIT_COLOR = "#d400d4"
SPEED_COLORS = ("#3b82f6", "#22c55e", "#f59e0b", "#ef4444")
ACCEL_COLORS = ("#5b8ff9", "#61d9a8", "#f6bd16", "#e8684a")
WZ_COLORS = ("#5b8ff9", "#61d9a8", "#f6bd16", "#e8684a")
BETA_COLORS = ("#5b8ff9", "#61d9a8", "#f6bd16", "#e8684a")


def equal_bands(maximum: float, band_count: int) -> List[Tuple[float, float]]:
    if maximum <= 0:
        raise ValueError("分档最大值必须大于 0")
    if band_count <= 0:
        raise ValueError("分档数量必须大于 0")
    return [
        (maximum * index / band_count, maximum * (index + 1) / band_count)
        for index in range(band_count)
    ]


def classify_value(value: float, maximum: float, band_count: int) -> int:
    value = abs(value)
    if value > maximum * (1.0 + OVER_LIMIT_TOLERANCE_RATIO):
        return band_count
    if value >= maximum:
        return band_count - 1
    return min(band_count - 1, int(value / maximum * band_count + 1e-12))


def trajectory_segment_color(
    mode: str,
    node: TrajectoryNode,
    max_speed_mmps: float,
    max_accel_mmps2: float,
    max_wz_radps: float,
    max_beta_radps2: float,
) -> str:
    if mode == ANALYSIS_MODE_SPEED:
        band = classify_value(node.speed_mmps, max_speed_mmps, 4)
        return OVER_LIMIT_COLOR if band == 4 else SPEED_COLORS[band]
    if mode == ANALYSIS_MODE_ACCEL:
        band = classify_value(node.a_total_mmps2, max_accel_mmps2, 4)
        return OVER_LIMIT_COLOR if band == 4 else ACCEL_COLORS[band]
    if mode == ANALYSIS_MODE_WZ:
        band = classify_value(node.wz_radps, max_wz_radps, 4)
        return OVER_LIMIT_COLOR if band == 4 else WZ_COLORS[band]
    if mode == ANALYSIS_MODE_BETA:
        band = classify_value(node.beta_radps2, max_beta_radps2, 4)
        return OVER_LIMIT_COLOR if band == 4 else BETA_COLORS[band]
    return NORMAL_COLOR


def legend_entries(
    mode: str,
    max_speed_mmps: float,
    max_accel_mmps2: float,
    max_wz_radps: float,
    max_beta_radps2: float,
) -> List[Tuple[str, str]]:
    if mode == ANALYSIS_MODE_SPEED:
        bands = equal_bands(max_speed_mmps / 1000.0, 4)
        entries = [
            (f"{lower:.2f}~{upper:.2f} m/s", color)
            for (lower, upper), color in zip(bands, SPEED_COLORS)
        ]
        entries.append((f"> {max_speed_mmps / 1000.0:.2f} m/s", OVER_LIMIT_COLOR))
        return entries
    if mode == ANALYSIS_MODE_ACCEL:
        bands = equal_bands(max_accel_mmps2 / 1000.0, 4)
        entries = [
            (f"{lower:.2f}~{upper:.2f} m/s²", color)
            for (lower, upper), color in zip(bands, ACCEL_COLORS)
        ]
        entries.append(
            (f"> {max_accel_mmps2 / 1000.0:.2f} m/s²", OVER_LIMIT_COLOR)
        )
        return entries
    if mode == ANALYSIS_MODE_WZ:
        bands = equal_bands(max_wz_radps, 4)
        entries = [
            (f"{lower:.2f}~{upper:.2f} rad/s", color)
            for (lower, upper), color in zip(bands, WZ_COLORS)
        ]
        entries.append((f"> {max_wz_radps:.2f} rad/s", OVER_LIMIT_COLOR))
        return entries
    if mode == ANALYSIS_MODE_BETA:
        bands = equal_bands(max_beta_radps2, 4)
        entries = [
            (f"{lower:.2f}~{upper:.2f} rad/s²", color)
            for (lower, upper), color in zip(bands, BETA_COLORS)
        ]
        entries.append((f"> {max_beta_radps2:.2f} rad/s²", OVER_LIMIT_COLOR))
        return entries
    return [("普通路径", NORMAL_COLOR)]


def node_hover_text(node: TrajectoryNode) -> str:
    return "\n".join(
        (
            f"s={node.s_mm:.1f} mm",
            f"x={node.x_mm:.1f} mm, y={node.y_mm:.1f} mm",
            f"yaw={node.yaw_rad * 180.0 / 3.141592653589793:.2f}°",
            f"v={node.speed_mmps:.1f} mm/s",
            f"vx={node.vx_mmps:.1f}, vy={node.vy_mmps:.1f} mm/s",
            f"wz={node.wz_radps:.3f} rad/s",
            f"a_t={node.a_t_mmps2:.1f} mm/s²",
            f"a_n={node.a_n_mmps2:.1f} mm/s²",
            f"a_total={node.a_total_mmps2:.1f} mm/s²",
            f"beta={node.beta_radps2:.3f} rad/s²",
            f"curvature={node.curvature_kappa_per_m:.5f} 1/m",
            f"max wheel={node.max_wheel_rpm:.2f} rpm",
            f"constraint={node.constraint_source}",
        )
    )
