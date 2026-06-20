"""Phase 6 topology gate models for directed leg optimization."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any


class TopologyGateDirection(StrEnum):
    ANY = "ANY"
    POSITIVE = "POSITIVE"
    NEGATIVE = "NEGATIVE"


@dataclass(frozen=True)
class TopologyGate:
    gate_id: str
    ax_mm: float
    ay_mm: float
    bx_mm: float
    by_mm: float
    direction: TopologyGateDirection = TopologyGateDirection.ANY

    @property
    def center(self) -> tuple[float, float]:
        return ((self.ax_mm + self.bx_mm) * 0.5, (self.ay_mm + self.by_mm) * 0.5)

    @property
    def vector(self) -> tuple[float, float]:
        return (self.bx_mm - self.ax_mm, self.by_mm - self.ay_mm)

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, index: int = 0) -> "TopologyGate":
        gate_id = str(data.get("gate_id", data.get("id", f"GATE_{index + 1}")))
        point_a = _point_from_dict(data, "a", ("x1_mm", "y1_mm", "ax_mm", "ay_mm"))
        point_b = _point_from_dict(data, "b", ("x2_mm", "y2_mm", "bx_mm", "by_mm"))
        direction = TopologyGateDirection(str(data.get("direction", TopologyGateDirection.ANY.value)))
        return cls(
            gate_id=gate_id,
            ax_mm=point_a[0],
            ay_mm=point_a[1],
            bx_mm=point_b[0],
            by_mm=point_b[1],
            direction=direction,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "gate_id": self.gate_id,
            "a": {"x_mm": self.ax_mm, "y_mm": self.ay_mm},
            "b": {"x_mm": self.bx_mm, "y_mm": self.by_mm},
            "direction": self.direction.value,
        }


@dataclass(frozen=True)
class TopologyGateCrossing:
    gate_id: str
    path_segment_index: int
    path_ratio: float
    gate_ratio: float
    signed_direction: float
    global_path_parameter: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "gate_id": self.gate_id,
            "path_segment_index": self.path_segment_index,
            "path_ratio": self.path_ratio,
            "gate_ratio": self.gate_ratio,
            "signed_direction": self.signed_direction,
            "global_path_parameter": self.global_path_parameter,
        }


@dataclass(frozen=True)
class TopologyValidationResult:
    success: bool
    crossings: tuple[TopologyGateCrossing, ...] = ()
    errors: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "crossings": [crossing.to_dict() for crossing in self.crossings],
            "errors": list(self.errors),
            "warnings": list(self.warnings),
        }


def topology_gates_from_profile(profile: object) -> tuple[TopologyGate, ...]:
    if not isinstance(profile, dict):
        return ()
    raw_gates = profile.get("gates", profile.get("ordered_gates", ()))
    if not isinstance(raw_gates, list):
        return ()
    return tuple(TopologyGate.from_dict(dict(item), index=index) for index, item in enumerate(raw_gates) if isinstance(item, dict))


def _point_from_dict(data: dict[str, Any], key: str, fallback_keys: tuple[str, str, str, str]) -> tuple[float, float]:
    value = data.get(key)
    if isinstance(value, dict):
        return (float(value.get("x_mm", value.get("x", 0.0))), float(value.get("y_mm", value.get("y", 0.0))))
    x_primary, y_primary, x_alt, y_alt = fallback_keys
    return (float(data.get(x_primary, data.get(x_alt, 0.0))), float(data.get(y_primary, data.get(y_alt, 0.0))))
