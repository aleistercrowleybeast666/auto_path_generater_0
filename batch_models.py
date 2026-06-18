# -*- coding: utf-8 -*-
"""V3.5 batch route case and leg-template models."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from path_models import (
    YAW_ROTATION_POLICIES,
    YAW_ROTATION_SHORTEST,
    parse_int,
)

BATCH_FORMAT = "HJMB_PATH_BATCH_JSON_V35"


@dataclass
class RouteCase:
    traj_id: int
    pickup_order: List[int] = field(default_factory=list)
    drop_order: List[int] = field(default_factory=list)
    sweep_direction: str = "LEFT_TO_RIGHT"
    yaw_rotation_policy: str = YAW_ROTATION_SHORTEST
    action_template_name: str = ""

    @classmethod
    def from_dict(cls, data: dict) -> "RouteCase":
        policy = str(data.get("yaw_rotation_policy", YAW_ROTATION_SHORTEST)).upper()
        if policy not in YAW_ROTATION_POLICIES:
            raise ValueError(f"route_case yaw_rotation_policy={policy!r} 非法")
        return cls(
            traj_id=parse_int(data.get("traj_id", 0), "route_case.traj_id"),
            pickup_order=[parse_int(value, "route_case.pickup_order") for value in data.get("pickup_order", [])],
            drop_order=[parse_int(value, "route_case.drop_order") for value in data.get("drop_order", [])],
            sweep_direction=str(data.get("sweep_direction", "LEFT_TO_RIGHT")),
            yaw_rotation_policy=policy,
            action_template_name=str(data.get("action_template_name", "")),
        )

    @property
    def ordered_sites(self) -> List[int]:
        return self.pickup_order + self.drop_order


@dataclass
class LegTemplate:
    from_site_id: int
    to_site_id: int
    waypoints: List[dict] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict) -> "LegTemplate":
        return cls(
            from_site_id=parse_int(data.get("from_site_id", 0), "leg_template.from_site_id"),
            to_site_id=parse_int(data.get("to_site_id", 0), "leg_template.to_site_id"),
            waypoints=[dict(item) for item in data.get("waypoints", [])],
        )

    @property
    def key(self) -> Tuple[int, int]:
        return self.from_site_id, self.to_site_id


def load_route_cases(data: List[dict]) -> List[RouteCase]:
    return [RouteCase.from_dict(item) for item in data]


def load_leg_templates(data: List[dict]) -> Dict[Tuple[int, int], LegTemplate]:
    templates = [LegTemplate.from_dict(item) for item in data]
    result: Dict[Tuple[int, int], LegTemplate] = {}
    for template in templates:
        if template.key in result:
            raise ValueError(f"leg_template {template.key} 重复")
        result[template.key] = template
    return result


def validate_route_case_coverage(
    route_cases: List[RouteCase],
    require_full_360: bool = True,
) -> List[str]:
    errors: List[str] = []
    ids = [case.traj_id for case in route_cases]
    duplicates = sorted({traj_id for traj_id in ids if ids.count(traj_id) > 1})
    if duplicates:
        errors.append(f"route_cases traj_id 重复: {duplicates}")
    out_of_range = [traj_id for traj_id in ids if not 0 <= traj_id <= 359]
    if out_of_range:
        errors.append(f"route_cases traj_id 超出 0~359: {out_of_range}")
    if require_full_360:
        missing = sorted(set(range(360)) - set(ids))
        if missing:
            errors.append(f"route_cases 缺少 traj_id: {missing[:20]}{'...' if len(missing) > 20 else ''}")
    return errors
