"""Phase 3 candidate and transition requirement models."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from hjmb_pathgen.models.enums import RouteFamily, UnloadMask, YawPolicy


@dataclass(frozen=True)
class UnloadStep:
    step_index: int
    unload_mask: UnloadMask
    target_ranks: tuple[int, ...]
    bean_types: tuple[str, ...]
    physical_sites: tuple[str, ...]
    vehicle_bins: tuple[str, ...]
    anchor_site: str
    yaw_ddeg: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "step_index": self.step_index,
            "unload_mask": self.unload_mask.value,
            "target_ranks": list(self.target_ranks),
            "bean_types": list(self.bean_types),
            "physical_sites": list(self.physical_sites),
            "vehicle_bins": list(self.vehicle_bins),
            "anchor_site": self.anchor_site,
            "yaw_ddeg": self.yaw_ddeg,
        }


@dataclass(frozen=True)
class CandidatePlan:
    candidate_id: str
    semantic_hash: str
    traj_id: int
    route_family: RouteFamily
    pickup_position_order: tuple[str, ...]
    pickup_arrival_state_order: tuple[str, ...]
    drop_target_rank_order: tuple[int, ...]
    vehicle_bin_assignment: dict[str, str]
    unload_sequence: tuple[UnloadStep, ...]
    yaw_direction: YawPolicy
    yaw_sequence_ddeg: tuple[int, ...]
    source_actions: tuple[dict[str, Any], ...]
    estimated_mechanism_time_ms: int
    warnings: tuple[str, ...] = ()
    unavailable_reasons: tuple[str, ...] = ()
    locked_by_user: bool = False

    @property
    def stop_count(self) -> int:
        return len(self.unload_sequence)

    @property
    def is_available(self) -> bool:
        return not self.unavailable_reasons

    def to_dict(self) -> dict[str, Any]:
        return {
            "candidate_id": self.candidate_id,
            "semantic_hash": self.semantic_hash,
            "traj_id": self.traj_id,
            "route_family": self.route_family.name,
            "pickup_position_order": list(self.pickup_position_order),
            "pickup_arrival_state_order": list(self.pickup_arrival_state_order),
            "drop_target_rank_order": list(self.drop_target_rank_order),
            "vehicle_bin_assignment": self.vehicle_bin_assignment,
            "unload_sequence": [step.to_dict() for step in self.unload_sequence],
            "yaw_direction": self.yaw_direction.value,
            "yaw_sequence_ddeg": list(self.yaw_sequence_ddeg),
            "source_actions": list(self.source_actions),
            "estimated_mechanism_time_ms": self.estimated_mechanism_time_ms,
            "warnings": list(self.warnings),
            "unavailable_reasons": list(self.unavailable_reasons),
            "locked_by_user": self.locked_by_user,
            "stop_count": self.stop_count,
        }


@dataclass(frozen=True)
class TransitionRequirement:
    requirement_id: str
    semantic_hash: str
    from_state_id: str
    to_state_id: str
    route_family: str
    topology_profile: str
    from_pose: dict[str, Any]
    to_pose: dict[str, Any]
    dependency_hashes: dict[str, Any]
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "requirement_id": self.requirement_id,
            "semantic_hash": self.semantic_hash,
            "from_state_id": self.from_state_id,
            "to_state_id": self.to_state_id,
            "route_family": self.route_family,
            "topology_profile": self.topology_profile,
            "from_pose": self.from_pose,
            "to_pose": self.to_pose,
            "dependency_hashes": self.dependency_hashes,
            "reason": self.reason,
        }
