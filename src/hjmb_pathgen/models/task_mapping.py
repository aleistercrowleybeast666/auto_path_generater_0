"""Phase 3 task-mapping models derived from traj_id.csv."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any


class BeanType(StrEnum):
    YELLOW = "YELLOW"
    GREEN = "GREEN"
    WHITE = "WHITE"


class PickupSlot(StrEnum):
    PICK_1 = "PICK_1"
    PICK_2 = "PICK_2"
    PICK_3 = "PICK_3"


class PhysicalDropSite(StrEnum):
    F_DROP_4 = "F_DROP_4"
    F_DROP_5 = "F_DROP_5"
    F_DROP_6 = "F_DROP_6"
    F_DROP_7 = "F_DROP_7"
    F_DROP_8 = "F_DROP_8"


LABEL_TO_BEAN = {
    1: BeanType.YELLOW,
    2: BeanType.GREEN,
    3: BeanType.WHITE,
}

EMPTY_LABELS = {4, 5}
PICKUP_SLOTS = (PickupSlot.PICK_1, PickupSlot.PICK_2, PickupSlot.PICK_3)
PHYSICAL_DROP_SITES = (
    PhysicalDropSite.F_DROP_4,
    PhysicalDropSite.F_DROP_5,
    PhysicalDropSite.F_DROP_6,
    PhysicalDropSite.F_DROP_7,
    PhysicalDropSite.F_DROP_8,
)


@dataclass(frozen=True)
class TrajCsvRow:
    traj_id: int
    file_name: str
    bean_code: int
    drop_code: int
    pick_assignment: dict[str, str]
    label_positions: dict[str, str]
    source_row_number: int
    source_row_hash: str
    raw_fields: dict[str, str]

    def target_position_signature(self) -> tuple[str, str, str]:
        return tuple(self.label_positions[str(label)] for label in (1, 2, 3))

    def to_route_row_dict(self) -> dict[str, Any]:
        return {
            "traj_id": self.traj_id,
            "file_name": self.file_name,
            "bean_code": self.bean_code,
            "drop_code": self.drop_code,
            "pick_assignment": self.pick_assignment,
            "label_positions": self.label_positions,
            "source_row_number": self.source_row_number,
            "source_row_hash": self.source_row_hash,
            "raw_fields": self.raw_fields,
        }


@dataclass(frozen=True)
class DropTarget:
    target_rank: int
    bean_type: BeanType
    label_number: int
    physical_site: PhysicalDropSite
    physical_order_index: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "target_rank": self.target_rank,
            "bean_type": self.bean_type.value,
            "label_number": self.label_number,
            "physical_site": self.physical_site.value,
            "physical_order_index": self.physical_order_index,
        }


def drop_targets_from_label_positions(label_positions: dict[str, str]) -> tuple[DropTarget, ...]:
    targets = []
    for label, bean_type in LABEL_TO_BEAN.items():
        site = PhysicalDropSite(label_positions[str(label)])
        targets.append(
            {
                "bean_type": bean_type,
                "label_number": label,
                "physical_site": site,
                "physical_order_index": _physical_order(site),
            }
        )
    targets.sort(key=lambda item: item["physical_order_index"])
    return tuple(
        DropTarget(
            target_rank=index + 1,
            bean_type=item["bean_type"],
            label_number=item["label_number"],
            physical_site=item["physical_site"],
            physical_order_index=item["physical_order_index"],
        )
        for index, item in enumerate(targets)
    )


def _physical_order(site: PhysicalDropSite) -> int:
    return int(site.value[-1])
