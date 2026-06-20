"""Read/write deterministic competition rules and generate the 360 mappings."""

from __future__ import annotations

import hashlib
import itertools
import json
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_domain.competition_task_config import (
    CompetitionTaskConfigV40,
    EXPECTED_BIN_REACHABILITY,
    EXPECTED_STATION_ROUTE_RULES,
    EXPECTED_UNLOAD_POSE_CATALOG,
    PHYSICAL_DROP_SITES,
    TASK_CONFIG_FORMAT,
)
from hjmb_pathgen.py_domain.route_case import RouteCaseRowV40, RouteCaseTableV40
from hjmb_pathgen.py_io.codecs.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.py_io.persistence.atomic_writer import atomic_write_bytes


def default_competition_task_config_dict() -> dict[str, Any]:
    return {
        "format": TASK_CONFIG_FORMAT,
        "config_version": 1,
        "traj_id_mapping": {
            "formula": "traj_id = bean_code * 60 + drop_code",
            "bean_code_permutations": [
                ["YELLOW", "GREEN", "WHITE"],
                ["YELLOW", "WHITE", "GREEN"],
                ["GREEN", "YELLOW", "WHITE"],
                ["GREEN", "WHITE", "YELLOW"],
                ["WHITE", "YELLOW", "GREEN"],
                ["WHITE", "GREEN", "YELLOW"],
            ],
            "pickup_slot_order": ["PICK_1", "PICK_2", "PICK_3"],
            "physical_drop_site_order": list(PHYSICAL_DROP_SITES),
            "drop_code_generation": "ORDERED_TARGET_PLACEMENT_LEXICOGRAPHIC",
            "empty_label_assignment": "REMAINING_SITES_ASCENDING_TO_LABEL_4_THEN_5",
            "expected_case_count": 360,
        },
        "label_semantics": {
            "1": "YELLOW",
            "2": "GREEN",
            "3": "WHITE",
            "4": "EMPTY",
            "5": "EMPTY",
        },
        "drop_stations": {
            "P_DROP_3": {"station_number": 3, "physical_sites": ["F_DROP_4", "F_DROP_5"]},
            "P_DROP_2": {"station_number": 2, "physical_sites": ["F_DROP_6"]},
            "P_DROP_1": {"station_number": 1, "physical_sites": ["F_DROP_7", "F_DROP_8"]},
        },
        "bin_reachability": {
            key: list(value) for key, value in EXPECTED_BIN_REACHABILITY.items()
        },
        "unload_pose_catalog": {
            key: {
                "station_site": value["station_site"],
                "unload_mask": value["unload_mask"],
                "assignments": dict(value["assignments"]),
            }
            for key, value in EXPECTED_UNLOAD_POSE_CATALOG.items()
        },
        "route_families": {
            "PICK_1_TO_3": {
                "display_name": "LEFT",
                "pickup_position_order": ["PICK_1", "PICK_2", "PICK_3"],
                "pickup_arrival_state_order": ["P_PICK_1", "P_PICK_2L", "P_PICK_3"],
                "drop_station_order": ["P_DROP_3", "P_DROP_2", "P_DROP_1"],
                "yaw_direction": "CW_ONLY",
            },
            "PICK_3_TO_1": {
                "display_name": "RIGHT",
                "pickup_position_order": ["PICK_3", "PICK_2", "PICK_1"],
                "pickup_arrival_state_order": ["P_PICK_3", "P_PICK_2R", "P_PICK_1"],
                "drop_station_order": ["P_DROP_1", "P_DROP_2", "P_DROP_3"],
                "yaw_direction": "CCW_ONLY",
            },
        },
        "automatic_selection": {
            "primary_objective": "MIN_UNLOAD_STOP_COUNT",
            "station_set_route_rules": dict(EXPECTED_STATION_ROUTE_RULES),
            "tie_default": "PICK_1_TO_3",
        },
    }


def default_competition_task_config() -> CompetitionTaskConfigV40:
    return CompetitionTaskConfigV40.from_dict(default_competition_task_config_dict())


def load_competition_task_config(path: str | Path) -> CompetitionTaskConfigV40:
    raw = Path(path).read_bytes()
    if raw.startswith(b"\xef\xbb\xbf"):
        raise ValueError(f"UTF-8 BOM is not allowed: {path}")
    data = json.loads(raw.decode("utf-8"))
    if not isinstance(data, dict):
        raise ValueError("competition task config must be a JSON object")
    return CompetitionTaskConfigV40.from_dict(data)


def save_competition_task_config(path: str | Path, config: CompetitionTaskConfigV40) -> None:
    path = Path(path)
    data = (json.dumps(config.to_dict(), ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")

    def validator(temp: Path) -> None:
        if load_competition_task_config(temp) != config:
            raise ValueError("competition task config write-back mismatch")

    atomic_write_bytes(path, data, validator=validator)


def ensure_competition_task_config(path: str | Path) -> CompetitionTaskConfigV40:
    path = Path(path)
    if path.exists():
        return load_competition_task_config(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    config = default_competition_task_config()
    save_competition_task_config(path, config)
    return config


def build_route_case_table_from_task_config(path: str | Path) -> RouteCaseTableV40:
    path = Path(path)
    config = load_competition_task_config(path)
    mapping = config.traj_id_mapping
    pickup_slots = tuple(str(item) for item in mapping.get("pickup_slot_order", ("PICK_1", "PICK_2", "PICK_3")))
    site_order = tuple(str(item) for item in mapping["physical_drop_site_order"])
    cases: list[RouteCaseRowV40] = []
    for bean_code, beans in enumerate(mapping["bean_code_permutations"]):
        for drop_code, target_sites in enumerate(itertools.permutations(site_order, 3)):
            traj_id = bean_code * 60 + drop_code
            empty_sites = [site for site in site_order if site not in target_sites]
            pick_assignment = dict(zip(pickup_slots, beans, strict=True))
            label_positions = {
                "1": target_sites[0],
                "2": target_sites[1],
                "3": target_sites[2],
                "4": empty_sites[0],
                "5": empty_sites[1],
            }
            semantic = {
                "traj_id": traj_id,
                "bean_code": bean_code,
                "drop_code": drop_code,
                "pick_assignment": pick_assignment,
                # Labels 4 and 5 are both empty.  Their swap is deliberately
                # excluded from task semantics and from source_row_hash.
                "target_label_positions": {
                    key: label_positions[key] for key in ("1", "2", "3")
                },
            }
            cases.append(
                RouteCaseRowV40(
                    traj_id=traj_id,
                    file_name=f"P{traj_id:04d}.BIN",
                    bean_code=bean_code,
                    drop_code=drop_code,
                    pick_assignment=pick_assignment,
                    label_positions=label_positions,
                    source_row_hash=canonical_json_crc32_hex(semantic),
                )
            )
    if len(cases) != 360:
        raise RuntimeError(f"generated case count is {len(cases)}, expected 360")
    source_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    # RouteCaseTableV40 retains the historical field names source_csv and
    # source_csv_sha256 for V4.0 compatibility.  The value now points to the
    # JSON rule file; no CSV is required by the normal workflow.
    source_name = f"{path.parent.name}/{path.name}"
    return RouteCaseTableV40(
        source_csv=source_name,
        source_csv_sha256=source_hash,
        cases=tuple(cases),
    )
