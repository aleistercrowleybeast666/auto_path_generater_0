from __future__ import annotations

import csv
import itertools
import json
import sys
from io import StringIO
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hjmb_pathgen.py_io.codecs.csv_codec import EXPECTED_TRAJ_HEADERS
from hjmb_pathgen.py_domain.project import ProjectV40

FIXTURE_ROOT = Path(__file__).resolve().parent / "fixtures" / "v40"
ROOT = Path(__file__).resolve().parents[1]

BEAN_NAMES = ("黄豆", "绿豆", "白芸豆")
SITE_NAMES = ("④号位", "⑤号位", "⑥号位", "⑦号位", "⑧号位")
ACTION_PROFILE_KEYS = (
    "PREP_PICK_1",
    "PREP_PICK_2L",
    "PREP_PICK_2R",
    "PREP_PICK_3",
    "PICK",
    "PREP_STORE_1",
    "PREP_STORE_2",
    "PREP_STORE_3",
    "STORE",
    "DROP_1",
    "DROP_2",
    "DROP_3",
    "DROP_12",
    "DROP_23",
)


def phase3_project_dict() -> dict:
    data = json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8"))
    data["action_profiles"] = {
        key: {
            "mode": "STOP_AND_WAIT",
            "timeout_ms": 1000,
            "post_wait_ms": 0,
            "estimated_time_ms": _profile_estimate(key),
        }
        for key in ACTION_PROFILE_KEYS
    }
    data["topology_profiles"] = {
        "PICK_1_TO_3": {"profile_id": "S_LTR_PHASE3"},
        "PICK_3_TO_1": {"profile_id": "S_RTL_PHASE3"},
    }
    return data


def phase3_project() -> ProjectV40:
    return ProjectV40.from_dict(phase3_project_dict())


def make_valid_traj_csv_bytes(
    *,
    bom: bool = False,
    lineterminator: str = "\n",
    mutate_row: Callable[[list[list[str]]], None] | None = None,
    mutate_header: Callable[[list[str]], None] | None = None,
) -> bytes:
    header = list(EXPECTED_TRAJ_HEADERS)
    if mutate_header is not None:
        mutate_header(header)
    rows = [header]
    bean_permutations = list(itertools.permutations(BEAN_NAMES))
    drop_permutations = list(itertools.permutations(SITE_NAMES, 3))
    for bean_code, beans in enumerate(bean_permutations):
        for drop_code, target_sites in enumerate(drop_permutations):
            traj_id = bean_code * 60 + drop_code
            empty_sites = [site for site in SITE_NAMES if site not in target_sites]
            rows.append(
                [
                    str(traj_id),
                    f"P{traj_id:04d}.BIN",
                    str(bean_code),
                    str(drop_code),
                    beans[0],
                    beans[1],
                    beans[2],
                    target_sites[0],
                    target_sites[1],
                    target_sites[2],
                    empty_sites[0],
                    empty_sites[1],
                ]
            )
    if mutate_row is not None:
        mutate_row(rows)
    buffer = StringIO(newline="")
    writer = csv.writer(buffer, lineterminator=lineterminator)
    writer.writerows(rows)
    data = buffer.getvalue().encode("utf-8")
    return b"\xef\xbb\xbf" + data if bom else data


def root_traj_csv_path() -> Path:
    return ROOT / "traj_id.csv"


def _profile_estimate(key: str) -> int:
    if key.startswith("DROP_"):
        return 20
    if key in {"PICK", "STORE"}:
        return 15
    return 5
