"""Project-level task mapping to route_case_table.json services.

The primary source is task_config/competition_task_config.json.  Chinese
traj_id.csv remains a legacy import/testing path only.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from hjmb_pathgen.py_domain.route_case import RouteCaseTableV40
from hjmb_pathgen.py_io.codecs.csv_codec import TrajCsvTable, load_traj_id_csv
from hjmb_pathgen.py_io.codecs.json_codec import load_route_case_table, save_route_case_table
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.competition_task_config_service import (
    build_route_case_table_from_task_config,
    ensure_competition_task_config,
)


@dataclass(frozen=True)
class RouteCaseTableBuildResult:
    route_case_table: RouteCaseTableV40
    output_path: Path
    source_kind: str = "TASK_CONFIG_JSON"
    csv_table: TrajCsvTable | None = None


def build_route_case_table_from_csv(csv_path: str | Path) -> RouteCaseTableV40:
    return load_traj_id_csv(csv_path).to_route_case_table()


def write_route_case_table(layout: ProjectLayout) -> RouteCaseTableBuildResult:
    config = ensure_competition_task_config(layout.competition_task_config_json)
    del config  # validation side effect is intentional
    route_case_table = build_route_case_table_from_task_config(layout.competition_task_config_json)
    save_route_case_table(layout.route_case_table_json, route_case_table)
    loaded = load_route_case_table(layout.route_case_table_json)
    if loaded != route_case_table:
        raise RuntimeError(f"route_case_table write-back mismatch: {layout.route_case_table_json}")
    return RouteCaseTableBuildResult(
        route_case_table=route_case_table,
        output_path=layout.route_case_table_json,
        source_kind="TASK_CONFIG_JSON",
    )


def write_route_case_table_from_legacy_csv(layout: ProjectLayout) -> RouteCaseTableBuildResult:
    csv_table = load_traj_id_csv(layout.traj_id_csv)
    route_case_table = csv_table.to_route_case_table()
    save_route_case_table(layout.route_case_table_json, route_case_table)
    return RouteCaseTableBuildResult(
        route_case_table=route_case_table,
        output_path=layout.route_case_table_json,
        source_kind="LEGACY_CSV",
        csv_table=csv_table,
    )
