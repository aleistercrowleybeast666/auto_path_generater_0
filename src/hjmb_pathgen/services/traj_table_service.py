"""Project-level traj_id.csv to route_case_table.json services."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from hjmb_pathgen.codec.csv_codec import TrajCsvTable, load_traj_id_csv
from hjmb_pathgen.codec.json_codec import load_route_case_table, save_route_case_table
from hjmb_pathgen.models.route_case import RouteCaseTableV40
from hjmb_pathgen.services.project_service import ProjectLayout


@dataclass(frozen=True)
class RouteCaseTableBuildResult:
    csv_table: TrajCsvTable
    route_case_table: RouteCaseTableV40
    output_path: Path


def build_route_case_table_from_csv(csv_path: str | Path) -> RouteCaseTableV40:
    return load_traj_id_csv(csv_path).to_route_case_table()


def write_route_case_table(layout: ProjectLayout) -> RouteCaseTableBuildResult:
    csv_table = load_traj_id_csv(layout.traj_id_csv)
    route_case_table = csv_table.to_route_case_table()
    save_route_case_table(layout.route_case_table_json, route_case_table)
    loaded = load_route_case_table(layout.route_case_table_json)
    if loaded != route_case_table:
        raise RuntimeError(f"route_case_table write-back mismatch: {layout.route_case_table_json}")
    return RouteCaseTableBuildResult(csv_table=csv_table, route_case_table=route_case_table, output_path=layout.route_case_table_json)
