"""Deterministic competition task mapping and unload geometry rules.

This JSON replaces the Chinese ``traj_id.csv`` as the primary source for the
360 draw-result mappings.  The file is intentionally English-only and keeps
stable competition rules separate from measured project poses.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .errors import V40ValidationError, reject_unknown_fields, require_fields

TASK_CONFIG_FORMAT = "HJMB_COMPETITION_TASK_CONFIG_JSON_V40"
TASK_CONFIG_VERSION = 1

BEAN_TYPES = ("YELLOW", "GREEN", "WHITE")
PICK_SLOTS = ("PICK_1", "PICK_2", "PICK_3")
PHYSICAL_DROP_SITES = ("F_DROP_4", "F_DROP_5", "F_DROP_6", "F_DROP_7", "F_DROP_8")
VEHICLE_BINS = ("BIN_1", "BIN_2", "BIN_3")
LOGICAL_DROP_STATIONS = ("P_DROP_1", "P_DROP_2", "P_DROP_3")

TOP_LEVEL_FIELDS = {
    "format",
    "config_version",
    "traj_id_mapping",
    "label_semantics",
    "drop_stations",
    "bin_reachability",
    "unload_pose_catalog",
    "route_families",
    "automatic_selection",
}

UNLOAD_POSE_PROFILE_IDS = (
    "DROP_F4_BIN_1",
    "DROP_F5_BIN_1",
    "DROP_F5_BIN_2",
    "DROP_F6_BIN_1",
    "DROP_F6_BIN_2",
    "DROP_F6_BIN_3",
    "DROP_F7_BIN_2",
    "DROP_F7_BIN_3",
    "DROP_F8_BIN_3",
    "DROP_F45_BIN_12",
    "DROP_F78_BIN_23",
)

EXPECTED_BIN_REACHABILITY = {
    "F_DROP_4": ("BIN_1",),
    "F_DROP_5": ("BIN_1", "BIN_2"),
    "F_DROP_6": ("BIN_1", "BIN_2", "BIN_3"),
    "F_DROP_7": ("BIN_2", "BIN_3"),
    "F_DROP_8": ("BIN_3",),
}

EXPECTED_UNLOAD_POSE_CATALOG = {
    "DROP_F4_BIN_1": {"station_site": "P_DROP_3", "unload_mask": "BIN_1", "assignments": {"F_DROP_4": "BIN_1"}},
    "DROP_F5_BIN_1": {"station_site": "P_DROP_3", "unload_mask": "BIN_1", "assignments": {"F_DROP_5": "BIN_1"}},
    "DROP_F5_BIN_2": {"station_site": "P_DROP_3", "unload_mask": "BIN_2", "assignments": {"F_DROP_5": "BIN_2"}},
    "DROP_F6_BIN_1": {"station_site": "P_DROP_2", "unload_mask": "BIN_1", "assignments": {"F_DROP_6": "BIN_1"}},
    "DROP_F6_BIN_2": {"station_site": "P_DROP_2", "unload_mask": "BIN_2", "assignments": {"F_DROP_6": "BIN_2"}},
    "DROP_F6_BIN_3": {"station_site": "P_DROP_2", "unload_mask": "BIN_3", "assignments": {"F_DROP_6": "BIN_3"}},
    "DROP_F7_BIN_2": {"station_site": "P_DROP_1", "unload_mask": "BIN_2", "assignments": {"F_DROP_7": "BIN_2"}},
    "DROP_F7_BIN_3": {"station_site": "P_DROP_1", "unload_mask": "BIN_3", "assignments": {"F_DROP_7": "BIN_3"}},
    "DROP_F8_BIN_3": {"station_site": "P_DROP_1", "unload_mask": "BIN_3", "assignments": {"F_DROP_8": "BIN_3"}},
    "DROP_F45_BIN_12": {"station_site": "P_DROP_3", "unload_mask": "BIN_12", "assignments": {"F_DROP_4": "BIN_1", "F_DROP_5": "BIN_2"}},
    "DROP_F78_BIN_23": {"station_site": "P_DROP_1", "unload_mask": "BIN_23", "assignments": {"F_DROP_7": "BIN_2", "F_DROP_8": "BIN_3"}},
}

EXPECTED_STATION_ROUTE_RULES = {
    "1": "PICK_3_TO_1",
    "2": "PICK_1_TO_3",
    "3": "PICK_1_TO_3",
    "1,2": "PICK_3_TO_1",
    "1,3": "PICK_1_TO_3",
    "2,3": "PICK_1_TO_3",
    "1,2,3": "PICK_1_TO_3",
}


@dataclass(frozen=True)
class CompetitionTaskConfigV40:
    traj_id_mapping: dict[str, Any]
    label_semantics: dict[str, Any]
    drop_stations: dict[str, Any]
    bin_reachability: dict[str, tuple[str, ...]]
    unload_pose_catalog: dict[str, dict[str, Any]]
    route_families: dict[str, dict[str, Any]]
    automatic_selection: dict[str, Any]
    config_version: int = TASK_CONFIG_VERSION
    format: str = TASK_CONFIG_FORMAT

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CompetitionTaskConfigV40":
        reject_unknown_fields(data, TOP_LEVEL_FIELDS, "CompetitionTaskConfigV40")
        require_fields(data, TOP_LEVEL_FIELDS, "CompetitionTaskConfigV40")
        if data.get("format") != TASK_CONFIG_FORMAT:
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "format", "unexpected task config format",
                actual=data.get("format"), expected=TASK_CONFIG_FORMAT,
            )
        if int(data.get("config_version", -1)) != TASK_CONFIG_VERSION:
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "config_version", "unsupported task config version",
                actual=data.get("config_version"), expected=TASK_CONFIG_VERSION,
            )

        mapping = dict(data["traj_id_mapping"])
        bean_permutations = mapping.get("bean_code_permutations")
        if not isinstance(bean_permutations, list) or len(bean_permutations) != 6:
            raise V40ValidationError("CompetitionTaskConfigV40", "traj_id_mapping.bean_code_permutations", "must contain six permutations")
        normalized_permutations: list[list[str]] = []
        for index, row in enumerate(bean_permutations):
            values = [str(item) for item in row] if isinstance(row, list) else []
            if sorted(values) != sorted(BEAN_TYPES):
                raise V40ValidationError(
                    "CompetitionTaskConfigV40", f"traj_id_mapping.bean_code_permutations[{index}]",
                    "must be a permutation of YELLOW/GREEN/WHITE", actual=values,
                )
            normalized_permutations.append(values)
        mapping["bean_code_permutations"] = normalized_permutations
        if list(mapping.get("physical_drop_site_order", ())) != list(PHYSICAL_DROP_SITES):
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "traj_id_mapping.physical_drop_site_order",
                "must be F_DROP_4 through F_DROP_8", actual=mapping.get("physical_drop_site_order"),
                expected=list(PHYSICAL_DROP_SITES),
            )
        if str(mapping.get("drop_code_generation")) != "ORDERED_TARGET_PLACEMENT_LEXICOGRAPHIC":
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "traj_id_mapping.drop_code_generation",
                "unsupported drop-code generation rule", actual=mapping.get("drop_code_generation"),
            )
        if str(mapping.get("formula")) != "traj_id = bean_code * 60 + drop_code":
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "traj_id_mapping.formula",
                "unexpected traj_id formula", actual=mapping.get("formula"),
            )

        labels = dict(data["label_semantics"])
        expected_labels = {"1": "YELLOW", "2": "GREEN", "3": "WHITE", "4": "EMPTY", "5": "EMPTY"}
        if labels != expected_labels:
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "label_semantics",
                "competition label meanings are fixed", actual=labels, expected=expected_labels,
            )

        stations = {str(key): dict(value) for key, value in dict(data["drop_stations"]).items()}
        expected_station_sites = {
            "P_DROP_3": ["F_DROP_4", "F_DROP_5"],
            "P_DROP_2": ["F_DROP_6"],
            "P_DROP_1": ["F_DROP_7", "F_DROP_8"],
        }
        if set(stations) != set(expected_station_sites):
            raise V40ValidationError("CompetitionTaskConfigV40", "drop_stations", "must contain P_DROP_1/P_DROP_2/P_DROP_3")
        for station, expected_sites in expected_station_sites.items():
            if list(stations[station].get("physical_sites", ())) != expected_sites:
                raise V40ValidationError(
                    "CompetitionTaskConfigV40", f"drop_stations.{station}.physical_sites",
                    "unexpected station geometry", actual=stations[station].get("physical_sites"), expected=expected_sites,
                )

        reachability_raw = dict(data["bin_reachability"])
        if set(reachability_raw) != set(PHYSICAL_DROP_SITES):
            raise V40ValidationError("CompetitionTaskConfigV40", "bin_reachability", "must cover F_DROP_4..F_DROP_8")
        reachability: dict[str, tuple[str, ...]] = {}
        for site in PHYSICAL_DROP_SITES:
            bins = tuple(str(item) for item in reachability_raw[site])
            if not bins or any(item not in VEHICLE_BINS for item in bins):
                raise V40ValidationError(
                    "CompetitionTaskConfigV40", f"bin_reachability.{site}", "contains invalid vehicle bin", actual=bins,
                )
            reachability[site] = bins
        if reachability != EXPECTED_BIN_REACHABILITY:
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "bin_reachability",
                "bin reachability is fixed by the mechanism geometry",
                actual={key: list(value) for key, value in reachability.items()},
                expected={key: list(value) for key, value in EXPECTED_BIN_REACHABILITY.items()},
            )

        catalog = {str(key): dict(value) for key, value in dict(data["unload_pose_catalog"]).items()}
        if set(catalog) != set(UNLOAD_POSE_PROFILE_IDS):
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "unload_pose_catalog", "must contain the eleven supported operations",
                actual=sorted(catalog), expected=list(UNLOAD_POSE_PROFILE_IDS),
            )
        for profile_id, spec in catalog.items():
            station = str(spec.get("station_site", ""))
            if station not in LOGICAL_DROP_STATIONS:
                raise V40ValidationError(
                    "CompetitionTaskConfigV40", f"unload_pose_catalog.{profile_id}.station_site", "invalid logical drop station", actual=station,
                )
            assignments = dict(spec.get("assignments", {}))
            if not assignments:
                raise V40ValidationError(
                    "CompetitionTaskConfigV40", f"unload_pose_catalog.{profile_id}.assignments", "must not be empty",
                )
            for site, vehicle_bin in assignments.items():
                if site not in PHYSICAL_DROP_SITES or vehicle_bin not in VEHICLE_BINS:
                    raise V40ValidationError(
                        "CompetitionTaskConfigV40", f"unload_pose_catalog.{profile_id}.assignments", "invalid physical-site/bin assignment",
                        actual=assignments,
                    )
                if vehicle_bin not in reachability[site]:
                    raise V40ValidationError(
                        "CompetitionTaskConfigV40", f"unload_pose_catalog.{profile_id}.assignments", "assignment violates reachability",
                        actual=assignments,
                    )
            if set(assignments) != set(stations[station]["physical_sites"]) and len(assignments) > 1:
                raise V40ValidationError(
                    "CompetitionTaskConfigV40", f"unload_pose_catalog.{profile_id}.assignments",
                    "dual unload must cover the complete right-angle pair for its station", actual=assignments,
                )
        normalized_catalog = {
            key: {
                "station_site": str(value.get("station_site", "")),
                "unload_mask": str(value.get("unload_mask", "")),
                "assignments": dict(value.get("assignments", {})),
            }
            for key, value in catalog.items()
        }
        if normalized_catalog != EXPECTED_UNLOAD_POSE_CATALOG:
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "unload_pose_catalog",
                "the eleven supported unload operations are fixed",
                actual=normalized_catalog, expected=EXPECTED_UNLOAD_POSE_CATALOG,
            )

        routes = {str(key): dict(value) for key, value in dict(data["route_families"]).items()}
        if set(routes) != {"PICK_1_TO_3", "PICK_3_TO_1"}:
            raise V40ValidationError("CompetitionTaskConfigV40", "route_families", "must contain both route families")
        automatic = dict(data["automatic_selection"])
        if str(automatic.get("primary_objective")) != "MIN_UNLOAD_STOP_COUNT":
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "automatic_selection.primary_objective",
                "unsupported automatic objective", actual=automatic.get("primary_objective"),
                expected="MIN_UNLOAD_STOP_COUNT",
            )
        rules = {str(key): str(value) for key, value in dict(automatic.get("station_set_route_rules", {})).items()}
        if rules != EXPECTED_STATION_ROUTE_RULES:
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "automatic_selection.station_set_route_rules",
                "unexpected left/right route rules", actual=rules, expected=EXPECTED_STATION_ROUTE_RULES,
            )
        if str(automatic.get("tie_default")) != "PICK_1_TO_3":
            raise V40ValidationError(
                "CompetitionTaskConfigV40", "automatic_selection.tie_default",
                "equal routes must default to the left route", actual=automatic.get("tie_default"),
                expected="PICK_1_TO_3",
            )

        return cls(
            traj_id_mapping=mapping,
            label_semantics=labels,
            drop_stations=stations,
            bin_reachability=reachability,
            unload_pose_catalog=catalog,
            route_families=routes,
            automatic_selection=automatic,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": self.format,
            "config_version": self.config_version,
            "traj_id_mapping": self.traj_id_mapping,
            "label_semantics": self.label_semantics,
            "drop_stations": self.drop_stations,
            "bin_reachability": {key: list(value) for key, value in self.bin_reachability.items()},
            "unload_pose_catalog": self.unload_pose_catalog,
            "route_families": self.route_families,
            "automatic_selection": self.automatic_selection,
        }

    def station_for_physical_site(self, physical_site: str) -> str:
        for station, spec in self.drop_stations.items():
            if physical_site in spec.get("physical_sites", []):
                return station
        raise KeyError(physical_site)

    def pose_profile_for_assignments(
        self,
        station_or_assignments: str | dict[str, str],
        assignments: dict[str, str] | None = None,
    ) -> str | None:
        if assignments is None:
            station: str | None = None
            raw_assignments = station_or_assignments
        else:
            station = str(station_or_assignments)
            raw_assignments = assignments
        if not isinstance(raw_assignments, dict):
            return None
        normalized = dict(sorted((str(k), str(v)) for k, v in raw_assignments.items()))
        for profile_id, spec in self.unload_pose_catalog.items():
            if station is not None and str(spec.get("station_site")) != station:
                continue
            candidate = dict(sorted((str(k), str(v)) for k, v in dict(spec.get("assignments", {})).items()))
            if candidate == normalized:
                return profile_id
        return None
