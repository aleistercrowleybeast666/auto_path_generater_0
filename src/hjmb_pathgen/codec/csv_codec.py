"""Strict traj_id.csv parser for Phase 3."""

from __future__ import annotations

import csv
import hashlib
import io
import itertools
import re
from dataclasses import dataclass
from pathlib import Path

from hjmb_pathgen.codec.canonical_json import canonical_json_crc32_hex
from hjmb_pathgen.models.errors import CsvFormatError, CsvValidationError
from hjmb_pathgen.models.route_case import RouteCaseRowV40, RouteCaseTableV40
from hjmb_pathgen.models.task_mapping import BeanType, TrajCsvRow
from hjmb_pathgen.services.path_naming import bin_name

EXPECTED_TRAJ_HEADERS = (
    "traj_id",
    "文件名",
    "bean_code",
    "drop_code",
    "①号位豆子",
    "②号位豆子",
    "③号位豆子",
    "数字1在几号位",
    "数字2在几号位",
    "数字3在几号位",
    "数字4在几号位",
    "数字5在几号位",
)

PICKUP_COLUMNS = ("①号位豆子", "②号位豆子", "③号位豆子")
LABEL_COLUMNS = tuple(f"数字{label}在几号位" for label in range(1, 6))

BEAN_NAME_TO_TYPE = {
    "黄豆": BeanType.YELLOW,
    "绿豆": BeanType.GREEN,
    "白芸豆": BeanType.WHITE,
}

DROP_SITE_NAME_TO_KEY = {
    "④号位": "F_DROP_4",
    "⑤号位": "F_DROP_5",
    "⑥号位": "F_DROP_6",
    "⑦号位": "F_DROP_7",
    "⑧号位": "F_DROP_8",
}

PICKUP_COLUMN_TO_SLOT = {
    "①号位豆子": "PICK_1",
    "②号位豆子": "PICK_2",
    "③号位豆子": "PICK_3",
}


@dataclass(frozen=True)
class TrajCsvTable:
    source_csv: str
    source_csv_sha256: str
    rows: tuple[TrajCsvRow, ...]

    def to_route_case_table(self) -> RouteCaseTableV40:
        route_rows = tuple(RouteCaseRowV40.from_dict(row.to_route_row_dict()) for row in sorted(self.rows, key=lambda item: item.traj_id))
        return RouteCaseTableV40(source_csv=self.source_csv, source_csv_sha256=self.source_csv_sha256, cases=route_rows)


def load_traj_id_csv(path: str | Path) -> TrajCsvTable:
    path = Path(path)
    return parse_traj_id_csv_bytes(path.read_bytes(), source_csv=path.name)


def parse_traj_id_csv_bytes(data: bytes, *, source_csv: str = "traj_id.csv") -> TrajCsvTable:
    text = _decode_utf8_sig(data, source_csv)
    csv_rows = list(csv.reader(io.StringIO(text, newline="")))
    if not csv_rows:
        raise CsvFormatError("traj_id.csv", "$", "CSV file is empty", actual=source_csv)
    _validate_header(csv_rows[0], source_csv)
    data_rows = _strip_trailing_blank_rows(csv_rows[1:])
    middle_blank_errors = _middle_blank_errors(data_rows)
    data_rows = [(index, row) for index, row in data_rows if not _is_blank_row(row)]

    errors: list[str] = list(middle_blank_errors)
    if len(data_rows) != 360:
        errors.append(f"data row count actual={len(data_rows)} expected=360")

    parsed_rows: list[TrajCsvRow] = []
    for row_index, cells in data_rows:
        row_number = row_index + 2
        row = _parse_business_row(row_number, cells, errors)
        if row is not None:
            parsed_rows.append(row)

    if not errors:
        errors.extend(_global_validation_errors(parsed_rows))
    if errors:
        raise CsvValidationError("traj_id.csv", "$", "CSV validation failed", actual=errors)

    return TrajCsvTable(
        source_csv=source_csv,
        source_csv_sha256=hashlib.sha256(data).hexdigest(),
        rows=tuple(sorted(parsed_rows, key=lambda row: row.traj_id)),
    )


def reconstruct_semantic_row(row: TrajCsvRow | RouteCaseRowV40) -> dict[str, str]:
    raw_fields = getattr(row, "raw_fields", None)
    if raw_fields:
        return {header: str(raw_fields[header]) for header in EXPECTED_TRAJ_HEADERS}
    return _semantic_row_from_normalized(row)


def _decode_utf8_sig(data: bytes, source_csv: str) -> str:
    try:
        return data.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise CsvFormatError("traj_id.csv", "$", f"{source_csv} is not UTF-8", actual=str(exc), expected="UTF-8 or UTF-8 with BOM") from exc


def _validate_header(header: list[str], source_csv: str) -> None:
    if header == list(EXPECTED_TRAJ_HEADERS):
        return
    duplicate = sorted({item for item in header if header.count(item) > 1})
    missing = [item for item in EXPECTED_TRAJ_HEADERS if item not in header]
    extra = [item for item in header if item not in EXPECTED_TRAJ_HEADERS]
    raise CsvFormatError(
        "traj_id.csv",
        "header",
        f"{source_csv} header must exactly match official columns",
        actual={"header": header, "duplicate": duplicate, "missing": missing, "extra": extra},
        expected=list(EXPECTED_TRAJ_HEADERS),
    )


def _strip_trailing_blank_rows(rows: list[list[str]]) -> list[tuple[int, list[str]]]:
    indexed = list(enumerate(rows))
    while indexed and _is_blank_row(indexed[-1][1]):
        indexed.pop()
    return indexed


def _middle_blank_errors(rows: list[tuple[int, list[str]]]) -> list[str]:
    errors = []
    last_nonblank = max((index for index, row in rows if not _is_blank_row(row)), default=-1)
    for index, row in rows:
        if index < last_nonblank and _is_blank_row(row):
            errors.append(f"row {index + 2}: blank row is only allowed at file end")
    return errors


def _is_blank_row(row: list[str]) -> bool:
    return not row or all(cell == "" for cell in row)


def _parse_business_row(row_number: int, cells: list[str], errors: list[str]) -> TrajCsvRow | None:
    if len(cells) != len(EXPECTED_TRAJ_HEADERS):
        errors.append(f"row {row_number}: column count actual={len(cells)} expected={len(EXPECTED_TRAJ_HEADERS)}")
        return None
    raw = dict(zip(EXPECTED_TRAJ_HEADERS, cells, strict=True))
    invalid_cells = [(column, value) for column, value in raw.items() if value != value.strip() or value == ""]
    if invalid_cells:
        for column, value in invalid_cells:
            errors.append(f"row {row_number}, column {column}: empty or whitespace-padded value is not allowed actual={value!r}")
        return None

    try:
        traj_id = _parse_decimal(raw["traj_id"], row_number, "traj_id")
        bean_code = _parse_decimal(raw["bean_code"], row_number, "bean_code")
        drop_code = _parse_decimal(raw["drop_code"], row_number, "drop_code")
        pick_assignment = _parse_pick_assignment(raw, row_number)
        label_positions = _parse_label_positions(raw, row_number)
    except CsvValidationError as exc:
        errors.append(str(exc))
        return None

    source_row_hash = canonical_json_crc32_hex(
        {"columns": [{"name": header, "value": raw[header]} for header in EXPECTED_TRAJ_HEADERS]}
    )
    return TrajCsvRow(
        traj_id=traj_id,
        file_name=raw["文件名"],
        bean_code=bean_code,
        drop_code=drop_code,
        pick_assignment=pick_assignment,
        label_positions=label_positions,
        source_row_number=row_number,
        source_row_hash=source_row_hash,
        raw_fields=raw,
    )


def _parse_decimal(value: str, row_number: int, column: str) -> int:
    if not re.fullmatch(r"[0-9]+", value):
        raise CsvValidationError("traj_id.csv", f"row {row_number}.{column}", "expected decimal integer", actual=value)
    return int(value)


def _parse_pick_assignment(raw: dict[str, str], row_number: int) -> dict[str, str]:
    result: dict[str, str] = {}
    seen = []
    for column in PICKUP_COLUMNS:
        value = raw[column]
        bean = BEAN_NAME_TO_TYPE.get(value)
        if bean is None:
            raise CsvValidationError("traj_id.csv", f"row {row_number}.{column}", "unknown bean name", actual=value, expected=sorted(BEAN_NAME_TO_TYPE))
        result[PICKUP_COLUMN_TO_SLOT[column]] = bean.value
        seen.append(bean.value)
    if sorted(seen) != sorted(bean.value for bean in BeanType):
        raise CsvValidationError("traj_id.csv", f"row {row_number}.pickup", "pickup beans must be one permutation of YELLOW/GREEN/WHITE", actual=seen)
    return result


def _parse_label_positions(raw: dict[str, str], row_number: int) -> dict[str, str]:
    result: dict[str, str] = {}
    seen = []
    for label, column in enumerate(LABEL_COLUMNS, start=1):
        value = raw[column]
        site = DROP_SITE_NAME_TO_KEY.get(value)
        if site is None:
            raise CsvValidationError("traj_id.csv", f"row {row_number}.{column}", "unknown physical drop site", actual=value, expected=sorted(DROP_SITE_NAME_TO_KEY))
        result[str(label)] = site
        seen.append(site)
    if sorted(seen) != sorted(DROP_SITE_NAME_TO_KEY.values()):
        raise CsvValidationError("traj_id.csv", f"row {row_number}.label_positions", "labels 1..5 must cover F_DROP_4..F_DROP_8 exactly once", actual=seen)
    return result


def _global_validation_errors(rows: list[TrajCsvRow]) -> list[str]:
    errors: list[str] = []
    _validate_traj_ids(rows, errors)
    _validate_bean_drop_grid(rows, errors)
    _validate_pick_permutations(rows, errors)
    _validate_drop_semantics(rows, errors)
    return errors


def _validate_traj_ids(rows: list[TrajCsvRow], errors: list[str]) -> None:
    seen: dict[int, list[int]] = {}
    for row in rows:
        seen.setdefault(row.traj_id, []).append(row.source_row_number)
        if row.file_name != bin_name(row.traj_id):
            errors.append(f"row {row.source_row_number}: file_name {row.file_name} does not match traj_id {row.traj_id}")
        if not 0 <= row.traj_id <= 359:
            errors.append(f"row {row.source_row_number}: traj_id out of range actual={row.traj_id} expected=0..359")
    duplicates = {traj_id: lines for traj_id, lines in seen.items() if len(lines) > 1}
    if duplicates:
        errors.append(f"duplicate traj_id values: {duplicates}")
    missing = sorted(set(range(360)) - set(seen))
    if missing:
        suffix = "..." if len(missing) > 20 else ""
        errors.append(f"missing traj_id values: {missing[:20]}{suffix}")


def _validate_bean_drop_grid(rows: list[TrajCsvRow], errors: list[str]) -> None:
    by_bean: dict[int, list[TrajCsvRow]] = {}
    for row in rows:
        if not 0 <= row.bean_code <= 5:
            errors.append(f"row {row.source_row_number}: bean_code out of range actual={row.bean_code} expected=0..5")
        if not 0 <= row.drop_code <= 59:
            errors.append(f"row {row.source_row_number}: drop_code out of range actual={row.drop_code} expected=0..59")
        expected_traj_id = row.bean_code * 60 + row.drop_code
        if row.traj_id != expected_traj_id:
            errors.append(f"row {row.source_row_number}: traj_id formula mismatch actual={row.traj_id} expected={expected_traj_id}")
        by_bean.setdefault(row.bean_code, []).append(row)
    for bean_code in range(6):
        group = by_bean.get(bean_code, [])
        if len(group) != 60:
            errors.append(f"bean_code {bean_code}: row count actual={len(group)} expected=60")
        drop_codes = {row.drop_code for row in group}
        if drop_codes != set(range(60)):
            errors.append(f"bean_code {bean_code}: drop_code coverage mismatch")


def _validate_pick_permutations(rows: list[TrajCsvRow], errors: list[str]) -> None:
    signatures_by_code: dict[int, set[tuple[tuple[str, str], ...]]] = {}
    for row in rows:
        signature = tuple(sorted(row.pick_assignment.items()))
        signatures_by_code.setdefault(row.bean_code, set()).add(signature)
    for bean_code, signatures in signatures_by_code.items():
        if len(signatures) != 1:
            errors.append(f"bean_code {bean_code}: pickup assignment drifts within its 60 rows")
    expected = {tuple(zip(("PICK_1", "PICK_2", "PICK_3"), perm, strict=True)) for perm in itertools.permutations([bean.value for bean in BeanType])}
    actual = {next(iter(signatures)) for signatures in signatures_by_code.values() if signatures}
    if actual != expected:
        errors.append("bean_code pickup permutations must cover all six YELLOW/GREEN/WHITE permutations exactly once")


def _validate_drop_semantics(rows: list[TrajCsvRow], errors: list[str]) -> None:
    by_drop: dict[int, list[TrajCsvRow]] = {}
    for row in rows:
        by_drop.setdefault(row.drop_code, []).append(row)
    signatures = set()
    for drop_code in range(60):
        group = by_drop.get(drop_code, [])
        if len(group) != 6:
            errors.append(f"drop_code {drop_code}: row count across bean_code actual={len(group)} expected=6")
            continue
        first = group[0].label_positions
        for row in group[1:]:
            if row.label_positions != first:
                errors.append(f"drop_code {drop_code}: label_positions drift across bean_code")
                break
        signatures.add(group[0].target_position_signature())
    if len(signatures) != 60:
        errors.append(f"drop_code target signatures actual={len(signatures)} expected=60 unique 5P3 placements")


def _semantic_row_from_normalized(row: TrajCsvRow | RouteCaseRowV40) -> dict[str, str]:
    bean_to_name = {bean.value: name for name, bean in BEAN_NAME_TO_TYPE.items()}
    site_to_name = {site: name for name, site in DROP_SITE_NAME_TO_KEY.items()}
    pick_assignment = row.pick_assignment
    label_positions = row.label_positions
    return {
        "traj_id": str(row.traj_id),
        "文件名": row.file_name,
        "bean_code": str(row.bean_code),
        "drop_code": str(row.drop_code),
        "①号位豆子": bean_to_name[str(pick_assignment["PICK_1"])],
        "②号位豆子": bean_to_name[str(pick_assignment["PICK_2"])],
        "③号位豆子": bean_to_name[str(pick_assignment["PICK_3"])],
        "数字1在几号位": site_to_name[str(label_positions["1"])],
        "数字2在几号位": site_to_name[str(label_positions["2"])],
        "数字3在几号位": site_to_name[str(label_positions["3"])],
        "数字4在几号位": site_to_name[str(label_positions["4"])],
        "数字5在几号位": site_to_name[str(label_positions["5"])],
    }
