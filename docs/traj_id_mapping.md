# Phase 3 traj_id Mapping

`traj_id.csv` is the only authority for the 360 competition mappings. Phase 3
parses UTF-8 with or without BOM and rejects non-UTF-8 input, unknown headers,
unknown Chinese business values, middle blank rows, duplicate IDs, missing IDs,
and any 6x60 grid inconsistency.

## Official Columns

The parser requires this exact column order:

```text
traj_id
文件名
bean_code
drop_code
①号位豆子
②号位豆子
③号位豆子
数字1在几号位
数字2在几号位
数字3在几号位
数字4在几号位
数字5在几号位
```

Rows preserve `source_row_number`, the official raw fields, and a stable
`source_row_hash` computed from the official column order. The top-level
`source_csv_sha256` is computed from the original CSV bytes, so BOM and line
endings remain traceable.

## Normalization

Bean labels map to internal enums:

```text
黄豆   -> YELLOW
绿豆   -> GREEN
白芸豆 -> WHITE
```

Physical drop labels map to:

```text
④号位 -> F_DROP_4
⑤号位 -> F_DROP_5
⑥号位 -> F_DROP_6
⑦号位 -> F_DROP_7
⑧号位 -> F_DROP_8
```

`bean_code` must cover the six pickup bean permutations, each with all
`drop_code` values 0..59. `traj_id` must equal `bean_code * 60 + drop_code`.

## route_case_table.json

`write_route_case_table()` writes `route_case_table.json` with the Phase 2 JSON
codec and immediate read-back validation. It stores normalized mapping only:
no route candidates, no optimized legs, no dense nodes, and no selected plan.
