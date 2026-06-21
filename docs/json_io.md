# V4.0 JSON IO

Phase 2 implements strict typed JSON codecs in
`src/hjmb_pathgen/py_io/codecs/json_codec.py`.

## Supported Documents

The public APIs cover the Phase 2 JSON classes and the independent round-1 leg-template documents:

```text
load_project / save_project
load_route_case_table / save_route_case_table
load_leg_library / save_leg_library
load_case / save_case
load_portable_case / save_portable_case
load_leg_templates / save_leg_templates
load_leg_template_instances / save_leg_template_instances
load_leg_template_validation_report / save_leg_template_validation_report
```

The loaded models are:

```text
ProjectV40
RouteCaseTableV40
LegLibraryV40
CaseManifestV40
PortableCaseV40
LegTemplatesV40
LegTemplateInstancesV40
LegTemplateValidationReportV40
```

## Strict Rules

JSON input must be UTF-8 without BOM. The codec rejects invalid JSON, non-object
top-level values, unknown fields, V3.x formats, and deleted V3.x fields such as
`trigger_s_mm`, `window_start`, `expire_s_mm`, and action-level `flags`.

Dense V4.0 leg nodes are allowed to use their protocol `flags` field.

Case and portable-case loads enforce filename/traj_id identity when using the
file APIs:

```text
P0007.json          -> traj_id 7
P0007.portable.json -> traj_id 7
```

## Stable Output

The JSON dump is deterministic:

- UTF-8 without BOM
- `sort_keys=True`
- two-space indentation
- trailing newline
- `allow_nan=False`

`save_*` APIs write with the Phase 2 atomic writer and validate by immediately
loading the temporary file before replacement.
