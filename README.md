# HJMB Path Generator

This repository contains the legacy HJMB V3.5 spatial trajectory
editor/planner/codec plus the phased HJMB V4.0 path-generation pipeline. V4.0 now has typed
models, strict JSON/BIN codecs, project layout services, atomic writes, a
synthetic dense-leg compiler, portable Case export, unified single/partial batch
output services, strict `traj_id.csv` parsing, deterministic task candidates,
plan locking services, 360 Case draft generation, strict manual site
configuration, site pose presets, manual free-path retiming, and robust
`z=v^2` time parameterization for finite geometry. Phase 5 adds explicit
collision configuration, three robot footprint models, continuous spatial
collision validation, collision reports, and formal export guards. Phase 6 adds
directed leg optimization, topology gates, Bezier XY curves, two-window yaw
profiles, leg-library review states, and CLI/service APIs for optimizing one
directed leg at a time. Phase 7/8 add unique-leg reuse, final case assembly,
mode-scoped TASK_COMPILED and MANUAL_FREE working outputs, explicit
`bin/final/` export, process-worker orchestration, and a V4 workflow UI shell.

Generated working cases are not automatically approved. Final BIN export is an
explicit operation that selects `TASK_COMPILED` or `MANUAL_FREE` and writes only
to `bin/final/Pxxxx.BIN` after export guards pass.

See:

- `AGENTS.md` for persistent project instructions.
- `TARGET.md` for the phased V4.0 delivery plan.
- `HJMB_path_file_protocol_v4.0.txt` for the future authoritative V4.0 protocol.
- `docs/baseline_audit.md` for the Phase 0 baseline audit.
- `docs/development.md` for reproducible setup and validation commands.
- `docs/architecture.md` for the Phase 1 package layout.
- `docs/protocol_v40_implementation.md` for V4.0 protocol implementation notes.
- `docs/project_layout.md` for Phase 2 project directories.
- `docs/json_io.md` and `docs/bin_io.md` for Phase 2 codecs.
- `docs/atomic_writes.md` for atomic output behavior.
- `docs/phase2_acceptance.md` for the current Phase 2 fixture and checks.
- `docs/traj_id_mapping.md`, `docs/task_compiler.md`,
  `docs/candidate_plans.md`, and `docs/case_drafts.md` for Phase 3 behavior.
- `docs/phase3_acceptance.md` for the current Phase 3 checks.
- `docs/time_parameterization.md` and `docs/phase4_acceptance.md` for Phase 4
  project configuration, manual paths, and robust retiming.
- `docs/collision_model.md`, `docs/collision_configuration.md`,
  `docs/continuous_collision_check.md`, `docs/path_validation.md`, and
  `docs/phase5_acceptance.md` for Phase 5 collision validation.
- `docs/leg_definition.md`, `docs/topology_gates.md`,
  `docs/curve_representation.md`, `docs/yaw_window_optimization.md`,
  `docs/leg_optimizer.md`, `docs/leg_library.md`, and
  `docs/phase6_acceptance.md` for Phase 6 directed leg optimization.
- `docs/phase7_audit_and_fixes.md`, `docs/output_mode_layout.md`,
  `docs/task_compiled_workflow.md`, `docs/manual_free_workflow.md`,
  `docs/final_export_workflow.md`, `docs/leg_clear_and_regenerate.md`,
  `docs/ui_architecture.md`, `docs/worker_process.md`, and
  `docs/phase8_acceptance.md` for Phase 8 workflow behavior.
- `docs/phase9_acceptance.md` plus the Chinese user manuals under `docs/`
  for final verification, packaging, delivery, and operator workflows.
- `docs/gui_field_editor.md`, `docs/gui_fixed_sites.md`,
  `docs/gui_manual_free.md`, `docs/gui_route_leg.md`,
  `docs/gui_actions.md`, `docs/gui_regression_fix.md`, and
  `docs/phase9_gui_addendum.md` for the final source-level GUI regression fix.

## Environment

Primary development and acceptance environment:

```powershell
python --version
```

Expected primary interpreter:

```text
Python 3.14.x, official Windows x64 CPython
```

Python 3.13 compatibility is allowed. MSYS2/MinGW Python is not required.

Install:

```powershell
python -m venv .venv
.\.venv\Scripts\python -m pip install --upgrade pip
.\.venv\Scripts\python -m pip install -e .
```

PySide6 must be a release that supports Python 3.14. The project metadata
requires `PySide6>=6.10.1,<7`.

## Run

```powershell
python hjmb_path_editor.py
```

`hjmb_path_editor.py` still exports the legacy V3.5 `MainWindow` symbols for
old tests and scripts. Running it as an application launches the V4 workflow UI.

Headless import smoke check:

```powershell
python -c "import hjmb_path_editor; print('import ok')"
```

Package import smoke check:

```powershell
python -c "import sys; sys.path.insert(0, 'src'); import hjmb_pathgen; print(hjmb_pathgen.__version__)"
```

## Current V3.5 Behavior

- `START` begins from rest; first node has `vx/vy/wz=0`.
- Editable point types are `START`, `WAYPOINT`, and `ARRIVAL`.
- `ARRIVAL` nodes stop fully; the last `ARRIVAL` is the V3.5 end node.
- Yaw anchors are `START` and `ARRIVAL`, with `SHORTEST`, `CW_ONLY`, and
  `CCW_ONLY` policies.
- Path modes are `FREE` and `FIXED_8`.
- Fixed V3.5 sites are `P_START`, `P_PICK_1`, `P_PICK_2L`, `P_PICK_2R`,
  `P_PICK_3`, `P_DROP_1`, `P_DROP_2`, and `P_DROP_3`.
- Mechanical action modes are `STOP_AND_WAIT`, `ASYNC`, and `KINEMATIC`.
- `ASYNC` has no trigger field.
- `KINEMATIC` has generated `check_start_s_mm`; old window/expire fields are
  rejected.
- `min_wait_ms` has been replaced by `post_wait_ms`.

## Current V3.5 BIN

The current code is V3.5:

```text
BeanTrajectoryHeaderV35_t       64 bytes
BeanTrajectoryNodeV35_t         16 bytes
BeanMechanicalActionV35_t       22 bytes
```

Python struct formats:

```text
Header  <4sBBBBHHHHHHBBHIIIIIIHHHHHHHH
Node    <HhhhhhhBB
Action  <BBBBHHHHHHHHH
```

`Header.version=35`. CRC uses `zlib.crc32()` with `file_crc32` cleared before
calculation.

## Phase 2 V4.0 Data Pipeline

V4.0 constants, enums, typed JSON models, CRC/canonical JSON helpers, JSON/BIN
codecs, and service-layer project output helpers live under `src/hjmb_pathgen`.

The implemented V4.0 packed formats are:

```text
Header  104 bytes
Node     16 bytes
Segment  24 bytes
Action   22 bytes
```

Phase 2 public service modules include:

```text
hjmb_pathgen.codec.json_codec
hjmb_pathgen.codec.bin_codec
hjmb_pathgen.services.project_service
hjmb_pathgen.services.case_compiler
hjmb_pathgen.services.output_service
hjmb_pathgen.services.batch_service
```

A minimal programmatic project can be created with:

```python
from pathlib import Path
from hjmb_pathgen.codec.json_codec import load_project
from hjmb_pathgen.services.project_service import ProjectLayout

project = load_project("tests/fixtures/v40/minimal_project.json")
layout = ProjectLayout.create(Path("example_v40_project"), project)
print(layout.status().status)
```

Synthetic single-case output is covered by tests using
`tests/fixtures/v40/synthetic_case.json` and
`tests/fixtures/v40/synthetic_leg_library.json`. Portable cases use
`Pxxxx.portable.json` with embedded dense legs and regenerate byte-identical BIN
for the same semantic Case.

Phase 2 remains the strict codec/output foundation. Phase 3 adds task drafting
on top, but executable competition BIN output still requires later optimized
leg assembly.

## Phase 3 traj_id Task Drafts

Phase 3 public service modules include:

```text
hjmb_pathgen.codec.csv_codec
hjmb_pathgen.services.traj_table_service
hjmb_pathgen.services.task_compiler
hjmb_pathgen.services.case_draft_service
hjmb_pathgen.services.plan_lock_service
```

Minimal CLI examples:

```powershell
python -m hjmb_pathgen.cli validate-traj-table --project example_v40_project
python -m hjmb_pathgen.cli build-route-case-table --project example_v40_project
python -m hjmb_pathgen.cli list-candidates --project example_v40_project --traj-id 0
python -m hjmb_pathgen.cli generate-case-draft --project example_v40_project --traj-id 0
python -m hjmb_pathgen.cli generate-all-case-drafts --project example_v40_project
python -m hjmb_pathgen.cli lock-plan --project example_v40_project --traj-id 0 --candidate-id C_PICK_1_TO_3_00000000
python -m hjmb_pathgen.cli unlock-plan --project example_v40_project --traj-id 0
```

`generate-all-case-drafts` writes `cases/P0000.json` through `P0359.json` plus
Phase 3 reports. It does not write formal `bin/Pxxxx.BIN` files.

## Phase 4 Project Configuration and Retiming

Phase 4 public service modules include:

```text
hjmb_pathgen.services.project_config_service
hjmb_pathgen.services.site_preset_service
hjmb_pathgen.services.manual_path_service
hjmb_pathgen.planning.time_parameterization
```

Minimal CLI examples:

```powershell
python -m hjmb_pathgen.cli validate-project-config --project example_v40_project
python -m hjmb_pathgen.cli export-site-preset --project example_v40_project --name measured_a
python -m hjmb_pathgen.cli import-site-preset --project example_v40_project --preset example_v40_project\presets\measured_a.site_poses.json --preview
python -m hjmb_pathgen.cli apply-site-preset --project example_v40_project --preset example_v40_project\presets\measured_a.site_poses.json
python -m hjmb_pathgen.cli plan-manual-case --project example_v40_project --case manual_case.json
python -m hjmb_pathgen.cli retime-case --project example_v40_project --case manual_case.json
```

Manual free-path retiming is a Phase 4 developer capability. It produces V40
nodes and manual override semantics for finite manual geometry, but it is not a
final 360 competition BIN generation.

## Phase 5 Collision Validation

Phase 5 public service modules include:

```text
hjmb_pathgen.services.collision_config_service
hjmb_pathgen.services.path_validation_service
hjmb_pathgen.services.export_guard_service
hjmb_pathgen.collision
```

`project.json` must now configure:

- `vehicle.footprint.r_large_mm`
- `vehicle.footprint.r_small_mm`
- `vehicle.footprint.collision_resolution_mm`
- `vehicle.footprint.strict_validation_resolution_mm`
- `vehicle.footprint.numerical_epsilon_mm`
- `vehicle.footprint.pickup_arc_segments`
- `vehicle.footprint.field_boundary_footprint_profile`
- two cylinders, three pickup boxes, five drop boxes, and the nominal field
  boundary under `field_objects`

Run explicit validation commands:

```powershell
python -m hjmb_pathgen.cli validate-collision-config --project example_v40_project
python -m hjmb_pathgen.cli validate-current-case-collision --project example_v40_project --case manual_case.json --report example_v40_project\reports\collision\P0000_collision.json
python -m hjmb_pathgen.cli show-collision-report --report example_v40_project\reports\collision\P0000_collision.json
```

Editing sites, footprint radii, obstacle geometry, or path geometry marks old
collision results stale by hash semantics. The software does not automatically
rerun collision validation after edits.

## Phase 6 Directed Leg Optimization

Phase 6 public service modules include:

```text
hjmb_pathgen.planning.leg_optimizer
hjmb_pathgen.services.leg_optimization_service
hjmb_pathgen.services.leg_library_service
hjmb_pathgen.services.leg_stale_service
```

Minimal CLI examples:

```powershell
python -m hjmb_pathgen.cli list-transition-requirements --project example_v40_project --case example_v40_project\cases\P0000.json
python -m hjmb_pathgen.cli optimize-leg --project example_v40_project --case example_v40_project\cases\P0000.json --transition-id TR_00000000 --profile STANDARD --replace
python -m hjmb_pathgen.cli validate-leg --project example_v40_project --leg-id LEG_000000000000
python -m hjmb_pathgen.cli approve-leg --project example_v40_project --leg-id LEG_000000000000
python -m hjmb_pathgen.cli lock-leg --project example_v40_project --leg-id LEG_000000000000
python -m hjmb_pathgen.cli unlock-leg --project example_v40_project --leg-id LEG_000000000000
python -m hjmb_pathgen.cli show-leg --project example_v40_project --leg-id LEG_000000000000
```

The optimizer works on one directed transition at a time and does not auto-run
after edits. Phase 7 generation commands consume the explicit leg library; they
do not silently optimize missing legs.

## Phase 7 Final Generation

Formal V4.0 completion is bound to the final drop: the last drop ARRIVAL is the
unique `FINISH_ARM`, the final action is `DROP_* STOP_AND_WAIT`, and formal BIN
output rejects `SAFE_END`, `FINISH_CLEAR`, and half-plane finish fields.

Minimal CLI examples:

```powershell
python -m hjmb_pathgen.cli audit-phase6 --project example_v40_project
python -m hjmb_pathgen.cli collect-unique-legs --project example_v40_project
python -m hjmb_pathgen.cli show-leg-status --project example_v40_project
python -m hjmb_pathgen.cli optimize-missing-legs --project example_v40_project --profile STANDARD
python -m hjmb_pathgen.cli evaluate-case-candidates --project example_v40_project --traj-id 0
python -m hjmb_pathgen.cli generate-one --project example_v40_project --traj-id 0
python -m hjmb_pathgen.cli generate-all --project example_v40_project
python -m hjmb_pathgen.cli validate-one --project example_v40_project --traj-id 0
python -m hjmb_pathgen.cli validate-all --project example_v40_project
python -m hjmb_pathgen.cli export-portable --project example_v40_project --traj-id 0
python -m hjmb_pathgen.cli show-batch-report --project example_v40_project
```

## Phase 9 Final Delivery

Phase 9 adds deterministic delivery checks, a synthetic 360-case example
project, final-drop BIN audits, and a PyInstaller onedir build spec. The
example project is reproducible validation data; it is not measured competition
calibration.

Minimal final verification commands:

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli phase9-protocol-report --protocol HJMB_path_file_protocol_v4.0.txt --output release\phase9_validation\protocol_report.json
python -m hjmb_pathgen.cli create-example-project --output release\phase9_validation\example_360 --source-traj traj_id.csv --generate-outputs
python -m hjmb_pathgen.cli phase9-golden-manifest --project release\phase9_validation\example_360 --output release\phase9_validation\golden_manifest_v40.json
python -m hjmb_pathgen.cli phase9-performance --project release\phase9_validation\example_360 --operation golden-manifest --output release\phase9_validation\performance_golden_manifest.json
```

Build the Windows onedir package:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build_pyinstaller_onedir.ps1
powershell -ExecutionPolicy Bypass -File scripts\archive_onedir.ps1
```

The tracked PyInstaller spec is `packaging/HJMB_Path_Generator.spec`; generated
release outputs live under `release/`, `build/`, and `dist/` and are ignored by
Git.

## CLI

```powershell
python path_codec_cli.py plan example_path.json
python path_codec_cli.py summary example_path.json
python path_codec_cli.py build example_path.json P0000.BIN
python path_codec_cli.py check P0000.BIN
```

`P0000.BIN` is generated output and is ignored by Git.

## Tests

Phase 2 focused tests:

```powershell
python -m unittest tests.unit.test_phase2_codecs tests.unit.test_phase2_services -v
```

V4.0 package tests:

```powershell
python -m unittest discover -s tests -v
```

Phase 3 focused tests:

```powershell
python -m unittest tests.unit.test_phase3_csv_codec tests.unit.test_phase3_task_compiler -v
python -m unittest tests.integration.test_phase3_official_traj_id -v
```

Phase 4 focused tests:

```powershell
python -m unittest tests.unit.test_phase4_project_config tests.unit.test_phase4_site_presets tests.unit.test_phase4_manual_path tests.unit.test_phase4_time_parameterization -v
```

Phase 5 focused tests:

```powershell
python -m unittest tests.unit.test_phase5_collision_geometry tests.unit.test_phase5_path_validation -v
```

Phase 6 focused tests:

```powershell
python -m unittest -v tests.unit.test_phase6_geometry_yaw_topology tests.unit.test_phase6_leg_optimizer tests.unit.test_phase6_leg_library
```

Full regression:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -m unittest -v
```

```powershell
python -m py_compile hjmb_path_editor.py path_codec_cli.py path_models.py path_geometry.py trajectory_planner.py trajectory_graphics.py batch_models.py batch_generator.py v35_test_utils.py
```
