# Phase 0 Baseline Audit

Date: 2026-06-19

## Summary

The current repository is a V3.5 path editor/codec/planner baseline with V4.0
planning documents at the repository root. The source code does not implement
V4.0 JSON/BIN codecs, V4.0 data models, continuous collision checking, shortest
time optimization, or 360-case V4.0 task compilation.

Primary acceptance environment for this baseline is official Windows x64 CPython
3.14. Python 3.13 compatibility is allowed, but MSYS2/MinGW Python is not part of
the required baseline.

## Confirmed From Source

- Main GUI entry point: `hjmb_path_editor.py`.
- GUI framework: PySide6 widgets.
- Current window title: `HJMB 空间轨迹编辑器 V3.5`.
- Current project JSON format: `HJMB_PATH_EDITOR_JSON_V35`.
- Current batch JSON format: `HJMB_PATH_BATCH_JSON_V35`.
- Current BIN codec version: `VERSION = 35`.
- Current V3.5 BIN sizes:
  - Header: 64 bytes
  - Node: 16 bytes
  - Action: 22 bytes
- Current codec has no V4.0 Segment table implementation.
- Current CLI entry point: `path_codec_cli.py`.
- Current CLI commands: `build`, `check`, `summary`, `plan`.
- Current planner implementation is in `trajectory_planner.py`.
- Current geometry sampling is in `path_geometry.py`.
- Current batch helpers are `batch_models.py` and `batch_generator.py`.
- Current fixed-site model has 8 sites:
  `P_START`, `P_PICK_1`, `P_PICK_2L`, `P_PICK_2R`, `P_PICK_3`,
  `P_DROP_1`, `P_DROP_2`, `P_DROP_3`.
- Current code explicitly rejects older V3.3/V3.4-style JSON fields such as
  `cut_in`, `stop_required`, `gate_id`, `trigger_s_mm`, `window_start`,
  `window_end`, and `min_wait_ms`.
- `AGENTS.md`, `TARGET.md`, and `HJMB_path_file_protocol_v4.0.txt` are present
  at the repository root.
- `traj_id.csv` is not present in this repository at Phase 0.

## Confirmed From Tests

Current test files:

- `test_batch_v35.py`
- `test_hjmb_path_editor_v35.py`
- `test_path_codec_v35.py`
- `test_path_geometry_v35.py`
- `test_trajectory_graphics_v35.py`
- `test_trajectory_planner_v35.py`

Current tests cover V3.5 behavior including:

- V3.5 struct sizes and CRC32 vector.
- V3.5 JSON/BIN build and parse round trip.
- V3.5 legacy JSON rejection.
- File name and `traj_id` validation.
- START/ARRIVAL velocity boundary behavior.
- V3.5 action modes: `STOP_AND_WAIT`, `ASYNC`, `KINEMATIC`.
- Automatic `KINEMATIC.check_start_s_mm` derivation.
- GUI table behavior and fixed-site editing behavior.
- V3.5 batch helper coverage and directed leg-template reuse.

Latest Phase 0 test result is recorded below after validation.

## Present Only In Design Documents

The following are described by `TARGET.md` or
`HJMB_path_file_protocol_v4.0.txt`, but are not implemented in the current source:

- V4.0 protocol constants and codecs:
  Header 104, Node 16, Segment 24, Action 22, BIN version 40.
- V4.0 JSON models:
  `project.json`, `route_case_table.json`, `leg_library.json`,
  `cases/Pxxxx.json`, portable Case JSON.
- V4.0 Segment table and finish policy.
- `traj_id.csv` as the only authority for all 360 mappings.
- V4.0 route families `PICK_1_TO_3` and `PICK_3_TO_1`.
- V4.0 vehicle-bin unload mask planning.
- Continuous collision checking and the three V4.0 footprint models.
- Time-optimal planning based on `z = v^2`.
- Directed optimized leg library with stale invalidation.
- Worker-process planning, cancellation, progress reporting, and atomic V4.0
  output generation.

## Not Implemented

- V4.0 JSON/BIN load, encode, decode, or round trip.
- V4.0 legacy rejection messages beyond the existing V3.5 rejection behavior.
- V4.0 route case table generation from `traj_id.csv`.
- V4.0 single-vs-batch byte identity guarantees.
- V4.0 continuous collision validation.
- V4.0 dynamic validation for wheel rpm/topology/collision.
- V4.0 package or PyInstaller delivery.

## Unable To Confirm

- Whether any removed V2.x/V3.x protocol documents are intentionally retained
  elsewhere. They are currently deleted in the working tree before this Phase 0
  cleanup and were not restored.
- Whether the root `P0000/P0001` JSON/BIN files were intended as durable example
  artifacts. They match generated-output naming and are not referenced by tests.

## Repository Hygiene Findings

Before cleanup, the repository contained:

- A root `.venv/` directory.
- A tracked `__pycache__/` directory with CPython 3.13/3.14 bytecode.
- Root generated outputs: `P0000.BIN`, `P0000.json`, `P0001.BIN`, `P0001.json`.
- Rendered PDF reference images: `pdf_page7_field*.png`.
- A local VS Code workspace file.
- No root `.gitignore`.

Phase 0 adds `.gitignore` rules for virtual environments, Python caches, test
caches, generated BIN/JSON outputs, batch output directories, logs, build
outputs, local IDE files, and rendered reference images.

## Dependency Baseline

`pyproject.toml` now declares:

- Python: `>=3.13,<3.15`.
- Runtime dependency: `PySide6>=6.10.1,<7`.
- Tests use standard-library `unittest`; no external test runner is required.

The verified local interpreter during audit:

```text
C:\Users\chdxm\AppData\Local\Programs\Python\Python314\python.exe
Python 3.14.0, MSC v.1944 64 bit (AMD64)
```

The verified local PySide6 version during audit:

```text
PySide6 6.10.1
```

## Validation Record

Environment:

```text
Python executable: C:\Users\chdxm\AppData\Local\Programs\Python\Python314\python.exe
Python: 3.14.0 (MSC v.1944 64 bit AMD64)
PySide6: 6.10.1
pip: 26.0.1
```

Commands and results:

```powershell
python -m pip install -e . --dry-run
```

Result: passed after allowing network for build dependency resolution.
`hjmb-path-generator==0.0.0` editable metadata was prepared successfully.
`PySide6>=6.10.1,<7` was already satisfied by local `PySide6 6.10.1`.

```powershell
python -c "import hjmb_path_editor; print('import ok')"
```

Result: passed, output `import ok`.

```powershell
python -c "from PySide6.QtWidgets import QApplication; import os; os.environ['QT_QPA_PLATFORM']='offscreen'; import hjmb_path_editor as h; app=QApplication([]); win=h.MainWindow(); print('window ok')"
```

Result: passed, output `window ok`.

```powershell
python -m py_compile batch_generator.py batch_models.py hjmb_path_editor.py path_codec_cli.py path_geometry.py path_models.py trajectory_graphics.py trajectory_planner.py v35_test_utils.py
```

Result: passed.

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -m unittest -v
```

Result: passed.

```text
Ran 46 tests in 0.567s
OK
```

Phase 0 acceptance conclusion: passed for the current official Windows x64
CPython 3.14 baseline. Python 3.13 compatibility is declared but not locally
executed because the machine only exposes Python 3.14 and Python 3.7 via `py -0p`.
