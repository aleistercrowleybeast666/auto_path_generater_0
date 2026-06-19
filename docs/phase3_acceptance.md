# Phase 3 Acceptance Notes

Phase 3 implements strict `traj_id.csv` parsing, deterministic route-case table
generation, task candidate compilation, user plan locking services, 360 Case
draft generation, and transition-requirement reporting.

It intentionally does not implement Phase 4+ speed planning, collision
checking, optimized legs, dense nodes, formal BIN output, firmware, or major UI
workflow changes.

## Implemented Modules

```text
src/hjmb_pathgen/codec/csv_codec.py
src/hjmb_pathgen/models/task_mapping.py
src/hjmb_pathgen/models/task_plan.py
src/hjmb_pathgen/services/action_source_compiler.py
src/hjmb_pathgen/services/task_compiler.py
src/hjmb_pathgen/services/traj_table_service.py
src/hjmb_pathgen/services/case_draft_service.py
src/hjmb_pathgen/services/plan_lock_service.py
src/hjmb_pathgen/utils/yaw_unwrap.py
src/hjmb_pathgen/cli/
```

## Real traj_id.csv

The repository root `traj_id.csv` is used by the Phase 3 integration test.

```text
Encoding: UTF-8 with BOM accepted by utf-8-sig parser
Data rows: 360
SHA256: 5cb6c215dafce49b32cbf31f1a579ca44b1212e4a2e7f6eb6b45208e069d1219
Header: traj_id, 文件名, bean_code, drop_code, ①号位豆子, ②号位豆子, ③号位豆子, 数字1在几号位, 数字2在几号位, 数字3在几号位, 数字4在几号位, 数字5在几号位
```

Official mapping statistics from the integration path:

```text
route_case_table canonical hash: 50e9541c
case drafts generated: 360
case draft failures: 0
candidate total: 1584
candidate count per Case: min 2, max 6
dual unload candidates: 864
unique transition requirements: 25
formal BIN generated: 0
```

## Verification Commands

Latest local focused results:

```powershell
python -m unittest tests.unit.test_phase3_csv_codec tests.unit.test_phase3_task_compiler -v
```

Result: `Ran 16 tests`, `OK`.

```powershell
python -m unittest tests.integration.test_phase3_official_traj_id -v
```

Result: `Ran 1 test`, `OK`.

Run the full suite before final acceptance:

```powershell
python -m compileall -q src tests
python -m unittest discover -s tests -v
$env:QT_QPA_PLATFORM='offscreen'
python -m unittest -v
```

Latest full local results:

```powershell
python -m compileall -q src tests
```

Result: passed.

```powershell
python -m unittest discover -s tests -v
```

Result: `Ran 45 tests`, `OK`.

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -m unittest -v
```

Result: `Ran 91 tests`, `OK`.

Smoke checks:

```powershell
python -c "import hjmb_path_editor; print('import ok')"
$env:QT_QPA_PLATFORM='offscreen'
python -c "from PySide6.QtWidgets import QApplication; import hjmb_path_editor as h; app=QApplication([]); win=h.MainWindow(); print('window ok')"
python -c "import sys; sys.path.insert(0, 'src'); from hjmb_pathgen.codec.csv_codec import load_traj_id_csv; print(len(load_traj_id_csv('traj_id.csv').rows))"
```

Result: all passed.

Phase 3 acceptance conclusion: passed for the implemented scope. Phase 4 has
not started.
