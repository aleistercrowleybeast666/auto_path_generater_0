# Phase 9 Acceptance

Phase 9 adds final delivery infrastructure:

- protocol conformance report
- output layout report
- golden manifest generation
- final-drop BIN audit
- synthetic example project generation
- PyInstaller onedir spec
- release clean/archive scripts
- final user operation docs

## Acceptance Run

Environment:

- Date: 2026-06-19
- OS: Windows 11
- Python: 3.14.0
- PyInstaller: 6.16.0

Commands and results:

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli phase9-protocol-report --protocol HJMB_path_file_protocol_v4.0.txt --output release\phase9_validation\protocol_report.json
```

Result: passed. Header/node/segment/action sizes are 104/16/24/22, BIN version is 40, CRC vector passed.

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli create-example-project --output release\phase9_validation\example_360_cached --source-traj traj_id.csv --generate-outputs
```

Result: generated 360 task cases, 360 task BIN files, 66 synthetic unique directed legs, 0 generation failures.

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli validate-all --project release\phase9_validation\example_360_cached
```

Result: 360 cases, 0 failures, 0 invalid cases.

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli phase9-output-layout --project release\phase9_validation\example_360_cached --output release\phase9_validation\output_layout_report.json
```

Result: passed.

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli export-final --project release\phase9_validation\example_360_cached --traj-id 0 --source TASK_COMPILED
python -m hjmb_pathgen.cli phase9-final-drop-audit --bin release\phase9_validation\example_360_cached\bin\final\P0000.BIN --output release\phase9_validation\final_drop_audit_P0000.json
```

Result: final export passed for approved smoke case P0000. Final-drop audit passed.

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli phase9-golden-manifest --project release\phase9_validation\example_360_cached --output release\phase9_validation\golden_manifest_v40.json
```

Result:

- case_count: 360
- final_bin_count: 1
- manifest_sha256: `ca01af0dd48bfee3a983140dc5927cba0a9e90150b4b74912c29c3efff349abc`
- all included BIN round trips are byte-identical
- all included task cases pass final-drop audit

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli phase9-performance --project release\phase9_validation\example_360_cached --operation golden-manifest --output release\phase9_validation\performance_golden_manifest.json
```

Result:

- elapsed_ms: 5452
- peak_memory_bytes: 2558002

```powershell
python -m unittest tests.unit.test_phase7_generation tests.unit.test_phase9_delivery
python -m unittest discover
python -m py_compile hjmb_path_editor.py path_codec_cli.py path_models.py path_geometry.py trajectory_planner.py trajectory_graphics.py batch_models.py batch_generator.py v35_test_utils.py
```

Result:

- Phase7/Phase9 focused tests: 11 tests passed
- Full unittest discovery: 144 tests passed
- py_compile: passed

```powershell
python -m PyInstaller --noconfirm packaging\HJMB_Path_Generator.spec
powershell -ExecutionPolicy Bypass -File scripts\archive_source.ps1 -Output release\HJMB_Path_Generator_V4.0_source.zip
powershell -ExecutionPolicy Bypass -File scripts\archive_onedir.ps1 -DistDir dist\HJMB_Path_Generator_V4.0 -Output release\HJMB_Path_Generator_V4.0_onedir.zip
```

Result:

- onedir: `dist\HJMB_Path_Generator_V4.0`
- executable: `dist\HJMB_Path_Generator_V4.0\HJMB_Path_Generator.exe`
- source release manifest, source zip, and onedir zip are generated under `release\`.
- Archive SHA256 values are emitted by the archive scripts after the final documentation state is fixed.

## Known Boundary

- The included example project is synthetic and reproducible. It is not a measured competition calibration.
- PyInstaller build succeeded on the development machine, but clean-machine GUI launch validation was not performed here. Run the onedir package on the final target Windows machine before event use.
- One earlier pre-cache 360 generation attempt timed out and left a temporary validation directory under `release\phase9_validation\example_360`; it is ignored by Git and was not deleted automatically.
