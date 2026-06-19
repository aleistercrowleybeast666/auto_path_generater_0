# V4.0 Protocol Implementation Notes

Phase 1 implemented the executable V4.0 data contract needed before later
planning and project workflows. Phase 2 adds strict JSON/BIN IO, project
directories, atomic writes, and synthetic dense-leg assembly.

## Constants And Enums

Protocol constants and enum values are defined in:

- `src/hjmb_pathgen/models/protocol.py`
- `src/hjmb_pathgen/models/enums.py`

Implemented constants include:

- BIN version: `40`
- Header size: `104`
- Node size: `16`
- Segment size: `24`
- Action size: `22`
- Nominal field: `4000 x 2000 mm`
- Competition `traj_id`: `0..359`
- JSON formats:
  - `HJMB_PATH_PROJECT_JSON_V40`
  - `HJMB_ROUTE_CASE_TABLE_JSON_V40`
  - `HJMB_LEG_LIBRARY_JSON_V40`
  - `HJMB_ROUTE_CASE_JSON_V40`

## Packed BIN Layout

`src/hjmb_pathgen/codec/binary_layout.py` defines the exact protocol formats:

```text
HEADER_FMT  = "<4sBBBBBBBBHHBBHHHHHBBHIIIIIIIIIHHHHHHHHBbhHHHHHHI"
NODE_FMT    = "<HhhhhhhBB"
SEGMENT_FMT = "<HHHHHBBBBIIH"
ACTION_FMT  = "<BBBBHHHHHHHHH"
```

Import-time assertions verify:

```text
struct.calcsize(HEADER_FMT)  == 104
struct.calcsize(NODE_FMT)    == 16
struct.calcsize(SEGMENT_FMT) == 24
struct.calcsize(ACTION_FMT)  == 22
```

The codec supports:

- Header, node, segment, and action encode/decode.
- Offsets and file size validation.
- Version, magic, structure-size validation.
- Required header flags and unknown flag rejection.
- Reserved field validation.
- CRC validation with `Header.file_crc32` zeroed during calculation.
- `encode(decode(bytes)) == bytes` for the minimal and synthetic fixtures.
- Filename/traj_id identity through `hjmb_pathgen.codec.bin_codec`.
- Protocol-level START/ARRIVAL/FINISH, segment, and action-mode validation.

## JSON Models

Typed V4.0 models are implemented with dataclasses and explicit validation:

- `ProjectV40`
- `RouteCaseTableV40`
- `LegLibraryV40`
- `CaseManifestV40`
- `PortableCaseV40`
- `CompiledTrajectoryV40`
- `HeaderV40`
- `NodeV40`
- `SegmentV40`
- `ActionV40`

Unknown fields are rejected by default. V3.x formats and deleted fields are
rejected by `src/hjmb_pathgen/codec/legacy_rejection.py`.

## CRC And Canonical JSON

CRC-32/IEEE is implemented in `src/hjmb_pathgen/codec/crc32.py` using
`zlib.crc32(data) & 0xFFFFFFFF`.

Canonical JSON helpers are implemented in
`src/hjmb_pathgen/codec/canonical_json.py`:

- UTF-8 output.
- Recursive object-key sorting through `json.dumps(sort_keys=True)`.
- Stable separators without extra whitespace.
- `allow_nan=False` to reject NaN and Infinity.
- CRC-32 and hex helper APIs.

## Fixtures

Readable fixtures:

- `tests/fixtures/v40/minimal_project.json`
- `tests/fixtures/v40/minimal_route_case_table.json`
- `tests/fixtures/v40/minimal_leg_library.json`
- `tests/fixtures/v40/minimal_case.json`
- `tests/fixtures/v40/minimal_portable_case.json`
- `tests/fixtures/v40/synthetic_leg_library.json`
- `tests/fixtures/v40/synthetic_case.json`
- `tests/fixtures/v40/synthetic_portable_case.json`
- `tests/fixtures/legacy/v35_minimal.json`

Binary fixture:

- `tests/fixtures/v40/minimal.bin`

Rebuild `minimal.bin`:

```powershell
python -c "import sys; sys.path.insert(0, 'src'); from pathlib import Path; from hjmb_pathgen.codec.fixtures import write_minimal_bin; write_minimal_bin(Path('tests/fixtures/v40/minimal.bin'))"
```

## Phase 2 Services

Phase 2 service APIs are in:

- `hjmb_pathgen.services.project_service`
- `hjmb_pathgen.services.case_compiler`
- `hjmb_pathgen.services.portable_service`
- `hjmb_pathgen.services.output_service`
- `hjmb_pathgen.services.batch_service`
- `hjmb_pathgen.services.atomic_writer`

The synthetic compiler assembles already compiled dense legs only. It does not
parse `traj_id.csv`, choose route families, optimize legs, or check collisions.

## Tests

```powershell
python -m unittest discover -s tests -v
```

Full regression including legacy V3.5 tests:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -m unittest -v
```

Phase 1 intentionally does not implement Phase 2+ workflows: no atomic project
directory writes, no complete 360 compiler, no optimizer, no collision checker,
and no worker-process UI workflow.
