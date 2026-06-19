# Phase 3 Case Drafts

Phase 3 generates reviewable Case drafts under `cases/Pxxxx.json`. These files
use the Phase 2 `CaseManifestV40` model and `storage_mode=REFERENCED`, but they
do not contain optimized legs or dense nodes yet.

## Contents

Each draft contains:

```text
source_mapping       normalized CSV row plus raw trace fields
selected_plan        selected candidate and candidate summaries
arrival_states       pickup/drop semantic arrival states
leg_refs             empty in Phase 3
actions.source       FIFO source mechanical actions
actions.compiled     empty in Phase 3
finish               copied from project.finish_policy
estimates            mechanism estimate only; motion time is unknown
review               DRAFT / INCOMPLETE_LEGS
hashes               source row, selected candidate, project task config
```

`actions.source` intentionally contains semantic references such as
`arrival_state_id`; it does not contain runtime `arrival_id`,
`check_start_s_mm`, trigger windows, expiry fields, or fake compiled actions.

## Reports

Full draft generation writes:

```text
reports/task_compile_summary.csv
reports/task_compile_report.json
reports/unique_transition_requirements.json
```

Transition requirements are derived from the selected plan and are only future
leg optimization requirements. They are directional and include full semantic
dependency hashes. Phase 3 does not optimize these requirements.

## No BIN Output

Case drafts are not executable. Formal BIN output requires later phases to
optimize or approve directed legs, assemble global nodes, compile actions, and
run full validation. Phase 3 therefore does not write `bin/Pxxxx.BIN`.
