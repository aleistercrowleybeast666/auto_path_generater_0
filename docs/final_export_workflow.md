# Final Export Workflow

Final export writes `bin/final/Pxxxx.BIN`.

The user must explicitly select a source:

- `TASK_COMPILED`
- `MANUAL_FREE`

The selected source case must pass the formal export guard. In particular, generated cases remain blocked until `review.approved=true` is set by explicit review.

The final export path is never written by ordinary generate commands.
