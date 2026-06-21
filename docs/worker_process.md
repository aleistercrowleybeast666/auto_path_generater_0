# Worker Process

Long-running jobs use `hjmb_pathgen.py_workers.worker_process`.

Supported worker jobs:

- `generate-one`
- `generate-all`
- `optimize-missing-legs`
- `validate-all`
- `export-final`
- `validate-leg-template`
- `validate-all-leg-templates`

The worker uses a spawned process and queue messages for progress, result, cancellation, and error reporting. The UI polls those messages and keeps heavy jobs outside the Qt main thread.

Some lower-level optimizer loops are still cooperative only at service boundaries; deeper cancellation checkpoints can be added inside the optimizer in a later phase.
