# Output Mode Layout

Phase 8 separates working outputs by path source:

```text
cases/task_compiled/Pxxxx.json
cases/manual_free/Pxxxx.json
bin/task_compiled/Pxxxx.BIN
bin/manual_free/Pxxxx.BIN
bin/final/Pxxxx.BIN
portable/task_compiled/Pxxxx.portable.json
portable/manual_free/Pxxxx.portable.json
```

The older flat paths are retained only for compatibility APIs and legacy files. New Phase 7/8 generation uses the mode directories.

`bin/final/` is not a working directory. It is written only by explicit final export after the selected source case passes export guards.
