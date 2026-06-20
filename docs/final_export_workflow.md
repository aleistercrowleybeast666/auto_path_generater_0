# Final export workflow

Final export accepts `MANUAL`, `SEMI_AUTO`, or `FULL_AUTO`. It reloads and fully validates the selected mode Case, requires approval and a non-STALE state, verifies the source manifest and BIN round trip, then atomically writes `bin/final/Pxxxx.BIN` after overwrite confirmation.
