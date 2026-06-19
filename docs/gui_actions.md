# GUI Actions

The `Actions` tab shows both action layers from the selected/generated Case:

- source actions: editable task intent when present
- compiled actions: read-only compiled FIFO actions

The tab explicitly documents the final-drop completion chain:

```text
final drop ARRIVAL -> STOP_AND_WAIT DROP_* -> DONE -> post_wait -> FIFO empty -> complete
```

The GUI does not expose any `SAFE_END` or `FINISH_CLEAR` setting.
