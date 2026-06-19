# UI Architecture

The V4 UI entry point is `hjmb_pathgen.ui.main_window.V4MainWindow`.

The main window is a composition shell. Field drawing, graphics items, table
models, and workflow tabs live in separate modules under `src/hjmb_pathgen/ui`.

Primary modules:

- `field_view.py`: V4 QGraphicsView field canvas
- `graphics_items.py`: editable point and yaw-handle items
- `ui_state.py`: project loading and shared GUI state
- `models/`: Qt table models
- `tabs/`: Project/Sites, Manual Free, Route/Leg, Leg Library, Task/360,
  Actions, Vehicle/Collision, Planning, and Reports/Final tabs

Long-running commands are started by explicit buttons and routed through the
process worker. Editing field points, yaw handles, tables, or manual sparse
paths only marks the GUI dirty/STALE and never starts planning automatically.

The old V3.5 `MainWindow` remains importable for compatibility tests, but
application startup uses the V4 workflow UI and V4 data models.
