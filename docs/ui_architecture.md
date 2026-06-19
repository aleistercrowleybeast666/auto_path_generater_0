# UI Architecture

The V4 UI entry point is `hjmb_pathgen.ui.main_window.V4MainWindow`.

The window is organized around Phase 8 workflow tabs:

- Project/Sites
- Vehicle/Collision
- Route/Leg
- Actions
- Leg Library
- Task/360
- Manual Free
- Planning
- Reports/Final

Long-running commands are started by explicit buttons and routed through the process worker. The old V3.5 `MainWindow` remains importable for compatibility tests, but application startup now uses the V4 workflow UI.
