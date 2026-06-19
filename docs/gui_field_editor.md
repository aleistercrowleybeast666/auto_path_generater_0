# GUI Field Editor

The V4 GUI now uses `hjmb_pathgen.ui.field_view.V4FieldView` for all field
canvases.

Core behavior:

- always draws the nominal 4000 x 2000 mm field, grid, axes, origin, and start area
- reads obstacles from `project.json.field_objects`
- reads site poses from `project.json.sites`
- uses world coordinates in millimeters with `x` right and `y` up
- supports wheel zoom, middle-button panning, fit-to-field, and structured scene dumps

Editing a point or yaw handle updates the in-memory V4 model, marks the GUI
dirty/STALE, and does not start planning, validation, optimization, or BIN
writing.
