# GUI Fixed Sites

The `Project/Sites` tab loads the exact ten V4 site keys:

- `P_START`
- `P_PICK_1`
- `P_PICK_2L`
- `P_PICK_2R`
- `P_PICK_3`
- `F_DROP_4`
- `F_DROP_5`
- `F_DROP_6`
- `F_DROP_7`
- `F_DROP_8`

`P_START` and pickup-side poses have yaw handles. `F_DROP_4` through
`F_DROP_8` have only `x/y`; their unload yaw comes from `unload_profiles`, not
from a shared physical drop-site yaw.

Workflow:

1. Select a site in the table.
2. Double-click the field to set its `x/y` and `configured=true`.
3. Drag the site marker to adjust position.
4. Drag the yaw handle for `P_START`/pickup poses.
5. Save explicitly with `保存 project.json`.

No edit automatically replans or exports.
