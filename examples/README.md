# HJMB V4.0 Examples

`HJMB_Path_Project_Example` is a synthetic reproducibility project.

It contains:

- `project.json`
- `traj_id.csv`
- `route_case_table.json`
- `leg_library.json`
- mode-separated output directories

It is not measured competition calibration data. Before competition use,
remeasure all ten sites, obstacle geometry, vehicle footprint, wheel parameters,
dynamics, action timing, unload yaw, and safety margins.

Regenerate the example from source:

```powershell
$env:PYTHONPATH='src'
python -m hjmb_pathgen.cli create-example-project --output examples\HJMB_Path_Project_Example --source-traj traj_id.csv
```
