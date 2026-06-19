# Changelog

## 4.0.0

- Added V4.0 JSON/BIN models, strict codecs, CRC and canonical JSON hashing.
- Added traj_id.csv normalization, task candidate compilation, plan locking, and 360 draft generation.
- Added robust manual retiming, collision validation, directed leg optimization, leg library review states, and final case assembly.
- Added TASK_COMPILED and MANUAL_FREE mode-separated working outputs and explicit `bin/final` export.
- Added final-drop completion validation: unique final `FINISH_ARM`, final `DROP_* STOP_AND_WAIT`, reserved `SAFE_END`/`FINISH_CLEAR` zero.
- Added V4 workflow UI shell, worker process orchestration, Phase9 reports, example project generation, and PyInstaller onedir packaging files.
