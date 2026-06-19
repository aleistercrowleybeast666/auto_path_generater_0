# 最终导出与 SD 卡复制

最终导出只写：

```text
bin/final/Pxxxx.BIN
```

命令：

```powershell
python -m hjmb_pathgen.cli export-final --project <project> --traj-id 0 --source TASK_COMPILED
python -m hjmb_pathgen.cli export-final --project <project> --traj-id 0 --source MANUAL_FREE
```

导出前必须通过 export guard。未批准、STALE、FAILED、碰撞未通过、保留位非零或旧完赛模式都会阻止导出。
