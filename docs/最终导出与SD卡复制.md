# 最终导出与 SD 卡复制

先在当前模式完成生成、完整验证与审批，再点击“设为最终版本”。源 BIN 位于 `bin/manual`、`bin/semi_auto` 或 `bin/full_auto`，最终文件写入 `bin/final/Pxxxx.BIN`。

final 导出会重新检查 Case 非 STALE、审批状态、源 manifest、BIN 回读和最终放货完赛条件。目标已存在时必须由用户确认覆盖。复制到 SD 卡前应以 `bin/final` 为唯一来源。
