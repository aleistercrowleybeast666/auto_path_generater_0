# 三模式输出布局

正式输出按 generation_mode 隔离：`cases/<mode>`、`bin/<mode>`、`portable/<mode>`，mode 为 `manual`、`semi_auto`、`full_auto`。比赛选定文件单独写 `bin/final`。

同 traj_id 三模式可共存且禁止互相覆盖。旧平铺和旧两模式布局只能通过显式迁移器处理。
