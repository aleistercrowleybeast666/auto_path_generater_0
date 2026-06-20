# HJMB Path Generator V4.0 项目约束

本仓库只实现 V4.0 桌面路径生成软件。BIN 协议保持 version 40，结构大小固定为 Header 104、Node 16、Segment 24、Action 22 字节。

## 当前架构

- 用户模式只有 `MANUAL`、`SEMI_AUTO`、`FULL_AUTO`。
- 正式 Python 业务代码只放在 `src/hjmb_pathgen/py_*` 包中；根目录只允许薄入口 `hjmb_path_editor.py`。
- 正式入口不得导入 `py_legacy`；旧 V3.5 仅允许作为显式迁移输入。
- GUI 只有“路径编辑”和“最优路段与批量生成”两个核心页面。
- 编辑只标记 `STALE`，不得自动运行耗时规划。

## 数据语义

- 项目公共姿态恰好为 `P_START`、`P_PICK_1`、`P_PICK_2L`、`P_PICK_2R`、`P_PICK_3`。
- Case 逻辑锚点为上述 5 点和 `P_DROP_1/2/3`，共 8 点。
- `P_PICK_2L/R` 是同一物理 PICK_2 的两个接近姿态。
- 5 个物理放货箱只属于 `project.field_objects.drop_boxes`，不得伪装为路线 site。
- `traj_id.csv` 是 360 映射的唯一权威。

## 模式规则

- `MANUAL`：点和动作由用户给出；不移动点、不搜索几何；仍执行确定性曲线、速度规划、碰撞、动作编译和完整验证。
- `SEMI_AUTO`：保存完整 8 锚点；锚点锁定；优化锚点间曲线和 yaw，使用 `leg_library`。
- `FULL_AUTO`：按 traj_id 生成任务、候选、动作和路段，选择总时间最短的可行方案；结果只读。
- `FULL_AUTO -> SEMI_AUTO` 必须显式复制，保留来源 hash，绝不修改或覆盖原全自动 Case。

## 输出目录

Case、BIN、portable 分别写入 `manual/`、`semi_auto/`、`full_auto/`。只有人工明确选择并通过完整验证与审批后，才写 `bin/final/`。旧平铺、`manual_free/`、`task_compiled/` 只由显式迁移器读取；迁移不得覆盖、不得猜测，必须支持 dry-run 和报告。

## 完赛与动作

- START 速度为零，每个 ARRIVAL 必停。
- 机械动作严格 FIFO；支持 STOP_AND_WAIT、ASYNC、KINEMATIC。
- 最终动作必须是 DROP_1/2/3/12/23 的 STOP_AND_WAIT，并绑定最终唯一 `FINISH_ARM` ARRIVAL。
- 正式输出中 `SAFE_END`、`FINISH_CLEAR` 和旧 finish 字段必须为零。

## 修改与验证

- 保持纯计算层不依赖 PySide6，UI 不实现优化、碰撞或 BIN 编码。
- 文件写入先验证再原子替换，不留下半写结果。
- 新增或修改行为必须有测试；报告准确命令和结果。
- 不提交缓存、生成的 Case/BIN、日志、build/dist/release。
- 本轮不得运行 PyInstaller，不得生成 exe 或发布 zip。
