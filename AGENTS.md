# HJMB Path Generator V4.0 项目约束

本仓库只实现 V4.0 桌面路径生成软件。BIN 协议保持 version 40，结构大小固定为 Header 104、Node 16、Segment 24、Action 22 字节。

## 默认工作方式

* 默认采用最小改动原则，只完成用户明确要求的工作。
* 不主动扩大任务范围，不顺便重构、清理、优化或实现未来功能。
* 优先读取用户明确指定的文件和目录，不先扫描整个仓库。
* 引用长文档时，只读取用户指定的章节；没有必要时不通读全文。
* 简单文件操作不制定计划，不搜索引用，不运行构建或测试，除非用户明确要求。
* 代码修改只运行与本次改动直接相关的最小测试，不默认运行全量回归。
* 不主动更新 README、协议、注释、配置和文档，除非它们属于明确要求。
* 不主动创建额外脚本、备份文件、迁移工具或兼容层。
* 遇到歧义时，不通过大范围仓库调查自行扩展需求；只提出一个必要问题。
* 达到用户给出的完成条件后立即停止，不继续寻找额外问题。
* 最终回复保持简短，只报告修改文件、验证结果和未完成事项。
* 删除操作仅作用于明确列出的路径；不得推断其他文件也应该删除。

## 当前架构

- 用户模式只有 `MANUAL`、`SEMI_AUTO`、`FULL_AUTO`。
- 正式 Python 业务代码只放在 `src/hjmb_pathgen/py_*` 包中；根目录只允许薄入口 `hjmb_path_editor.py`。
- 正式入口不得导入 `py_legacy`；旧 V3.5 仅允许作为显式迁移输入。
- 当前真实入口 `v35_exact_main_window` 的右侧标签依次包含路径点、机械动作、固定8点/批量、Leg 模板和规划参数；Leg 模板是编辑页，不是第四种生成模式。
- 编辑只标记 `STALE`，不得自动运行耗时规划。
- Leg 模板页不得改写或旁路 traj_id 输入、下拉选择和刷新保持逻辑。

## 数据语义

- 项目公共姿态恰好为 `P_START`、`P_PICK_1`、`P_PICK_2L`、`P_PICK_2R`、`P_PICK_3`。
- Case 逻辑锚点为上述 5 点和 `P_DROP_1/2/3`，共 8 点。
- `P_PICK_2L/R` 是同一物理 PICK_2 的两个接近姿态。
- 5 个物理放货箱只属于 `project.field_objects.drop_boxes`，不得伪装为路线 site。
- `project.json.sites.P_DROP_1/2/3.yaw_ddeg=0xFFFF` 只表示固定倒货位不约束方向；实际倒货 yaw 必须来自 `unload_pose_profiles` 的 11 个已标定操作姿态。
- 双倒仅允许 F4+F5/BIN_12 和 F7+F8/BIN_23。
- `task_config/competition_task_config.json` 是正常工作流中 360 映射、储箱可达性、双倒组合和左右路线规则的唯一权威；`traj_id.csv` 仅允许显式旧工程迁移或回归比对。

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
