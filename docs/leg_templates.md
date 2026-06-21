# V4.0 人工 Leg 模板数据与服务

数据层不增加生成模式或 BIN 字段。第 2 轮在真实主窗口中增加第4个“Leg 模板”标签，并继续复用本文件所列服务。

项目根目录中的源文件为 `leg_templates.json`，格式常量为
`HJMB_LEG_TEMPLATES_JSON_V40`、`config_version=1`。它只保存方向、逻辑端点、
启用状态和人工 XY 引导点，不保存 yaw、traj_id、机械动作或 BIN 节点。

验证输出写入：

- `leg_template_instances.json`：格式 `HJMB_LEG_TEMPLATE_INSTANCES_JSON_V40`；按精确固定姿态或
  configured unload pose profile 展开，成功实例包含正式规划器生成且经严格复验的 `LegV40` 快照。
- `reports/leg_template_validation_report.json`：格式
  `HJMB_LEG_TEMPLATE_VALIDATION_REPORT_JSON_V40`；记录每模板和每实例状态、结构化失败原因与指标。

三个文档均为 UTF-8 无 BOM、稳定键序、严格顶层/记录 schema，并通过同目录临时文件、写回解析和
原子替换保存。源模板不会直接进入 `project.json` 或 `leg_library.json`。

纯服务入口位于 `hjmb_pathgen.py_services.leg_template_service`：

```text
sync_leg_templates / sync_leg_templates_for_layout
expand_leg_template_instances
validate_leg_template
validate_leg_template_for_layout
validate_all_enabled_templates / validate_all_enabled_templates_for_layout
leg_template_instance_is_current_and_passed
```

同步只维护 16 个合法方向槽位，不运行规划；配置变化保留 enabled/waypoints 并使旧结论 STALE。
验证复用正式 Bezier、自动 yaw、拓扑门、连续车体碰撞和 V4 时间参数化链路。只有当前模板与依赖
hash 下状态为 PASSED 的实例才具备后续接入资格，本轮不接入 FULL_AUTO。

## GUI 第4页

页面顶部提供同步、重新加载、保存、验证当前、验证全部已启用、取消及三个 JSON 输出。模板表显示16个稳定槽位；场地图复用 V4 坐标变换、障碍物和虚拟门图层。用户只能添加、拖动、重排和删除 XY waypoint，不能编辑 yaw、速度、控制柄或机械动作。

waypoint 改动只生成防抖轻量预览并标记草稿；点击“确定/验证当前模板”后才通过 worker 调用严格验证服务。后台任务携带模板 hash、依赖 hash、页面 revision 和 job token，旧任务不能覆盖更新后的草稿或项目。

内部文件位置不变。页面的“输出模板 JSON”“输出实例 JSON”“输出验证报告”和“全部输出”只原子复制并校验现有结果，不触发规划，不生成 BIN。`PASSED` 表示全部精确实例通过，`PARTIAL` 表示部分通过，`FAILED` 表示全部失败。FULL_AUTO 来源策略仍未接入。
