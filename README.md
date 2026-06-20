# HJMB Path Generator V4.0

HJMB V4.0 是离线路径编辑、路段优化、360 Case 编译、连续碰撞验证和 BIN 编码工具。当前正式界面采用三模式、两页面架构，V3.5 仅作为显式迁移输入。

## 运行

要求官方 Windows x64 CPython 3.14（兼容 3.13）和 PySide6。

```powershell
python -m pip install -e .
python hjmb_path_editor.py
```

CLI 安装后使用 `hjmb-pathgen --help`。源码运行可用：

```powershell
$env:PYTHONPATH = "src"
python -m hjmb_pathgen.py_app.cli_main --help
```

## 三种模式

- `MANUAL`：用户编辑任意 START/WAYPOINT/ARRIVAL 和全部机械动作。程序不移动点、不搜索几何，但会进行确定性曲线、V4 速度规划、碰撞、四轮约束、动作编译和验证。
- `SEMI_AUTO`：用户编辑 8 个逻辑锚点、yaw、动作和辅助点策略；规划器锁定锚点，只优化锚点间路段并使用 `leg_library.json`。
- `FULL_AUTO`：单条只选择 traj_id；系统生成任务、候选、动作和路段并选总时间最短的可行方案。支持当前 ID 和全部 360，结果默认只读，可显式复制为半自动 Case。

8 个逻辑点为 `P_START`、`P_PICK_1`、`P_PICK_2L`、`P_PICK_2R`、`P_PICK_3`、`P_DROP_1`、`P_DROP_2`、`P_DROP_3`。项目只保存前 5 个公共姿态；5 个物理放货箱保存在 `field_objects.drop_boxes`。

## GUI

界面只有两个核心页面：

1. 路径编辑：模式与 traj_id、场地图元、点表、动作表、路径预览；FULL_AUTO 只读并支持“转为半自动编辑”。
2. 最优路段与批量生成：Case 摘要、leg 表、当前/缺失路段优化、清除重算、验证、当前 ID/360 批量任务、进度和取消。

顶部工具栏统一提供新建/打开 V4 项目、保存项目、打开/保存 Case、生成、验证、导出模式 BIN、设为 final、撤销和重做。编辑操作只标记 `STALE`，不会触发后台规划。

## 运行项目目录

```text
HJMB_Path_Project/
├─ project.json
├─ traj_id.csv
├─ route_case_table.json
├─ leg_library.json
├─ presets/
├─ cases/{manual,semi_auto,full_auto}/
├─ bin/{manual,semi_auto,full_auto,final}/
├─ portable/{manual,semi_auto,full_auto}/
├─ reports/
├─ drafts/
└─ cache/
```

同一 traj_id 的三模式文件可以共存，互不覆盖。旧 `manual_free`、`task_compiled` 和平铺输出只能通过“迁移旧 V4 输出目录”显式迁移；默认 dry-run，有冲突则停止并写报告。

## V3.5 导入

菜单“导入旧 V3.5 工程”只读取旧 JSON，并转换为 `MANUAL` 或 `SEMI_AUTO`；不能转换为 `FULL_AUTO`。可表达的机械动作会迁移，不支持项和不完整的最终放货约束会进入报告/警告。正式入口不加载旧 editor、planner 或 BIN 实现，也不输出 `_v35` 文件。

## 协议不变量

- BIN version 40；Header/Node/Segment/Action 为 104/16/24/22 字节。
- START 和 ARRIVAL 速度为零；最后 DROP_* STOP_AND_WAIT 绑定最终唯一 FINISH_ARM。
- 正式输出不使用 SAFE_END、FINISH_CLEAR 或旧半平面 finish 字段。
- 单条和批量对同一输入必须生成字节一致 BIN；所有正式写入采用验证后原子替换。

## 测试

```powershell
python -m unittest discover -v
```

分层测试位于 `tests/unit` 与 `tests/integration`。本次源码重构不运行 PyInstaller，不生成 exe 或发布压缩包。

更多说明见 [docs/三模式使用说明.md](docs/三模式使用说明.md)、[docs/文件与目录结构.md](docs/文件与目录结构.md) 和 [docs/最终GUI说明.md](docs/最终GUI说明.md)。
