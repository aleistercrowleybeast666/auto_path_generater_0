# HJMB 空间轨迹编辑器 V3.3

这是搬运豆子机器人 HJMB V3.3 空间轨迹文件的 PySide6 编辑器、规划器和
命令行编解码工具。V3.3 与 V3.2 及更早版本不兼容，工程不会自动迁移旧
JSON 或 BIN。

## 运行

```bash
pip install PySide6
python hjmb_path_editor.py
```

GUI 保留 4000 x 2000 mm 场地图、双击加点、拖动点、拖动 yaw、表格与画布
双向选择，并新增：

- `CUT_IN`、`WAYPOINT`、`ARRIVAL` 三种编辑点；
- 局部二次贝塞尔圆角与弧长稠密采样；
- CUT_IN/ARRIVAL yaw 锚点与受物理限制的全路径平滑 yaw 规划；
- 速度、合成线加速度、角速度、角加速度互斥分档着色；
- V3.3 20 字节机械动作与 `0xFE` 加速度 Gate；
- `PREP_STORE_1/2/3 + STORE` 放入暂存区；
- `DROP_1/2/3/12/23` 从暂存区放到放豆区域，不提供 `DROP_13/123`；
- 动态切入预览、正式轨迹时间、机械阻塞估算和总时间；
- 规划参数、切入参数、车辆几何和机械持续时间编辑。

程序启动和新建工程时使用空路径，不自动载入示例点。“清空”会在确认后删除
全部编辑点和机械动作，同时保留当前规划参数与车辆参数。

WAYPOINT 不决定 yaw，其 `yaw_ddeg` 固定为 `0xFF`，画布不显示方向线或角度
控制点。CUT_IN 提供起始 yaw，ARRIVAL 提供后续 yaw 锚点；两个锚点之间使用
五次平滑函数分配 yaw，并通过角速度、角加速度和轮速约束自动限制路径速度。

“导入配置 JSON / 导出配置 JSON”只读写当前人工配置，包括三种编辑点、动作、
车辆、切入和规划参数。自动生成的稠密节点、速度曲线和分析结果不会写入 JSON；
每次导入后都会重新规划，便于其他程序批量生成配置后逐条人工校验。

合成线加速度固定按 `0~0.4`、`0.4~0.8`、`0.8~1.2`、`1.2~1.6 m/s²`
和 `>1.6 m/s²` 五个区域显示；角速度固定按 `0~1.5`、`1.5~3.0`、
`3.0~4.5`、`4.5~6.0 rad/s` 和 `>6.0 rad/s` 五个区域显示。

参数或编辑点变化后使用 220 ms 防抖重新规划。规划失败时保留编辑点并清除旧
轨迹着色。

## 模块

- `path_models.py`：V3.3 JSON、编辑点、动作、车辆和规划结果模型；
- `path_geometry.py`：局部二次贝塞尔、弧长重采样、切线和曲率；
- `trajectory_planner.py`：yaw、物理速度、麦轮约束、切入与动作规划；
- `trajectory_graphics.py`：分档、颜色、图例和 hover 文本；
- `path_codec_cli.py`：V3.3 BIN 编解码、CRC、严格校验和 CLI；
- `hjmb_path_editor.py`：PySide6 主窗口与交互。

## 几何与速度

普通 `WAYPOINT` 使用局部二次贝塞尔圆角：

```text
d = min(corner_trim_mm, 0.45*|AB|, 0.45*|BC|)
Q(t) = (1-t)^2*P_in + 2*(1-t)*t*B + t^2*P_out
```

`CUT_IN`、`ARRIVAL` 和 `exact_pass=true` 的 `WAYPOINT` 会成为真实导出节点。
曲线先自适应离散，再按弧长重采样；任意节点间距不超过 50 mm，到达点附近
进一步细化。

规划器分别计算：

```text
a_n     = v^2 * |kappa|
a_total = sqrt(a_t^2 + a_n^2)
beta    = q*a_t + q'*v^2
wz      = q*v
```

正向和反向扫描在合成平移加速度、横向加速度、角速度、角加速度和麦轮软限幅
的交集中选择尽可能大的切向加速度。首节点速度固定为 `cut_in.target_speed_mmps`，
最后和所有 STOP 节点速度为 0。

麦轮逆解集中在 `trajectory_planner.mecanum_wheel_rpm()`。正常规划使用
`wheel_plan_limit_rpm`，`wheel_hard_limit_rpm` 仅作为固件运行保护值。

## V3.3 BIN

协议依据为 `HJMB_path_file_protocol_v3.3.txt`：

```text
BeanTrajectoryHeaderV33_t       64 bytes
BeanTrajectoryNodeV33_t         16 bytes
BeanMechanicalActionV33_t       20 bytes
```

Python struct：

```text
Header  <4sBBBBHHHHHHBBHIIIIIIHHHHHHHH
Node    <HhhhhhhBB
Action  <BBBBHHHHHHHH
```

CRC 使用 `zlib.crc32()`；计算时将 Header 偏移 24 的 `file_crc32` 清零。BIN
保存规划后的稠密空间节点，不保存稀疏编辑点。

## CLI

```bash
python path_codec_cli.py plan example_path.json
python path_codec_cli.py summary example_path.json
python path_codec_cli.py build example_path.json P0000.BIN
python path_codec_cli.py check P0000.BIN
```

`build` 会重新规划、严格校验、写入 BIN 并回读；`check` 校验结构、offset、
reserved、Gate、动作和 CRC。

## 时间估算

- 正式轨迹：`dt = 2*ds/(v_i+v_{i+1})`，写入 `Header.planned_time_ms`；
- 动态切入：使用非零终端速度梯形/三角形规划，仅用于预览；
- 机械时间：累加 `HOLD_PATH` 和未被覆盖的 `REQUIRED_AT_END` 动作持续时间；
- 总预计：切入预览 + 正式轨迹 + 机械阻塞估算。

切入预览和机械估算不会写入 `planned_time_ms`。

## 车辆参数

`example_path.json` 中的轮半径 `76 mm`、旋转半径 `260 mm` 是可运行的示例
初值，不代表实车已确认数据。导出实车文件前必须按 STM32 麦轮逆解核对：

- `wheel_radius_mm`
- `rotation_radius_mm`
- `mecanum_convention`
- `wheel_plan_limit_rpm`
- `wheel_hard_limit_rpm`

零值或非法几何参数会阻止规划和导出。

## 测试

```bash
python -m unittest -v
python -m compileall .
```

测试覆盖 64/16/20 字节结构、CRC、offset/reserved、JSON 到 BIN 往返、Gate 和
动作规则、局部贝塞尔、精确点、曲率、合成加速度、`beta`、麦轮软限幅、分档和
非零终端速度切入预览。
