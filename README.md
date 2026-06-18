# HJMB 空间轨迹编辑器 V3.5

这是搬运豆子机器人 HJMB V3.5 空间轨迹文件的 PySide6 编辑器、规划器和命令行编解码工具。V3.5 与 V3.4 及更早 JSON/BIN 不兼容；旧文件会被明确拒绝，需要按 V3.5 重新配置并重新导出。

## 运行

```bash
pip install PySide6
python hjmb_path_editor.py
```

## V3.5 要点

- 固定 `START` 从静止直接起步，首节点 `vx/vy/wz=0`。
- 编辑点仅保留 `START`、`WAYPOINT`、`ARRIVAL`。
- `ARRIVAL` 必停，最后一个 `ARRIVAL` 自动作为 `END`。
- yaw 锚点只有 `START` 和 `ARRIVAL`，支持 `SHORTEST/CW_ONLY/CCW_ONLY`。
- 支持 `FREE` 与 `FIXED_8` 点位模式。
- 固定 8 点使用路径点前缀：`P_START`、`P_PICK_1`、`P_PICK_2L`、`P_PICK_2R`、`P_PICK_3`、`P_DROP_1`、`P_DROP_2`、`P_DROP_3`。
- 机械动作保留 `STOP_AND_WAIT`、`ASYNC`、`KINEMATIC` 三种模式。
- `ASYNC` 不再配置 trigger，成为 FIFO 队首后立即请求启动。
- `KINEMATIC` 不再配置 window/expire，规划器自动计算 `check_start_s_mm`。
- `min_wait_ms` 已改为 `post_wait_ms`，表示机械模块 `DONE` 后的附加硬等待。
- `ARRIVAL` departure lock 由绑定的 `STOP_AND_WAIT` 动作自动推导，不写入 JSON/BIN 额外表。

## BIN

协议依据为 `HJMB_path_file_protocol_v3.5.txt`：

```text
BeanTrajectoryHeaderV35_t       64 bytes
BeanTrajectoryNodeV35_t         16 bytes
BeanMechanicalActionV35_t       22 bytes
```

Python struct：

```text
Header  <4sBBBBHHHHHHBBHIIIIIIHHHHHHHH
Node    <HhhhhhhBB
Action  <BBBBHHHHHHHHH
```

`Header.version=35`，文件 flags 必含 `FILE_FLAG_AUTO_ACTION_START`。CRC 使用 `zlib.crc32()`，计算时将 Header 偏移 24 的 `file_crc32` 清零。

## CLI

```bash
python path_codec_cli.py plan example_path.json
python path_codec_cli.py summary example_path.json
python path_codec_cli.py build example_path.json P0000.BIN
python path_codec_cli.py check P0000.BIN
```

`plan` 会重新规划、解析动作、计算自动 `check_start_s_mm` 和 departure locks，并回读 BIN 做严格校验。

## 批量

批量工程格式为 `HJMB_PATH_BATCH_JSON_V35`。P0000~P0359 的 route case 映射必须集中维护，至少包含 `traj_id`、取豆顺序、放豆顺序、扫场方向、yaw 旋转策略和动作模板名。有向路段模板使用 `from_site_id -> to_site_id`，正反向不自动等价。

## 测试

```bash
python -m unittest -v
python -m compileall .
```

当前测试覆盖 64/16/22 字节结构、CRC、version=35、旧 JSON/BIN 拒绝、FREE/FIXED_8 解析、START/ARRIVAL 速度边界、arrival_id、yaw 策略、V3.5 动作 mode、自动 `check_start_s_mm`、departure lock、批量 case 和 GUI 表列。
