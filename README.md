# HJMB 路径编辑器 V2.5

这是用于搬运豆子机器人路径文件生成的 PySide6 图形化工具。V2.5 将路径点和机械
动作 FIFO 分离，并使用 Gate 控制 PICK、DROP 等关键动作的解锁和底盘等待。

## 功能

- 保留 4000 x 2000 mm 场地、坐标系、双击加点、拖动点和拖动 yaw；
- “路径点”页编辑坐标、点类型、`gate_id`、`marker_id` 和点 flags；
- “机械动作”页独立编辑动作 FIFO、解锁 Gate、动作 flags 和超时；
- Gate 在画布上显示为 `G0`、`G1` 等，并支持按路径顺序重编号；
- 保存/打开 V2.5 JSON，导出并严格校验 V2.5 BIN；
- 显式导入 V2.0 JSON/BIN，转换后要求另存，避免覆盖原文件；
- CLI 不依赖 PySide6，可用于自动生成和校验 BIN。

## 运行

```bash
pip install PySide6
python hjmb_path_editor.py
```

## V2.5 模型

路径点中的 V2.0 `action` 字节已改为 `gate_id`。机械动作保存在独立 FIFO 中：

- `unlock_gate_id = 0xFF`：动作到达队首后立即执行；
- `ACTION_FLAG_LOCKED`：等待指定 Gate；
- `ACTION_FLAG_HOLD_PATH`：Gate 到达后等待该动作完成；
- `ACTION_FLAG_REQUIRED_AT_END`：路径结束前必须确认动作完成。

PICK 和 DROP 必须设置 `LOCKED|HOLD_PATH`，并引用位于 `ARRIVE_SCAN` 点的 Gate。
Gate 必须唯一，并按路径顺序连续编号为 `0, 1, 2, ...`。

## BIN 格式

协议详见 `HJMB_path_file_protocol_v2.5.txt`：

```text
BeanPathHeaderV25_t       32 bytes
BeanPathPointV25_t[]      12 bytes each
BeanMechanicalAction_t[]   8 bytes each
```

- `version = 25`
- Header：`<4sBBBBHHIBBBB3I`
- Point：`<hhhBBBBBB`
- Action：`<BBBBHH`
- 文件大小：`32 + point_count*12 + action_count*8`
- CRC32 使用 `zlib.crc32()`，计算时将偏移 12 的 CRC 字段置 0。

JSON 顶层格式为：

```json
{
  "format": "HJMB_PATH_EDITOR_JSON_V25",
  "traj_id": 0,
  "points": [],
  "actions": []
}
```

## 命令行

```bash
python path_codec_cli.py build example_path.json P0000.BIN
python path_codec_cli.py check P0000.BIN
```

`check` 成功时输出 `traj_id`、`point_count`、`action_count` 和 `gate_count`。

## V2.0 迁移

打开 V2.0 JSON/BIN 时，工具会提取原路径点上的动作并生成 V2.5 FIFO。PICK、
DROP 及原 `WAIT_ACTION` 动作会创建 Gate 并设置 `LOCKED|HOLD_PATH`；原 bit0
会从路径点 flags 中清除。转换结果不会绑定原 JSON 路径，必须另存为 V2.5 文件。

## 测试

```bash
python -m unittest -v
```

测试覆盖 V2.5 往返、CRC/长度错误、Gate 和动作规则，以及 V2 JSON/BIN 迁移。
