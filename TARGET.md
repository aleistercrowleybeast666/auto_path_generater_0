# V4.0 三模式最终重构目标

当前交付目标是一次完成以下 8 个可验收部分：

1. **源码结构**：根目录仅薄 GUI 入口；正式代码全部进入 `py_app/py_ui/py_domain/py_planning/py_services/py_io/py_workers/py_utils`，V3.5 reader/model 仅在 `py_legacy`。
2. **三模式模型**：正式 Case 统一使用 `generation_mode=MANUAL|SEMI_AUTO|FULL_AUTO`，旧 `path_source` 值只允许迁移器读取。
3. **8 点与物理对象**：项目 5 个公共取货侧姿态；Case 8 个逻辑点；场地保持 2 圆柱、3 取货箱、5 放货箱。
4. **规划行为**：MANUAL 不搜索几何；SEMI_AUTO 锁定锚点并优化 leg；FULL_AUTO 支持当前 ID 与全部 360；编辑不自动规划。
5. **GUI**：只有“路径编辑”“最优路段与批量生成”两页；工具栏统一 V4 JSON/BIN/final；全自动只读并可显式转半自动。
6. **输出与迁移**：三模式独立目录；同 traj_id 可共存；旧平铺和旧两模式目录显式、无覆盖、原子迁移；V3.5 动作可表达部分必须保留并报告不支持项。
7. **协议与安全**：BIN packed 格式不变；连续碰撞、轮速、动作 FIFO、最终放货完赛、审批和 STALE 守卫保持有效。
8. **验收**：目录/import、三模式、8 点、GUI、输出、规划、worker、codec、collision、optimizer、batch 和 final-drop 回归全部通过；不执行打包。

## 完成定义

- 正式入口不加载 `py_legacy`，不存在重复正式实现或 V3.5 输出按钮。
- MANUAL、SEMI_AUTO、FULL_AUTO 各自可保存 Case、生成 BIN/portable，并能从任一合格模式显式发布 final。
- 清除 leg 只修改路段库并将依赖 Case 标记 `STALE`，不自动重算。
- 协议结构断言保持 `40/104/16/24/22`。
- README 和 `docs/` 对用户流程、目录、迁移和 GUI 的描述与实现一致。
