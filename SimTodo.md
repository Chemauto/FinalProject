# SimTodo - IsaacLab 到 GO2 迁移清单（当前版本）

## A. 当前状态（已完成）

- [x] IsaacLab 行走策略 checkpoint 已就绪：`WalkFlatNew.pt`
- [x] `walkisaacsim` 工具已注册：`move_isaac` / `get_isaac_config`
- [x] 交互端支持 `LLM_MODEL` 环境变量（可切 14b）
- [x] 新增 `/tool` 直连模式（配额不足仍可联调）
- [x] 修复环境版本误触发（移动后不再无意义重规划）

---

## B. 近期目标（仿真先收敛）

- [x] 让 LLM 在 IsaacLab 场景下稳定调用 `move_isaac`
- [x] 完成最小场景评测：前进/侧移/旋转组合动作
- [x] 加入可复现日志与结果表（成功率、时长、失败原因）

完成说明：
- 已完成 `/tool` 与自然语言双路径联调
- 已验证左移/前进/旋转动作可执行
- 已观察并修复环境版本误触发重规划问题

---

## C. 阶段1：IsaacLab 行走闭环（本周）

### C1. 运行环境
- [x] 启动 IsaacLab 任务 `Template-Velocity-Go2-Walk-Flat-Ros-v0`
- [x] 确认 UDP 端口 `5555` 正常收命令
- [x] 确认 `/tool get_isaac_config` 返回正常

验收标准：
- 直连模式连续执行 10 次不报错

### C2. 动作联调
- [x] `/tool move_isaac {"direction":"forward","distance":1.0}`
- [x] `/tool move_isaac {"direction":"left","distance":1.0}`
- [x] `/tool move_isaac {"direction":"rotate_right","distance":90}`
- [x] 组合动作（前进+转向+侧移）

验收标准：
- 动作方向、距离/角度与预期一致

### C3. LLM 调度联调
- [x] 自然语言触发：`左移1米` / `前进1米` / `右转90度`
- [x] 观察低层是否稳定选择 `move_isaac`
- [x] 避免无效重规划（环境版本不应每步变化）

验收标准：
- 5 条简单指令中，工具选择正确率 >= 90%

---

## D. 阶段2：扩展 RL 技能

### D1. 技能定义
- [ ] `navigate_to_pose`
- [ ] `push_object`
- [ ] `climb_step`
- [ ] `stop`

### D2. 训练与导出
- [ ] 在 IsaacLab 为每个技能定义 obs/action/reward/done
- [ ] 训练并导出 checkpoint
- [ ] 固化推理输入输出协议

验收标准：
- 每个技能离线测试 SR >= 80%（可按实际调整）

---

## E. 阶段3：任务级仿真评测（对齐论文思路）

- [ ] Box obstruction（10次）
- [ ] Box usage（10次）
- [ ] Integrated（10次）

记录指标：
- [ ] SR（成功率）
- [ ] OT（总耗时）
- [ ] PT（规划耗时）
- [ ] 重规划次数与触发原因

产出：
- [ ] `results/sim_eval.md`

---

## F. 阶段4：GO2 实机迁移

### F1. 执行层迁移
- [ ] 将 `move_isaac` 后端替换为 GO2 控制接口（API 名称保持不变）
- [ ] 增加速度/角速度限幅
- [ ] 增加通信超时自动 `stop`

### F2. 感知状态迁移
- [ ] 将 `build_env_state_snapshot()` 数据源切换到实机传感器
- [ ] 保持字段兼容：`position/environment_version/obstacles`

### F3. 安全测试
- [ ] 急停测试
- [ ] 异常断连测试
- [ ] 低速模式首测

验收标准：
- 至少完成 1 个端到端任务并可重复

---

## G. 当下优先级（按顺序）

1. [ ] 固化 IsaacLab 启动与 `/tool` 回归脚本
2. [ ] 跑 20 条基础动作指令，形成动作准确率记录
3. [ ] 跑 10 条自然语言动作指令，记录工具选择正确率
4. [ ] 进入 `push_object` 技能训练与接入
