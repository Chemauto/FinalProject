# FinalProject - 机器人多输入自适应规划与执行系统

## 1. 目标（按你当前需求定义）

系统最终要支持三类输入并融合决策：

- `input1`：语音识别结果（ASR 转文本）
- `input2`：相机/雷达等传感器数据 + VLM 环境描述（含结构化 JSON）
- `input3`：机器人原子技能库（如 `walk`/`climb`/`push`，可直接调用）

LLM 基于 `input1 + input2 + input3` 生成：

- 面向任务的 `SkillTree`（技能树）
- 当前可执行的 `Skeleton`（短时执行骨架）

并通过机器人状态反馈与环境反馈进行自适应重规划。

---

## 2. 当前代码状态（简述）

项目已经具备以下基础：

- 双层 LLM：高层规划 + 低层执行（`LLM_Module/high_level_llm.py`, `LLM_Module/low_level_llm.py`）
- 自适应控制框架：`LLM_Module/adaptive_controller.py`
- 环境监控器与异常类型：`LLM_Module/execution_monitor.py`
- 任务队列：`LLM_Module/task_queue.py`
- 技能注册中心：`Robot_Module/skill.py`

当前缺口：

- 未形成明确的“高频执行环 / 低频规划环”双循环调度
- `env_state` 尚未形成稳定实时输入管线（多传感器融合不足）
- 缺少统一数据契约（`input1/input2/input3/ExecutionTrace/ReplanDecision`）

---

## 3. 双频循环目标架构

### A) 执行环（高频，10-30Hz，尽量不调用 LLM）

目标：稳定执行 `Skeleton[next]`

- 输入：`Skeleton[next] + RobotState + 最新局部环境状态`
- 输出：`SkillResult`，追加到 `ExecutionTrace`

特点：

- 仅做技能调用与轻量规则判断
- 不做长上下文推理
- 保障实时性和控制稳定性

### B) 规划/重规划环（低频，1-2Hz 或事件触发）

目标：判断是否重规划，以及重规划粒度

- 输入：`Observation + ExecutionTrace + SkillTree`
- 输出：`ReplanDecision`

`ReplanDecision` 可选：

- 保持当前骨架继续执行
- 局部换分支（替换某个子树）
- 全量重建 `SkillTree`

---

## 4. 统一数据契约（建议 JSON）

### 4.1 input1（语音文本）

```json
{
  "text": "去二楼把红色箱子推到门口",
  "lang": "zh",
  "confidence": 0.93,
  "timestamp": 1700000000.12
}
```

### 4.2 input2（多传感器 + VLM）

```json
{
  "sensor_frame": {
    "camera": {"objects": [{"id": "box_1", "label": "red_box", "x": 1.2, "y": 0.8}]},
    "radar": {"obstacles": [{"x": 2.1, "y": 0.4, "r": 0.3}]},
    "robot_state": {"pose": {"x": 0.0, "y": 0.0, "yaw": 0.1}, "battery": 0.78}
  },
  "vlm_summary": "前方2米有障碍，左侧可绕行，红色箱子在右前方",
  "environment_version": 42,
  "timestamp": 1700000000.23
}
```

### 4.3 input3（技能注册表）

```json
{
  "skills": [
    {"name": "walk", "args_schema": {"target": "pose"}},
    {"name": "climb", "args_schema": {"stairs_id": "string"}},
    {"name": "push", "args_schema": {"object_id": "string", "direction": "vector"}}
  ]
}
```

### 4.4 SkillTree / Skeleton / ExecutionTrace / ReplanDecision

```json
{
  "skill_tree": {
    "goal": "deliver_box",
    "children": [
      {"id": "n1", "skill": "walk", "status": "pending"},
      {"id": "n2", "skill": "push", "status": "pending"}
    ]
  },
  "skeleton": [{"node_id": "n1"}, {"node_id": "n2"}],
  "execution_trace": [
    {"node_id": "n1", "status": "success", "latency_ms": 120, "state_delta": {"x": 0.6, "y": 0.0}}
  ],
  "replan_decision": {
    "need_replan": true,
    "scope": "branch",
    "reason": "new_obstacle",
    "replace_node_ids": ["n2"]
  }
}
```

---

## 5. 代码修改计划（先做这个）

> 目标：最少改动先跑通，再迭代增强。

### Phase 1：定义接口与状态总线

- 新增统一数据结构（可先放在 `LLM_Module/contracts.py`）
- 明确 `input1/input2/input3` 的字段与校验
- 将 `ExecutionTrace`、`ReplanDecision` 结构固定

涉及文件：

- `LLM_Module/`（新增 `contracts.py`）
- `Interactive_Module/interactive.py`（接入输入封装）

### Phase 2：拆分双频循环

- 在 `AdaptiveController` 中拆分：
  - 高频执行环（技能调度）
  - 低频规划环（评估与重规划）
- 将当前串行执行改为“执行线程/协程 + 规划线程/协程”

涉及文件：

- `LLM_Module/adaptive_controller.py`
- `LLM_Module/task_queue.py`（补充 skeleton 游标与并发安全）

### Phase 3：接入评价器（是否重规划）

- 新增 `Evaluator`（可放 `LLM_Module/evaluator.py`）
- 统一评估信号：
  - 技能失败率
  - 卡住/振荡
  - 环境版本变化
  - 目标可达性变化
- 输出标准化 `ReplanDecision`

涉及文件：

- `LLM_Module/evaluator.py`（新增）
- `LLM_Module/execution_monitor.py`（输出喂给 evaluator）

### Phase 4：多输入融合

- `input1`：预留 ASR adapter（本地/远程皆可）
- `input2`：传感器聚合器 + VLM 文本摘要拼接
- `input3`：从 `Robot_Module/skill.py` 动态拉取技能 schema

涉及文件：

- `Interactive_Module/interactive.py`
- `LLM_Module/high_level_llm.py`
- `Robot_Module/skill.py`

### Phase 5：最小验证

- 增加两类回归测试：
  - 正常执行不重规划
  - 障碍突发触发 branch replan

---

## 6. 建议的最小目录增量

```text
LLM_Module/
  contracts.py      # 输入/输出数据契约
  evaluator.py      # 重规划评价器
  adaptive_controller.py  # 双频循环调度（改造）
```

---

## 7. 运行方式（当前）

```bash
# 安装依赖
pip install -r requirements.txt

# 设置 API KEY
export Test_API_KEY=your_key

# 终端1：启动仿真器
python3 Sim_Module/sim2d/simulator.py

# 终端2：启动交互
python3 Interactive_Module/interactive.py
```

---

## 8. 下一步实施原则

- 先固定数据契约，再做循环拆分
- 高频环不调用 LLM，只做执行与轻量规则
- 低频环只做评估与重规划，避免抖动
- 每次重规划都写入可追踪原因（可观测、可回放）

