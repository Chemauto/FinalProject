# FinalProject 项目详解

面向交互导航的双层 LLM 系统（当前重点：IsaacLab 行走策略联调，后续迁移 GO2 实机）。本文件目标是让读者只读这一份文档即可理解项目结构、运行流程、使用方式与扩展路径。

## 1. 项目定位与目标

- 以“高层任务规划 + 低层工具调用”的双层 LLM 架构，实现自然语言到机器人动作执行的完整链路。
- 支持最小闭环：任务分解 → 工具选择 → 执行反馈 → 重试/重规划。
- 当前主路径对接 IsaacLab 行走策略（UDP 控制），后续迁移到 GO2 实机接口时保持工具 API 不变。

## 2. 快速开始（IsaacLab 主路径）

### 2.1 启动 IsaacLab（加载行走策略）

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python scripts/rsl_rl/play.py \
  --task Template-Velocity-Go2-Walk-Flat-Ros-v0 \
  --checkpoint /home/xcj/work/IsaacLab/IsaacLabBisShe/ModelBackup/WalkPolicy/WalkFlatNew.pt
```

### 2.2 启动交互端

```bash
cd /home/xcj/FinalProject/Interactive_Module
python interactive.py
```

### 2.3 首次验证（推荐）

优先使用“直连工具模式”验证工具链路：

```text
/tool get_isaac_config
/tool move_isaac {"direction":"left","distance":1.0}
```

当工具调用可正常驱动 IsaacLab 后，再切换自然语言输入：

```text
左移1米
右转90度
```

## 3. 运行前准备

### 3.1 依赖与环境

- Python 环境与依赖：参考 `requirements.txt`。
- ROS2 环境：用于 `ros_topic_comm.py` 提供的命令与状态通信。
- IsaacLab 环境：用于实际行走控制与策略验证。

### 3.2 环境变量

建议在 `.env` 或 shell 中设置：

```bash
Test_API_KEY=your_api_key
LLM_MODEL=qwen3-14b
```

说明：

- `LLM_MODEL` 控制高层/低层文本模型。
- VLM 使用本地 Ollama 模型 `qwen3-vl:4b`（与文本模型无关）。

## 4. 核心流程（输入 → 规划 → 执行 → 反馈）

### 4.1 输入

- `input1`：用户指令文本（未来可替换为 ASR 输入）。
- `input2`：环境状态快照（位置、障碍、环境版本）。
- `input3`：工具列表（由 `Robot_Module/skill.py` 动态注册）。

### 4.2 执行流程

1. `interactive.py` 获取用户输入。
2. `build_env_state_snapshot()` 组装环境快照。
3. `LLMAgent.run_pipeline()` 启动双层执行。
4. `HighLevelLLM` 生成任务序列。
5. `AdaptiveController` 逐步执行任务。
6. `LowLevelLLM` 选择工具与参数并调用。
7. 根据反馈判断是否重试或重规划。
8. 输出 `results`（每步状态、动作、结果、错误）。

### 4.3 输出

- 结构化执行结果列表（`status/action/task/result/error`）。
- 终端日志（规划、进度、重规划原因）。

## 5. 项目结构与职责

### 5.1 目录结构（关键路径）

```text
Interactive_Module/interactive.py
  ├─ 读取用户输入（文本）
  ├─ 采样环境状态（build_env_state_snapshot）
  ├─ /tool 直连工具模式（绕过 LLM）
  └─ 调用 LLMAgent.run_pipeline(...)

LLM_Module/
  ├─ llm_core.py            # 统一入口 LLMAgent
  ├─ high_level_llm.py      # 任务规划 / 重规划生成
  ├─ low_level_llm.py       # 工具选择与参数生成
  ├─ adaptive_controller.py # 执行循环 + 简化评估 + 重规划
  ├─ vlm_core.py            # 图像理解（可选）
  └─ prompts/               # 规划与VLM提示词

Robot_Module/
  ├─ skill.py               # MCP工具注册中心
  └─ module/walkisaacsim.py # IsaacLab UDP控制工具

Sim_Module/sim2d/
  └─ simulator.py           # 2D仿真器（可选）
```

### 5.2 关键模块说明

- `Interactive_Module/interactive.py`
  - CLI 入口，统一处理输入、环境快照、工具列表。
  - 支持两种执行方式：
    - `LLM模式`：自然语言 → 任务规划 → 工具调用。
    - `直连工具模式`：`/tool ...` 直接调用技能。

- `LLM_Module/llm_core.py`
  - 初始化高层/低层/自适应/VLM。
  - 统一暴露 `run_pipeline()` 作为执行入口。

- `LLM_Module/high_level_llm.py`
  - 将用户目标拆分为子任务。
  - 失败时触发重规划，生成新的任务序列。

- `LLM_Module/low_level_llm.py`
  - 将子任务映射为具体工具调用。
  - 负责工具选择与参数生成。

- `LLM_Module/adaptive_controller.py`
  - 驱动任务队列执行。
  - 执行评估（失败率/环境变化）并触发重试或重规划。

- `Robot_Module/skill.py`
  - MCP 工具注册中心。
  - 负责统一输出给 LLM 的可用工具列表。

- `Robot_Module/module/walkisaacsim.py`
  - UDP 控制 IsaacLab 行走策略。
  - 提供 `move_isaac` / `get_isaac_config` 工具。

## 6. 交互方式与模式说明

### 6.1 LLM 模式（推荐）

适合完整链路验证：

```text
左移1米
右转90度
```

### 6.2 直连工具模式（调试优先）

适合在 API 配额不足或需要定位工具问题时使用：

```text
/tool get_isaac_config
/tool move_isaac {"direction":"left","distance":1.0}
```

## 7. IsaacLab 工具说明

### 7.1 move_isaac

用途：统一的移动控制（前后左右移动与旋转）。

参数：

- `direction`：`forward` / `backward` / `left` / `right` / `rotate_left` / `rotate_right`
- `distance`：移动距离（米）或旋转角度（度），必须 > 0
- `speed`：速度（m/s 或 rad/s），不传则使用默认值

示例：

```text
/tool move_isaac {"direction":"forward","distance":1.0}
/tool move_isaac {"direction":"rotate_right","distance":90.0,"speed":0.5}
```

### 7.2 get_isaac_config

用途：返回速度范围、方向、默认配置与运行前置条件。

```text
/tool get_isaac_config
```

## 8. 常见问题与排查

- 403 `AllocationQuota.FreeTierOnly`
  - 说明云端模型配额不足。
  - 解决方案：使用 `/tool ...` 直连工具模式继续联调。

- 工具能调用但机器人无动作
  - 确认 IsaacLab 已启动并加载策略。
  - 确认 UDP 端口与主机配置正确。

- 规划频繁重启或无输出
  - 检查环境快照是否正常生成。
  - 检查 LLM 模型/Key 是否可用。

## 9. 扩展与二次开发

### 9.1 新增技能

1. 新增模块到 `Robot_Module/module/`。
2. 在 `skill.py` 中注册新工具。
3. 更新提示词或任务规则以覆盖新技能。

### 9.2 接入 GO2 实机

- 保持工具 API 不变（`move_isaac` 或同名接口）。
- 替换底层执行逻辑为实机通信接口。

### 9.3 增加场景测试

- 扩展 IsaacLab 场景（如 push_object / climb_step）。
- 使用相同接口进行多场景评测，保持 LLM 侧逻辑不变。

## 10. 参考入口与关键文件索引

- 交互入口：`Interactive_Module/interactive.py`
- LLM 入口：`LLM_Module/llm_core.py`
- 工具注册：`Robot_Module/skill.py`
- IsaacLab 工具：`Robot_Module/module/walkisaacsim.py`
- ROS 通信：`ros_topic_comm.py`
- 2D 仿真器：`Sim_Module/sim2d/simulator.py`

---

如需更深入的调用链说明或自定义扩展（技能、传感器、VLM），可在此文档基础上继续扩展对应章节。
