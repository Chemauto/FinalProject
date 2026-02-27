# FinalProject

面向交互导航的双层 LLM 系统（当前重点：IsaacLab 行走策略联调，后续迁移 GO2 实机）。

## 1. 当前能力

- 支持双层 LLM 流程：高层任务分解 + 低层工具调用
- 支持简化自适应执行：失败重试、重规划触发
- 已接入 IsaacLab 行走工具：`move_isaac` / `get_isaac_config`
- 支持“直连工具模式”绕过 LLM（用于 API 配额不足时调试）

## 2. 项目框架

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

## 3. 输入到输出的数据流

### 3.1 输入

- `input1`：用户指令文本（后续可替换 ASR）
- `input2`：环境状态快照（位置、障碍、环境版本）
- `input3`：技能工具列表（从 `skill.py` 动态注册）

### 3.2 流程

1. `interactive.py` 获取用户输入
2. 组装环境快照 `build_env_state_snapshot()`
3. `LLMAgent.run_pipeline()` 启动执行
4. `HighLevelLLM` 生成任务序列
5. `AdaptiveController` 逐步执行任务
6. `LowLevelLLM` 选择工具（如 `move_isaac`）并调用
7. 根据反馈判断是否重规划
8. 输出 `results`（每步状态、动作、结果、错误）

### 3.3 输出

- 结构化执行结果列表：`status/action/task/result/error`
- 终端日志：规划、进度、重规划原因

## 4. 模块作用

- `Interactive_Module/interactive.py`
  - 命令行入口
  - 支持两种执行方式：
    - `LLM模式`：自然语言 -> 规划 -> 工具调用
    - `直连工具模式`：`/tool ...` 直接调用技能

- `LLM_Module/llm_core.py`
  - 统一初始化高层/低层/自适应/VLM
  - 暴露 `run_pipeline()`

- `LLM_Module/high_level_llm.py`
  - 把用户目标拆成子任务
  - 支持失败后的重规划任务生成

- `LLM_Module/low_level_llm.py`
  - 对子任务做 function calling 决策
  - 选择具体工具与参数

- `LLM_Module/adaptive_controller.py`
  - 任务队列执行
  - 简化评估（失败率/环境变化）
  - 重试与重规划

- `Robot_Module/skill.py`
  - 注册并暴露 MCP 工具给 LLM

- `Robot_Module/module/walkisaacsim.py`
  - 通过 UDP 向 IsaacLab 发送速度命令
  - 承接 `move_isaac/get_isaac_config`

## 5. 环境变量

建议在 `.env` 设置：

```bash
Test_API_KEY=your_api_key
LLM_MODEL=qwen3-14b
```

说明：
- `LLM_MODEL` 控制高层与低层文本模型
- `qwen3-vl:4b` 是 VLM 的本地 Ollama 模型，不同于文本模型

## 6. IsaacLab 联调（当前主路径）

### 6.1 启动 IsaacLab（加载你的行走策略）

```bash
cd /home/xcj/work/IsaacLab/IsaacLabBisShe
python scripts/rsl_rl/play.py \
  --task Template-Velocity-Go2-Walk-Flat-Ros-v0 \
  --checkpoint /home/xcj/work/IsaacLab/IsaacLabBisShe/ModelBackup/WalkPolicy/WalkFlatNew.pt
```

### 6.2 启动交互端

```bash
cd /home/xcj/FinalProject/Interactive_Module
python interactive.py
```

### 6.3 测试方式

- 直连工具（推荐先测）
```text
/tool get_isaac_config
/tool move_isaac {"direction":"left","distance":1.0}
```

- LLM 自然语言
```text
左移1米
右转90度
```

## 7. 已知问题与说明

- 若出现 403 `AllocationQuota.FreeTierOnly`：表示云端模型配额不足，不是代码逻辑错误。
- 配额不足时，优先使用 `/tool ...` 直连工具模式继续联调。
- `environment_version` 已改为“仅环境签名变化时递增”，避免机器人移动引发误重规划。

## 8. 下一步

- 在 IsaacLab 扩展 `push_object / climb_step` 技能
- 完成三类场景评测（Box obstruction/usage/integrated）
- 迁移执行后端到 GO2 实机接口（保持同一工具API）
