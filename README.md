# FinalProject

一个面向“语音指令 + 环境感知 + 技能库”的双层 LLM 机器人任务系统。
当前版本优先支持仿真闭环，并为后续迁移到 Unitree GO2 预留接口。

## 1. 项目目标

- 接收用户指令（可来自 ASR）
- 融合环境感知（相机/雷达/机器人状态，可接 VLM 描述）
- 基于技能库自动规划并执行任务
- 在执行失败或环境变化时进行简化重规划

## 2. 当前框架（精简版）

```text
User/ASR
  -> Interactive_Module/interactive.py
  -> LLM_Module/llm_core.py (LLMAgent)
      -> high_level_llm.py      # 任务分解/重规划生成
      -> low_level_llm.py       # 单步工具选择与参数生成
      -> adaptive_controller.py # 执行闭环 + 评价 + 重规划
      -> vlm_core.py            # 视觉理解（可选）
  -> Robot_Module/skill.py
      -> Robot_Module/module/*  # 原子技能实现
  -> Sim_Module/sim2d/simulator.py (当前默认执行环境)
```

## 3. 输入到输出的数据流

### 3.1 输入

- `input1`（指令输入）
  - 来源：控制台文本（后续可替换 ASR）
  - 示例：`"去目标点并避开障碍"`

- `input2`（环境输入）
  - 来源：`build_env_state_snapshot()`
  - 典型字段：
    - `position`: 机器人位姿
    - `environment_version`: 环境版本（用于判断变化）
    - `camera.objects` / `radar.obstacles`

- `input3`（技能输入）
  - 来源：`Robot_Module/skill.py` 动态注册的工具定义
  - 内容：技能名、参数 schema、描述

### 3.2 处理流程

1. `interactive.py` 收集用户指令与环境快照
2. `LLMAgent.run_pipeline()` 整合输入并启动流程
3. `HighLevelLLM.plan_tasks()` 生成任务序列（可参考 VLM 描述）
4. `AdaptiveController` 逐步调度任务执行
5. `LowLevelLLM.execute_task()` 选择工具并调用具体技能
6. 执行反馈进入评价器（失败率/卡住/环境变化）
7. 若需要重规划：`HighLevelLLM.replan_tasks()` 生成新任务并插入
8. 返回执行结果列表（成功/失败/重规划后的结果）

### 3.3 输出

- 主输出：任务执行结果 `results`
  - 每步包含：`status/action/task/result/error`
- 日志输出：
  - 规划结果
  - 当前步骤进度
  - 是否触发重规划与原因

## 4. 核心模块说明

### 4.1 LLM_Module

- `llm_core.py`
  - 外部统一入口 `LLMAgent`
  - 负责初始化并串联高层、低层、自适应控制器和 VLM

- `high_level_llm.py`
  - 将用户目标分解为可执行子任务序列
  - 当执行异常时，根据失败原因和环境上下文重新规划

- `low_level_llm.py`
  - 对当前子任务进行工具决策（function calling）
  - 生成技能参数并调用执行函数

- `adaptive_controller.py`
  - 维护执行循环与任务队列
  - 基于执行反馈做简化评估
  - 触发局部/全局重规划并继续执行

- `vlm_core.py`
  - 可选视觉理解模块
  - 支持本地 Ollama 或兼容 OpenAI API 的图像理解

- `prompts/`
  - 存放规划与视觉理解提示词模板

### 4.2 Interactive_Module

- `interactive.py`
  - 命令行入口
  - 注册技能、读取用户输入、提取图片路径、组装环境状态
  - 调用 `LLMAgent.run_pipeline()` 执行完整流程

### 4.3 Robot_Module

- `skill.py`
  - 技能注册中心（MCP 工具导出）
  - 提供工具定义给 LLM 进行调用

- `module/`
  - 具体原子技能实现（移动、追击、视觉等）
  - 当前以仿真链路为主，后续可替换为 GO2 实机控制实现

### 4.4 Sim_Module

- `sim2d/simulator.py`
  - 当前默认仿真环境
  - 承接动作命令并反馈机器人状态

### 4.5 ros_topic_comm.py

- 提供模块间状态与动作通信封装
- 当前承担仿真通信，也可作为后续实机通信适配层入口

## 5. 运行方式（当前）

```bash
pip install -r requirements.txt
export Test_API_KEY=your_key

# 终端1：启动仿真器
python3 Sim_Module/sim2d/simulator.py

# 终端2：启动交互
python3 Interactive_Module/interactive.py
```

## 6. 当前状态与下一步

- 当前可用：仿真端双层 LLM + 简化重规划闭环
- 正在推进：按 `TODO.md` 完成仿真评估后迁移 GO2

建议阅读顺序：
1. `TODO.md`
2. `Interactive_Module/interactive.py`
3. `LLM_Module/llm_core.py`
4. `LLM_Module/adaptive_controller.py`
5. `Robot_Module/skill.py`
