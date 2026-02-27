# LLM_Module（精简核心版）

本目录只保留任务规划、执行控制、重规划和 VLM 感知的核心文件。

## 目录结构

```text
LLM_Module/
  __init__.py
  llm_core.py
  high_level_llm.py
  low_level_llm.py
  adaptive_controller.py
  vlm_core.py
  prompts/
    planning_prompt_2d.yaml
    vlm_perception.yaml
```

## 每个文件作用

- `llm_core.py`
  - 对外主入口 `LLMAgent`
  - 负责初始化高层/低层/VLM/自适应控制器
  - 提供 `run_pipeline(...)` 一键流程

- `high_level_llm.py`
  - 读取用户指令（可融合 VLM 环境理解）
  - 生成任务序列（tasks）
  - 失败后生成重规划任务（replan_tasks）

- `low_level_llm.py`
  - 对“当前单步任务”选择工具和参数
  - 调用 `execute_tool_fn`
  - 返回执行状态（success/failed/requires_replanning）

- `adaptive_controller.py`
  - 执行闭环（任务队列 + 反馈评估 + 自动重规划）
  - 维护简化评价器：失败率、环境版本变化、卡住检测
  - 决策重规划级别并触发 `high_level_llm.replan_tasks`

- `vlm_core.py`
  - 视觉理解模块
  - 支持本地 Ollama 或兼容 OpenAI API 的 VLM 推理

- `prompts/planning_prompt_2d.yaml`
  - 高层任务分解提示词模板

- `prompts/vlm_perception.yaml`
  - VLM 环境描述提示词模板

## 最小运行链路

1. `interactive.py` 构造工具列表与用户输入  
2. 调用 `LLMAgent.run_pipeline(...)`  
3. `HighLevelLLM` 生成任务  
4. `AdaptiveController` 驱动执行与反馈评估  
5. `LowLevelLLM` 逐步调用机器人技能  
6. 失败或环境变化时自动重规划

## 快速使用

```python
from LLM_Module.llm_core import LLMAgent

agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml",
    enable_vlm=True,
    enable_adaptive=True,
)
```

