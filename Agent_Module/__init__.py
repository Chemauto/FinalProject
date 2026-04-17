"""
Agent_Module — 合并后的智能体模块（原 Interactive_Module + LLM_Module）。

核心类：
- Planner: 高层任务规划（CoT 推理链）
- ParameterCalculator / load_object_facts: object_facts 加载与参数提取
- TaskExecutor: 低层任务执行
- PipelineRunner: 完整 plan→execute 流水线（含重规划）
- AgentRuntime / InteractiveSessionState: TUI 状态管理

提示词位置：
- prompts/highlevel_prompt.yaml  (规划提示词)
- prompts/lowlevel_prompt.yaml  (执行提示词)
"""

from .planner import Planner, LLMAgent, HighLevelPlanner
from .parameter import ParameterCalculator, load_object_facts
from .executor import TaskExecutor, LowLevelExecutor, execute_tool
from .replanner import (
    PipelineRunner,
    AgentRuntime,
    InteractiveSessionState,
    run_agent_turn,
    main,
)

__version__ = "3.0.0"
__all__ = [
    "Planner",
    "LLMAgent",
    "HighLevelPlanner",
    "ParameterCalculator",
    "load_object_facts",
    "TaskExecutor",
    "LowLevelExecutor",
    "execute_tool",
    "PipelineRunner",
    "AgentRuntime",
    "InteractiveSessionState",
    "run_agent_turn",
    "main",
]
