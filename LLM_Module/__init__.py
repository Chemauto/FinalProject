"""
LLM Module - 大语言模型模块 (双层LLM架构)

功能:
- 上层LLM: 任务规划 (用户指令 → 子任务序列)
- 下层LLM: 执行控制 (子任务 → 技能调用)
- 支持多种LLM提供商
- 自适应重新规划

核心类 (新架构):
- HighLevelLLM: 高层LLM任务规划器
- LowLevelLLM: 低层LLM执行控制器
- TaskQueue: 任务队列管理
- ExecutionMonitor: 执行监控器
- AdaptiveController: 自适应控制器

核心类 (向后兼容):
- LLMAgent: 双层LLM代理类 (内部使用新架构)

提示词位置:
- prompts/planning_prompt_*.yaml  (规划提示词)
- prompts/execution_prompt_*.yaml  (执行提示词)
"""

# 新的模块化架构
from .high_level_llm import HighLevelLLM
from .low_level_llm import LowLevelLLM, ExecutionStatus
from .task_queue import TaskQueue, Task, TaskStatus
from .execution_monitor import ExecutionMonitor, Anomaly, AnomalyType
from .adaptive_controller import AdaptiveController, ReplanLevel

# 向后兼容：旧的LLMAgent类（内部使用新的模块化架构）
from .llm_core import LLMAgent

__version__ = '2.0.0'
__all__ = [
    # 新架构
    'HighLevelLLM',
    'LowLevelLLM',
    'ExecutionStatus',
    'TaskQueue',
    'Task',
    'TaskStatus',
    'ExecutionMonitor',
    'Anomaly',
    'AnomalyType',
    'AdaptiveController',
    'ReplanLevel',

    # 向后兼容
    'LLMAgent',
]
