"""
LLM Module - 精简版双层LLM模块

核心能力:
- 上层LLM: 任务规划 (用户指令 → 子任务序列)
- 下层LLM: 执行控制 (子任务 → 技能调用)
- 自适应重规划闭环
- VLM环境理解（可选）

导出类:
- HighLevelLLM: 高层LLM任务规划器
- LowLevelLLM: 低层LLM执行控制器
- AdaptiveController: 自适应控制器
- VLMCore: 视觉理解模块
- LLMAgent: 双层LLM代理类 (内部使用新架构)

提示词位置:
- prompts/planning_prompt_*.yaml
- prompts/vlm_perception.yaml
"""

# 新的模块化架构
from .high_level_llm import HighLevelLLM
from .low_level_llm import LowLevelLLM, ExecutionStatus
from .adaptive_controller import AdaptiveController, ReplanLevel
from .vlm_core import VLMCore

# 向后兼容：旧的LLMAgent类（内部使用新的模块化架构）
from .llm_core import LLMAgent

__version__ = '3.0.0'
__all__ = [
    # 新架构
    'HighLevelLLM',
    'LowLevelLLM',
    'ExecutionStatus',
    'AdaptiveController',
    'ReplanLevel',
    'VLMCore',

    # 向后兼容
    'LLMAgent',
]
