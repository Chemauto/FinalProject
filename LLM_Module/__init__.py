"""
LLM Module - 大语言模型模块 (双层LLM架构)

功能:
- 上层LLM: 任务规划 (用户指令 → 子任务序列)
- 下层LLM: 执行控制 (子任务 → 技能调用)
- 支持多种LLM提供商

核心类:
- LLMAgent: 双层LLM代理类

提示词位置:
- prompts/highlevel_prompt.yaml  (规划提示词)
- prompts/lowlevel_prompt.yaml  (执行提示词)
"""

from .llm_core import LLMAgent

__version__ = '2.0.0'
__all__ = ['LLMAgent']
