"""Planner_Module schema。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class TaskIntent:
    """规划器输出的任务意图。"""

    step: int
    task: str
    type: str = "未分类"
    function: str = "待LLM决定"
    reason: str = "未提供规划依据"
    parameter_context: dict[str, Any] = field(default_factory=dict)
    calculated_parameters: dict[str, Any] = field(default_factory=dict)
