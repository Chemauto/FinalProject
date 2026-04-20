"""Excu_Module schema。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ExecutionFeedback:
    """单步执行反馈。"""

    success: bool
    action: str
    task: str
    task_type: str = ""
    feedback: dict[str, Any] = field(default_factory=dict)
    result: dict[str, Any] = field(default_factory=dict)
    assessment_message: str = ""
