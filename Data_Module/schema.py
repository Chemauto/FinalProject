"""Data_Module 数据层 schema。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class RobotStateSnapshot:
    """机器人状态快照，组合硬件状态 + 视觉感知 + 物体事实。"""

    state: dict[str, Any] = field(default_factory=dict)
    visual_context: dict[str, Any] | None = None
    scene_facts: dict[str, Any] | None = None
    object_facts: dict[str, Any] | None = None
    connected: bool = False
    task_type: str | None = None
