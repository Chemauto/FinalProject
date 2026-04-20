"""任务分发注册表。"""

from __future__ import annotations

import importlib
import os

_TASK_REGISTRY: dict[str, list[str]] = {
    "bishe": [
        "Robot_Module.tasks.bishe.walk",
        "Robot_Module.tasks.bishe.navigation",
        "Robot_Module.tasks.bishe.nav_climb",
        "Robot_Module.tasks.bishe.climb_align",
        "Robot_Module.tasks.bishe.climb",
        "Robot_Module.tasks.bishe.push_box",
        "Robot_Module.tasks.bishe.way_select",
    ],
}

DEFAULT_TASK = os.getenv("FINALPROJECT_ACTION_TASK", os.getenv("FINALPROJECT_TASK", "bishe")).strip().lower() or "bishe"


def register_tools(mcp, *, task: str | None = None):
    """加载技能模块并注册到 MCP。"""
    from Excu_Module.skill_registry import register_navigation_model_uses

    register_navigation_model_uses({4, 5})

    selected = str(task or DEFAULT_TASK).strip().lower()
    if selected not in _TASK_REGISTRY:
        available = ", ".join(sorted(_TASK_REGISTRY))
        raise ValueError(f"未知 Action task: {selected}，可选: {available}")

    registry = {}
    for module_path in _TASK_REGISTRY[selected]:
        module = importlib.import_module(module_path)
        register_fn = getattr(module, "register_tools")
        registry.update(register_fn(mcp))

    return registry
