"""Action 技能注册入口。

根据 FINALPROJECT_ACTION_TASK 环境变量加载对应任务目录下的技能模块。
每个技能模块只需实现 register_tools(mcp) 函数。
"""

from __future__ import annotations

import importlib
import os

DEFAULT_ACTION_TASK = os.getenv("FINALPROJECT_ACTION_TASK", os.getenv("FINALPROJECT_TASK", "bishe")).strip().lower() or "bishe"

ACTION_TOOL_NAMES: set[str] = set()

_TASK_SKILL_MODULES = {
    "bishe": [
        "Robot_Module.module.Action.Task.Bishe.walk",
        "Robot_Module.module.Action.Task.Bishe.navigation",
        "Robot_Module.module.Action.Task.Bishe.nav_climb",
        "Robot_Module.module.Action.Task.Bishe.climb_align",
        "Robot_Module.module.Action.Task.Bishe.climb",
        "Robot_Module.module.Action.Task.Bishe.push_box",
        "Robot_Module.module.Action.Task.Bishe.way_select",
    ],
}


def get_current_task(task: str | None = None) -> str:
    selected = str(task or DEFAULT_ACTION_TASK).strip().lower()
    if selected not in _TASK_SKILL_MODULES:
        available = ", ".join(sorted(_TASK_SKILL_MODULES))
        raise ValueError(f"未知 Action task: {selected}，可选: {available}")
    return selected


def register_tools(mcp, *, task: str | None = None):
    """加载技能模块并注册到 MCP。每个模块暴露 register_tools(mcp) -> dict。"""
    from Excu_Module.skill_registry import register_navigation_model_uses

    # 注册导航 model_use 码：4=navigation, 5=nav_climb
    register_navigation_model_uses({4, 5})

    selected = get_current_task(task)
    registry = {}
    for module_path in _TASK_SKILL_MODULES[selected]:
        module = importlib.import_module(module_path)
        register_fn = getattr(module, "register_tools")
        registry.update(register_fn(mcp))

    ACTION_TOOL_NAMES.update(registry.keys())
    return registry
