from __future__ import annotations

import importlib
import os


DEFAULT_ACTION_TASK = os.getenv("FINALPROJECT_ACTION_TASK", os.getenv("FINALPROJECT_TASK", "bishe")).strip().lower() or "bishe"

ACTION_TOOL_NAMES = {
    "walk",
    "navigation",
    "nav_climb",
    "climb_align",
    "climb",
    "push_box",
    "way_select",
}

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
    selected = get_current_task(task)
    registry = {}
    for module_path in _TASK_SKILL_MODULES[selected]:
        module = importlib.import_module(module_path)
        register_fn = getattr(module, "register_tool")
        registry.update(register_fn(mcp))
    return registry
