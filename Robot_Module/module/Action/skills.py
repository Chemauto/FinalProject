from __future__ import annotations

import importlib
import os
from pathlib import Path


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


def _register_bishe_hooks() -> None:
    """Register Bishe-specific hooks (context, rule planner, nav codes, prompts)."""
    from Excu_Module.skill_registry import (
        register_context_hook,
        register_rule_planner,
        register_lowlevel_prompt,
        register_highlevel_prompt,
        register_navigation_model_uses,
    )
    from Robot_Module.module.Action.Task.Bishe._bishe_helpers import (
        build_bishe_context,
        NAVIGATION_MODEL_USES,
    )
    from Robot_Module.module.Action.Task.Bishe.bishe_planner import plan_box_assist_navigation

    bishe_dir = Path(__file__).parent / "Task" / "Bishe"

    register_context_hook(build_bishe_context)
    register_rule_planner(plan_box_assist_navigation)
    register_navigation_model_uses(NAVIGATION_MODEL_USES)
    register_lowlevel_prompt(bishe_dir / "lowlevel_prompt.yaml")
    register_highlevel_prompt(bishe_dir / "highlevel_prompt.yaml")


def register_tools(mcp, *, task: str | None = None):
    from Excu_Module.skill_registry import register_skill

    selected = get_current_task(task)

    # Register Bishe-specific hooks before loading skill modules
    if selected == "bishe":
        _register_bishe_hooks()

    registry = {}
    for module_path in _TASK_SKILL_MODULES[selected]:
        module = importlib.import_module(module_path)

        # Prefer SkillBase-based registration
        skill_instance = getattr(module, "_skill", None)
        if skill_instance is not None:
            register_skill(skill_instance)
            registry.update(skill_instance.register_tool(mcp))
        else:
            # Legacy fallback
            register_fn = getattr(module, "register_tool")
            registry.update(register_fn(mcp))
    return registry
