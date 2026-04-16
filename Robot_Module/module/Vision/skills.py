from __future__ import annotations

import importlib
import os


DEFAULT_VISION_TASK = os.getenv("FINALPROJECT_VISION_TASK", os.getenv("FINALPROJECT_TASK", "bishe")).strip().lower() or "bishe"

VISION_TOOL_NAMES = {
    "vlm",
}

_TASK_REGISTRIES = {
    "bishe": "Robot_Module.module.Vision.Task.Bishe.registry",
}


def get_current_task(task: str | None = None) -> str:
    selected = str(task or DEFAULT_VISION_TASK).strip().lower()
    if selected not in _TASK_REGISTRIES:
        available = ", ".join(sorted(_TASK_REGISTRIES))
        raise ValueError(f"未知 Vision task: {selected}，可选: {available}")
    return selected


def register_tools(mcp, *, task: str | None = None):
    selected = get_current_task(task)
    module = importlib.import_module(_TASK_REGISTRIES[selected])
    register_fn = getattr(module, "register_tools")
    return register_fn(mcp)
