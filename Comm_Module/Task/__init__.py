from __future__ import annotations

import importlib
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


@dataclass(frozen=True)
class TaskBackend:
    """具体任务后端定义。"""

    task_type: str
    description: str
    data_module: str
    state_schema: dict[str, Any]
    data_getter_name: str = "get_data"
    sync_object_facts_from_live_data_name: str | None = None
    sync_runtime_overrides_from_user_input_name: str | None = None


@dataclass(frozen=True)
class TaskRegistration:
    task_type: str
    module_path: str
    description: str = ""


_TASK_REGISTRY: dict[str, TaskRegistration] = {}
_DEFAULT_TASK_TYPE = os.getenv("FINALPROJECT_ROBOT_TYPE", "sim")


def _load_backend_callable(backend: TaskBackend, function_name: str | None) -> Callable[..., Any] | None:
    if function_name is None:
        return None

    module = importlib.import_module(backend.data_module)
    function = getattr(module, function_name, None)
    if function is None:
        raise ValueError(f"{backend.data_module} 未导出 {function_name}")
    if not callable(function):
        raise TypeError(f"{backend.data_module}.{function_name} 必须是可调用对象")
    return function


def register_task(task_type: str, module_path: str, description: str = "") -> None:
    normalized_type = str(task_type).strip().lower()
    if not normalized_type:
        raise ValueError("task_type 不能为空")
    _TASK_REGISTRY[normalized_type] = TaskRegistration(
        task_type=normalized_type,
        module_path=module_path,
        description=description,
    )


def list_task_types() -> list[str]:
    return sorted(_TASK_REGISTRY.keys())


def get_default_task_type() -> str:
    return _DEFAULT_TASK_TYPE


def load_task_backend(task_type: str | None = None) -> TaskBackend:
    selected_type = str(task_type or _DEFAULT_TASK_TYPE).strip().lower()
    registration = _TASK_REGISTRY.get(selected_type)
    if registration is None:
        available = ", ".join(list_task_types()) or "(空)"
        raise ValueError(f"未知 task_type: {selected_type}，可选: {available}")

    module = importlib.import_module(registration.module_path)
    backend = getattr(module, "TASK_DATA", None)
    if backend is None:
        raise ValueError(f"{registration.module_path} 未导出 TASK_DATA")
    if not isinstance(backend, TaskBackend):
        raise TypeError(f"{registration.module_path}.TASK_DATA 必须是 TaskBackend")
    return backend


def get_task_state(task_type: str | None = None) -> dict[str, Any]:
    from Comm_Module.Status.get_state import get_state

    return get_state(task_type=task_type)


def get_task_data(task_type: str | None = None) -> dict[str, Any]:
    backend = load_task_backend(task_type)
    data_getter = _load_backend_callable(backend, backend.data_getter_name)
    if data_getter is None:
        raise NotImplementedError(f"task_type={backend.task_type} 未实现数据获取")
    return data_getter()


def sync_object_facts_from_live_data(
    object_facts_path: str | Path,
    *,
    user_input: str = "",
    task_type: str | None = None,
) -> dict[str, Any] | None:
    backend = load_task_backend(task_type)
    sync_function = _load_backend_callable(backend, backend.sync_object_facts_from_live_data_name)
    if sync_function is None:
        raise NotImplementedError(f"task_type={backend.task_type} 未实现 live data 同步")
    return sync_function(
        object_facts_path=object_facts_path,
        user_input=user_input,
    )


def sync_runtime_overrides_from_user_input(
    object_facts_path: str | Path,
    *,
    user_input: str = "",
    task_type: str | None = None,
) -> dict[str, Any] | None:
    backend = load_task_backend(task_type)
    sync_function = _load_backend_callable(backend, backend.sync_runtime_overrides_from_user_input_name)
    if sync_function is None:
        raise NotImplementedError(f"task_type={backend.task_type} 未实现 runtime override 同步")
    return sync_function(
        object_facts_path=object_facts_path,
        user_input=user_input,
    )


register_task(
    task_type="sim",
    module_path="Comm_Module.Task.Sim.Data",
    description="IsaacLab EnvTest 仿真后端",
)


__all__ = [
    "TaskBackend",
    "get_default_task_type",
    "get_task_data",
    "get_task_state",
    "list_task_types",
    "load_task_backend",
    "register_task",
    "sync_object_facts_from_live_data",
    "sync_runtime_overrides_from_user_input",
]
