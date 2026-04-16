from __future__ import annotations

from importlib import import_module


_EXPORTS = {
    "DEFAULT_CONTROL_FILES": "Comm_Module.Task.Sim.get_data",
    "DEFAULT_ENVTEST_REPO_ROOT": "Comm_Module.Task.Sim.get_data",
    "DEFAULT_OBJECT_FACTS_PATH": "Comm_Module.Task.Sim.get_data",
    "STATE_SCHEMA": "Comm_Module.Task.Sim.Data",
    "TASK_DATA": "Comm_Module.Task.Sim.Data",
    "get_data": "Comm_Module.Task.Sim.get_data",
    "sync_object_facts_from_live_data": "Comm_Module.Task.Sim.get_data",
    "sync_object_facts_from_live_envtest": "Comm_Module.Task.Sim.get_data",
    "sync_object_facts_from_status_text": "Comm_Module.Task.Sim.get_data",
    "sync_runtime_overrides_from_user_input": "Comm_Module.Task.Sim.get_data",
}


def get_state():
    from Comm_Module.Status.get_state import get_state as resolve_state

    return resolve_state(task_type="sim")


def __getattr__(name: str):
    module_path = _EXPORTS.get(name)
    if module_path is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_path)
    return getattr(module, name)


__all__ = list(_EXPORTS.keys()) + ["get_state"]
