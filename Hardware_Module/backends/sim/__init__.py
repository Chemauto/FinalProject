from __future__ import annotations

from importlib import import_module


_EXPORTS = {
    "DEFAULT_CONTROL_FILES": "Hardware_Module.backends.sim.data",
    "DEFAULT_ENVTEST_REPO_ROOT": "Hardware_Module.backends.sim.data",
    "DEFAULT_OBJECT_FACTS_PATH": "Hardware_Module.backends.sim.data",
    "STATE_SCHEMA": "Hardware_Module.backends.sim.schema",
    "TASK_DATA": "Hardware_Module.backends.sim.schema",
    "get_data": "Hardware_Module.backends.sim.data",
    "sync_object_facts_from_live_data": "Hardware_Module.backends.sim.data",
    "sync_object_facts_from_live_envtest": "Hardware_Module.backends.sim.data",
    "sync_object_facts_from_status_text": "Hardware_Module.backends.sim.data",
    "sync_runtime_overrides_from_user_input": "Hardware_Module.backends.sim.data",
}


def get_state():
    from Hardware_Module.schema import get_state as resolve_state

    return resolve_state(task_type="sim")


def __getattr__(name: str):
    module_path = _EXPORTS.get(name)
    if module_path is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_path)
    return getattr(module, name)


__all__ = list(_EXPORTS.keys()) + ["get_state"]
