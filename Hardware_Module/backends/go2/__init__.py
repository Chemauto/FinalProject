from __future__ import annotations

from importlib import import_module


_EXPORTS = {
    "ROS2_TOPICS": "Hardware_Module.backends.go2.schema",
    "STATE_SCHEMA": "Hardware_Module.backends.go2.schema",
    "TASK_DATA": "Hardware_Module.backends.go2.schema",
    "get_data": "Hardware_Module.backends.go2.data",
    "sync_object_facts_from_live_data": "Hardware_Module.backends.go2.data",
    "sync_runtime_overrides_from_user_input": "Hardware_Module.backends.go2.data",
}


def __getattr__(name: str):
    module_path = _EXPORTS.get(name)
    if module_path is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(module_path)
    return getattr(module, name)


__all__ = list(_EXPORTS.keys())
