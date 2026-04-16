"""Comm_Module - 通用状态与执行通信模块。"""


def get_state(*args, **kwargs):
    from .Status.get_state import get_state as _get_state

    return _get_state(*args, **kwargs)


def list_registered_task_types():
    from .Status.get_state import list_registered_task_types as _list_registered_task_types

    return _list_registered_task_types()

__all__ = ["get_state", "list_registered_task_types"]
