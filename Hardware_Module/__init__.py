"""Hardware_Module - 硬件抽象层，统一状态获取与后端管理。"""


def get_state(*args, **kwargs):
    from .schema import get_state as _get_state

    return _get_state(*args, **kwargs)


def list_registered_task_types():
    from .schema import list_registered_task_types as _list_registered_task_types

    return _list_registered_task_types()


__all__ = ["get_state", "list_registered_task_types"]
