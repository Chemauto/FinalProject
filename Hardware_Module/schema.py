#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通用状态获取接口。

Hardware_Module/schema.py — 统一状态入口与通用解析。

用法:
    from Hardware_Module import get_state
    state = get_state()           # 自动检测 task_type
    obs = state["observation"]    # agent 的观测
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from typing import Any

from Hardware_Module.registry import (
    get_default_task_type,
    get_task_data,
    list_task_types,
    load_task_backend,
)


_DEFAULT_TASK_TYPE = get_default_task_type()
_MISSING = object()
_FIELD_KEYS = {"source", "default", "literal"}


def _is_field_spec(schema: Any) -> bool:
    return isinstance(schema, dict) and any(key in schema for key in _FIELD_KEYS)


def _resolve_path(payload: Any, source: str | None) -> Any:
    if not source:
        return _MISSING

    current = payload
    for part in source.split("."):
        if isinstance(current, dict):
            if part not in current:
                return _MISSING
            current = current[part]
            continue

        if isinstance(current, list) and part.isdigit():
            index = int(part)
            if 0 <= index < len(current):
                current = current[index]
                continue
        return _MISSING
    return current


def _parse_field(schema: dict[str, Any], payload: dict[str, Any]) -> Any:
    if "literal" in schema:
        return copy.deepcopy(schema["literal"])

    value = _resolve_path(payload, schema.get("source"))
    if value is not _MISSING:
        return copy.deepcopy(value)

    if "default" in schema:
        return copy.deepcopy(schema["default"])
    return None


def _parse_schema(schema: Any, payload: dict[str, Any]) -> Any:
    if _is_field_spec(schema):
        return _parse_field(schema, payload)

    if isinstance(schema, dict):
        return {key: _parse_schema(value, payload) for key, value in schema.items()}

    if isinstance(schema, list):
        return [_parse_schema(item, payload) for item in schema]

    return copy.deepcopy(schema)


def get_state(task_type: str | None = None) -> dict[str, Any]:
    """获取当前任务/机器人/环境的状态。"""
    selected_type = task_type or _DEFAULT_TASK_TYPE
    backend = load_task_backend(selected_type)
    raw_data = get_task_data(backend.task_type)
    state = _parse_schema(backend.state_schema, raw_data)

    if not isinstance(state, dict):
        raise TypeError(f"task_type={backend.task_type} 的 state_schema 顶层必须是 dict")

    state.setdefault("connected", False)
    state.setdefault("task_type", backend.task_type)
    state.setdefault("robot_type", state.get("task_type", backend.task_type))
    state.setdefault("observation", {})
    state.setdefault("runtime", {})
    return state


def list_registered_task_types() -> list[str]:
    """返回所有已注册 task_type。"""
    return list_task_types()


def main() -> int:
    parser = argparse.ArgumentParser(description="获取当前机器人/环境状态。")
    parser.add_argument(
        "--type",
        type=str,
        default=None,
        dest="task_type",
        help="任务后端类型（默认 sim）",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        default=False,
        help="JSON 格式输出",
    )
    parser.add_argument(
        "--list-types",
        action="store_true",
        default=False,
        help="列出可用 task_type",
    )
    args = parser.parse_args()

    if args.list_types:
        for rt in list_registered_task_types():
            print(rt)
        return 0

    try:
        state = get_state(task_type=args.task_type)
    except (TypeError, ValueError, ImportError) as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(state, ensure_ascii=False, indent=2))
    else:
        connected = "已连接" if state.get("connected") else "未连接"
        print(f"task_type:      {state.get('task_type', state.get('robot_type', '-'))}")
        print(f"连接状态:       {connected}")
        if state.get("connected"):
            obs = state.get("observation", {})
            if obs:
                print("observation:")
                for key, value in obs.items():
                    if isinstance(value, (dict, list)) and value:
                        print(f"  {key}: {json.dumps(value, ensure_ascii=False, indent=2)}")
                    else:
                        print(f"  {key}: {value}")
            else:
                print("observation:   (空)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
