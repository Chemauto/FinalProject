#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Data_Module/facts.py — object_facts 加载、标准化与同步。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


# ── Normalization helpers ──────────────────────────────────────────────


def _normalize_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y", "是"}:
            return True
        if normalized in {"false", "0", "no", "n", "否"}:
            return False
    return default


def _normalize_number(value: Any, field_name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field_name} 必须是数字，收到: {value!r}") from error


def _normalize_vec3(value: Any, field_name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field_name} 必须是长度为 3 的数组")
    return [_normalize_number(item, field_name) for item in value]


def _normalize_object(payload: dict[str, Any], index: int) -> dict[str, Any]:
    object_id = str(payload.get("id", "")).strip()
    if not object_id:
        raise ValueError(f"objects[{index}].id 不能为空")
    object_type = str(payload.get("type", "")).strip() or "unknown"
    return {
        "id": object_id,
        "type": object_type,
        "center": _normalize_vec3(payload.get("center"), f"objects[{index}].center"),
        "size": _normalize_vec3(payload.get("size"), f"objects[{index}].size"),
        "movable": _normalize_bool(payload.get("movable"), default=False),
    }


def _normalize_constraints(payload: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in payload.items():
        if isinstance(value, bool):
            result[key] = value
        elif isinstance(value, (int, float)):
            result[key] = float(value)
        else:
            result[key] = value
    return result


def normalize_object_facts(payload: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("object_facts 顶层必须是 JSON 对象")
    constraints_payload = payload.get("constraints") or {}
    if constraints_payload and not isinstance(constraints_payload, dict):
        raise ValueError("constraints 必须是 JSON 对象")
    objects_payload = payload.get("objects") or []
    if not isinstance(objects_payload, list):
        raise ValueError("objects 必须是数组")
    constraints = _normalize_constraints(constraints_payload)
    return {
        "navigation_goal": _normalize_vec3(payload.get("navigation_goal"), "navigation_goal"),
        "robot_pose": _normalize_vec3(payload.get("robot_pose"), "robot_pose"),
        "constraints": constraints,
        "objects": [_normalize_object(item, index) for index, item in enumerate(objects_payload)],
    }


# ── Public API ─────────────────────────────────────────────────────────


def load_object_facts(path: str | Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    file_path = Path(path)
    if not file_path.exists():
        return None
    payload = json.loads(file_path.read_text(encoding="utf-8"))
    return normalize_object_facts(payload)
