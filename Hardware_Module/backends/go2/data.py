#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Go2 真机数据获取 — 通过 ROS2 topic 订阅获取状态。"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import Any

# ── ROS2 可选导入 ──────────────────────────────────────────
_rclpy_ok = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Odometry
    from std_msgs.msg import String
    _rclpy_ok = True
except ImportError:
    pass


# ── topic 名字 ────────────────────────────────────────────
from Hardware_Module.backends.go2.schema import ROS2_TOPICS


# ── 全局缓存 ──────────────────────────────────────────────
_latest_state: dict[str, Any] = {}
_node = None
_subscribers = []


class _Go2StateNode:
    """ROS2 订阅节点，持续缓存最新状态。"""

    def __init__(self):
        if not _rclpy_ok:
            return

        try:
            if not rclpy.ok():
                rclpy.init()
        except Exception:
            return

        self.node = rclpy.create_node("finalproject_go2_state_subscriber")
        self._odom_data: dict[str, Any] = {}
        self._skill_data: dict[str, Any] = {}
        self._scene_objects: list[dict[str, Any]] = []
        self._connected = False
        self._last_update = 0.0

        # 订阅里程计
        self.node.create_subscription(
            Odometry,
            ROS2_TOPICS["odom"],
            self._odom_callback,
            10,
        )

        # 订阅技能状态（JSON string）
        self.node.create_subscription(
            String,
            ROS2_TOPICS["skill_status"],
            self._skill_callback,
            10,
        )

        # 订阅场景物体（JSON string）
        self.node.create_subscription(
            String,
            ROS2_TOPICS["scene_objects"],
            self._scene_callback,
            10,
        )

    def _odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self._odom_data = {
            "x": pos.x,
            "y": pos.y,
            "z": pos.z,
        }
        self._connected = True
        self._last_update = time.time()

    def _skill_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            data = {"raw": msg.data}
        self._skill_data = data
        self._last_update = time.time()

    def _scene_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                self._scene_objects = data
        except (json.JSONDecodeError, TypeError):
            pass

    def spin_once(self, timeout_sec: float = 0.1):
        """非阻塞 spin 一次。"""
        if not _rclpy_ok or not hasattr(self, "node"):
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            pass

    def build_state(self) -> dict[str, Any]:
        """把缓存数据组装成统一 state dict。"""
        odom = self._odom_data
        skill = self._skill_data

        return {
            "connected": self._connected,
            "task_type": "go2",
            "timestamp": self._last_update,
            "odom": odom,
            "skill_status": skill,
            "scene_objects": self._scene_objects,
        }


def _get_or_create_node() -> _Go2StateNode | None:
    """获取或创建全局 ROS2 订阅节点。"""
    global _node
    if _node is None and _rclpy_ok:
        _node = _Go2StateNode()
    return _node


def _spin_and_get() -> dict[str, Any]:
    """spin 一次并返回最新状态。"""
    node = _get_or_create_node()
    if node is None:
        return {"connected": False, "task_type": "go2"}
    node.spin_once(timeout_sec=0.05)
    return node.build_state()


# ── 公共接口（与 Sim 后端对齐） ───────────────────────────


def get_data() -> dict[str, Any]:
    """获取 Go2 当前状态数据（供 Hardware_Module 状态解析使用）。

    返回的 dict 结构对应 schema.py 中的 STATE_SCHEMA。
    """
    raw = _spin_and_get()

    odom = raw.get("odom", {})
    skill = raw.get("skill_status", {})

    return {
        "connected": raw.get("connected", False),
        "task_type": "go2",
        "timestamp": raw.get("timestamp"),
        "odom": {
            "position": [odom.get("x", 0.0), odom.get("y", 0.0), odom.get("z", 0.0)],
        },
        "skill_status": skill,
        "scene_objects": raw.get("scene_objects", []),
    }


def sync_object_facts_from_live_data(
    object_facts_path: str | Path = "",
    *,
    user_input: str = "",
) -> dict[str, Any] | None:
    """从 ROS2 实时状态同步 object_facts.json。"""
    from Hardware_Module.backends.sim.data import (
        update_object_facts_runtime,
        extract_runtime_overrides,
        extract_navigation_goal_override,
    )

    path = Path(object_facts_path)
    if not path.exists():
        return None

    raw = _spin_and_get()
    if not raw.get("connected"):
        return None

    odom = raw.get("odom", {})
    robot_pose = odom.get("position")
    skill = raw.get("skill_status", {})
    scene_objects = raw.get("scene_objects", [])

    # 构建 snapshot 格式（与 Sim 后端兼容）
    snapshot = {
        "robot_pose": robot_pose,
        "goal": skill.get("goal"),
        "model_use": skill.get("model_use"),
        "start": skill.get("start"),
        "skill": skill.get("skill"),
        "scene_id": skill.get("scene_id"),
        "timestamp": raw.get("timestamp"),
    }

    # 构建 runtime_objects 格式（与 Sim 后端兼容）
    runtime_objects = []
    for obj in scene_objects:
        if isinstance(obj, dict):
            runtime_objects.append({
                "id": obj.get("id", ""),
                "type": obj.get("type", ""),
                "center": obj.get("center") or obj.get("position"),
                "size": obj.get("size"),
                "movable": obj.get("movable", False),
            })

    return update_object_facts_runtime(
        path,
        snapshot=snapshot,
        user_input=user_input,
        scene_objects=scene_objects,
        runtime_objects=runtime_objects,
        replace_objects=True,
    )


def sync_runtime_overrides_from_user_input(
    object_facts_path: str | Path = "",
    *,
    user_input: str = "",
) -> dict[str, Any] | None:
    """从用户输入提取覆盖值写入 object_facts.json。"""
    from Hardware_Module.backends.sim.data import (
        update_object_facts_runtime,
        extract_runtime_overrides,
        extract_navigation_goal_override,
    )

    path = Path(object_facts_path)
    if not path.exists():
        return None

    overrides = extract_runtime_overrides(user_input)
    nav_override = extract_navigation_goal_override(user_input)

    snapshot = {}
    if overrides:
        snapshot.update(overrides)
    if nav_override is not None:
        snapshot["navigation_goal_override"] = nav_override

    if not snapshot and nav_override is None:
        return None

    return update_object_facts_runtime(
        path,
        snapshot=snapshot,
        user_input=user_input,
    )
