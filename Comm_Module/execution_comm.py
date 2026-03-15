#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""执行通信模块。

负责在上层决策和执行器之间传递两类消息：
1. `/robot/skill_command`：高层技能执行命令
2. `/robot/execution_feedback`：执行完成/失败反馈
"""

from __future__ import annotations

import asyncio
import json
import sys
import time
import uuid
from typing import Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SKILL_COMMAND_TOPIC = "/robot/skill_command"
EXECUTION_FEEDBACK_TOPIC = "/robot/execution_feedback"

_ros_initialized = False
_skill_command_publisher = None
_skill_command_subscriber = None
_execution_feedback_publisher = None
_execution_feedback_subscriber = None


def _ros_init() -> None:
    """初始化 ROS2，上下文在单进程内只初始化一次。"""
    global _ros_initialized
    if _ros_initialized:
        return
    try:
        rclpy.init()
        _ros_initialized = True
        print("[execution_comm] ROS 已初始化", file=sys.stderr)
    except Exception as error:
        print(f"[execution_comm] ROS 初始化跳过: {error}", file=sys.stderr)
        _ros_initialized = True


def create_action_id(skill_name: str) -> str:
    """生成技能执行 ID。"""
    return f"{skill_name}-{uuid.uuid4().hex[:8]}"


class SkillCommandPublisher:
    """技能命令发布器。"""

    def __init__(self):
        _ros_init()
        self.node = Node("skill_command_publisher")
        self.publisher = self.node.create_publisher(String, SKILL_COMMAND_TOPIC, 10)
        print(f"[SkillCommandPublisher] 已创建: {SKILL_COMMAND_TOPIC}", file=sys.stderr)
        time.sleep(0.2)

    def publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(msg)
        print(f"[SkillCommandPublisher] 发布命令: {payload}", file=sys.stderr)

    def wait_for_subscribers(self, timeout_sec: float = 2.0, poll_interval: float = 0.05) -> bool:
        """等待至少一个技能执行器订阅当前命令话题。"""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.publisher.get_subscription_count() > 0:
                return True
            time.sleep(poll_interval)
        return self.publisher.get_subscription_count() > 0


class SkillCommandSubscriber:
    """技能命令订阅器。"""

    def __init__(self, callback: Callable[[dict], None]):
        _ros_init()
        self.node = Node("skill_command_subscriber")
        self.callback = callback
        self.subscription = self.node.create_subscription(
            String,
            SKILL_COMMAND_TOPIC,
            self._message_callback,
            10,
        )
        print(f"[SkillCommandSubscriber] 已创建: {SKILL_COMMAND_TOPIC}", file=sys.stderr)

    def _message_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.callback(payload)
        except Exception as error:
            print(f"[SkillCommandSubscriber] 解析失败: {error}", file=sys.stderr)

    def spin_once(self, timeout_sec: float = 0.001) -> None:
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            pass


class ExecutionFeedbackPublisher:
    """执行反馈发布器。"""

    def __init__(self):
        _ros_init()
        self.node = Node("execution_feedback_publisher")
        self.publisher = self.node.create_publisher(String, EXECUTION_FEEDBACK_TOPIC, 10)
        print(f"[ExecutionFeedbackPublisher] 已创建: {EXECUTION_FEEDBACK_TOPIC}", file=sys.stderr)
        time.sleep(0.2)

    def publish(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(msg)
        print(f"[ExecutionFeedbackPublisher] 发布反馈: {payload}", file=sys.stderr)


class ExecutionFeedbackSubscriber:
    """执行反馈订阅器。"""

    def __init__(self):
        _ros_init()
        self.node = Node("execution_feedback_subscriber")
        self._feedback_cache: dict[str, dict] = {}
        self.subscription = self.node.create_subscription(
            String,
            EXECUTION_FEEDBACK_TOPIC,
            self._message_callback,
            10,
        )
        print(f"[ExecutionFeedbackSubscriber] 已创建: {EXECUTION_FEEDBACK_TOPIC}", file=sys.stderr)
        time.sleep(0.2)

    def _message_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            action_id = payload.get("action_id")
            if action_id:
                self._feedback_cache[action_id] = payload
        except Exception as error:
            print(f"[ExecutionFeedbackSubscriber] 解析失败: {error}", file=sys.stderr)

    def spin_once(self, timeout_sec: float = 0.001) -> None:
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            pass

    def pop_feedback(self, action_id: str) -> dict | None:
        return self._feedback_cache.pop(action_id, None)


def get_skill_command_publisher() -> SkillCommandPublisher:
    global _skill_command_publisher
    if _skill_command_publisher is None:
        _skill_command_publisher = SkillCommandPublisher()
    return _skill_command_publisher


def get_skill_command_subscriber(callback: Callable[[dict], None]) -> SkillCommandSubscriber:
    global _skill_command_subscriber
    if _skill_command_subscriber is None:
        _skill_command_subscriber = SkillCommandSubscriber(callback)
    return _skill_command_subscriber


def get_execution_feedback_publisher() -> ExecutionFeedbackPublisher:
    global _execution_feedback_publisher
    if _execution_feedback_publisher is None:
        _execution_feedback_publisher = ExecutionFeedbackPublisher()
    return _execution_feedback_publisher


def get_execution_feedback_subscriber() -> ExecutionFeedbackSubscriber:
    global _execution_feedback_subscriber
    if _execution_feedback_subscriber is None:
        _execution_feedback_subscriber = ExecutionFeedbackSubscriber()
    return _execution_feedback_subscriber


def publish_skill_command(skill_name: str, parameters: dict, action_id: str | None = None) -> str:
    """发布技能执行命令，返回 action_id。"""
    action_id = action_id or create_action_id(skill_name)
    publisher = get_skill_command_publisher()
    if not publisher.wait_for_subscribers():
        raise RuntimeError(f"未检测到执行器订阅 {SKILL_COMMAND_TOPIC}")

    payload = {
        "action_id": action_id,
        "skill": skill_name,
        "parameters": parameters,
        "timestamp": time.time(),
    }
    publisher.publish(payload)
    return action_id


def publish_execution_feedback(
    action_id: str,
    skill_name: str,
    signal: str,
    message: str,
    result: dict | None = None,
) -> None:
    """发布执行反馈。"""
    payload = {
        "action_id": action_id,
        "skill": skill_name,
        "signal": signal,
        "message": message,
        "result": result or {},
        "timestamp": time.time(),
    }
    get_execution_feedback_publisher().publish(payload)


async def wait_for_execution_feedback(
    action_id: str,
    timeout_sec: float = 10.0,
    poll_interval: float = 0.05,
) -> dict:
    """异步等待某个 action_id 的执行反馈。"""
    subscriber = get_execution_feedback_subscriber()
    start_time = time.time()

    while time.time() - start_time < timeout_sec:
        subscriber.spin_once(timeout_sec=0.001)
        feedback = subscriber.pop_feedback(action_id)
        if feedback is not None:
            return feedback
        await asyncio.sleep(poll_interval)

    return {
        "action_id": action_id,
        "signal": "FAILURE",
        "message": f"等待执行反馈超时（{timeout_sec:.1f}s）",
        "result": {},
    }
