from __future__ import annotations

import ast
import asyncio
import json
import math
import os
import re
import socket
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

# ── ROS2 可选导入（Go2 真机后端） ──────────────────────────
_rclpy_ok = False
_ros2_publishers: dict[str, Any] = {}
_ros2_node = None

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Int32, Bool as BoolMsg
    from geometry_msgs.msg import Twist, PoseStamped
    _rclpy_ok = True
except ImportError:
    pass


# ── ROS2 topic 名字（TODO: 替换为实际 topic） ──────────────
ROS2_TOPICS = {
    "skill_command": "/go2/skill_command",       # JSON: {model_use, velocity, goal, start}
    "velocity_command": "/go2/cmd_vel",          # geometry_msgs/Twist
    "goal_command": "/go2/goal_pose",            # geometry_msgs/PoseStamped
}


DEFAULT_TARGET = "前方目标点"

# ── Universal model-use code: 0 = idle for all projects ────────────
MODEL_USE_IDLE = 0

DEFAULT_STATUS_POLL_SEC = 0.5
DEFAULT_STATUS_READY_TIMEOUT_SEC = 2.0
DEFAULT_COMMAND_SETTLE_SEC = 0.15
DEFAULT_STOP_SETTLE_SEC = 0.1

GOAL_AUTO_TOKENS = {
    "",
    "auto",
    "scene",
    "default",
}


@dataclass(frozen=True)
class ExecutionRuntimeConfig:
    task_type: str
    backend: str
    udp_host: str
    udp_port: int
    model_use_file: Path
    velocity_file: Path
    goal_file: Path
    start_file: Path
    reset_file: Path
    command_settle_sec: float
    stop_settle_sec: float
    auto_idle_after_skill: bool
    way_select_policy: str


def read_env_float(name: str, default: float) -> float:
    raw = os.getenv(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def read_env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name, "").strip().lower()
    if not raw:
        return default
    return raw in {"1", "true", "yes", "y", "on"}


def read_env_int(name: str, default: int) -> int:
    raw = os.getenv(name, "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def load_runtime_config(task_type: str | None = None) -> ExecutionRuntimeConfig:
    backend = os.getenv("FINALPROJECT_NAV_BACKEND", "file").strip().lower()
    if backend not in {"file", "udp", "ros"}:
        backend = "file"

    way_select_policy = os.getenv("FINALPROJECT_WAY_SELECT_POLICY", "walk").strip().lower()
    if way_select_policy not in {"walk", "navigation"}:
        way_select_policy = "walk"

    resolved_task_type = str(task_type or os.getenv("FINALPROJECT_ROBOT_TYPE", "sim")).strip().lower() or "sim"
    return ExecutionRuntimeConfig(
        task_type=resolved_task_type,
        backend=backend,
        udp_host=os.getenv("FINALPROJECT_NAV_UDP_HOST", "127.0.0.1").strip() or "127.0.0.1",
        udp_port=read_env_int("FINALPROJECT_NAV_UDP_PORT", 5566),
        model_use_file=Path(os.getenv("FINALPROJECT_MODEL_USE_FILE", "/tmp/model_use.txt")),
        velocity_file=Path(os.getenv("FINALPROJECT_VELOCITY_FILE", "/tmp/envtest_velocity_command.txt")),
        goal_file=Path(os.getenv("FINALPROJECT_GOAL_FILE", "/tmp/envtest_goal_command.txt")),
        start_file=Path(os.getenv("FINALPROJECT_START_FILE", "/tmp/envtest_start.txt")),
        reset_file=Path(os.getenv("FINALPROJECT_RESET_FILE", "/tmp/envtest_reset.txt")),
        command_settle_sec=read_env_float("FINALPROJECT_NAV_COMMAND_SETTLE_SEC", DEFAULT_COMMAND_SETTLE_SEC),
        stop_settle_sec=read_env_float("FINALPROJECT_NAV_STOP_SETTLE_SEC", DEFAULT_STOP_SETTLE_SEC),
        auto_idle_after_skill=read_env_bool("FINALPROJECT_NAV_AUTO_IDLE", True),
        way_select_policy=way_select_policy,
    )


def make_action_id(skill_name: str) -> str:
    return f"{skill_name}-{uuid.uuid4().hex[:8]}"


def build_feedback(
    skill: str,
    message: str,
    signal: str = "SUCCESS",
    *,
    validation: dict[str, Any] | None = None,
) -> dict[str, Any]:
    feedback: dict[str, Any] = {
        "signal": signal,
        "skill": skill,
        "message": message,
    }
    if validation is not None:
        feedback["validation"] = validation
    return feedback


def resolve_feedback_backend(feedback: dict[str, Any]) -> str:
    result = feedback.get("result") or {}
    backend = result.get("backend")
    if isinstance(backend, str) and backend:
        return backend
    return load_runtime_config().backend


def speak(text: str) -> None:
    if text:
        print(f"[go2.speech] {text}", file=sys.stderr)


def log_skill(skill_name: str, detail: str) -> None:
    print(f"[go2.skill] {skill_name}: {detail}", file=sys.stderr)


def ensure_parent_dir(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def write_text(path: Path, text: str) -> None:
    ensure_parent_dir(path)
    path.write_text(text.strip() + "\n", encoding="utf-8")


def format_vector_text(values: Sequence[float], keep_dims: int = 3) -> str:
    return " ".join(str(float(value)) for value in values[:keep_dims])


def coerce_goal_vector(values: Sequence[Any]) -> list[float] | None:
    if len(values) < 3:
        return None
    try:
        return [float(value) for value in values[:4]]
    except (TypeError, ValueError):
        return None


def parse_goal_value(value: Any) -> list[float] | str | None:
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        return coerce_goal_vector(value)

    text = str(value).strip()
    if not text:
        return None
    if text.lower() in GOAL_AUTO_TOKENS:
        return "auto"

    parsed: Any = None
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        try:
            parsed = ast.literal_eval(text)
        except (ValueError, SyntaxError):
            parsed = None

    if isinstance(parsed, (list, tuple)):
        goal_vector = coerce_goal_vector(parsed)
        if goal_vector is not None:
            return goal_vector

    numeric_tokens = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)
    if len(numeric_tokens) >= 3:
        return [float(token) for token in numeric_tokens[:4]]
    return None


def estimate_linear_duration(distance: float, speed: float, buffer_sec: float = 0.5) -> float:
    return max(0.5, distance / max(abs(speed), 0.05) + buffer_sec)


def estimate_goal_skill_duration(goal_value: list[float] | str | None, fallback_sec: float) -> float:
    if not isinstance(goal_value, list) or len(goal_value) < 2:
        return fallback_sec

    try:
        from Excu_Module.state import extract_robot_pose, load_live_state

        robot_pose = extract_robot_pose(load_live_state())
    except Exception:
        robot_pose = None

    if not robot_pose or len(robot_pose) < 2:
        return fallback_sec

    planar_distance = math.hypot(float(goal_value[0]) - robot_pose[0], float(goal_value[1]) - robot_pose[1])
    return max(fallback_sec, planar_distance / 0.35 + 1.0)



# ── ROS2 发布函数（Go2 真机） ────────────────────────────


def _ensure_ros2_node():
    """确保 ROS2 节点和 publisher 已创建。"""
    global _ros2_node, _ros2_publishers

    if not _rclpy_ok:
        return False

    if _ros2_node is not None:
        return True

    try:
        if not rclpy.ok():
            rclpy.init()
        _ros2_node = rclpy.create_node("finalproject_command_publisher")

        _ros2_publishers["skill_command"] = _ros2_node.create_publisher(
            String, ROS2_TOPICS["skill_command"], 10,
        )
        _ros2_publishers["velocity_command"] = _ros2_node.create_publisher(
            Twist, ROS2_TOPICS["velocity_command"], 10,
        )
        _ros2_publishers["goal_command"] = _ros2_node.create_publisher(
            PoseStamped, ROS2_TOPICS["goal_command"], 10,
        )
        return True
    except Exception as e:
        print(f"[ROS2] 初始化失败: {e}", file=sys.stderr)
        _ros2_node = None
        return False


def _ros2_publish_command(
    model_use: int | None = None,
    velocity: list[float] | None = None,
    goal: list[float] | str | None = None,
    start: bool | None = None,
    reset: int | None = None,
) -> None:
    """通过 ROS2 topic 发布命令到 Go2。

    发布两条消息：
    1. /go2/skill_command (JSON) — 完整技能状态
    2. /go2/cmd_vel 或 /go2/goal_pose — 具体运动命令
    """
    if not _ensure_ros2_node():
        return

    # 1. 发布 skill_command (JSON)
    skill_msg: dict[str, Any] = {}
    if model_use is not None:
        skill_msg["model_use"] = int(model_use)
    if start is not None:
        skill_msg["start"] = start
    if reset is not None:
        skill_msg["reset"] = int(reset)
    if goal is not None and goal != "auto":
        skill_msg["goal"] = goal if isinstance(goal, list) else list(goal)
    if velocity is not None:
        skill_msg["velocity"] = velocity

    if skill_msg:
        msg = String()
        msg.data = json.dumps(skill_msg)
        _ros2_publishers["skill_command"].publish(msg)

    # 2. 发布 velocity_command (Twist)
    if velocity is not None:
        twist = Twist()
        twist.linear.x = float(velocity[0])
        twist.linear.y = float(velocity[1] if len(velocity) > 1 else 0.0)
        twist.linear.z = float(velocity[2] if len(velocity) > 2 else 0.0)
        _ros2_publishers["velocity_command"].publish(twist)

    # 3. 发布 goal_command (PoseStamped)
    if goal is not None and goal != "auto":
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = _ros2_node.get_clock().now().to_msg()
        if isinstance(goal, (list, tuple)):
            pose.pose.position.x = float(goal[0])
            pose.pose.position.y = float(goal[1]) if len(goal) > 1 else 0.0
            pose.pose.position.z = float(goal[2]) if len(goal) > 2 else 0.0
            if len(goal) > 3:
                # yaw → quaternion
                yaw = float(goal[3])
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
        _ros2_publishers["goal_command"].publish(pose)


def apply_envtest_command(
    config: ExecutionRuntimeConfig,
    *,
    model_use: int | None = None,
    velocity: Sequence[float] | None = None,
    goal: list[float] | str | None = None,
    start: bool | None = None,
    reset: int | None = None,
) -> None:
    if config.backend == "ros":
        _ros2_publish_command(
            model_use=model_use,
            velocity=list(velocity) if velocity is not None else None,
            goal=goal,
            start=start,
            reset=reset,
        )
        return

    if config.backend == "udp":
        fields: list[str] = []
        if model_use is not None:
            fields.append(f"model_use={int(model_use)}")
        if velocity is not None:
            fields.append(f"velocity={','.join(str(float(value)) for value in velocity[:3])}")
        if goal is not None:
            if goal == "auto":
                fields.append("goal=auto")
            else:
                fields.append(f"goal={','.join(str(float(value)) for value in goal[:3])}")
        if start is not None:
            fields.append(f"start={1 if start else 0}")
        if reset is not None:
            fields.append(f"reset={int(reset)}")
        if not fields:
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto("; ".join(fields).encode("utf-8"), (config.udp_host, config.udp_port))
        finally:
            sock.close()
        return

    if model_use is not None:
        write_text(config.model_use_file, str(int(model_use)))
    if velocity is not None:
        write_text(config.velocity_file, format_vector_text(velocity, keep_dims=3))
    if goal is not None:
        write_text(config.goal_file, "auto" if goal == "auto" else format_vector_text(goal, keep_dims=3))
    if start is not None:
        write_text(config.start_file, "1" if start else "0")
    if reset is not None:
        write_text(config.reset_file, str(int(reset)))


async def stop_envtest_skill(config: ExecutionRuntimeConfig) -> None:
    apply_envtest_command(config, start=False)
    await asyncio.sleep(config.stop_settle_sec)
    stop_fields: dict[str, Any] = {"velocity": [0.0, 0.0, 0.0]}
    if config.auto_idle_after_skill:
        stop_fields["model_use"] = MODEL_USE_IDLE
    apply_envtest_command(config, **stop_fields)
