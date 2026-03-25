"""
四足导航技能模块 (Navigation Skills Module)

当前只暴露 4 个技能给 LLM：
1. walk
2. climb
3. push_box
4. way_select

当前版本优先对接 IsaacLab EnvTest 的控制文件/UDP 协议，
同时保留可选 ROS 执行反馈通道，避免破坏现有上层接口。
"""

from __future__ import annotations

import argparse
import asyncio
import ast
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

try:
    from Comm_Module.execution_comm import publish_skill_command, wait_for_execution_feedback
except Exception:  # pragma: no cover - 运行环境未装 ROS2 时走本地控制后端
    publish_skill_command = None
    wait_for_execution_feedback = None


CLIMB_LIMIT_METERS = 0.3
DEFAULT_TARGET = "前方目标点"

MODEL_USE_IDLE = 0
MODEL_USE_WALK = 1
MODEL_USE_CLIMB = 2
MODEL_USE_PUSH_BOX = 3
MODEL_USE_NAVIGATION = 4

DEFAULT_WALK_SPEED_MPS = 0.6
DEFAULT_LATERAL_SPEED_MPS = 0.5
DEFAULT_CLIMB_SPEED_MPS = 0.6
DEFAULT_WAY_SELECT_FORWARD_BIAS_M = 0.2
DEFAULT_WAY_SELECT_DURATION_SEC = 3.0
DEFAULT_WALK_BUFFER_SEC = 0.5
DEFAULT_CLIMB_BASE_DURATION_SEC = 4.0
DEFAULT_CLIMB_EXECUTION_SEC = 15.0
DEFAULT_PUSH_BOX_DURATION_SEC = 15.0
DEFAULT_NAVIGATION_DURATION_SEC = 6.0
DEFAULT_COMMAND_SETTLE_SEC = 0.15
DEFAULT_STOP_SETTLE_SEC = 0.1

GOAL_AUTO_TOKENS = {
    "",
    "auto",
    "scene",
    "default",
    "高台旁边",
    "平台旁边",
    "高台前方",
    "目标点附近",
}


@dataclass(frozen=True)
class EnvTestRuntimeConfig:
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


def _read_env_float(name: str, default: float) -> float:
    raw = os.getenv(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _read_env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name, "").strip().lower()
    if not raw:
        return default
    return raw in {"1", "true", "yes", "y", "on"}


def _read_env_str(name: str, default: str) -> str:
    raw = os.getenv(name, "").strip()
    return raw or default


def _load_runtime_config() -> EnvTestRuntimeConfig:
    backend = os.getenv("FINALPROJECT_NAV_BACKEND", "file").strip().lower()
    if backend not in {"file", "udp", "ros"}:
        backend = "file"

    way_select_policy = os.getenv("FINALPROJECT_WAY_SELECT_POLICY", "walk").strip().lower()
    if way_select_policy not in {"walk", "navigation"}:
        way_select_policy = "walk"

    return EnvTestRuntimeConfig(
        backend=backend,
        udp_host=os.getenv("FINALPROJECT_NAV_UDP_HOST", "127.0.0.1").strip() or "127.0.0.1",
        udp_port=int(os.getenv("FINALPROJECT_NAV_UDP_PORT", "5566")),
        model_use_file=Path(os.getenv("FINALPROJECT_MODEL_USE_FILE", "/tmp/model_use.txt")),
        velocity_file=Path(os.getenv("FINALPROJECT_VELOCITY_FILE", "/tmp/envtest_velocity_command.txt")),
        goal_file=Path(os.getenv("FINALPROJECT_GOAL_FILE", "/tmp/envtest_goal_command.txt")),
        start_file=Path(os.getenv("FINALPROJECT_START_FILE", "/tmp/envtest_start.txt")),
        reset_file=Path(os.getenv("FINALPROJECT_RESET_FILE", "/tmp/envtest_reset.txt")),
        command_settle_sec=_read_env_float("FINALPROJECT_NAV_COMMAND_SETTLE_SEC", DEFAULT_COMMAND_SETTLE_SEC),
        stop_settle_sec=_read_env_float("FINALPROJECT_NAV_STOP_SETTLE_SEC", DEFAULT_STOP_SETTLE_SEC),
        auto_idle_after_skill=_read_env_bool("FINALPROJECT_NAV_AUTO_IDLE", True),
        way_select_policy=way_select_policy,
    )


def _envtest_repo_root() -> Path:
    return Path(_read_env_str("FINALPROJECT_ENVTEST_ROOT", "/home/xcj/work/IsaacLab/IsaacLabBisShe"))


def _envtest_socket_client_path() -> Path:
    default_path = _envtest_repo_root() / "Socket" / "envtest_socket_client.py"
    return Path(_read_env_str("FINALPROJECT_ENVTEST_SOCKET_CLIENT", str(default_path)))


def _climb_uses_socket_client(config: EnvTestRuntimeConfig) -> bool:
    if config.backend == "ros":
        return False
    return _read_env_bool("FINALPROJECT_CLIMB_USE_SOCKET_CLIENT", True)


def _format_height(height: float) -> str:
    return f"{height:.2f}"


def _make_action_id(skill_name: str) -> str:
    return f"{skill_name}-{uuid.uuid4().hex[:8]}"


def _build_feedback(skill: str, message: str, signal: str = "SUCCESS") -> dict[str, str]:
    return {
        "signal": signal,
        "skill": skill,
        "message": message,
    }


def _speak(text: str) -> None:
    if text:
        print(f"[go2.speech] {text}", file=sys.stderr)


def _log_skill(skill_name: str, detail: str) -> None:
    print(f"[go2.skill] {skill_name}: {detail}", file=sys.stderr)


def _normalize_direction(direction: str) -> str:
    normalized = direction.strip().lower()
    mapping = {
        "left": "left",
        "right": "right",
        "左": "left",
        "右": "right",
        "左边": "left",
        "右边": "right",
        "左侧": "left",
        "右侧": "right",
    }
    if normalized not in mapping:
        raise ValueError("direction 只能是 left/right 或 左/右")
    return mapping[normalized]


def _resolve_feedback_backend(feedback: dict[str, Any]) -> str:
    result = feedback.get("result") or {}
    backend = result.get("backend")
    if isinstance(backend, str) and backend:
        return backend
    return _load_runtime_config().backend


def _ensure_parent_dir(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def _write_text(path: Path, text: str) -> None:
    _ensure_parent_dir(path)
    path.write_text(text.strip() + "\n", encoding="utf-8")


def _format_vector_text(values: Sequence[float], keep_dims: int = 3) -> str:
    limited_values = [float(value) for value in values[:keep_dims]]
    return " ".join(str(value) for value in limited_values)


def _coerce_goal_vector(values: Sequence[Any]) -> list[float] | None:
    if len(values) < 3:
        return None
    try:
        parsed = [float(value) for value in values[:4]]
    except (TypeError, ValueError):
        return None
    if len(parsed) >= 3:
        return parsed
    return None


def _parse_goal_value(value: Any) -> list[float] | str | None:
    if value is None:
        return None

    if isinstance(value, (list, tuple)):
        return _coerce_goal_vector(value)

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
        goal_vector = _coerce_goal_vector(parsed)
        if goal_vector is not None:
            return goal_vector

    numeric_tokens = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)
    if len(numeric_tokens) >= 3:
        return [float(token) for token in numeric_tokens[:4]]

    return None


def _default_object_facts_path() -> Path:
    project_root = Path(__file__).resolve().parents[2]
    return Path(os.getenv("FINALPROJECT_OBJECT_FACTS_PATH", str(project_root / "config" / "object_facts.json")))


def _load_object_facts() -> dict[str, Any] | None:
    path = _default_object_facts_path()
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _extract_robot_pose() -> list[float]:
    object_facts = _load_object_facts() or {}
    robot_pose = object_facts.get("robot_pose") or [0.0, 0.0, 0.0]
    if not isinstance(robot_pose, list) or len(robot_pose) < 3:
        return [0.0, 0.0, 0.0]
    try:
        return [float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])]
    except (TypeError, ValueError):
        return [0.0, 0.0, 0.0]


def _select_side_anchor_y(direction: str) -> float | None:
    object_facts = _load_object_facts() or {}
    objects = object_facts.get("objects") or []
    direction_sign = 1 if direction == "left" else -1
    candidates: list[tuple[float, float]] = []

    for obj in objects:
        center = obj.get("center") or []
        if not isinstance(center, list) or len(center) < 2:
            continue
        try:
            center_x = float(center[0])
            center_y = float(center[1])
        except (TypeError, ValueError):
            continue
        if direction_sign > 0 and center_y >= 0:
            candidates.append((abs(center_x), center_y))
        if direction_sign < 0 and center_y < 0:
            candidates.append((abs(center_x), center_y))

    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def _resolve_way_select_navigation_goal(direction: str, lateral_distance: float) -> list[float]:
    robot_x, robot_y, robot_z = _extract_robot_pose()
    forward_bias = _read_env_float("FINALPROJECT_WAY_SELECT_FORWARD_BIAS_M", DEFAULT_WAY_SELECT_FORWARD_BIAS_M)
    direction_sign = 1.0 if direction == "left" else -1.0

    anchor_y = _select_side_anchor_y(direction)
    if anchor_y is None:
        target_y = robot_y + direction_sign * lateral_distance
    elif direction == "left":
        target_y = max(anchor_y, robot_y + lateral_distance)
    else:
        target_y = min(anchor_y, robot_y - lateral_distance)

    return [
        round(robot_x + forward_bias, 3),
        round(target_y, 3),
        round(robot_z, 3),
    ]


def _estimate_linear_duration(distance: float, speed: float, buffer_sec: float = DEFAULT_WALK_BUFFER_SEC) -> float:
    return max(0.5, distance / max(abs(speed), 0.05) + buffer_sec)


def _estimate_climb_duration(height: float) -> float:
    return max(
        DEFAULT_CLIMB_BASE_DURATION_SEC,
        DEFAULT_CLIMB_BASE_DURATION_SEC + height * 8.0,
    )


def _estimate_goal_skill_duration(goal_value: list[float] | str | None, fallback_sec: float) -> float:
    if not isinstance(goal_value, list) or len(goal_value) < 2:
        return fallback_sec

    robot_x, robot_y, _ = _extract_robot_pose()
    goal_x = float(goal_value[0])
    goal_y = float(goal_value[1])
    planar_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
    return max(fallback_sec, planar_distance / 0.35 + 1.0)


def _apply_envtest_command(
    config: EnvTestRuntimeConfig,
    *,
    model_use: int | None = None,
    velocity: Sequence[float] | None = None,
    goal: list[float] | str | None = None,
    start: bool | None = None,
    reset: int | None = None,
) -> None:
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
        _write_text(config.model_use_file, str(int(model_use)))
    if velocity is not None:
        _write_text(config.velocity_file, _format_vector_text(velocity, keep_dims=3))
    if goal is not None:
        if goal == "auto":
            _write_text(config.goal_file, "auto")
        else:
            _write_text(config.goal_file, _format_vector_text(goal, keep_dims=3))
    if start is not None:
        _write_text(config.start_file, "1" if start else "0")
    if reset is not None:
        _write_text(config.reset_file, str(int(reset)))


async def _stop_envtest_skill(config: EnvTestRuntimeConfig) -> None:
    _apply_envtest_command(config, start=False)
    await asyncio.sleep(config.stop_settle_sec)
    stop_fields: dict[str, Any] = {"velocity": [0.0, 0.0, 0.0]}
    if config.auto_idle_after_skill:
        stop_fields["model_use"] = MODEL_USE_IDLE
    _apply_envtest_command(config, **stop_fields)


async def _run_envtest_socket_client(
    config: EnvTestRuntimeConfig,
    *args: str,
) -> dict[str, Any]:
    client_path = _envtest_socket_client_path()
    if not client_path.is_file():
        raise FileNotFoundError(f"未找到 envtest_socket_client.py: {client_path}")

    command = [
        sys.executable,
        str(client_path),
        "--host",
        config.udp_host,
        "--port",
        str(config.udp_port),
        *args,
    ]
    process = await asyncio.create_subprocess_exec(
        *command,
        cwd=str(client_path.parent.parent),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    stdout, stderr = await process.communicate()
    output = {
        "command": command,
        "returncode": process.returncode,
        "stdout": stdout.decode("utf-8", errors="ignore").strip(),
        "stderr": stderr.decode("utf-8", errors="ignore").strip(),
    }
    if process.returncode != 0:
        message = output["stderr"] or output["stdout"] or "socket client 执行失败"
        raise RuntimeError(message)
    return output


async def _stop_envtest_skill_via_socket_client(config: EnvTestRuntimeConfig) -> None:
    await _run_envtest_socket_client(config, "--start", "0")
    await asyncio.sleep(config.stop_settle_sec)
    await _run_envtest_socket_client(config, "--velocity", "0.0", "0.0", "0.0")
    if config.auto_idle_after_skill:
        await _run_envtest_socket_client(config, "--model_use", str(MODEL_USE_IDLE))


async def _wait_skill_feedback(
    skill_name: str,
    parameters: dict[str, Any],
    local_success_message: str,
    wait_feedback: bool = True,
    timeout_sec: float = 20.0,
    *,
    model_use: int | None = None,
    velocity_command: Sequence[float] | None = None,
    goal_command: list[float] | str | None = None,
    execution_time_sec: float | None = None,
) -> dict[str, Any]:
    action_id = _make_action_id(skill_name)
    config = _load_runtime_config()
    planned_result = {
        "mode": "planned_only",
        "backend": config.backend,
        "model_use": model_use,
        "parameters": parameters,
        "velocity_command": list(velocity_command) if velocity_command is not None else None,
        "goal_command": goal_command,
        "estimated_execution_time_sec": execution_time_sec,
    }

    if not wait_feedback:
        return {
            "action_id": action_id,
            **_build_feedback(skill_name, local_success_message),
            "result": planned_result,
        }

    if config.backend == "ros":
        if publish_skill_command is None or wait_for_execution_feedback is None:
            raise RuntimeError("当前环境未安装 ROS2 依赖，无法使用 ros 执行后端")
        action_id = publish_skill_command(skill_name, parameters, action_id=action_id)
        feedback = await wait_for_execution_feedback(action_id, timeout_sec=timeout_sec)
        feedback.setdefault("action_id", action_id)
        feedback.setdefault("skill", skill_name)
        feedback.setdefault("result", {})
        return feedback

    if model_use is None:
        raise ValueError(f"{skill_name} 缺少 EnvTest 所需的 model_use")

    execution_time_sec = max(0.5, float(execution_time_sec or 0.5))
    _apply_envtest_command(
        config,
        model_use=model_use,
        velocity=velocity_command,
        goal=goal_command,
    )
    await asyncio.sleep(config.command_settle_sec)
    _apply_envtest_command(config, start=True)

    actual_wait_sec = min(timeout_sec, execution_time_sec)
    await asyncio.sleep(actual_wait_sec)
    await _stop_envtest_skill(config)

    if execution_time_sec > timeout_sec:
        return {
            "action_id": action_id,
            **_build_feedback(
                skill_name,
                f"{skill_name} 预计执行 {execution_time_sec:.1f}s，超过超时阈值 {timeout_sec:.1f}s",
                signal="FAILURE",
            ),
            "result": {
                **planned_result,
                "mode": "envtest_timeout",
                "backend": config.backend,
                "wait_time_sec": actual_wait_sec,
            },
        }

    return {
        "action_id": action_id,
        **_build_feedback(skill_name, local_success_message),
        "result": {
            **planned_result,
            "mode": "envtest_control",
            "backend": config.backend,
            "wait_time_sec": actual_wait_sec,
        },
    }


async def _execute_climb_via_socket_client(
    *,
    height: float,
    stage: str,
    target: str,
    velocity_command: Sequence[float],
    execution_time_sec: float,
    wait_feedback: bool,
    config: EnvTestRuntimeConfig,
    timeout_sec: float = 20.0,
) -> dict[str, Any]:
    action_id = _make_action_id("climb")
    planned_result = {
        "mode": "planned_only",
        "backend": "socket_client",
        "model_use": MODEL_USE_CLIMB,
        "parameters": {
            "height": height,
            "stage": stage,
            "target": target,
            "velocity_command": list(velocity_command),
        },
        "velocity_command": list(velocity_command),
        "goal_command": None,
        "estimated_execution_time_sec": execution_time_sec,
    }
    if not wait_feedback:
        return {
            "action_id": action_id,
            **_build_feedback("climb", f"已完成{stage}" if "攀爬" in stage else f"已完成{stage}攀爬"),
            "result": planned_result,
        }

    await _run_envtest_socket_client(
        config,
        "--model_use",
        str(MODEL_USE_CLIMB),
        "--velocity",
        *(str(float(value)) for value in velocity_command[:3]),
    )
    await asyncio.sleep(config.command_settle_sec)
    await _run_envtest_socket_client(config, "--start", "1")

    actual_wait_sec = min(timeout_sec, execution_time_sec)
    await asyncio.sleep(actual_wait_sec)
    await _stop_envtest_skill_via_socket_client(config)

    if execution_time_sec > timeout_sec:
        return {
            "action_id": action_id,
            **_build_feedback(
                "climb",
                f"climb 预计执行 {execution_time_sec:.1f}s，超过超时阈值 {timeout_sec:.1f}s",
                signal="FAILURE",
            ),
            "result": {
                **planned_result,
                "mode": "envtest_timeout",
                "wait_time_sec": actual_wait_sec,
            },
        }

    return {
        "action_id": action_id,
        **_build_feedback("climb", f"已完成{stage}" if "攀爬" in stage else f"已完成{stage}攀爬"),
        "result": {
            **planned_result,
            "mode": "envtest_socket_client",
            "wait_time_sec": actual_wait_sec,
        },
    }


async def execute_walk_skill(
    route_side: str = "前方",
    distance: float = 1.0,
    target: str = DEFAULT_TARGET,
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, Any]:
    """行走技能。"""
    if distance <= 0:
        raise ValueError("distance 必须大于 0")

    walk_speed = abs(_read_env_float("FINALPROJECT_WALK_SPEED_MPS", DEFAULT_WALK_SPEED_MPS))
    velocity_command = [walk_speed, 0.0, 0.0]
    execution_time_sec = _estimate_linear_duration(distance, walk_speed)

    _speak(speech)
    _log_skill(
        "walk",
        f"沿{route_side}路径行走 {distance:.2f} 米，目标={target}，速度命令={velocity_command}，预计执行 {execution_time_sec:.2f} 秒",
    )
    feedback = await _wait_skill_feedback(
        "walk",
        {
            "route_side": route_side,
            "distance": distance,
            "target": target,
            "velocity_command": velocity_command,
        },
        f"已完成沿{route_side}路线行走",
        wait_feedback=wait_feedback,
        model_use=MODEL_USE_WALK,
        velocity_command=velocity_command,
        execution_time_sec=execution_time_sec,
    )
    status = "success" if feedback.get("signal") == "SUCCESS" else "failure"
    backend_label = _resolve_feedback_backend(feedback)
    return {
        "skill": "walk",
        "route_side": route_side,
        "distance": distance,
        "target": target,
        "speech": speech,
        "action_id": feedback.get("action_id"),
        "execution_feedback": feedback,
        "execution_result": feedback.get("result", {}),
        "control_command": {
            "model_use": MODEL_USE_WALK,
            "velocity": velocity_command,
            "estimated_execution_time_sec": round(execution_time_sec, 3),
        },
        "backend": backend_label,
        "status": status,
    }


async def execute_climb_skill(
    height: float,
    stage: str = "高台",
    target: str = DEFAULT_TARGET,
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, Any]:
    """攀爬技能。"""
    if height <= 0:
        raise ValueError("height 必须大于 0")
    if height > CLIMB_LIMIT_METERS:
        raise ValueError(
            f"当前 demo 约束为最大攀爬高度 {CLIMB_LIMIT_METERS:.2f} 米，收到 {height:.2f} 米"
        )

    config = _load_runtime_config()
    climb_speed = abs(_read_env_float("FINALPROJECT_CLIMB_SPEED_MPS", DEFAULT_CLIMB_SPEED_MPS))
    velocity_command = [climb_speed, 0.0, 0.0]
    execution_time_sec = max(0.5, _read_env_float("FINALPROJECT_CLIMB_DURATION_SEC", DEFAULT_CLIMB_EXECUTION_SEC))

    _speak(speech)
    _log_skill(
        "climb",
        f"攀爬{stage}，高度 {height:.2f} 米，目标={target}，速度命令={velocity_command}，预计执行 {execution_time_sec:.2f} 秒",
    )

    if _climb_uses_socket_client(config):
        feedback = await _execute_climb_via_socket_client(
            height=height,
            stage=stage,
            target=target,
            velocity_command=velocity_command,
            execution_time_sec=execution_time_sec,
            wait_feedback=wait_feedback,
            config=config,
        )
    else:
        feedback = await _wait_skill_feedback(
            "climb",
            {
                "height": height,
                "stage": stage,
                "target": target,
                "velocity_command": velocity_command,
            },
            f"已完成{stage}" if "攀爬" in stage else f"已完成{stage}攀爬",
            wait_feedback=wait_feedback,
            model_use=MODEL_USE_CLIMB,
            velocity_command=velocity_command,
            execution_time_sec=execution_time_sec,
        )
    status = "success" if feedback.get("signal") == "SUCCESS" else "failure"
    backend_label = _resolve_feedback_backend(feedback)
    return {
        "skill": "climb",
        "height": height,
        "stage": stage,
        "target": target,
        "speech": speech,
        "action_id": feedback.get("action_id"),
        "execution_feedback": feedback,
        "execution_result": feedback.get("result", {}),
        "control_command": {
            "model_use": MODEL_USE_CLIMB,
            "velocity": velocity_command,
            "estimated_execution_time_sec": round(execution_time_sec, 3),
        },
        "backend": backend_label,
        "status": status,
    }


async def execute_push_box_skill(
    box_height: float,
    target_position: str = "高台旁边",
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, Any]:
    """推箱子技能。"""
    if box_height <= 0:
        raise ValueError("box_height 必须大于 0")

    goal_command = _parse_goal_value(target_position)
    if goal_command is None:
        goal_command = "auto"
    execution_time_sec = _estimate_goal_skill_duration(goal_command, DEFAULT_PUSH_BOX_DURATION_SEC)

    _speak(speech)
    _log_skill(
        "push_box",
        f"推动高度 {box_height:.2f} 米的箱子到{target_position}，目标命令={goal_command}",
    )
    feedback = await _wait_skill_feedback(
        "push_box",
        {
            "box_height": box_height,
            "target_position": target_position,
            "goal_command": goal_command,
        },
        f"已完成推箱子到{target_position}",
        wait_feedback=wait_feedback,
        model_use=MODEL_USE_PUSH_BOX,
        goal_command=goal_command,
        execution_time_sec=execution_time_sec,
    )
    status = "success" if feedback.get("signal") == "SUCCESS" else "failure"
    backend_label = _resolve_feedback_backend(feedback)
    return {
        "skill": "push_box",
        "box_height": box_height,
        "target_position": target_position,
        "speech": speech,
        "action_id": feedback.get("action_id"),
        "execution_feedback": feedback,
        "execution_result": feedback.get("result", {}),
        "control_command": {
            "model_use": MODEL_USE_PUSH_BOX,
            "goal": goal_command,
            "estimated_execution_time_sec": round(execution_time_sec, 3),
        },
        "backend": backend_label,
        "status": status,
    }


async def execute_way_select_skill(
    direction: str,
    lateral_distance: float = 0.5,
    target: str = DEFAULT_TARGET,
    speech: str = "",
    wait_feedback: bool = True,
) -> dict[str, Any]:
    """路线选择技能。"""
    normalized_direction = _normalize_direction(direction)
    direction_label = "左侧" if normalized_direction == "left" else "右侧"

    if lateral_distance <= 0:
        raise ValueError("lateral_distance 必须大于 0")

    if not speech:
        speech = f"机器人当前位于中间位置，先切换到{direction_label}路线。"

    config = _load_runtime_config()
    control_mode = config.way_select_policy
    goal_command: list[float] | str | None = None
    velocity_command: list[float] | None = None
    model_use = MODEL_USE_WALK

    if control_mode == "navigation":
        model_use = MODEL_USE_NAVIGATION
        goal_command = _resolve_way_select_navigation_goal(normalized_direction, lateral_distance)
        execution_time_sec = _estimate_goal_skill_duration(goal_command, DEFAULT_NAVIGATION_DURATION_SEC)
        base_skill_call = {
            "skill": "navigation",
            "goal": goal_command,
            "target": f"{direction_label}路线入口",
        }
    else:
        lateral_speed = abs(DEFAULT_LATERAL_SPEED_MPS)
        signed_lateral_speed = lateral_speed if normalized_direction == "left" else -lateral_speed
        velocity_command = [0.0, signed_lateral_speed, 0.0]
        execution_time_sec = DEFAULT_WAY_SELECT_DURATION_SEC
        base_skill_call = {
            "skill": "walk",
            "route_side": f"{direction_label}路线入口",
            "distance": round(lateral_speed * execution_time_sec, 3),
            "requested_lateral_distance": lateral_distance,
            "target": f"{direction_label}路线入口",
            "velocity_command": velocity_command,
            "execution_time_sec": execution_time_sec,
        }

    _speak(speech)
    _log_skill(
        "way_select",
        f"选择{direction_label}路线，控制模式={control_mode}，目标={target}，速度命令={velocity_command}，预计执行 {execution_time_sec:.2f} 秒",
    )
    feedback = await _wait_skill_feedback(
        "way_select",
        {
            "direction": normalized_direction,
            "lateral_distance": lateral_distance,
            "target": target,
            "control_mode": control_mode,
            "velocity_command": velocity_command,
            "goal_command": goal_command,
        },
        f"已成功切换到{direction_label}路线",
        wait_feedback=wait_feedback,
        model_use=model_use,
        velocity_command=velocity_command,
        goal_command=goal_command,
        execution_time_sec=execution_time_sec,
    )
    status = "success" if feedback.get("signal") == "SUCCESS" else "failure"
    backend_label = _resolve_feedback_backend(feedback)

    return {
        "skill": "way_select",
        "direction": normalized_direction,
        "lateral_distance": lateral_distance,
        "target": target,
        "speech": speech,
        "base_skill_call": base_skill_call,
        "action_id": feedback.get("action_id"),
        "execution_feedback": feedback,
        "execution_result": feedback.get("result", {}),
        "control_command": {
            "model_use": model_use,
            "velocity": velocity_command,
            "goal": goal_command,
            "control_mode": control_mode,
            "estimated_execution_time_sec": round(execution_time_sec, 3),
        },
        "backend": backend_label,
        "status": status,
    }


async def execute_case_2_flow(
    target: str = DEFAULT_TARGET,
    left_height: float = 0.2,
    right_height: float = 0.0,
    lateral_distance: float = 0.5,
    forward_distance: float = 1.0,
) -> dict[str, Any]:
    """case 2 本地 demo：右侧绕行。"""
    if left_height <= 0 or left_height > CLIMB_LIMIT_METERS:
        raise ValueError("case 2 要求左侧高台高度在 (0, 0.3] 米范围内")
    if right_height > 0.05:
        raise ValueError("case 2 要求右侧无障碍或近似无障碍")

    speech = (
        f"前方左侧检测到高台，高度约 {_format_height(left_height)} 米，"
        f"右侧障碍高度约 {_format_height(right_height)} 米。"
        f"按照简单快速原则，先选择右侧路线，再调用行走技能前往{target}。"
    )

    execution = [
        await execute_way_select_skill(
            direction="right",
            lateral_distance=lateral_distance,
            target=target,
            speech=speech,
            wait_feedback=False,
        ),
        await execute_walk_skill(
            route_side="右侧",
            distance=forward_distance,
            target=target,
            speech=f"已进入右侧路线，继续行走到{target}。",
            wait_feedback=False,
        ),
    ]

    return {
        "status": "success",
        "scene_type": "case_2_left_platform_right_clear",
        "target": target,
        "observation": {
            "left_height": left_height,
            "right_height": right_height,
            "climb_limit": CLIMB_LIMIT_METERS,
        },
        "execution": execution,
    }


async def execute_case_4_flow(
    target: str = DEFAULT_TARGET,
    left_height: float = 0.4,
    right_height: float = 0.4,
    box_height: float = 0.2,
    lateral_distance: float = 0.5,
    forward_distance: float = 1.0,
    box_side: str = "left",
) -> dict[str, Any]:
    """case 4 本地 demo：选择箱子路线，推箱子后二段攀爬。"""
    if left_height <= CLIMB_LIMIT_METERS or right_height <= CLIMB_LIMIT_METERS:
        raise ValueError("case 4 要求左右高台都超过最大攀爬高度")
    if box_height <= 0 or box_height > CLIMB_LIMIT_METERS:
        raise ValueError("case 4 要求箱子高度在 (0, 0.3] 米范围内")

    climb_from_box_height = max(left_height, right_height) - box_height
    if climb_from_box_height <= 0 or climb_from_box_height > CLIMB_LIMIT_METERS:
        raise ValueError("case 4 的平台高度和箱子高度组合不满足二段攀爬条件")

    box_side_label = "左侧" if _normalize_direction(box_side) == "left" else "右侧"
    speech = (
        f"前方左右两侧均检测到高台，左侧高度约 {_format_height(left_height)} 米，"
        f"右侧高度约 {_format_height(right_height)} 米，超过最大攀爬高度 "
        f"{_format_height(CLIMB_LIMIT_METERS)} 米。检测到{box_side_label}存在可推动箱子，"
        f"高度约 {_format_height(box_height)} 米。按照简单快速原则，先选择有箱子的路线，"
        f"再执行推箱子和攀爬动作前往{target}。"
    )

    execution = [
        await execute_way_select_skill(
            direction=box_side,
            lateral_distance=lateral_distance,
            target=target,
            speech=speech,
            wait_feedback=False,
        ),
        await execute_push_box_skill(
            box_height=box_height,
            target_position="高台旁边",
            speech="开始推动箱子到高台旁边。",
            wait_feedback=False,
        ),
        await execute_climb_skill(
            height=box_height,
            stage="箱子顶部",
            target=target,
            speech="箱子已到位，先攀爬到箱子顶部。",
            wait_feedback=False,
        ),
        await execute_climb_skill(
            height=climb_from_box_height,
            stage="箱子到高台的二段攀爬",
            target=target,
            speech="继续从箱子顶部攀爬到高台。",
            wait_feedback=False,
        ),
        await execute_walk_skill(
            route_side="高台上方",
            distance=forward_distance,
            target=target,
            speech=f"已经到达高台，继续行走到{target}。",
            wait_feedback=False,
        ),
    ]

    return {
        "status": "success",
        "scene_type": "case_4_high_platform_with_box",
        "target": target,
        "observation": {
            "left_height": left_height,
            "right_height": right_height,
            "box_height": box_height,
            "box_side": _normalize_direction(box_side),
            "climb_limit": CLIMB_LIMIT_METERS,
        },
        "execution": execution,
    }


def register_tools(mcp):
    """注册 4 个导航技能。"""

    @mcp.tool()
    async def walk(
        route_side: str = "前方",
        distance: float = 1.0,
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        """行走技能。

        用于机器人沿当前路线继续前进。
        当前默认通过 EnvTest 控制文件/UDP 协议触发 IsaacLab 技能执行。
        """
        result = await execute_walk_skill(
            route_side=route_side,
            distance=distance,
            target=target,
            speech=speech,
        )
        return json.dumps(result, ensure_ascii=False)

    @mcp.tool()
    async def climb(
        height: float,
        stage: str = "高台",
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        """攀爬技能。

        用于机器人攀爬箱子或高台。
        当前默认通过 EnvTest 控制文件/UDP 协议触发 IsaacLab 技能执行。
        """
        result = await execute_climb_skill(
            height=height,
            stage=stage,
            target=target,
            speech=speech,
        )
        return json.dumps(result, ensure_ascii=False)

    @mcp.tool()
    async def push_box(
        box_height: float,
        target_position: str = "高台旁边",
        speech: str = "",
    ) -> str:
        """推箱子技能。

        用于把可推动箱子移动到高台旁边。
        `target_position` 支持 `[x, y, z]` / `"x,y,z"` / `"auto"`。
        """
        result = await execute_push_box_skill(
            box_height=box_height,
            target_position=target_position,
            speech=speech,
        )
        return json.dumps(result, ensure_ascii=False)

    @mcp.tool()
    async def way_select(
        direction: str,
        lateral_distance: float = 0.5,
        target: str = DEFAULT_TARGET,
        speech: str = "",
    ) -> str:
        """路线选择技能。

        机器人初始位于中间位置时，优先调用该技能切换到左侧或右侧路线。
        默认使用 walk 模式做横向切换，也可通过环境变量切到 navigation 模式。
        """
        result = await execute_way_select_skill(
            direction=direction,
            lateral_distance=lateral_distance,
            target=target,
            speech=speech,
        )
        return json.dumps(result, ensure_ascii=False)

    print("[navigation.py:register_tools] 导航技能模块已注册 (4 个工具)", file=sys.stderr)

    return {
        "walk": walk,
        "climb": climb,
        "push_box": push_box,
        "way_select": way_select,
    }


async def _run_cli_demo(scene: str) -> dict[str, Any]:
    """命令行 demo 入口。"""
    if scene == "case2":
        return await execute_case_2_flow()
    if scene == "case4":
        return await execute_case_4_flow()
    raise ValueError(f"不支持的场景: {scene}")


def main() -> None:
    """支持直接运行模块做本地场景演示。"""
    parser = argparse.ArgumentParser(description="四足导航技能演示")
    parser.add_argument("scene", choices=["case2", "case4"], help="要运行的演示场景")
    args = parser.parse_args()

    import asyncio

    result = asyncio.run(_run_cli_demo(args.scene))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
