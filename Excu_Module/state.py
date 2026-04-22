from __future__ import annotations

import asyncio
import math
import time
from typing import Any, Sequence

from Hardware_Module import get_state

from .runtime import (
    DEFAULT_STATUS_POLL_SEC,
    DEFAULT_STATUS_READY_TIMEOUT_SEC,
    read_env_float,
    read_env_int,
)

DEFAULT_NAV_ARRIVAL_TOL_M = 0.13
DEFAULT_NAV_STABLE_POSITION_DELTA_M = 0.08
DEFAULT_NAV_REQUIRED_STABLE_POLLS = 3
DEFAULT_STATE_STALE_SEC = 2.0


def load_live_state(task_type: str | None = None) -> dict[str, Any]:
    state = get_state(task_type=task_type)
    return state if isinstance(state, dict) else {}


def extract_robot_pose(state: dict[str, Any]) -> list[float] | None:
    pose = (state.get("observation") or {}).get("agent_position")
    return round_pose(pose)


def extract_scene_objects(state: dict[str, Any]) -> list[dict[str, Any]]:
    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    obstacles = environment.get("obstacles")
    if isinstance(obstacles, list):
        return [item for item in obstacles if isinstance(item, dict)]
    runtime = state.get("runtime") or {}
    scene_objects = runtime.get("scene_objects")
    if isinstance(scene_objects, list):
        return [item for item in scene_objects if isinstance(item, dict)]
    return []


def extract_runtime_snapshot(state: dict[str, Any]) -> dict[str, Any]:
    runtime = state.get("runtime")
    if isinstance(runtime, dict):
        snapshot = runtime.get("snapshot")
        if isinstance(snapshot, dict):
            return snapshot
        compact_snapshot = {}
        for key in ("timestamp", "goal", "skill", "model_use", "start", "pose_command", "vel_command"):
            if key in runtime:
                compact_snapshot[key] = runtime.get(key)
        if compact_snapshot:
            return compact_snapshot
    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    action_result = observation.get("action_result") or {}
    return {
        "goal": environment.get("goal"),
        "skill": action_result.get("skill"),
        "model_use": action_result.get("model_use"),
        "start": action_result.get("start"),
    }


def extract_status_timestamp(state: dict[str, Any]) -> float | None:
    snapshot = extract_runtime_snapshot(state)
    try:
        return float(snapshot.get("timestamp"))
    except (TypeError, ValueError):
        return None


def extract_goal(state: dict[str, Any]) -> list[float] | None:
    snapshot = extract_runtime_snapshot(state)
    return round_pose(snapshot.get("goal"))


def extract_skill_name(state: dict[str, Any]) -> str | None:
    skill = extract_runtime_snapshot(state).get("skill")
    return str(skill) if isinstance(skill, str) and skill else None


def extract_model_use(state: dict[str, Any]) -> int | None:
    value = extract_runtime_snapshot(state).get("model_use")
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def extract_start_flag(state: dict[str, Any]) -> bool | None:
    value = extract_runtime_snapshot(state).get("start")
    return value if isinstance(value, bool) else None


def round_pose(values: Sequence[float] | None, keep_dims: int = 3) -> list[float] | None:
    if values is None:
        return None
    try:
        return [round(float(value), 3) for value in values[:keep_dims]]
    except (TypeError, ValueError):
        return None


def relative_target_pose(start_pose: Sequence[float], distance: float, direction: str | None) -> list[float]:
    target = [float(value) for value in start_pose[:3]]
    if direction == "forward":
        target[0] += float(distance)
    elif direction == "backward":
        target[0] -= float(distance)
    elif direction == "left":
        target[1] -= float(distance)
    elif direction == "right":
        target[1] += float(distance)
    return [round(value, 3) for value in target]


def directional_progress(dx: float, dy: float, direction: str | None) -> float:
    if direction == "forward":
        return dx
    if direction == "backward":
        return -dx
    if direction == "left":
        return -dy
    if direction == "right":
        return dy
    return math.hypot(dx, dy)


def summarize_state(state: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(state, dict):
        return None
    summary: dict[str, Any] = {
        "connected": bool(state.get("connected")),
        "task_type": state.get("task_type"),
    }
    timestamp = extract_status_timestamp(state)
    if timestamp is not None:
        summary["timestamp"] = round(timestamp, 3)
    for key, value in {
        "skill": extract_skill_name(state),
        "model_use": extract_model_use(state),
        "start": extract_start_flag(state),
        "robot_pose": extract_robot_pose(state),
        "goal": extract_goal(state),
    }.items():
        if value is not None:
            summary[key] = value

    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    scene_id = environment.get("scene_id")
    if scene_id is not None:
        summary["scene_id"] = scene_id
    return summary


async def wait_for_live_state(
    *,
    task_type: str | None,
    timeout_sec: float,
    min_timestamp: float | None = None,
) -> dict[str, Any] | None:
    poll_sec = max(0.1, read_env_float("FINALPROJECT_STATUS_POLL_SEC", DEFAULT_STATUS_POLL_SEC))
    deadline = time.time() + max(poll_sec, float(timeout_sec))

    while time.time() < deadline:
        state = load_live_state(task_type=task_type)
        if state.get("connected"):
            timestamp = extract_status_timestamp(state)
            if min_timestamp is None or timestamp is None or timestamp > min_timestamp:
                return state
        await asyncio.sleep(poll_sec)
    return None


async def wait_for_navigation_completion(
    *,
    task_type: str | None,
    goal_command: list[float],
    timeout_sec: float,
) -> dict[str, Any]:
    poll_sec = max(0.1, read_env_float("FINALPROJECT_STATUS_POLL_SEC", DEFAULT_STATUS_POLL_SEC))
    arrival_tol_m = abs(read_env_float("FINALPROJECT_NAV_ARRIVAL_TOL_M", DEFAULT_NAV_ARRIVAL_TOL_M))
    stable_delta_m = abs(
        read_env_float("FINALPROJECT_NAV_STABLE_POSITION_DELTA_M", DEFAULT_NAV_STABLE_POSITION_DELTA_M)
    )
    required_stable_polls = max(
        1,
        read_env_int("FINALPROJECT_NAV_REQUIRED_STABLE_POLLS", DEFAULT_NAV_REQUIRED_STABLE_POLLS),
    )
    stale_sec = abs(read_env_float("FINALPROJECT_STATUS_STALE_SEC", DEFAULT_STATE_STALE_SEC))
    ready_timeout_sec = max(
        poll_sec,
        read_env_float("FINALPROJECT_STATUS_READY_TIMEOUT_SEC", DEFAULT_STATUS_READY_TIMEOUT_SEC),
    )

    start_time = time.time()
    deadline = start_time + timeout_sec
    ready_deadline = start_time + ready_timeout_sec
    previous_pose: list[float] | None = None
    latest_state: dict[str, Any] | None = None
    latest_dist_xy: float | None = None
    latest_pose_delta: float | None = None
    stable_hits = 0
    observed_start = False

    from .skill_registry import get_navigation_model_uses
    navigation_model_uses = set(get_navigation_model_uses())

    while time.time() < deadline:
        await asyncio.sleep(poll_sec)
        state = load_live_state(task_type=task_type)
        if not state.get("connected"):
            if time.time() >= ready_deadline:
                return {
                    "ok": False, "reason": "未获取到实时状态",
                    "state": latest_state, "dist_xy": latest_dist_xy,
                    "pose_delta_xy": latest_pose_delta,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
            continue

        latest_state = state
        timestamp = extract_status_timestamp(state)
        if timestamp is not None and (time.time() - timestamp) > stale_sec:
            if time.time() >= ready_deadline:
                return {
                    "ok": False, "reason": f"实时状态超过 {stale_sec:.1f}s 未更新",
                    "state": latest_state, "dist_xy": latest_dist_xy,
                    "pose_delta_xy": latest_pose_delta,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
            continue

        robot_pose = extract_robot_pose(state)
        active_goal = extract_goal(state) or goal_command
        if robot_pose is None or active_goal is None or len(active_goal) < 2:
            continue

        latest_dist_xy = math.hypot(robot_pose[0] - active_goal[0], robot_pose[1] - active_goal[1])
        latest_pose_delta = None
        if previous_pose is not None:
            latest_pose_delta = math.hypot(robot_pose[0] - previous_pose[0], robot_pose[1] - previous_pose[1])
        previous_pose = robot_pose

        current_model_use = extract_model_use(state)
        start_flag = extract_start_flag(state)
        is_navigation_model = current_model_use in navigation_model_uses if current_model_use is not None else False
        if start_flag is True and is_navigation_model:
            observed_start = True
        is_stable_pose = latest_pose_delta is not None and latest_pose_delta <= stable_delta_m

        if latest_dist_xy <= arrival_tol_m and is_stable_pose:
            stable_hits += 1
            if stable_hits >= required_stable_polls:
                return {
                    "ok": True,
                    "reason": f"导航已到达目标附近 (dist_xy={latest_dist_xy:.3f}m)",
                    "state": latest_state, "dist_xy": latest_dist_xy,
                    "pose_delta_xy": latest_pose_delta, "stable_hits": stable_hits,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
        else:
            stable_hits = 0

        if observed_start and (start_flag is False or (current_model_use is not None and not is_navigation_model)):
            if latest_dist_xy is None or latest_dist_xy > arrival_tol_m:
                return {
                    "ok": False,
                    "reason": "导航在到达目标前已停止" if latest_dist_xy is None else f"导航在到达目标前已停止 (dist_xy={latest_dist_xy:.3f}m)",
                    "state": latest_state, "dist_xy": latest_dist_xy,
                    "pose_delta_xy": latest_pose_delta, "stable_hits": stable_hits,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }

    return {
        "ok": False,
        "reason": f"导航超时，{timeout_sec:.1f}s 内未到达目标" if latest_dist_xy is None else f"导航超时，{timeout_sec:.1f}s 内未到达目标 (dist_xy={latest_dist_xy:.3f}m)",
        "state": latest_state, "dist_xy": latest_dist_xy,
        "pose_delta_xy": latest_pose_delta, "stable_hits": stable_hits,
        "elapsed_sec": round(time.time() - start_time, 3),
    }


def build_navigation_validation(skill_name: str, goal_command: list[float], arrival_result: dict[str, Any]) -> dict[str, Any]:
    state = arrival_result.get("state")
    verified = isinstance(state, dict) and arrival_result.get("dist_xy") is not None
    summary = str(arrival_result.get("reason") or "").strip() or f"{skill_name} 未返回有效到达判定"
    return {
        "verified": verified,
        "meets_requirements": bool(arrival_result.get("ok")),
        "source": "comm_state",
        "summary": summary,
        "goal_command": round_pose(goal_command),
        "dist_xy": arrival_result.get("dist_xy"),
        "pose_delta_xy": arrival_result.get("pose_delta_xy"),
        "stable_hits": arrival_result.get("stable_hits"),
        "state": summarize_state(state),
        "elapsed_sec": arrival_result.get("elapsed_sec"),
    }


DEFAULT_DISPLACEMENT_TOL_M = 0.05
DEFAULT_DISPLACEMENT_POLL_SEC = 0.1
DEFAULT_DISPLACEMENT_STABLE_POLLS = 1
DEFAULT_DIRECTION_GRACE_SEC = 1.0


async def wait_for_displacement_completion(
    *,
    task_type: str | None,
    start_pose: list[float],
    distance: float,
    timeout_sec: float,
    direction: str | None = None,
) -> dict[str, Any]:
    """轮询等待位移完成。每 0.5s 读一次状态，检查从 start_pose 起的位移是否达到 distance。

    Args:
        task_type: 后端类型
        start_pose: 起始位姿 [x, y, z]
        distance: 目标位移距离（米）
        timeout_sec: 超时（秒）
        direction: 期望方向 "forward"/"backward"/"left"/"right"，用于方向校验
    """
    poll_sec = max(
        0.05,
        read_env_float("FINALPROJECT_DISPLACEMENT_POLL_SEC", DEFAULT_DISPLACEMENT_POLL_SEC),
    )
    arrival_tol = abs(read_env_float("FINALPROJECT_DISPLACEMENT_TOL_M", DEFAULT_DISPLACEMENT_TOL_M))
    required_stable = max(1, read_env_int("FINALPROJECT_DISPLACEMENT_STABLE_POLLS", DEFAULT_DISPLACEMENT_STABLE_POLLS))
    stale_sec = abs(read_env_float("FINALPROJECT_STATUS_STALE_SEC", DEFAULT_STATE_STALE_SEC))
    ready_timeout_sec = max(poll_sec, read_env_float("FINALPROJECT_STATUS_READY_TIMEOUT_SEC", DEFAULT_STATUS_READY_TIMEOUT_SEC))
    grace_sec = abs(read_env_float("FINALPROJECT_DIRECTION_GRACE_SEC", DEFAULT_DIRECTION_GRACE_SEC))

    start_time = time.time()
    deadline = start_time + timeout_sec
    ready_deadline = start_time + ready_timeout_sec
    grace_deadline = start_time + grace_sec
    latest_state: dict[str, Any] | None = None
    latest_disp: float | None = None
    stable_hits = 0
    observed_start = False
    target_pose = relative_target_pose(start_pose, distance, direction)

    while time.time() < deadline:
        await asyncio.sleep(poll_sec)
        state = load_live_state(task_type=task_type)
        if not state.get("connected"):
            if time.time() >= ready_deadline:
                return {
                    "ok": False, "reason": "未获取到实时状态",
                    "state": latest_state, "displacement": latest_disp,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
            continue

        latest_state = state
        timestamp = extract_status_timestamp(state)
        if timestamp is not None and (time.time() - timestamp) > stale_sec:
            if time.time() >= ready_deadline:
                return {
                    "ok": False, "reason": f"实时状态超过 {stale_sec:.1f}s 未更新",
                    "state": latest_state, "displacement": latest_disp,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
            continue

        robot_pose = extract_robot_pose(state)
        if robot_pose is None:
            continue

        in_grace = time.time() < grace_deadline

        dx = robot_pose[0] - start_pose[0]
        dy = robot_pose[1] - start_pose[1]
        progress = directional_progress(dx, dy, direction)
        latest_disp = progress

        # 方向校验：仅宽限期结束后检查
        if not in_grace and progress <= -arrival_tol:
            return {
                "ok": False, "reason": f"位移方向错误：期望{direction or '目标方向'}，实际反向 (dx={dx:.3f}, dy={dy:.3f})",
                "state": latest_state, "displacement": round(progress, 3),
                "target_pose": target_pose,
                "elapsed_sec": round(time.time() - start_time, 3),
            }

        # 位移达标判定
        remaining = max(0.0, distance - progress)
        if remaining <= arrival_tol:
            stable_hits += 1
            if stable_hits >= required_stable:
                return {
                    "ok": True,
                    "reason": f"位移完成 (displacement={progress:.3f}m, target={distance:.3f}m)",
                    "state": latest_state,
                    "displacement": round(progress, 3),
                    "dx": round(dx, 3), "dy": round(dy, 3),
                    "target_pose": target_pose,
                    "stable_hits": stable_hits,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
        else:
            stable_hits = 0

        # 只有确认技能曾经启动后，start=false 才表示提前停止。
        # ROS2/EnvTest 链路中 model_use/velocity 和 start 分两帧生效，启动前可能短暂看到旧的 idle 状态。
        start_flag = extract_start_flag(state)
        if start_flag is True:
            observed_start = True
        elif start_flag is False and observed_start:
            return {
                "ok": False,
                "reason": f"技能在位移完成前已停止 (displacement={progress:.3f}m, target={distance:.3f}m)",
                "state": latest_state, "displacement": round(progress, 3),
                "target_pose": target_pose,
                "elapsed_sec": round(time.time() - start_time, 3),
            }

    final_disp = latest_disp if latest_disp is not None else 0.0
    return {
        "ok": False,
        "reason": f"位移超时，{timeout_sec:.1f}s 内未达到目标 (displacement={final_disp:.3f}m, target={distance:.3f}m)",
        "state": latest_state, "displacement": latest_disp,
        "target_pose": target_pose,
        "elapsed_sec": round(time.time() - start_time, 3),
    }


def build_displacement_validation(skill_name: str, distance: float, arrival_result: dict[str, Any]) -> dict[str, Any]:
    """根据位移轮询结果构建 validation。"""
    state = arrival_result.get("state")
    verified = isinstance(state, dict) and arrival_result.get("displacement") is not None
    summary = str(arrival_result.get("reason") or "").strip() or f"{skill_name} 未返回有效位移判定"
    return {
        "verified": verified,
        "meets_requirements": bool(arrival_result.get("ok")),
        "source": "displacement_poll",
        "summary": summary,
        "target_distance": round(distance, 3),
        "actual_displacement": arrival_result.get("displacement"),
        "dx": arrival_result.get("dx"),
        "dy": arrival_result.get("dy"),
        "target_pose": arrival_result.get("target_pose"),
        "stable_hits": arrival_result.get("stable_hits"),
        "state": summarize_state(state),
        "elapsed_sec": arrival_result.get("elapsed_sec"),
    }
