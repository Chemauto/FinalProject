from __future__ import annotations

import asyncio
import math
import time
from typing import Any, Sequence

from Comm_Module import get_state

from .runtime import (
    DEFAULT_STATUS_POLL_SEC,
    DEFAULT_STATUS_READY_TIMEOUT_SEC,
    read_env_float,
    read_env_int,
)

DEFAULT_NAV_ARRIVAL_TOL_M = 0.15
DEFAULT_NAV_STABLE_POSITION_DELTA_M = 0.08
DEFAULT_NAV_REQUIRED_STABLE_POLLS = 3
DEFAULT_STATE_STALE_SEC = 2.0


def load_live_state(task_type: str | None = None) -> dict[str, Any]:
    state = get_state(task_type=task_type)
    return state if isinstance(state, dict) else {}


def extract_robot_pose(state: dict[str, Any] | None) -> list[float] | None:
    if not isinstance(state, dict):
        return None
    observation = state.get("observation") or {}
    pose = observation.get("agent_position")
    return round_pose(pose)


def extract_scene_objects(state: dict[str, Any] | None) -> list[dict[str, Any]]:
    if not isinstance(state, dict):
        return []
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


def extract_runtime_snapshot(state: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(state, dict):
        return {}
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


def extract_status_timestamp(state: dict[str, Any] | None) -> float | None:
    snapshot = extract_runtime_snapshot(state)
    try:
        return float(snapshot.get("timestamp"))
    except (TypeError, ValueError):
        return None


def extract_goal(state: dict[str, Any] | None) -> list[float] | None:
    snapshot = extract_runtime_snapshot(state)
    goal = snapshot.get("goal")
    return round_pose(goal)


def extract_skill_name(state: dict[str, Any] | None) -> str | None:
    snapshot = extract_runtime_snapshot(state)
    skill = snapshot.get("skill")
    return str(skill) if isinstance(skill, str) and skill else None


def extract_model_use(state: dict[str, Any] | None) -> int | None:
    snapshot = extract_runtime_snapshot(state)
    value = snapshot.get("model_use")
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def extract_start_flag(state: dict[str, Any] | None) -> bool | None:
    snapshot = extract_runtime_snapshot(state)
    value = snapshot.get("start")
    return value if isinstance(value, bool) else None


def round_pose(values: Sequence[float] | None, keep_dims: int = 3) -> list[float] | None:
    if values is None:
        return None
    try:
        return [round(float(value), 3) for value in values[:keep_dims]]
    except (TypeError, ValueError):
        return None


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

        from .skill_registry import get_navigation_model_uses
        if start_flag is False or (current_model_use is not None and current_model_use not in get_navigation_model_uses()):
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
        "state": summarize_state(state if isinstance(state, dict) else None),
        "elapsed_sec": arrival_result.get("elapsed_sec"),
    }
