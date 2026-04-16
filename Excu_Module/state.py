from __future__ import annotations

import asyncio
import math
import time
from typing import Any, Sequence

from Comm_Module import get_state

from .runtime import (
    DEFAULT_STATUS_POLL_SEC,
    DEFAULT_STATUS_READY_TIMEOUT_SEC,
    NAVIGATION_MODEL_USES,
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
        alignment = runtime.get("envtest_alignment")
        if isinstance(alignment, dict):
            compact_snapshot.update(
                {
                    "platform_1": alignment.get("platform_1"),
                    "platform_2": alignment.get("platform_2"),
                    "box": alignment.get("box"),
                }
            )
        if compact_snapshot:
            return compact_snapshot
    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    action_result = observation.get("action_result") or {}
    fallback_snapshot = {
        "goal": environment.get("goal"),
        "skill": action_result.get("skill"),
        "model_use": action_result.get("model_use"),
        "start": action_result.get("start"),
    }
    alignment = environment.get("envtest_alignment")
    if isinstance(alignment, dict):
        fallback_snapshot.update(
            {
                "platform_1": alignment.get("platform_1"),
                "platform_2": alignment.get("platform_2"),
                "box": alignment.get("box"),
            }
        )
    return fallback_snapshot


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


def extract_box_pose(state: dict[str, Any] | None) -> list[float] | None:
    """Extract box center position [x, y, z] from state snapshot or runtime_objects."""
    if not isinstance(state, dict):
        return None

    # Priority 1: runtime.snapshot.box.position (live position from status file)
    snapshot = extract_runtime_snapshot(state)
    box_asset = snapshot.get("box")
    if isinstance(box_asset, dict):
        position = box_asset.get("position")
        if isinstance(position, (list, tuple)) and len(position) >= 3:
            return round_pose(position)

    # Priority 2: runtime_objects with type=box / movable=True
    runtime = state.get("runtime") or {}
    runtime_objects = runtime.get("scene_objects")
    if isinstance(runtime_objects, list):
        for obj in runtime_objects:
            if not isinstance(obj, dict):
                continue
            obj_type = str(obj.get("type", "")).lower()
            movable = obj.get("movable")
            if obj_type == "box" or movable is True:
                center = obj.get("center")
                if isinstance(center, (list, tuple)) and len(center) >= 3:
                    return round_pose(center)

    # Priority 3: observation.environment.obstacles
    observation = state.get("observation") or {}
    environment = observation.get("environment") or {}
    obstacles = environment.get("obstacles")
    if isinstance(obstacles, list):
        for obj in obstacles:
            if not isinstance(obj, dict):
                continue
            obj_type = str(obj.get("type", "")).lower()
            movable = obj.get("movable")
            if obj_type == "box" or movable is True:
                center = obj.get("center") or obj.get("position")
                if isinstance(center, (list, tuple)) and len(center) >= 3:
                    return round_pose(center)

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
                    "ok": False,
                    "reason": "未获取到实时状态",
                    "state": latest_state,
                    "dist_xy": latest_dist_xy,
                    "pose_delta_xy": latest_pose_delta,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
            continue

        latest_state = state
        timestamp = extract_status_timestamp(state)
        if timestamp is not None and (time.time() - timestamp) > stale_sec:
            if time.time() >= ready_deadline:
                return {
                    "ok": False,
                    "reason": f"实时状态超过 {stale_sec:.1f}s 未更新",
                    "state": latest_state,
                    "dist_xy": latest_dist_xy,
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
                    "state": latest_state,
                    "dist_xy": latest_dist_xy,
                    "pose_delta_xy": latest_pose_delta,
                    "stable_hits": stable_hits,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
        else:
            stable_hits = 0

        if start_flag is False or (current_model_use is not None and current_model_use not in NAVIGATION_MODEL_USES):
            if latest_dist_xy is None or latest_dist_xy > arrival_tol_m:
                return {
                    "ok": False,
                    "reason": (
                        "导航在到达目标前已停止"
                        if latest_dist_xy is None
                        else f"导航在到达目标前已停止 (dist_xy={latest_dist_xy:.3f}m)"
                    ),
                    "state": latest_state,
                    "dist_xy": latest_dist_xy,
                    "pose_delta_xy": latest_pose_delta,
                    "stable_hits": stable_hits,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }

    return {
        "ok": False,
        "reason": (
            f"导航超时，{timeout_sec:.1f}s 内未到达目标"
            if latest_dist_xy is None
            else f"导航超时，{timeout_sec:.1f}s 内未到达目标 (dist_xy={latest_dist_xy:.3f}m)"
        ),
        "state": latest_state,
        "dist_xy": latest_dist_xy,
        "pose_delta_xy": latest_pose_delta,
        "stable_hits": stable_hits,
        "elapsed_sec": round(time.time() - start_time, 3),
    }


def _check_walk_done(
    state: dict[str, Any],
    before_pose: list[float],
    parameters: dict[str, Any],
) -> dict[str, Any]:
    """Check if walk has completed by measuring robot displacement."""
    current_pose = extract_robot_pose(state)
    if current_pose is None:
        return {"done": False}

    requested_distance = max(0.0, float(parameters.get("distance") or 0.0))
    minimum_distance = minimum_verified_motion(
        requested_distance,
        ratio_env="FINALPROJECT_WALK_VERIFY_RATIO",
        ratio_default=0.75,
        abs_tol_env="FINALPROJECT_WALK_VERIFY_ABS_TOL_M",
        abs_tol_default=0.15,
    )

    delta_x = current_pose[0] - before_pose[0]
    delta_y = current_pose[1] - before_pose[1]
    planar_distance = math.hypot(delta_x, delta_y)

    route_side = str(parameters.get("route_side") or "前方").strip().lower()
    axis = "x"
    direction_sign = 1.0
    if any(token in route_side for token in ("left", "左")):
        axis = "y"
    elif any(token in route_side for token in ("right", "右")):
        axis = "y"
        direction_sign = -1.0

    signed_progress = (delta_x if axis == "x" else delta_y) * direction_sign
    minimum_direction_progress = max(0.05, minimum_distance * 0.6)
    done = planar_distance >= minimum_distance and signed_progress >= minimum_direction_progress

    return {
        "done": done,
        "planar_distance": round(planar_distance, 3),
        "minimum_distance": round(minimum_distance, 3),
        "current_pose": current_pose,
    }


def _check_climb_done(
    state: dict[str, Any],
    before_pose: list[float],
    parameters: dict[str, Any],
) -> dict[str, Any]:
    """Check if climb has completed by measuring robot height gain."""
    current_pose = extract_robot_pose(state)
    if current_pose is None:
        return {"done": False}

    requested_height = max(0.0, float(parameters.get("height") or 0.0))
    minimum_height = minimum_verified_motion(
        requested_height,
        ratio_env="FINALPROJECT_CLIMB_VERIFY_RATIO",
        ratio_default=0.6,
        abs_tol_env="FINALPROJECT_CLIMB_VERIFY_ABS_TOL_M",
        abs_tol_default=0.08,
    )

    delta_z = current_pose[2] - before_pose[2]
    done = delta_z >= minimum_height

    return {
        "done": done,
        "vertical_progress": round(delta_z, 3),
        "minimum_height": round(minimum_height, 3),
        "current_pose": current_pose,
    }


def _estimate_push_expected_distance(
    before_box_pose: list[float],
    state: dict[str, Any],
    parameters: dict[str, Any],
) -> float:
    """Estimate expected push distance from box start to nearest platform edge."""
    goal_command = parameters.get("goal_command")
    if isinstance(goal_command, list) and len(goal_command) >= 2:
        return math.hypot(
            float(goal_command[0]) - before_box_pose[0],
            float(goal_command[1]) - before_box_pose[1],
        )

    # Auto mode: estimate from box to nearest platform edge
    scene_objects = extract_scene_objects(state)
    min_dist = float("inf")
    for obj in scene_objects:
        obj_type = str(obj.get("type", "")).lower()
        if obj_type != "platform":
            continue
        center = obj.get("center")
        size = obj.get("size")
        if not isinstance(center, list) or len(center) < 2:
            continue
        if not isinstance(size, list) or len(size) < 2:
            continue
        try:
            # Distance from box to nearest edge of platform
            cx, cy = float(center[0]), float(center[1])
            sx, sy = float(size[0]), float(size[1])
            nearest_x = max(cx - sx / 2.0, min(before_box_pose[0], cx + sx / 2.0))
            nearest_y = max(cy - sy / 2.0, min(before_box_pose[1], cy + sy / 2.0))
            dist = math.hypot(before_box_pose[0] - nearest_x, before_box_pose[1] - nearest_y)
            min_dist = min(min_dist, dist)
        except (TypeError, ValueError):
            continue

    return min_dist if min_dist < float("inf") else 0.5


def _check_push_box_done(
    state: dict[str, Any],
    before_box_pose: list[float],
    parameters: dict[str, Any],
) -> dict[str, Any]:
    """Check if push_box has completed.

    Success logic:
      - If goal_command is a concrete coordinate: box within arrival_tolerance (0.1m) of goal.
      - If goal_command is auto: box displacement >= minimum_verified_motion (computed from
        distance to nearest platform, same ratio-based approach as walk/climb).
    """
    current_box = extract_box_pose(state)
    if current_box is None:
        return {"done": False}

    box_delta_x = current_box[0] - before_box_pose[0]
    box_delta_y = current_box[1] - before_box_pose[1]
    box_planar_distance = math.hypot(box_delta_x, box_delta_y)
    arrival_tol = abs(read_env_float("FINALPROJECT_PUSH_BOX_ARRIVAL_TOL_M", 0.1))

    goal_command = parameters.get("goal_command")
    box_goal_dist_xy: float | None = None

    if isinstance(goal_command, list) and len(goal_command) >= 2:
        # Concrete target: box within arrival_tolerance of goal
        box_goal_dist_xy = math.hypot(
            current_box[0] - float(goal_command[0]),
            current_box[1] - float(goal_command[1]),
        )
        done = box_goal_dist_xy <= arrival_tol
    else:
        # Auto: require displacement >= minimum_verified_motion based on expected distance
        expected_distance = _estimate_push_expected_distance(before_box_pose, state, parameters)
        minimum_box_motion = minimum_verified_motion(
            expected_distance,
            ratio_env="FINALPROJECT_PUSH_BOX_VERIFY_RATIO",
            ratio_default=0.75,
            abs_tol_env="FINALPROJECT_PUSH_BOX_VERIFY_ABS_TOL_M",
            abs_tol_default=0.15,
        )
        done = box_planar_distance >= minimum_box_motion

    return {
        "done": done,
        "box_planar_distance": round(box_planar_distance, 3),
        "arrival_tol": round(arrival_tol, 3),
        "current_box_pose": current_box,
        "box_goal_dist_xy": round(box_goal_dist_xy, 3) if box_goal_dist_xy is not None else None,
    }


def _check_way_select_done(
    state: dict[str, Any],
    before_pose: list[float],
    parameters: dict[str, Any],
) -> dict[str, Any]:
    """Check if way_select has completed by measuring lateral displacement."""
    current_pose = extract_robot_pose(state)
    if current_pose is None:
        return {"done": False}

    requested_distance = max(0.0, float(parameters.get("lateral_distance") or 0.0))
    minimum_distance = minimum_verified_motion(
        requested_distance,
        ratio_env="FINALPROJECT_WAY_SELECT_VERIFY_RATIO",
        ratio_default=0.7,
        abs_tol_env="FINALPROJECT_WAY_SELECT_VERIFY_ABS_TOL_M",
        abs_tol_default=0.12,
    )

    direction = str(parameters.get("direction") or "").strip().lower()
    direction_sign = 1.0 if direction == "left" else -1.0

    delta_y = current_pose[1] - before_pose[1]
    lateral_progress = delta_y * direction_sign
    planar_distance = math.hypot(
        current_pose[0] - before_pose[0],
        current_pose[1] - before_pose[1],
    )
    minimum_lateral_progress = max(0.05, minimum_distance * 0.8)
    done = planar_distance >= minimum_distance and lateral_progress >= minimum_lateral_progress

    return {
        "done": done,
        "lateral_progress": round(lateral_progress, 3),
        "planar_distance": round(planar_distance, 3),
        "minimum_distance": round(minimum_distance, 3),
        "current_pose": current_pose,
    }


async def wait_for_skill_completion(
    *,
    skill_name: str,
    parameters: dict[str, Any],
    before_state: dict[str, Any],
    task_type: str | None,
    timeout_sec: float,
) -> dict[str, Any]:
    """Poll every 0.5s until the skill's completion condition is met or timeout.

    Returns a dict with ok/reason/state/elapsed_sec plus skill-specific metrics.
    """
    poll_sec = max(0.1, read_env_float("FINALPROJECT_STATUS_POLL_SEC", DEFAULT_STATUS_POLL_SEC))
    stale_sec = abs(read_env_float("FINALPROJECT_STATUS_STALE_SEC", DEFAULT_STATE_STALE_SEC))
    ready_timeout_sec = max(
        poll_sec,
        read_env_float("FINALPROJECT_STATUS_READY_TIMEOUT_SEC", DEFAULT_STATUS_READY_TIMEOUT_SEC),
    )

    start_time = time.time()
    deadline = start_time + timeout_sec
    ready_deadline = start_time + ready_timeout_sec
    latest_state: dict[str, Any] | None = None

    before_pose = extract_robot_pose(before_state)
    before_box = extract_box_pose(before_state)

    while time.time() < deadline:
        await asyncio.sleep(poll_sec)
        state = load_live_state(task_type=task_type)

        if not state.get("connected"):
            if time.time() >= ready_deadline:
                return {
                    "ok": False,
                    "reason": "未获取到实时状态",
                    "state": latest_state,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
            continue

        latest_state = state
        timestamp = extract_status_timestamp(state)
        if timestamp is not None and (time.time() - timestamp) > stale_sec:
            if time.time() >= ready_deadline:
                return {
                    "ok": False,
                    "reason": f"实时状态超过 {stale_sec:.1f}s 未更新",
                    "state": latest_state,
                    "elapsed_sec": round(time.time() - start_time, 3),
                }
            continue

        # Skill-specific completion check
        if skill_name == "walk" and before_pose is not None:
            result = _check_walk_done(state, before_pose, parameters)
        elif skill_name in {"climb", "climb_align"} and before_pose is not None:
            result = _check_climb_done(state, before_pose, parameters)
        elif skill_name == "push_box" and before_box is not None:
            result = _check_push_box_done(state, before_box, parameters)
        elif skill_name == "way_select" and before_pose is not None:
            result = _check_way_select_done(state, before_pose, parameters)
        else:
            continue

        if result.get("done"):
            return {
                "ok": True,
                "reason": f"{skill_name} 执行完成",
                "state": latest_state,
                "elapsed_sec": round(time.time() - start_time, 3),
                **result,
            }

    # Timeout
    return {
        "ok": False,
        "reason": f"{skill_name} 超时，{timeout_sec:.1f}s 内未完成",
        "state": latest_state,
        "elapsed_sec": round(time.time() - start_time, 3),
    }


def minimum_verified_motion(
    requested_distance: float,
    *,
    ratio_env: str,
    ratio_default: float,
    abs_tol_env: str,
    abs_tol_default: float,
) -> float:
    requested_distance = max(0.0, float(requested_distance))
    if requested_distance <= 0:
        return 0.05
    ratio = min(1.0, max(0.1, read_env_float(ratio_env, ratio_default)))
    abs_tol = abs(read_env_float(abs_tol_env, abs_tol_default))
    return min(requested_distance, max(0.05, max(requested_distance * ratio, requested_distance - abs_tol)))


def build_missing_validation(
    skill_name: str,
    reason: str,
    *,
    before_state: dict[str, Any] | None = None,
    after_state: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "verified": False,
        "meets_requirements": False,
        "source": "comm_state",
        "summary": f"{skill_name} 无法完成真实状态校验: {reason}",
        "before_state": summarize_state(before_state),
        "after_state": summarize_state(after_state),
    }


def build_motion_validation_base(
    skill_name: str,
    before_state: dict[str, Any] | None,
    after_state: dict[str, Any] | None,
) -> tuple[dict[str, Any], list[float], list[float]] | tuple[dict[str, Any], None, None]:
    if before_state is None:
        return build_missing_validation(skill_name, "缺少执行前实时状态", after_state=after_state), None, None
    if after_state is None:
        return build_missing_validation(skill_name, "缺少执行后实时状态", before_state=before_state), None, None

    before_pose = extract_robot_pose(before_state)
    after_pose = extract_robot_pose(after_state)
    if before_pose is None or after_pose is None:
        return (
            build_missing_validation(
                skill_name,
                "实时状态中缺少 robot_pose",
                before_state=before_state,
                after_state=after_state,
            ),
            None,
            None,
        )

    return {
        "verified": True,
        "source": "comm_state",
        "before_state": summarize_state(before_state),
        "after_state": summarize_state(after_state),
        "before_robot_pose": before_pose,
        "after_robot_pose": after_pose,
    }, before_pose, after_pose


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


def validate_walk_execution(
    parameters: dict[str, Any],
    before_state: dict[str, Any] | None,
    after_state: dict[str, Any] | None,
) -> dict[str, Any]:
    validation, before_pose, after_pose = build_motion_validation_base("walk", before_state, after_state)
    if before_pose is None or after_pose is None:
        return validation

    requested_distance = max(0.0, float(parameters.get("distance") or 0.0))
    minimum_distance = minimum_verified_motion(
        requested_distance,
        ratio_env="FINALPROJECT_WALK_VERIFY_RATIO",
        ratio_default=0.75,
        abs_tol_env="FINALPROJECT_WALK_VERIFY_ABS_TOL_M",
        abs_tol_default=0.15,
    )
    delta_x = after_pose[0] - before_pose[0]
    delta_y = after_pose[1] - before_pose[1]
    delta_z = after_pose[2] - before_pose[2]
    planar_distance = math.hypot(delta_x, delta_y)

    route_side = str(parameters.get("route_side") or "前方").strip().lower()
    axis = "x"
    direction_sign = 1.0
    direction_label = "前方"
    if any(token in route_side for token in ("left", "左")):
        axis = "y"
        direction_label = "左侧"
    elif any(token in route_side for token in ("right", "右")):
        axis = "y"
        direction_sign = -1.0
        direction_label = "右侧"

    signed_progress = (delta_x if axis == "x" else delta_y) * direction_sign
    minimum_direction_progress = max(0.05, minimum_distance * 0.6)
    meets_requirements = planar_distance >= minimum_distance and signed_progress >= minimum_direction_progress
    validation.update(
        {
            "meets_requirements": meets_requirements,
            "delta_pose": round_pose([delta_x, delta_y, delta_z]),
            "planar_distance": round(planar_distance, 3),
            "required_distance": round(requested_distance, 3),
            "minimum_required_distance": round(minimum_distance, 3),
            "direction_axis": axis,
            "direction_label": direction_label,
            "direction_progress": round(signed_progress, 3),
            "summary": (
                f"walk 真实状态校验{'通过' if meets_requirements else '失败'}: "
                f"机器人从 {validation['before_robot_pose']} 移动到 {validation['after_robot_pose']}，"
                f"平面位移 {planar_distance:.3f}m，{direction_label}方向有效位移 {signed_progress:.3f}m，"
                f"要求至少 {minimum_distance:.3f}m。"
            ),
        }
    )
    return validation


def validate_way_select_execution(
    parameters: dict[str, Any],
    before_state: dict[str, Any] | None,
    after_state: dict[str, Any] | None,
) -> dict[str, Any]:
    validation, before_pose, after_pose = build_motion_validation_base("way_select", before_state, after_state)
    if before_pose is None or after_pose is None:
        return validation

    requested_distance = max(0.0, float(parameters.get("lateral_distance") or 0.0))
    minimum_distance = minimum_verified_motion(
        requested_distance,
        ratio_env="FINALPROJECT_WAY_SELECT_VERIFY_RATIO",
        ratio_default=0.7,
        abs_tol_env="FINALPROJECT_WAY_SELECT_VERIFY_ABS_TOL_M",
        abs_tol_default=0.12,
    )
    direction = str(parameters.get("direction") or "").strip().lower()
    direction_sign = 1.0 if direction == "left" else -1.0
    direction_label = "左侧" if direction_sign > 0 else "右侧"
    delta_x = after_pose[0] - before_pose[0]
    delta_y = after_pose[1] - before_pose[1]
    delta_z = after_pose[2] - before_pose[2]
    planar_distance = math.hypot(delta_x, delta_y)
    lateral_progress = delta_y * direction_sign
    minimum_lateral_progress = max(0.05, minimum_distance * 0.8)
    meets_requirements = planar_distance >= minimum_distance and lateral_progress >= minimum_lateral_progress
    validation.update(
        {
            "meets_requirements": meets_requirements,
            "delta_pose": round_pose([delta_x, delta_y, delta_z]),
            "planar_distance": round(planar_distance, 3),
            "required_lateral_distance": round(requested_distance, 3),
            "minimum_required_distance": round(minimum_distance, 3),
            "lateral_progress": round(lateral_progress, 3),
            "direction_label": direction_label,
            "summary": (
                f"way_select 真实状态校验{'通过' if meets_requirements else '失败'}: "
                f"机器人从 {validation['before_robot_pose']} 移动到 {validation['after_robot_pose']}，"
                f"平面位移 {planar_distance:.3f}m，{direction_label}方向位移 {lateral_progress:.3f}m，"
                f"要求至少 {minimum_distance:.3f}m。"
            ),
        }
    )
    return validation


def validate_climb_execution(
    parameters: dict[str, Any],
    before_state: dict[str, Any] | None,
    after_state: dict[str, Any] | None,
) -> dict[str, Any]:
    validation, before_pose, after_pose = build_motion_validation_base("climb", before_state, after_state)
    if before_pose is None or after_pose is None:
        return validation

    requested_height = max(0.0, float(parameters.get("height") or 0.0))
    minimum_height = minimum_verified_motion(
        requested_height,
        ratio_env="FINALPROJECT_CLIMB_VERIFY_RATIO",
        ratio_default=0.6,
        abs_tol_env="FINALPROJECT_CLIMB_VERIFY_ABS_TOL_M",
        abs_tol_default=0.08,
    )
    delta_x = after_pose[0] - before_pose[0]
    delta_y = after_pose[1] - before_pose[1]
    delta_z = after_pose[2] - before_pose[2]
    planar_distance = math.hypot(delta_x, delta_y)
    meets_requirements = delta_z >= minimum_height
    validation.update(
        {
            "meets_requirements": meets_requirements,
            "delta_pose": round_pose([delta_x, delta_y, delta_z]),
            "planar_distance": round(planar_distance, 3),
            "required_height": round(requested_height, 3),
            "minimum_required_height": round(minimum_height, 3),
            "vertical_progress": round(delta_z, 3),
            "summary": (
                f"climb 真实状态校验{'通过' if meets_requirements else '失败'}: "
                f"机器人从 {validation['before_robot_pose']} 移动到 {validation['after_robot_pose']}，"
                f"高度变化 {delta_z:.3f}m，平面位移 {planar_distance:.3f}m，"
                f"要求至少抬升 {minimum_height:.3f}m。"
            ),
        }
    )
    return validation


def validate_push_box_execution(
    parameters: dict[str, Any],
    before_state: dict[str, Any] | None,
    after_state: dict[str, Any] | None,
) -> dict[str, Any]:
    # --- Extract box positions (primary validation target) ---
    before_box = extract_box_pose(before_state)
    after_box = extract_box_pose(after_state)

    if before_box is None or after_box is None:
        # Cannot validate via box pose; fall back to robot motion only
        validation, before_pose, after_pose = build_motion_validation_base("push_box", before_state, after_state)
        if before_pose is None or after_pose is None:
            return validation

        delta_x = after_pose[0] - before_pose[0]
        delta_y = after_pose[1] - before_pose[1]
        delta_z = after_pose[2] - before_pose[2]
        planar_distance = math.hypot(delta_x, delta_y)
        minimum_robot_motion = abs(read_env_float("FINALPROJECT_PUSH_BOX_VERIFY_MIN_ROBOT_MOVE_M", 0.3))
        meets_requirements = planar_distance >= minimum_robot_motion
        validation.update(
            {
                "meets_requirements": meets_requirements,
                "validation_mode": "robot_fallback",
                "delta_pose": round_pose([delta_x, delta_y, delta_z]),
                "planar_distance": round(planar_distance, 3),
                "minimum_robot_motion": round(minimum_robot_motion, 3),
                "summary": (
                    "push_box 校验(后备-机器人位移): "
                    f"无法获取箱子位置，机器人从 {validation['before_robot_pose']} 移动到 "
                    f"{validation['after_robot_pose']}，平面位移 {planar_distance:.3f}m，"
                    f"最小运动要求 {minimum_robot_motion:.3f}m。"
                ),
            }
        )
        return validation

    # --- Primary: validate by box position ---
    box_delta_x = after_box[0] - before_box[0]
    box_delta_y = after_box[1] - before_box[1]
    box_delta_z = after_box[2] - before_box[2]
    box_planar_distance = math.hypot(box_delta_x, box_delta_y)
    arrival_tol = abs(read_env_float("FINALPROJECT_PUSH_BOX_ARRIVAL_TOL_M", 0.1))

    goal_command = parameters.get("goal_command")
    box_goal_dist_xy: float | None = None
    check_mode = ""

    if isinstance(goal_command, list) and len(goal_command) >= 2:
        # Concrete target: box within arrival_tolerance of goal
        box_goal_dist_xy = math.hypot(after_box[0] - float(goal_command[0]), after_box[1] - float(goal_command[1]))
        meets_requirements = box_goal_dist_xy <= arrival_tol
        check_mode = f"目标到达校验: 箱子距目标 {box_goal_dist_xy:.3f}m，容许误差 {arrival_tol:.3f}m"
    else:
        # Auto: displacement >= minimum_verified_motion based on expected distance
        expected_distance = _estimate_push_expected_distance(before_box, before_state, parameters)
        minimum_box_motion = minimum_verified_motion(
            expected_distance,
            ratio_env="FINALPROJECT_PUSH_BOX_VERIFY_RATIO",
            ratio_default=0.75,
            abs_tol_env="FINALPROJECT_PUSH_BOX_VERIFY_ABS_TOL_M",
            abs_tol_default=0.15,
        )
        meets_requirements = box_planar_distance >= minimum_box_motion
        check_mode = f"位移校验: 箱子位移 {box_planar_distance:.3f}m，预期距离 {expected_distance:.3f}m，最小要求 {minimum_box_motion:.3f}m"

    # Collect robot motion as auxiliary info
    before_robot = extract_robot_pose(before_state)
    after_robot = extract_robot_pose(after_state)
    robot_info = ""
    if before_robot is not None and after_robot is not None:
        r_delta_x = after_robot[0] - before_robot[0]
        r_delta_y = after_robot[1] - before_robot[1]
        robot_planar = math.hypot(r_delta_x, r_delta_y)
        robot_info = f" 机器人从 {before_robot} 移动到 {after_robot}，平面位移 {robot_planar:.3f}m。"

    summary = (
        f"push_box 真实状态校验{'通过' if meets_requirements else '失败'}: "
        f"箱子从 {before_box} 移动到 {after_box}，{check_mode}。"
    )
    summary += robot_info

    validation: dict[str, Any] = {
        "verified": True,
        "source": "comm_state",
        "validation_mode": "box_primary",
        "before_state": summarize_state(before_state),
        "after_state": summarize_state(after_state),
        "before_robot_pose": before_robot,
        "after_robot_pose": after_robot,
        "before_box_pose": before_box,
        "after_box_pose": after_box,
    }
    validation.update(
        {
            "meets_requirements": meets_requirements,
            "box_delta_pose": round_pose([box_delta_x, box_delta_y, box_delta_z]),
            "box_planar_distance": round(box_planar_distance, 3),
            "arrival_tol": round(arrival_tol, 3),
            "box_goal_dist_xy": round(box_goal_dist_xy, 3) if box_goal_dist_xy is not None else None,
            "summary": summary,
        }
    )
    return validation


def validate_live_execution(
    skill_name: str,
    parameters: dict[str, Any],
    before_state: dict[str, Any] | None,
    after_state: dict[str, Any] | None,
) -> dict[str, Any]:
    if skill_name == "walk":
        return validate_walk_execution(parameters, before_state, after_state)
    if skill_name == "way_select":
        return validate_way_select_execution(parameters, before_state, after_state)
    if skill_name in {"climb", "climb_align"}:
        return validate_climb_execution(parameters, before_state, after_state)
    if skill_name == "push_box":
        return validate_push_box_execution(parameters, before_state, after_state)
    return build_missing_validation(
        skill_name,
        "当前技能未定义通用状态校验规则",
        before_state=before_state,
        after_state=after_state,
    )
