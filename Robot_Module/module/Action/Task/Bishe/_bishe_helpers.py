"""Shared constants and geometry helpers for Bishe skills.

Extracted from Excu_Module/runtime.py (constants) and
LLM_Module/parameter_calculator.py (Go2 geometry functions).
"""

from __future__ import annotations

from math import hypot
import re
from typing import Any


# ── Model-use codes ────────────────────────────────────────────────
MODEL_USE_IDLE = 0
MODEL_USE_WALK = 1
MODEL_USE_CLIMB = 2
MODEL_USE_PUSH_BOX = 3
MODEL_USE_NAVIGATION = 4
MODEL_USE_NAV_CLIMB = 5
NAVIGATION_MODEL_USES = {MODEL_USE_NAVIGATION, MODEL_USE_NAV_CLIMB}

# ── Bishe-specific physical constants ──────────────────────────────
CLIMB_LIMIT_METERS = 0.3
DEFAULT_WALK_SPEED_MPS = 0.6
DEFAULT_LATERAL_SPEED_MPS = 0.5
DEFAULT_CLIMB_SPEED_MPS = 0.6
DEFAULT_WAY_SELECT_FORWARD_BIAS_M = 0.2
DEFAULT_WAY_SELECT_DURATION_SEC = 3.0
DEFAULT_WALK_BUFFER_SEC = 0.5
DEFAULT_CLIMB_BASE_DURATION_SEC = 4.0
DEFAULT_CLIMB_EXECUTION_SEC = 12.0
DEFAULT_PUSH_BOX_DURATION_SEC = 15.0
DEFAULT_NAVIGATION_DURATION_SEC = 6.0
DEFAULT_NAVIGATION_TIMEOUT_MARGIN_SEC = 5.0
DEFAULT_POST_PUSH_SETTLE_SEC = 1.2
DEFAULT_PRE_CLIMB_SETTLE_SEC = 0.8
DEFAULT_POST_ALIGN_SETTLE_SEC = 0.6

DEFAULT_LATERAL_DISTANCE = 0.5
WALK_DISTANCE_EPSILON = 0.05


# ── Go2 geometry helpers (from ParameterCalculator) ────────────────

def select_support_box(objects: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Select the shortest movable box from the scene objects."""
    boxes = [
        obj
        for obj in objects
        if obj.get("movable") and str(obj.get("type", "")).lower() == "box"
    ]
    if not boxes:
        return None
    return min(boxes, key=lambda item: object_height(item))


def select_target_platform(objects: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Select the tallest platform from the scene objects."""
    platforms = [
        obj
        for obj in objects
        if str(obj.get("type", "")).lower() == "platform"
    ]
    if not platforms:
        return None
    return max(platforms, key=lambda item: object_height(item))


def object_height(obj: dict[str, Any] | None) -> float:
    """Return the z-size (height) of an object."""
    if not obj:
        return 0.0
    size = obj.get("size") or [0.0, 0.0, 0.0]
    return round(float(size[2]), 3)


def coerce_vec3(value: Any) -> list[float] | None:
    """Coerce a value to [x, y, z] list of floats, or None."""
    if not isinstance(value, list) or len(value) < 3:
        return None
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except (TypeError, ValueError):
        return None


def infer_route_side(obj: dict[str, Any] | None) -> str | None:
    """Infer 'left' or 'right' from object center y-coordinate."""
    if not obj:
        return None
    center = obj.get("center") or [0.0, 0.0, 0.0]
    return "left" if float(center[1]) >= 0 else "right"


def side_label(side: str) -> str:
    """Convert 'left'/'right' to '左侧'/'右侧'."""
    return "左侧" if side == "left" else "右侧"


def build_adjacent_ground_position(
    support_box: dict[str, Any],
    target_platform: dict[str, Any],
) -> list[float]:
    """Compute target [x, y, z] for pushing box adjacent to platform edge."""
    box_center = support_box.get("center") or [0.0, 0.0, 0.0]
    box_size = support_box.get("size") or [0.0, 0.0, 0.0]
    platform_center = target_platform.get("center") or [0.0, 0.0, 0.0]
    platform_size = target_platform.get("size") or [0.0, 0.0, 0.0]

    x = round(float(platform_center[0]) - float(platform_size[0]) / 2 - float(box_size[0]) / 2, 3)
    y = round(float(box_center[1]), 3)
    z = round(float(box_center[2]), 3)
    return [x, y, z]


def estimate_pose_after_way_select(
    current_pose: list[float],
    route_side: str,
    lateral_distance: float,
    anchor_obj: dict[str, Any] | None,
) -> list[float]:
    """Estimate robot pose after a way_select (lateral displacement)."""
    target_y = current_pose[1] + lateral_distance if route_side == "left" else current_pose[1] - lateral_distance
    anchor_center = coerce_vec3((anchor_obj or {}).get("center"))
    if anchor_center:
        if route_side == "left":
            target_y = max(anchor_center[1], target_y)
        else:
            target_y = min(anchor_center[1], target_y)
    return [round(current_pose[0], 3), round(target_y, 3), round(current_pose[2], 3)]


def estimate_pose_after_climb(
    current_pose: list[float],
    stage_obj: dict[str, Any] | None,
    fallback_height: float,
) -> list[float]:
    """Estimate robot pose after a climb."""
    if not stage_obj:
        return [round(current_pose[0], 3), round(current_pose[1], 3), round(current_pose[2] + fallback_height, 3)]

    stage_center = coerce_vec3(stage_obj.get("center"))
    stage_size = coerce_vec3(stage_obj.get("size"))
    if not stage_center:
        return [round(current_pose[0], 3), round(current_pose[1], 3), round(current_pose[2] + fallback_height, 3)]

    top_z = stage_center[2] + stage_size[2] if stage_size else current_pose[2] + fallback_height
    return [round(stage_center[0], 3), round(stage_center[1], 3), round(top_z, 3)]


def infer_walk_route_label(
    task: dict[str, Any],
    current_pose: list[float],
    navigation_goal: list[float],
) -> str:
    """Infer the direction label for a walk task."""
    task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
    if "左侧" in task_text or "left" in task_text:
        return "左侧"
    if "右侧" in task_text or "right" in task_text:
        return "右侧"

    lateral_delta = navigation_goal[1] - current_pose[1]
    if abs(lateral_delta) <= 0.15:
        return "前方"
    return "左侧" if lateral_delta > 0 else "右侧"


def infer_way_select_route_side(
    task: dict[str, Any],
    default_side: str | None,
) -> str | None:
    """Infer route side (left/right) from way_select task text."""
    task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
    if re.search(r"(选择|切换到|到达|沿).{0,8}右侧路线|right", task_text):
        return "right"
    if re.search(r"(选择|切换到|到达|沿).{0,8}左侧路线|left", task_text):
        return "left"
    if "右侧" in task_text and "左侧" not in task_text:
        return "right"
    if "左侧" in task_text and "右侧" not in task_text:
        return "left"
    return default_side


def format_navigation_target(navigation_goal: list[float]) -> str:
    """Format navigation goal as a human-readable target string."""
    x, y, z = navigation_goal
    return f"目标点[{x:g}, {y:g}, {z:g}]"


def format_navigation_goal_command(navigation_goal: list[float]) -> str:
    """Format navigation goal as a command string."""
    x, y, z = navigation_goal
    return f"[{x:g}, {y:g}, {z:g}]"


def extract_requested_distance(task: dict[str, Any]) -> float | None:
    """Extract requested distance in meters from task text."""
    text = f"{task.get('task', '')} {task.get('reason', '')}"
    matches = re.findall(r"(\d+(?:\.\d+)?)\s*米", text)
    if not matches:
        return None
    try:
        return max(float(value) for value in matches)
    except ValueError:
        return None


def planar_distance_to_object(current_pose: list[float], obj: dict[str, Any]) -> float:
    """Compute 2D planar distance from current_pose to object center."""
    center = obj.get("center") or [0.0, 0.0, 0.0]
    return hypot(float(center[0]) - current_pose[0], float(center[1]) - current_pose[1])


def infer_task_side(task: dict[str, Any]) -> str | None:
    """Infer side from task text."""
    task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
    if "左侧" in task_text and "右侧" not in task_text:
        return "left"
    if "右侧" in task_text and "左侧" not in task_text:
        return "right"
    return None


def select_platform_on_side(
    objects: list[dict[str, Any]],
    side: str,
    current_pose: list[float] | None,
) -> dict[str, Any] | None:
    """Select nearest platform on the given side."""
    candidates = [
        obj
        for obj in objects
        if str(obj.get("type", "")).lower() == "platform"
        and infer_route_side(obj) == side
    ]
    if not candidates:
        return None
    if not current_pose:
        return min(candidates, key=lambda item: abs(float((item.get("center") or [0.0, 0.0, 0.0])[0])))
    return min(candidates, key=lambda item: planar_distance_to_object(current_pose, item))


def select_box_on_side(
    objects: list[dict[str, Any]],
    side: str,
    current_pose: list[float] | None,
) -> dict[str, Any] | None:
    """Select nearest movable box on the given side."""
    candidates = [
        obj
        for obj in objects
        if str(obj.get("type", "")).lower() == "box"
        and obj.get("movable")
        and infer_route_side(obj) == side
    ]
    if not candidates:
        return None
    if not current_pose:
        return min(candidates, key=lambda item: abs(float((item.get("center") or [0.0, 0.0, 0.0])[0])))
    return min(candidates, key=lambda item: planar_distance_to_object(current_pose, item))


def match_object_from_task(
    task: dict[str, Any],
    objects: list[dict[str, Any]],
    object_type: str,
) -> dict[str, Any] | None:
    """Match an object referenced by name in task text."""
    task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
    for obj in objects:
        if str(obj.get("type", "")).lower() != object_type:
            continue
        object_id = str(obj.get("id", "")).lower()
        if object_id and object_id in task_text:
            return obj
    return None


def task_targets_box(task: dict[str, Any], support_box: dict[str, Any] | None) -> bool:
    """Check if task text refers to a box."""
    task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
    if support_box and str(support_box.get("id", "")).lower() in task_text:
        return True
    return bool(re.search(r"箱子|box", task_text))


def task_targets_platform(task: dict[str, Any], platform: dict[str, Any] | None) -> bool:
    """Check if task text refers to a platform."""
    task_text = f"{task.get('task', '')} {task.get('reason', '')}".lower()
    if platform and str(platform.get("id", "")).lower() in task_text:
        return True
    return bool(re.search(r"平台|高台|platform", task_text))


def select_platform_for_task(
    task: dict[str, Any],
    objects: list[dict[str, Any]],
    preferred_side: str | None,
    default_platform: dict[str, Any] | None,
    current_pose: list[float] | None,
) -> dict[str, Any] | None:
    """Select the best platform for a specific task."""
    explicit_platform = match_object_from_task(task, objects, "platform")
    if explicit_platform:
        return explicit_platform

    task_side = infer_task_side(task) or preferred_side
    if task_side:
        side_platform = select_platform_on_side(objects, task_side, current_pose)
        if side_platform:
            return side_platform

    return default_platform


def select_box_for_task(
    task: dict[str, Any],
    objects: list[dict[str, Any]],
    preferred_side: str | None,
    default_box: dict[str, Any] | None,
    current_pose: list[float] | None,
) -> dict[str, Any] | None:
    """Select the best box for a specific task."""
    explicit_box = match_object_from_task(task, objects, "box")
    if explicit_box:
        return explicit_box

    task_side = infer_task_side(task) or preferred_side
    if task_side:
        side_box = select_box_on_side(objects, task_side, current_pose)
        if side_box:
            return side_box

    return default_box


def build_bishe_context(
    object_facts: dict[str, Any],
    tasks: list[dict[str, Any]],
) -> dict[str, Any]:
    """Build shared planning context for Bishe parameter calculation.

    This is the context hook registered via ``register_context_hook``.
    It extracts scene geometry and task-level metadata so that individual
    skill ``calculate_parameters()`` methods can share a consistent view.
    """
    objects = object_facts.get("objects") or []
    constraints = object_facts.get("constraints") or {}
    climb_limit = float(constraints.get("max_climb_height_m", CLIMB_LIMIT_METERS))
    navigation_goal = coerce_vec3(object_facts.get("navigation_goal"))
    current_pose = coerce_vec3(object_facts.get("robot_pose")) or [0.0, 0.0, 0.0]

    support_box = select_support_box(objects)
    target_platform = select_target_platform(objects)
    route_side = infer_route_side(support_box or target_platform)

    plan_uses_push_box = any(task.get("function") == "push_box" for task in tasks)
    total_climb_steps = sum(1 for task in tasks if task.get("function") == "climb")

    return {
        "current_pose": current_pose,
        "navigation_goal": navigation_goal,
        "current_route_side": route_side,
        "support_box": support_box,
        "target_platform": target_platform,
        "plan_uses_push_box": plan_uses_push_box,
        "total_climb_steps": total_climb_steps,
        "climb_count": 0,
        "climb_limit": climb_limit,
    }
