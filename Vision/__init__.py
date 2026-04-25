from Vision.vlm import VisionCore


def observe_environment(image_path="", emit=None):
    vision = VisionCore()
    visual_context = vision.describe_structured(image_path or None)
    scene_facts = VisionCore.build_scene_facts(visual_context)
    robot_state = read_robot_state()
    scene_facts = merge_robot_scene_facts(scene_facts, robot_state)
    if emit:
        emit("status", scene_facts.get("summary", "环境观察完成"))
    return {
        "status": "success",
        "image_source": str(vision.last_image_path or ""),
        "visual_context": visual_context,
        "scene_facts": scene_facts,
        "robot_state": robot_state,
    }
#观察环境图片并融合ROS2状态，返回规划用场景事实


def read_robot_state():
    try:
        from Executor.robot_ws import get_robot_state
        state = get_robot_state(timeout_sec=1)
    except Exception as error:
        return {"connected": False, "message": str(error), "scene_objects": []}
    return normalize_robot_state(state)
#读取WebSocket中的ROS2状态，失败时不阻断VLM观察


def normalize_robot_state(state):
    raw = state.get("raw") if isinstance(state.get("raw"), dict) else {}
    scene_objects = state.get("scene_objects") or raw.get("scene_objects") or []
    return {
        "connected": state.get("signal") != "FAILURE",
        "message": state.get("message", ""),
        "robot": state.get("robot") or {},
        "box_world": state.get("box_world") or {},
        "box_relative": state.get("box_relative") or {},
        "scene_objects": scene_objects if isinstance(scene_objects, list) else [],
        "skill_status": raw.get("skill_status") or {},
    }
#标准化机器人状态字段，给observe返回稳定结构


def merge_robot_scene_facts(scene_facts, robot_state):
    objects = robot_state.get("scene_objects") or []
    if not robot_state.get("connected") or not objects:
        return scene_facts

    merged = {
        "summary": summarize_objects(objects),
        "terrain_features": build_terrain_features(objects),
        "interactive_objects": build_interactive_objects(objects),
        "route_options": build_route_options(objects),
        "constraints": dict(scene_facts.get("constraints") or {"max_climb_height_m": VisionCore.MAX_CLIMB_HEIGHT_M}),
        "uncertainties": list(scene_facts.get("uncertainties") or []),
        "visual_summary": scene_facts.get("summary"),
    }
    return merged
#ROS2结构化物体优先，VLM摘要作为视觉补充


def build_terrain_features(objects):
    features = []
    for obj in objects:
        if obj.get("type") != "platform":
            continue
        center = obj.get("center") or [0, 0, 0]
        size = obj.get("size") or [0, 0, 0]
        height = float(size[2]) if len(size) > 2 else 0.0
        side = side_from_center(center)
        features.append({
            "side": side,
            "type": "platform",
            "height_m": height,
            "traversable": 0 < height <= VisionCore.MAX_CLIMB_HEIGHT_M,
            "description": f"{side_label(side)}平台 {obj.get('id', '')}".strip(),
            "object_id": obj.get("id"),
            "center": center,
            "size": size,
        })
    return features
#从scene_objects生成地形事实


def build_interactive_objects(objects):
    items = []
    for obj in objects:
        if obj.get("type") != "box":
            continue
        center = obj.get("center") or [0, 0, 0]
        size = obj.get("size") or [0, 0, 0]
        height = float(size[2]) if len(size) > 2 else 0.0
        side = side_from_center(center)
        items.append({
            "name": obj.get("id", "box"),
            "side": side,
            "height_m": height,
            "movable": bool(obj.get("movable", True)),
            "usable_as_step": height <= VisionCore.MAX_CLIMB_HEIGHT_M,
            "description": f"{side_label(side)}箱子 {obj.get('id', 'box')}",
            "center": center,
            "size": size,
        })
    return items
#从scene_objects生成可交互物体事实


def build_route_options(objects):
    options = {}
    for feature in build_terrain_features(objects):
        side = feature["side"]
        options[side] = {
            "direction": side,
            "status": "clear" if feature["traversable"] else "blocked",
            "reason": feature["description"],
            "requires_climb": feature["traversable"],
        }
    for side in ("left", "right"):
        options.setdefault(side, {
            "direction": side,
            "status": "clear",
            "reason": f"{side_label(side)}无结构化障碍",
            "requires_climb": False,
        })
    return list(options.values())
#根据结构化地形生成左右通行选项


def summarize_objects(objects):
    parts = []
    for obj in objects:
        center = obj.get("center") or [0, 0, 0]
        size = obj.get("size") or [0, 0, 0]
        side = side_label(side_from_center(center))
        height = float(size[2]) if len(size) > 2 else 0.0
        if obj.get("type") == "platform":
            parts.append(f"{side}存在约{height:g}米高台")
        elif obj.get("type") == "box":
            parts.append(f"{side}有可推动箱子 {obj.get('id', 'box')}")
    return "，".join(parts) if parts else "已获取ROS2结构化环境信息"
#生成融合后的环境摘要


def side_from_center(center):
    try:
        return "left" if float(center[1]) >= 0 else "right"
    except (TypeError, ValueError, IndexError):
        return "unknown"
#根据y坐标推断物体在机器人左右侧


def side_label(side):
    return "左侧" if side == "left" else "右侧" if side == "right" else "当前侧"
#把英文侧向转成中文说明
