state = {}


def reset_state():
    state.clear()
    state.update({
        "robot": {"x": 0, "y": 0, "z": 0, "yaw": 0},
        "box_world": {"x": 0, "y": 1, "z": 0},
        "box_relative": {"x": 0, "y": 1, "z": 0},
        "current_skill": None,
        "model_use": None,
        "start": None,
        "last_action": None,
        "latest_feedback": None,
        "raw": {},
    })
#重置执行状态，启动时和/reset后使用


def update_latest_state(payload):
    if not isinstance(payload, dict):
        return state
    body = payload.get("state") if isinstance(payload.get("state"), dict) else payload
    state["raw"] = dict(body)
    if isinstance(body.get("robot"), dict):
        state["robot"] = _normalize_robot(body["robot"])
    if isinstance(body.get("box_world"), dict):
        state["box_world"] = _normalize_position(body["box_world"])
    elif isinstance(body.get("box"), dict):
        state["box_world"] = _normalize_position(body["box"])
    if isinstance(body.get("box_relative"), dict):
        state["box_relative"] = _normalize_position(body["box_relative"])
    if "current_skill" in body:
        state["current_skill"] = body.get("current_skill")
    elif "skill" in body:
        state["current_skill"] = body.get("skill")
    if "model_use" in body:
        state["model_use"] = body.get("model_use")
    if "start" in body:
        state["start"] = body.get("start")
    return state
#接收服务器state消息，更新机器人、箱子、技能和运行标记


def update_latest_feedback(payload):
    if not isinstance(payload, dict):
        return state
    state["latest_feedback"] = dict(payload)
    skill = payload.get("skill") or payload.get("name") or "unknown"
    signal = payload.get("signal") or payload.get("status") or "UNKNOWN"
    state["last_action"] = f"{skill}:{signal}"
    return state
#接收服务器feedback消息，记录最近一次执行结果


def fmt_robot():
    robot = state.get("robot") or {"x": 0, "y": 0, "z": 0, "yaw": 0}
    return (
        f"robot=({_fmt(robot.get('x', 0))}, {_fmt(robot.get('y', 0))}, {_fmt(robot.get('z', 0))}), "
        f"yaw={_fmt(robot.get('yaw', 0))}"
    )
#格式化机器人坐标


def fmt_box_world():
    return _fmt_position("box_world")
#格式化箱子世界坐标


def fmt_box_relative():
    return _fmt_position("box_relative")
#格式化箱子相对机器人坐标


def format_latest_state():
    parts = [fmt_robot(), fmt_box_world(), fmt_box_relative()]
    current_skill = state.get("current_skill")
    if current_skill:
        parts.append(f"current_skill={current_skill}")
    if state.get("model_use") is not None:
        parts.append(f"model_use={state.get('model_use')}")
    if state.get("start") is not None:
        parts.append(f"start={state.get('start')}")
    return ", ".join(parts)
#格式化最新状态，给TUI和LLM摘要使用


def format_feedback():
    feedback = state.get("latest_feedback") or {}
    if not feedback:
        return "no feedback"
    skill = feedback.get("skill") or "unknown"
    signal = feedback.get("signal") or feedback.get("status") or "UNKNOWN"
    message = feedback.get("message") or ""
    return f"{skill} {signal}: {message}".strip()
#格式化最近一次执行反馈


def _normalize_position(payload):
    return {
        "x": _coerce_number(payload.get("x", 0)),
        "y": _coerce_number(payload.get("y", 0)),
        "z": _coerce_number(payload.get("z", 0)),
    }
#把服务器位置标准化为x/y/z数字


def _normalize_robot(payload):
    robot = _normalize_position(payload)
    robot["yaw"] = _coerce_number(payload.get("yaw", 0))
    return robot
#把机器人状态标准化为x/y/z/yaw数字


def _fmt_position(key):
    pos = state.get(key) or {"x": 0, "y": 0, "z": 0}
    return f"{key}=({_fmt(pos.get('x', 0))}, {_fmt(pos.get('y', 0))}, {_fmt(pos.get('z', 0))})"
#按robot或box格式化坐标


def _coerce_number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0
#把输入转成数字，失败时回退0


def _fmt(value):
    value = round(_coerce_number(value), 2)
    return int(value) if value == int(value) else value
#显示时去掉无意义的小数


reset_state()
