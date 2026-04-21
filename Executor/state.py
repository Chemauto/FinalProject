state = {}

def reset_state():
    state.clear()
    state.update({
        "robot": {"x": 0, "y": 0, "z": 0},
        "box": {"x": 0, "y": 1, "z": 0},
        "last_action": None,
    })
#重置假状态

def fmt_robot():
    robot = state["robot"]
    return f"robot=({_fmt(robot['x'])}, {_fmt(robot['y'])}, {_fmt(robot['z'])})"
#格式化机器人状态

def fmt_box():
    box = state["box"]
    return f"box=({_fmt(box['x'])}, {_fmt(box['y'])}, {_fmt(box['z'])})"
#格式化箱子状态

def _fmt(value):
    value = round(value, 2)
    return int(value) if value == int(value) else value
#显示时去掉多余小数

reset_state()
