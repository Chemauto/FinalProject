import math
import time
from Executor.state import fmt_box, fmt_robot, state

DT = 0.2
NAV_SPEED = 0.5
PUSH_SPEED = 0.2
CLIMB_SPEED = 0.1

def _sleep(enabled):
    if enabled:
        time.sleep(DT)
#控制是否真实等待

def _emit(emit, name, t):
    if emit:
        emit("status", f"{name} t={t:.1f}s {fmt_robot()}, {fmt_box()}")
#输出状态

def Nav(x, y, z, emit=None, speed=NAV_SPEED, sleep=True):
    t = 0
    while True:
        robot = state["robot"]
        dx, dy, dz = x - robot["x"], y - robot["y"], z - robot["z"]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance == 0:
            break
        step = min(speed * DT, distance)
        state["robot"] = {
            "x": round(robot["x"] + dx / distance * step, 2),
            "y": round(robot["y"] + dy / distance * step, 2),
            "z": round(robot["z"] + dz / distance * step, 2),
        }
        t = round(t + DT, 1)
        _emit(emit, "Nav", t)
        _sleep(sleep)
    state["last_action"] = f"Nav({x}, {y}, {z})"
    return fmt_robot()
#以默认0.5m/s直线导航

def walk(direction, v, duration=1.0, emit=None, sleep=True):
    axis = {"front": ("x", 1), "back": ("x", -1), "left": ("y", 1), "right": ("y", -1)}
    if direction not in axis:
        return f"unknown direction: {direction}"
    name, sign = axis[direction]
    t = 0
    while t < duration:
        state["robot"][name] = round(state["robot"][name] + sign * v * DT, 2)
        t = round(t + DT, 1)
        _emit(emit, f"walk({direction})", t)
        _sleep(sleep)
    state["last_action"] = f"walk({direction}, {v})"
    return fmt_robot()
#按速度v移动，默认持续1秒

def Push(x, y, z, emit=None, speed=PUSH_SPEED, sleep=True):
    t = 0
    while True:
        box = state["box"]
        dx, dy, dz = x - box["x"], y - box["y"], z - box["z"]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance == 0:
            break
        step = min(speed * DT, distance)
        state["box"] = {
            "x": round(box["x"] + dx / distance * step, 2),
            "y": round(box["y"] + dy / distance * step, 2),
            "z": round(box["z"] + dz / distance * step, 2),
        }
        state["robot"] = {"x": state["box"]["x"], "y": round(state["box"]["y"] - 1, 2), "z": state["box"]["z"]}
        t = round(t + DT, 1)
        _emit(emit, "Push", t)
        _sleep(sleep)
    state["last_action"] = f"Push({x}, {y}, {z})"
    return f"{fmt_box()}, {fmt_robot()}"
#以0.2m/s把箱子推到目标位置

def climb(height, emit=None, speed=CLIMB_SPEED, sleep=True):
    if height > 0.3:
        message = "failed: climb height > 0.3"
        if emit:
            emit("error", message)
        return message
    t = 0
    target = round(state["robot"]["z"] + height, 2)
    while state["robot"]["z"] < target:
        state["robot"]["z"] = round(min(target, state["robot"]["z"] + speed * DT), 2)
        t = round(t + DT, 1)
        _emit(emit, "climb", t)
        _sleep(sleep)
    state["last_action"] = f"climb({height})"
    return fmt_robot()
#以0.1m/s攀爬，最高0.3m
