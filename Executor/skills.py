from Executor.robot_ws import send_skill_command


def Nav(x, y, z, emit=None):
    return send_skill_command("nav", {"x": x, "y": y, "z": z}, emit=emit)
#发送导航启动信号，最终成功失败由服务器feedback决定


def NavClimb(x, y, z, emit=None):
    return send_skill_command("nav_climb", {"x": x, "y": y, "z": z}, emit=emit)
#发送导航攀爬启动信号，用于目标路径含可攀爬障碍物


def walk(direction, v=0.5, distance=0.0, emit=None):
    if direction not in {"front", "back", "left", "right"}:
        return {"signal": "FAILURE", "skill": "walk_skill", "message": f"unknown direction: {direction}"}
    return send_skill_command("walk_skill", {"direction": direction, "v": v, "distance": distance}, emit=emit)
#发送行走启动信号，direction限定为front/back/left/right，v默认0.5，distance单位米


def Push(x, y, yaw=0.0, z=None, emit=None):
    return send_skill_command("push", {"x": x, "y": y, "yaw": yaw}, emit=emit)
#发送推箱子启动信号，第三个参数按FQPlanner约定为yaw；旧z入参只为兼容，不下发


def climb(height, emit=None):
    if height > 0.5:
        return {"signal": "FAILURE", "skill": "climb", "message": "climb height > 0.5"}
    return send_skill_command("climb", {"height": height}, emit=emit)
#发送攀爬启动信号，0.5m表示借助箱子连续攀爬总高度
