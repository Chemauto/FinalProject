from Executor.robot_ws import send_skill_command


def Nav(x, y, z, emit=None):
    return send_skill_command("nav", {"x": x, "y": y, "z": z}, emit=emit)
#发送导航启动信号，最终成功失败由服务器feedback决定


def walk(direction, v=0.5, distance=0.0, emit=None):
    if direction not in {"front", "back", "left", "right"}:
        return {"signal": "FAILURE", "skill": "walk_skill", "message": f"unknown direction: {direction}"}
    return send_skill_command("walk_skill", {"direction": direction, "v": v, "distance": distance}, emit=emit)
#发送行走启动信号，direction限定为front/back/left/right，v默认0.5，distance单位米


def Push(x, y, z, emit=None):
    return send_skill_command("push", {"x": x, "y": y, "z": z}, emit=emit)
#发送推箱子启动信号，服务器负责控制和验收


def climb(height, emit=None):
    if height > 0.3:
        return {"signal": "FAILURE", "skill": "climb", "message": "climb height > 0.3"}
    return send_skill_command("climb", {"height": height}, emit=emit)
#发送攀爬启动信号，客户端只保留最高0.3m的基础约束
