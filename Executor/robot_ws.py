import asyncio
import json
import os
import uuid

from Executor import state

DEFAULT_WS_URL = "ws://127.0.0.1:8765"
DEFAULT_TIMEOUT_SEC = 60
DEFAULT_CONNECT_TIMEOUT_SEC = 3
_robot_client = None


def build_command_payload(skill, args, action_id=None):
    return {
        "type": "command",
        "action_id": action_id or f"{skill}-{uuid.uuid4().hex[:8]}",
        "skill": skill,
        "args": dict(args or {}),
    }
#构建发送给机器人服务器的技能启动消息


def build_healthcheck_payload():
    return {"type": "healthcheck"}
#构建发送给机器人服务端的健康检查消息


def build_get_state_payload():
    return {"type": "get_state"}
#构建读取机器人当前状态的只读消息


def set_robot_client(client):
    global _robot_client
    _robot_client = client
#注入测试客户端或外部客户端，传None恢复默认WebSocket


def check_connection(timeout_sec=None):
    if _robot_client is not None and hasattr(_robot_client, "check_connection"):
        return _robot_client.check_connection(timeout_sec=timeout_sec)
    return asyncio.run(_check_ws_connection(timeout_sec=timeout_sec))
#检查WebSocket服务端是否在线，并确认仿真状态源是否就绪


def get_robot_state(timeout_sec=None):
    if _robot_client is not None and hasattr(_robot_client, "get_robot_state"):
        return _robot_client.get_robot_state(timeout_sec=timeout_sec)
    return asyncio.run(_get_ws_state(timeout_sec=timeout_sec))
#通过WebSocket读取当前ROS2状态，不发送控制指令


def send_skill_command(skill, args, emit=None, timeout_sec=None):
    if _robot_client is not None:
        return _robot_client.send_skill_command(skill, args, emit=emit, timeout_sec=timeout_sec)
    return asyncio.run(_send_ws_command(skill, args, emit=emit, timeout_sec=timeout_sec))
#发送技能启动信号，并等待服务器返回最终反馈


async def _check_ws_connection(timeout_sec=None):
    try:
        import websockets
    except ImportError:
        return _health_failure("未安装websockets，请先执行 pip install websockets")

    url = os.getenv("ROBOT_WS_URL", DEFAULT_WS_URL)
    timeout = float(timeout_sec or os.getenv("ROBOT_WS_CONNECT_TIMEOUT_SEC", DEFAULT_CONNECT_TIMEOUT_SEC))

    try:
        async with websockets.connect(url) as ws:
            await ws.send(json.dumps(build_healthcheck_payload(), ensure_ascii=False))
            message = _parse_message(await asyncio.wait_for(ws.recv(), timeout=timeout))
            if message.get("type") == "health":
                message["ws_url"] = url
                return message
            if message.get("type") == "error":
                return _health_failure(message.get("message") or "服务器返回错误", url=url)
            return _health_failure(f"收到未知响应: {message}", url=url)
    except Exception as error:
        return _health_failure(f"WebSocket连接失败: {error}", url=url)
#发送健康检查消息，快速判断服务端是否在线


async def _get_ws_state(timeout_sec=None):
    try:
        import websockets
    except ImportError:
        return _state_failure("未安装websockets，请先执行 pip install websockets")

    url = os.getenv("ROBOT_WS_URL", DEFAULT_WS_URL)
    timeout = float(timeout_sec or os.getenv("ROBOT_WS_CONNECT_TIMEOUT_SEC", DEFAULT_CONNECT_TIMEOUT_SEC))

    try:
        async with websockets.connect(url) as ws:
            await ws.send(json.dumps(build_get_state_payload(), ensure_ascii=False))
            message = _parse_message(await asyncio.wait_for(ws.recv(), timeout=timeout))
            if message.get("type") == "state":
                message["signal"] = message.get("signal") or "SUCCESS"
                message["ws_url"] = url
                state.update_latest_state(message)
                return message
            if message.get("type") == "error":
                return _state_failure(message.get("message") or "服务器返回错误", url=url)
            return _state_failure(f"收到未知响应: {message}", url=url)
    except Exception as error:
        return _state_failure(f"WebSocket读取状态失败: {error}", url=url)
#读取一次机器人状态，给observe融合ROS2环境事实


async def _send_ws_command(skill, args, emit=None, timeout_sec=None):
    try:
        import websockets
    except ImportError:
        return _failure(skill, "未安装websockets，请先执行 pip install websockets")

    command = build_command_payload(skill, args)
    url = os.getenv("ROBOT_WS_URL", DEFAULT_WS_URL)
    timeout = float(timeout_sec or os.getenv("ROBOT_WS_TIMEOUT_SEC", DEFAULT_TIMEOUT_SEC))
    deadline = asyncio.get_running_loop().time() + timeout

    try:
        async with websockets.connect(url) as ws:
            await ws.send(json.dumps(command, ensure_ascii=False))
            if emit:
                emit("tool", f"sent {skill} action_id={command['action_id']}")
            return await _wait_feedback(ws, command, deadline, emit)
    except Exception as error:
        return _failure(skill, f"WebSocket通信失败: {error}", action_id=command["action_id"])
#打开WebSocket，发送命令，然后持续接收状态直到拿到feedback


async def _wait_feedback(ws, command, deadline, emit):
    while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            return _failure(command["skill"], "等待服务器反馈超时", action_id=command["action_id"])

        message = _parse_message(await asyncio.wait_for(ws.recv(), timeout=remaining))
        if message.get("type") == "state":
            state.update_latest_state(message)
            if emit:
                emit("status", state.format_latest_state())
            continue

        if message.get("type") == "feedback" and message.get("action_id") == command["action_id"]:
            state.update_latest_feedback(message)
            return message

        if message.get("type") == "error":
            feedback = _failure(command["skill"], message.get("message") or "服务器返回错误", action_id=command["action_id"])
            state.update_latest_feedback(feedback)
            return feedback
#处理服务器推送的state/error/feedback消息


def _parse_message(raw_message):
    if isinstance(raw_message, dict):
        return raw_message
    try:
        message = json.loads(raw_message)
    except (TypeError, json.JSONDecodeError):
        return {"type": "error", "message": f"无法解析服务器消息: {raw_message!r}"}
    return message if isinstance(message, dict) else {"type": "error", "message": "服务器消息不是JSON对象"}
#把服务器原始消息解析成字典


def _failure(skill, message, action_id=None):
    feedback = {
        "type": "feedback",
        "action_id": action_id or f"{skill}-local-failure",
        "skill": skill,
        "signal": "FAILURE",
        "message": message,
    }
    state.update_latest_feedback(feedback)
    return feedback
#构建本地失败反馈，保持和服务器feedback结构一致


def _health_failure(message, url=None):
    return {
        "type": "health",
        "signal": "FAILURE",
        "message": message,
        "status_json_ready": False,
        "ws_url": url or os.getenv("ROBOT_WS_URL", DEFAULT_WS_URL),
    }
#构建健康检查失败结果，给TUI直接显示


def _state_failure(message, url=None):
    return {
        "type": "state",
        "signal": "FAILURE",
        "connected": False,
        "message": message,
        "scene_objects": [],
        "raw": {},
        "ws_url": url or os.getenv("ROBOT_WS_URL", DEFAULT_WS_URL),
    }
#构建状态读取失败结果，observe可退回纯VLM
