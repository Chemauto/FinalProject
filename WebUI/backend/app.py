#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flask Web Server - Web UI 后端服务器
提供 REST API 和 WebSocket 支持
"""
import os
import sys
import json
import asyncio
from pathlib import Path
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from flask_sock import Sock

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

# WebUI 目录
webui_dir = Path(__file__).parent.parent
backend_dir = Path(__file__).parent

# 导入机器人服务
sys.path.insert(0, str(backend_dir))
from robot_service import get_robot_service, RobotService

# 创建 Flask 应用
app = Flask(__name__, static_folder='../frontend/dist', static_url_path='')
CORS(app)
sock = Sock(app)

# 全局变量
robot_service = None
connected_clients = []


def init_robot_service():
    """初始化机器人服务"""
    global robot_service
    try:
        robot_service = get_robot_service()
        print("✓ 机器人服务初始化成功")
        return True
    except Exception as e:
        print(f"✗ 机器人服务初始化失败: {e}")
        return False


# ==============================================================================
# REST API 路由
# ==============================================================================

@app.route('/')
def index():
    """返回前端页面"""
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/api/status', methods=['GET'])
def get_status():
    """获取服务状态"""
    if robot_service is None:
        return jsonify({"error": "服务未初始化"}), 503

    return jsonify(robot_service.get_status())


@app.route('/api/tools', methods=['GET'])
def get_tools():
    """获取可用工具列表"""
    if robot_service is None:
        return jsonify({"error": "服务未初始化"}), 503

    tools = robot_service.get_available_tools()

    # 简化工具信息用于前端显示
    simplified_tools = []
    for tool in tools:
        func = tool.get("function", {})
        simplified_tools.append({
            "name": func.get("name"),
            "description": func.get("description"),
            "parameters": list(func.get("parameters", {}).get("properties", {}).keys())
        })

    return jsonify(simplified_tools)


@app.route('/api/history', methods=['GET'])
def get_history():
    """获取任务历史"""
    if robot_service is None:
        return jsonify({"error": "服务未初始化"}), 503

    return jsonify(robot_service.get_task_history())


@app.route('/api/command', methods=['POST'])
def execute_command():
    """执行用户命令"""
    if robot_service is None:
        return jsonify({"error": "服务未初始化"}), 503

    data = request.get_json()
    user_input = data.get('input', '').strip()

    if not user_input:
        return jsonify({"error": "输入不能为空"}), 400

    if robot_service.is_processing:
        return jsonify({"error": "系统正在处理其他任务"}), 409

    # 异步执行命令
    def run_command():
        async def progress_callback(progress):
            # 通过 WebSocket 广播进度
            message = json.dumps({
                "type": "progress",
                "data": progress
            })
            for client in connected_clients:
                try:
                    client.send(message)
                except:
                    pass

        # 运行异步任务
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            result = loop.run_until_complete(
                robot_service.process_command(user_input, progress_callback)
            )

            # 发送完成消息
            completion_message = json.dumps({
                "type": "complete",
                "data": result
            })
            for client in connected_clients:
                try:
                    client.send(completion_message)
                except:
                    pass
        finally:
            loop.close()

    # 在后台线程中执行
    import threading
    thread = threading.Thread(target=run_command, daemon=True)
    thread.start()

    return jsonify({"success": True, "message": "命令已接受"}), 202


# ==============================================================================
# WebSocket 路由
# ==============================================================================

@sock.route('/ws')
def websocket_connection(ws):
    """WebSocket 连接处理"""
    connected_clients.append(ws)
    print(f"✓ 新客户端连接 (当前: {len(connected_clients)})")

    try:
        # 发送初始状态
        if robot_service:
            ws.send(json.dumps({
                "type": "status",
                "data": robot_service.get_status()
            }))

        # 保持连接并处理消息
        while True:
            message = ws.receive()
            if not message:
                break

            data = json.loads(message)

            if data.get('type') == 'ping':
                ws.send(json.dumps({"type": "pong"}))

    except Exception as e:
        print(f"✗ WebSocket 错误: {e}")
    finally:
        connected_clients.remove(ws)
        print(f"✓ 客户端断开 (当前: {len(connected_clients)})")


# ==============================================================================
# 错误处理
# ==============================================================================

@app.errorhandler(404)
def not_found(error):
    """404 错误处理"""
    return jsonify({"error": "未找到请求的资源"}), 404


@app.errorhandler(500)
def internal_error(error):
    """500 错误处理"""
    return jsonify({"error": "服务器内部错误"}), 500


# ==============================================================================
# 主函数
# ==============================================================================

def main():
    """主函数"""
    print("=" * 60)
    print("Robot Web UI Server")
    print("=" * 60)

    # 初始化机器人服务
    if not init_robot_service():
        print("\n✗ 启动失败：无法初始化机器人服务")
        print("  请确保已设置 Test_API_KEY 环境变量")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("服务器配置")
    print("=" * 60)
    print(f"  地址: http://localhost:5000")
    print(f"  WebSocket: ws://localhost:5000/ws")
    print(f"  前端目录: {app.static_folder}")
    print("=" * 60)
    print("\n按 Ctrl+C 停止服务器\n")

    # 启动服务器
    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    except KeyboardInterrupt:
        print("\n\n✓ 服务器已停止")


if __name__ == "__main__":
    main()
