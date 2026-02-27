#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Interface - 交互界面
连接 LLM_Module 和 Robot_Module，集成任务规划和工具执行
"""
import os
import sys
import asyncio
import time
import json
from pathlib import Path

# 取消代理设置（避免 OpenAI 客户端使用错误的代理）
for var in ['http_proxy', 'https_proxy', 'HTTP_PROXY', 'HTTPS_PROXY',
            'ALL_PROXY', 'all_proxy', 'no_proxy', 'NO_PROXY']:
    if var in os.environ:
        del os.environ[var]

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# 加载 .env 文件
try:
    from dotenv import load_dotenv
    env_file = project_root / ".env"
    if env_file.exists():
        load_dotenv(env_file)
        print(f"[加载 .env 文件] {env_file}", file=sys.stderr)
except ImportError:
    pass  # python-dotenv 未安装，跳过

from LLM_Module.llm_core import LLMAgent
from Robot_Module.skill import (
    get_skill_function,
    get_tool_definitions,
    register_all_modules
)

# 环境版本控制（仅在环境签名变化时递增，避免每帧都触发重规划）
_env_version_counter = 0
_last_env_signature = None


def build_env_state_snapshot() -> dict:
    """采样环境状态，作为 input2/sensor_frame 的基础数据。"""
    global _env_version_counter, _last_env_signature

    snapshot = {
        "timestamp": time.time(),
        "sensor_status": {
            "camera": "ok",
            "radar": "ok",
            "robot_state": "ok"
        },
        "camera": {"objects": []},
        "radar": {"obstacles": []},
        "robot_state": {
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "battery": 1.0
        },
        "environment_version": _env_version_counter
    }

    try:
        from ros_topic_comm import get_enemy_positions, get_robot_state, get_yolo_enemy_positions

        robot_state = get_robot_state() or {}
        snapshot["robot_state"]["pose"] = {
            "x": float(robot_state.get("x", 0.0)),
            "y": float(robot_state.get("y", 0.0)),
            "yaw": float(robot_state.get("angle", 0.0))
        }
        snapshot["position"] = {
            "x": snapshot["robot_state"]["pose"]["x"],
            "y": snapshot["robot_state"]["pose"]["y"],
            "z": 0.0
        }

        enemies = get_enemy_positions() or []
        yolo_enemies = get_yolo_enemy_positions() or []

        snapshot["camera"]["objects"] = enemies
        snapshot["radar"]["obstacles"] = yolo_enemies

        # 只基于“外部环境对象”构造版本签名，避免机器人自身位姿变化触发重规划
        enemy_sig = tuple(
            sorted(
                (
                    str(item.get("id", "")),
                    round(float(item.get("x", 0.0)), 2),
                    round(float(item.get("y", 0.0)), 2),
                )
                for item in enemies
                if isinstance(item, dict)
            )
        )
        obstacle_sig = tuple(
            sorted(
                (
                    str(item.get("id", item.get("label", ""))),
                    round(float(item.get("x", 0.0)), 2),
                    round(float(item.get("y", 0.0)), 2),
                )
                for item in yolo_enemies
                if isinstance(item, dict)
            )
        )
        current_sig = (enemy_sig, obstacle_sig)
        if _last_env_signature is None:
            _last_env_signature = current_sig
        elif current_sig != _last_env_signature:
            _env_version_counter += 1
            _last_env_signature = current_sig

        snapshot["environment_version"] = _env_version_counter

    except Exception as exc:
        snapshot["sensor_status"]["robot_state"] = "degraded"
        snapshot["sensor_error"] = str(exc)

    return snapshot


def execute_tool(function_name: str, function_args: dict) -> dict:
    """执行 Robot_Module 中的工具函数"""
    skill_func = get_skill_function(function_name)

    if not skill_func:
        return {"error": f"Unknown tool: {function_name}"}

    try:
        # 调用异步技能函数
        try:
            # 检查是否已有运行的事件循环
            asyncio.get_running_loop()
            # 如果有，使用 asyncio.ensure_future() 或直接 await（需要在异步上下文中）
            # 但由于 execute_tool 是同步函数，我们需要在循环中调度这个协程
            import threading

            # 在新线程中运行，避免阻塞当前循环
            result = None
            exception = None

            def run_in_new_loop():
                nonlocal result, exception
                try:
                    new_loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(new_loop)
                    result = new_loop.run_until_complete(skill_func(**function_args))
                    new_loop.close()
                except Exception as e:
                    exception = e

            thread = threading.Thread(target=run_in_new_loop)
            thread.start()
            thread.join()

            if exception:
                raise exception

        except RuntimeError:
            # 没有运行的事件循环，使用 asyncio.run()
            result = asyncio.run(skill_func(**function_args))

        # 估算执行时间
        if function_name in ['move_forward', 'move_backward']:
            distance = function_args.get('distance', 1.0)
            speed = function_args.get('speed', 0.3)
            delay = distance / speed if speed > 0 else 0
        elif function_name == 'detect_color_and_act':
            delay = 3.3  # 颜色检测+移动约3.3秒
        elif function_name == 'turn':
            angle = abs(function_args.get('angle', 90.0))
            angular_speed = function_args.get('angular_speed', 0.5)
            delay = (angle / 180.0 * 3.14159) / angular_speed if angular_speed > 0 else 0
        else:
            delay = 0

        return {"success": True, "result": result, "delay": delay}
    except Exception as e:
        return {"success": False, "error": str(e)}


def format_robot_config(tools):
    """格式化机器人配置信息"""
    config_lines = ["机器人类型: 2D仿真机器人（差速驱动）"]
    config_lines.append("\n可用技能:")

    for tool in tools:
        func = tool.get("function", {})
        name = func.get("name", "")
        desc = func.get("description", "")
        params = func.get("parameters", {}).get("properties", {})

        # 转义大括号
        desc = desc.replace("{", "{{").replace("}", "}}")

        config_lines.append(f"- {name}({', '.join(params.keys())}): {desc}")

    return "\n".join(config_lines)


def format_available_skills(tools):
    """格式化可用技能列表"""
    skills = []
    for tool in tools:
        func = tool.get("function", {})
        name = func.get("name", "")
        desc = func.get("description", "")
        params = func.get("parameters", {}).get("properties", {})

        # 转义大括号，避免被当作模板占位符
        desc = desc.replace("{", "{{").replace("}", "}}")

        param_str = ", ".join([f"{k}: {v.get('type', '')}" for k, v in params.items()])
        skills.append(f"  - {name}({param_str}): {desc}")

    return "\n".join(skills)


def load_dynamic_prompt(prompt_path, tools):
    """加载并动态填充提示词"""
    import yaml

    with open(prompt_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    # 获取模板
    prompt_template = data.get("prompt", "")

    # 动态生成配置信息
    robot_config = format_robot_config(tools)
    available_skills = format_available_skills(tools)

    # 填充模板
    prompt = prompt_template.format(
        robot_config=robot_config,
        available_skills=available_skills,
        user_input="{user_input}"  # 保留占位符
    )

    return prompt


def main():
    """主函数"""
    # 注册所有 Robot_Module 的工具函数
    register_all_modules()

    # 检查 API Key
    api_key = os.getenv('Test_API_KEY')
    if not api_key:
        print("❌ 错误: 未设置 Test_API_KEY 环境变量", file=sys.stderr)
        print("请设置: export Test_API_KEY=your_api_key_here", file=sys.stderr)
        sys.exit(1)

    # 从 Robot_Module 获取工具定义
    tools = get_tool_definitions()

    # 获取提示词路径
    prompt_path = project_root / "LLM_Module" / "prompts" / "planning_prompt_2d.yaml"
    llm_model = os.getenv("LLM_MODEL", "qwen3-32b")

    # 初始化 LLM Agent（启用自适应控制）
    llm_agent = LLMAgent(
        api_key=api_key,
        model=llm_model,
        prompt_path=str(prompt_path),
        enable_adaptive=True  # 启用自适应重新规划
    )

    # 动态加载并填充提示词，覆盖默认的模板
    dynamic_prompt = load_dynamic_prompt(prompt_path, tools)
    llm_agent.planning_prompt_template = dynamic_prompt

    # 显示欢迎信息
    print("="*60, file=sys.stderr)
    print("LLM Interactive Interface", file=sys.stderr)
    print("="*60, file=sys.stderr)
    print(f"API: {llm_agent.client.base_url}", file=sys.stderr)
    print(f"Model: {llm_agent.model}", file=sys.stderr)
    print(f"可用工具: {len(tools)} 个", file=sys.stderr)
    print("-"*60, file=sys.stderr)

    # 显示所有可用工具
    for tool in tools:
        func = tool.get("function", {})
        name = func.get("name", "")
        desc = func.get("description", "")
        params = func.get("parameters", {}).get("properties", {})

        print(f"  • {name}", file=sys.stderr)
        if params:
            param_list = [f"{k}({v.get('type', '')})" for k, v in params.items()]
            print(f"    参数: {', '.join(param_list)}", file=sys.stderr)
        print(f"    描述: {desc}", file=sys.stderr)
        print("", file=sys.stderr)

    print("-"*60, file=sys.stderr)
    print("提示: 确保已在另一个窗口启动仿真器", file=sys.stderr)
    print("  python3 Sim_Module/sim2d/simulator.py", file=sys.stderr)
    print("", file=sys.stderr)
    print("直连工具模式（不调用LLM）:", file=sys.stderr)
    print("  /tool get_isaac_config", file=sys.stderr)
    print('  /tool move_isaac {"direction":"forward","distance":1.0}', file=sys.stderr)
    print("", file=sys.stderr)
    print("输入 'quit' 或 'exit' 退出", file=sys.stderr)
    print("="*60, file=sys.stderr)

    # 主循环
    while True:
        try:
            # 获取用户输入
            user_input = input("\n💬 请输入指令: ").strip()

            if not user_input:
                continue

            if user_input.lower() in ['quit', 'exit', 'q']:
                print("👋 再见!", file=sys.stderr)
                break

            # ==================== 直连工具模式（绕过LLM） ====================
            # 用法：
            #   /tool get_isaac_config
            #   /tool move_isaac {"direction":"forward","distance":1.0}
            if user_input.startswith("/tool "):
                raw = user_input[len("/tool "):].strip()
                if not raw:
                    print("❌ 用法: /tool <tool_name> [json_args]", file=sys.stderr)
                    continue

                if " " in raw:
                    tool_name, args_text = raw.split(" ", 1)
                    args_text = args_text.strip()
                    try:
                        tool_args = json.loads(args_text) if args_text else {}
                        if not isinstance(tool_args, dict):
                            raise ValueError("json_args 必须是对象")
                    except Exception as exc:
                        print(f"❌ JSON参数解析失败: {exc}", file=sys.stderr)
                        continue
                else:
                    tool_name, tool_args = raw, {}

                direct_result = execute_tool(tool_name, tool_args)
                print(f"\n🔧 [直连工具结果] {tool_name}", file=sys.stderr)
                print(json.dumps(direct_result, ensure_ascii=False, indent=2), file=sys.stderr)
                continue
            # ================================================================

            # ==================== 提取图片路径 ====================
            import re
            image_path = None

            # 匹配图片路径（支持 .png, .jpg, .jpeg）
            # 格式1: "根据 /path/to/image.png 前进"
            # 格式2: "看看 /path/to/image.jpg 然后..."
            # 格式3: "/path/to/image.jpeg"
            image_patterns = [
                r'(?:根据|看看|观察|分析|检测)\s*([/\w\-./]+\.(?:png|jpg|jpeg))',
                r'([/\w\-./]+\.(?:png|jpg|jpeg))\s*(?:然后|并且|，|、)',
                r'^([/\w\-./]+\.(?:png|jpg|jpeg))$'
            ]

            for pattern in image_patterns:
                match = re.search(pattern, user_input)
                if match:
                    image_path = match.group(1)
                    # 从用户输入中移除图片路径部分
                    user_input = re.sub(pattern, '', user_input).strip()
                    print(f"🖼️  [检测到图片] {image_path}", file=sys.stderr)
                    break

            if not image_path:
                # 检查是否是纯图片路径
                if re.match(r'^[/\w\-./]+\.(?:png|jpg|jpeg)$', user_input):
                    image_path = user_input
                    user_input = "请分析当前环境并给出建议"
                    print(f"🖼️  [检测到图片] {image_path}", file=sys.stderr)
            # ===========================================================

            # 执行双层 LLM 流程
            results = llm_agent.run_pipeline(
                user_input=user_input,
                tools=tools,
                execute_tool_fn=execute_tool,
                image_path=image_path,  # 新增：VLM 环境图像路径
                env_state_provider=build_env_state_snapshot
            )

            # 显示结果摘要
            if results:
                # 兼容两种返回格式：
                # - 非自适应模式: {"success": True, ...}
                # - 自适应模式: {"status": "success", ...}
                success_count = sum(1 for r in results
                                   if r.get("success") is True or r.get("status") == "success")
                print(f"\n📊 [完成] {success_count}/{len(results)} 个任务成功", file=sys.stderr)

        except KeyboardInterrupt:
            print("\n\n👋 再见!", file=sys.stderr)
            break
        except Exception as e:
            print(f"\n❌ [错误] {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()
