#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Interface - 交互界面
连接 LLM_Module 和 Robot_Module，集成任务规划和工具执行
"""
import os
import sys
import asyncio
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


def execute_tool(function_name: str, function_args: dict) -> dict:
    """执行 Robot_Module 中的工具函数"""
    skill_func = get_skill_function(function_name)

    if not skill_func:
        return {"error": f"Unknown tool: {function_name}"}

    try:
        # 调用异步技能函数
        try:
            # 检查是否已有运行的事件循环
            loop = asyncio.get_running_loop()
            # 如果有，使用 asyncio.ensure_future() 或直接 await（需要在异步上下文中）
            # 但由于 execute_tool 是同步函数，我们需要在循环中调度这个协程
            import concurrent.futures
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

    # 初始化 LLM Agent（启用自适应控制）
    llm_agent = LLMAgent(
        api_key=api_key,
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
    print("  python3 Sim_Module/2d/simulator.py", file=sys.stderr)
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

            # 执行双层 LLM 流程
            results = llm_agent.run_pipeline(
                user_input=user_input,
                tools=tools,
                execute_tool_fn=execute_tool
            )

            # 显示结果摘要
            if results:
                success_count = sum(1 for r in results if r.get("success"))
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
