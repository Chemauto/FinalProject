#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
追击功能测试程序

测试目标追击功能：
1. 启动增强仿真器
2. 随机生成敌人
3. 通过自然语言控制机器人追击
"""

import sys
import os
from pathlib import Path

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# 加载 .env
try:
    from dotenv import load_dotenv
    env_file = project_root / ".env"
    if env_file.exists():
        load_dotenv(env_file)
        print(f"[加载 .env] {env_file}", file=sys.stderr)
except ImportError:
    pass

from LLM_Module.llm_core import LLMAgent
from Robot_Module.skill import (
    get_skill_function,
    get_tool_definitions,
    register_all_modules
)


def execute_tool(function_name: str, function_args: dict) -> dict:
    """执行工具函数"""
    skill_func = get_skill_function(function_name)
    if not skill_func:
        return {"error": f"Unknown tool: {function_name}"}
    try:
        import asyncio
        result = asyncio.run(skill_func(**function_args))
        return {"result": result}
    except Exception as e:
        return {"error": str(e)}


def format_robot_config(tools):
    """格式化机器人配置"""
    config_lines = ["机器人类型: 2D仿真机器人（差速驱动）"]
    config_lines.append("\n可用技能:")
    for tool in tools:
        func = tool.get("function", {})
        name = func.get("name", "")
        desc = func.get("description", "")
        params = func.get("parameters", {}).get("properties", {})
        config_lines.append(f"- {name}({', '.join(params.keys())}): {desc}")
    return "\n".join(config_lines)


def format_available_skills(tools):
    """格式化可用技能"""
    skills = []
    for tool in tools:
        func = tool.get("function", {})
        name = func.get("name", "")
        desc = func.get("description", "")
        params = func.get("parameters", {}).get("properties", {})
        param_str = ", ".join([f"{k}: {v.get('type', '')}" for k, v in params.items()])
        skills.append(f"  - {name}({param_str}): {desc}")
    return "\n".join(skills)


def load_chase_prompt(tools):
    """加载追击任务提示词"""
    import yaml

    prompt_content = """你是一个机器人任务规划专家。用户会给你一个追击任务，你需要将其分解为具体的工具调用。

{robot_config}

当前任务：追击地图上的敌人目标。

追击任务说明：
1. 机器人在2D平面地图上（800x600像素）
2. 地图上随机分布着敌人目标（红色圆圈）
3. 你可以使用追击工具自动追击目标
4. 也可以使用基础运动工具（前进、后退、旋转）

可用追击工具：
{available_skills}

用户输入: {user_input}

请输出任务分解（JSON格式）：
```json
{{
  "tasks": [
    {{"task": "任务描述", "tool": "工具名", "parameters": {{"参数": "值"}}}}
  ]
}}
```
"""

    robot_config = format_robot_config(tools)
    available_skills = format_available_skills(tools)

    return prompt_content.format(
        robot_config=robot_config,
        available_skills=available_skills,
        user_input="{user_input}"
    )


def main():
    """主测试函数"""
    print("="*60, file=sys.stderr)
    print("追击功能测试程序", file=sys.stderr)
    print("="*60, file=sys.stderr)

    # 检查API Key
    api_key = os.getenv('Test_API_KEY')
    if not api_key:
        print("❌ 错误: 未设置 Test_API_KEY", file=sys.stderr)
        sys.exit(1)

    # 注册所有工具
    register_all_modules()

    # 获取工具定义
    tools = get_tool_definitions()

    # 初始化LLM
    llm_agent = LLMAgent(api_key=api_key, prompt_path="")
    llm_agent.planning_prompt_template = load_chase_prompt(tools)

    print("\n" + "="*60, file=sys.stderr)
    print("使用说明", file=sys.stderr)
    print("="*60, file=sys.stderr)
    print("1. 先在另一个终端运行增强仿真器：", file=sys.stderr)
    print("   python Test_Module/enhanced_simulator.py", file=sys.stderr)
    print("", file=sys.stderr)
    print("2. 在仿真器中按 'R' 生成敌人", file=sys.stderr)
    print("", file=sys.stderr)
    print("3. 然后在这里输入追击命令", file=sys.stderr)
    print("", file=sys.stderr)
    print("示例命令：", file=sys.stderr)
    print("  - '追击坐标(700, 300)的目标'", file=sys.stderr)
    print("  - '追击最近的目标'", file=sys.stderr)
    print("  - '计算追击角度，我在(100, 300)朝向东方，目标在(700, 300)'", file=sys.stderr)
    print("="*60, file=sys.stderr)

    # 主循环
    while True:
        try:
            user_input = input("\n💬 请输入指令: ").strip()

            if not user_input:
                continue

            if user_input.lower() in ['quit', 'exit', 'q']:
                print("👋 再见!", file=sys.stderr)
                break

            # 执行任务
            print(f"\n[规划] {user_input}", file=sys.stderr)
            results = llm_agent.run_pipeline(
                user_input=user_input,
                tools=tools,
                execute_tool_fn=execute_tool
            )

            # 显示结果
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
