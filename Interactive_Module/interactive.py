#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Interface - 交互界面
连接 LLM_Module 和 Robot_Module，集成任务规划和工具执行
"""
import os
import sys
import asyncio
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
from LLM_Module.object_facts_loader import load_object_facts
from envtest_status_sync import (
    sync_object_facts_from_live_envtest,
    sync_runtime_overrides_from_user_input,
)
from Robot_Module.skill import (
    get_skill_function,
    get_tool_definitions,
    register_all_modules
)
from VLM_Module.vlm_core import VLMCore

ENABLE_VLM_CONTEXT = True
DEFAULT_OBJECT_FACTS_PATH = Path(
    os.getenv("FINALPROJECT_OBJECT_FACTS_PATH", str(project_root / "config" / "object_facts.json"))
)


def _report_object_facts_status(object_facts_path: str | Path) -> None:
    path = Path(object_facts_path)
    if path.exists():
        return

    example_path = path.with_name(f"{path.stem}.example{path.suffix}")
    if example_path.exists():
        print(
            f"[ObjectFacts] 未找到结构化物体信息文件: {path}；示例文件 {example_path} 不会自动加载，本次仅使用 VLM",
            file=sys.stderr,
        )
        return

    print(f"[ObjectFacts] 未找到结构化物体信息文件: {path}，本次仅使用 VLM", file=sys.stderr)


def _normalize_tool_result(function_name: str, raw_result):
    """把工具原始返回统一转换为结构化结果。"""
    parsed_result = raw_result
    if isinstance(raw_result, str):
        try:
            parsed_result = json.loads(raw_result)
        except json.JSONDecodeError:
            parsed_result = {"raw_result": raw_result}

    if not isinstance(parsed_result, dict):
        parsed_result = {"raw_result": parsed_result}

    status = parsed_result.get("status", "success")
    feedback = parsed_result.get("execution_feedback") or {
        "signal": "SUCCESS" if status == "success" else "FAILURE",
        "skill": function_name,
        "message": f"{function_name} 执行{'成功' if status == 'success' else '失败'}",
    }
    success = status == "success" and feedback.get("signal") == "SUCCESS"

    return {
        "success": success,
        "result": parsed_result,
        "raw_result": raw_result,
        "feedback": feedback,
    }


def execute_tool(function_name: str, function_args: dict) -> dict:
    """执行 Robot_Module 中的工具函数"""
    skill_func = get_skill_function(function_name)

    if not skill_func:
        return {"error": f"Unknown tool: {function_name}"}

    try:
        # 调用异步技能函数
        raw_result = asyncio.run(skill_func(**function_args))
        normalized = _normalize_tool_result(function_name, raw_result)

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

        return {
            "success": normalized["success"],
            "result": normalized["result"],
            "raw_result": normalized["raw_result"],
            "feedback": normalized["feedback"],
            "delay": delay,
        }
    except Exception as e:
        return {"error": str(e)}


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
        visual_context="{visual_context}",
        scene_facts="{scene_facts}",
        object_facts="{object_facts}",
        user_input="{user_input}"  # 保留占位符
    )

    return prompt


def build_llm_agent():
    """初始化工具列表和 LLM Agent。"""
    register_all_modules()

    api_key = os.getenv('Test_API_KEY')
    if not api_key:
        print("❌ 错误: 未设置 Test_API_KEY 环境变量", file=sys.stderr)
        print("请设置: export Test_API_KEY=your_api_key_here", file=sys.stderr)
        sys.exit(1)

    tools = get_tool_definitions()
    prompt_path = project_root / "LLM_Module" / "prompts" / "highlevel_prompt.yaml"
    llm_agent = LLMAgent(api_key=api_key, prompt_path=str(prompt_path))
    llm_agent.planning_prompt_template = load_dynamic_prompt(prompt_path, tools)
    return llm_agent, tools


def build_vlm_core():
    """按开关初始化 VLM。"""
    if not ENABLE_VLM_CONTEXT:
        return None
    try:
        return VLMCore()
    except Exception as error:
        print(f"[VLM] 初始化失败，已跳过: {error}", file=sys.stderr)
        return None


def show_welcome(llm_agent, tools, title="LLM Interactive Interface", input_hint="输入 'quit' 或 'exit' 退出"):
    """打印欢迎信息。"""
    print("="*60, file=sys.stderr)
    print(title, file=sys.stderr)
    print("="*60, file=sys.stderr)
    print(f"API: {llm_agent.client.base_url}", file=sys.stderr)
    print(f"Model: {llm_agent.model}", file=sys.stderr)
    print(f"VLM上下文: {'开启' if ENABLE_VLM_CONTEXT else '关闭'}", file=sys.stderr)
    print(f"可用工具: {len(tools)} 个", file=sys.stderr)
    print("-"*60, file=sys.stderr)

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
    print(input_hint, file=sys.stderr)
    print("="*60, file=sys.stderr)


def process_user_input(user_input: str, llm_agent, tools, vlm_core=None, object_facts_path: str | Path | None = None) -> bool:
    """处理一条用户输入。返回 False 表示退出。"""
    if not user_input:
        return True

    if user_input.lower() in ['quit', 'exit', 'q']:
        print("👋 再见!", file=sys.stderr)
        return False

    visual_context = None
    scene_facts = None
    object_facts = None

    try:
        effective_object_facts_path = object_facts_path or DEFAULT_OBJECT_FACTS_PATH
        synced_payload = sync_object_facts_from_live_envtest(
            effective_object_facts_path,
            user_input=user_input,
        )
        if synced_payload is not None:
            runtime_state = synced_payload.get("runtime_state") or {}
            print(
                "[EnvTestSync] 已同步实时场景: "
                f"scene_id={runtime_state.get('scene_id')}, "
                f"model_use={runtime_state.get('model_use')}, "
                f"objects={len(synced_payload.get('objects') or [])}",
                file=sys.stderr,
            )
        else:
            sync_runtime_overrides_from_user_input(
                effective_object_facts_path,
                user_input=user_input,
            )
        object_facts = load_object_facts(effective_object_facts_path)
        if object_facts is not None:
            print(f"[ObjectFacts] 已加载结构化物体信息: {effective_object_facts_path}", file=sys.stderr)
        else:
            _report_object_facts_status(effective_object_facts_path)
    except Exception as error:
        print(f"[ObjectFacts] 读取失败，忽略结构化物体信息: {error}", file=sys.stderr)

    if vlm_core is not None:
        try:
            if hasattr(vlm_core, "describe_structured"):
                structured_context = vlm_core.describe_structured()
                visual_context = json.dumps(structured_context, ensure_ascii=False)
                print(f"[VLM] 当前视觉描述: {visual_context}", file=sys.stderr)
                scene_facts = VLMCore.build_scene_facts(structured_context)
            else:
                visual_context = vlm_core.describe()
                print(f"[VLM] 当前视觉描述: {visual_context}", file=sys.stderr)
        except Exception as error:
            print(f"[VLM] 视觉描述失败，继续仅使用文字输入: {error}", file=sys.stderr)

    if object_facts is not None:
        scene_facts = VLMCore.merge_scene_facts(scene_facts, object_facts)

    results = llm_agent.run_pipeline(
        user_input=user_input,
        tools=tools,
        execute_tool_fn=execute_tool,
        visual_context=visual_context,
        scene_facts=scene_facts,
        object_facts=object_facts,
    )

    if results:
        success_count = sum(1 for result in results if result.get("success"))
        print(f"\n📊 [完成] {success_count}/{len(results)} 个任务成功", file=sys.stderr)
    return True


def main():
    """主函数"""
    llm_agent, tools = build_llm_agent()
    vlm_core = build_vlm_core()
    show_welcome(llm_agent, tools)

    # 主循环
    while True:
        try:
            user_input = input("\n💬 请输入指令: ").strip()
            if not process_user_input(user_input, llm_agent, tools, vlm_core):
                break
        except KeyboardInterrupt:
            print("\n\n👋 再见!", file=sys.stderr)
            break
        except Exception as e:
            print(f"\n❌ [错误] {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()
