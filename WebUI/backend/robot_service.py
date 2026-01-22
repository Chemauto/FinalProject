#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Service - Web UI 机器人服务层
封装 LLM_Module 和 Robot_Module 的交互逻辑
"""
import os
import sys
import asyncio
from pathlib import Path
from typing import Callable, Dict, List, Any

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

# 加载 .env 文件
try:
    from dotenv import load_dotenv
    env_file = project_root / ".env"
    if env_file.exists():
        load_dotenv(env_file)
except ImportError:
    pass

from LLM_Module.llm_core import LLMAgent
from Robot_Module.skill import (
    get_skill_function,
    get_tool_definitions,
    register_all_modules
)


class RobotService:
    """机器人服务类"""

    def __init__(self):
        """初始化服务"""
        self.api_key = os.getenv('Test_API_KEY')
        if not self.api_key:
            raise ValueError("未设置 Test_API_KEY 环境变量")

        # 注册所有 Robot_Module 的工具函数
        register_all_modules()

        # 从 Robot_Module 获取工具定义
        self.tools = get_tool_definitions()

        # 获取提示词路径
        prompt_path = project_root / "LLM_Module" / "prompts" / "planning_prompt_2d.yaml"

        # 初始化 LLM Agent
        self.llm_agent = LLMAgent(api_key=self.api_key, prompt_path=str(prompt_path))

        # 动态加载并填充提示词
        dynamic_prompt = self._load_dynamic_prompt(prompt_path, self.tools)
        self.llm_agent.planning_prompt_template = dynamic_prompt

        # 状态信息
        self.is_processing = False
        self.task_history: List[Dict[str, Any]] = []

    def _load_dynamic_prompt(self, prompt_path: Path, tools: List[Dict]) -> str:
        """加载并动态填充提示词"""
        import yaml

        with open(prompt_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        prompt_template = data.get("prompt", "")

        # 动态生成配置信息
        robot_config = self._format_robot_config(tools)
        available_skills = self._format_available_skills(tools)

        # 填充模板
        prompt = prompt_template.format(
            robot_config=robot_config,
            available_skills=available_skills,
            user_input="{user_input}"
        )

        return prompt

    def _format_robot_config(self, tools: List[Dict]) -> str:
        """格式化机器人配置信息"""
        config_lines = ["机器人类型: 2D仿真机器人（差速驱动）"]
        config_lines.append("\n可用技能:")

        for tool in tools:
            func = tool.get("function", {})
            name = func.get("name", "")
            desc = func.get("description", "")
            params = func.get("parameters", {}).get("properties", {})

            config_lines.append(f"- {name}({', '.join(params.keys())}): {desc}")

        return "\n".join(config_lines)

    def _format_available_skills(self, tools: List[Dict]) -> str:
        """格式化可用技能列表"""
        skills = []
        for tool in tools:
            func = tool.get("function", {})
            name = func.get("name", "")
            desc = func.get("description", "")
            params = func.get("parameters", {}).get("properties", {})

            param_str = ", ".join([f"{k}: {v.get('type', '')}" for k, v in params.items()])
            skills.append(f"  - {name}({param_str}): {desc}")

        return "\n".join(skills)

    def execute_tool(self, function_name: str, function_args: dict) -> dict:
        """执行 Robot_Module 中的工具函数"""
        skill_func = get_skill_function(function_name)

        if not skill_func:
            return {"error": f"Unknown tool: {function_name}"}

        try:
            # 调用异步技能函数
            result = asyncio.run(skill_func(**function_args))

            # 估算执行时间
            if function_name in ['move_forward', 'move_backward']:
                distance = function_args.get('distance', 1.0)
                speed = function_args.get('speed', 0.3)
                delay = distance / speed if speed > 0 else 0
            elif function_name == 'detect_color_and_act':
                delay = 3.3
            elif function_name == 'turn':
                angle = abs(function_args.get('angle', 90.0))
                angular_speed = function_args.get('angular_speed', 0.5)
                delay = (angle / 180.0 * 3.14159) / angular_speed if angular_speed > 0 else 0
            else:
                delay = 0

            return {"result": result, "delay": delay}
        except Exception as e:
            return {"error": str(e)}

    async def process_command(self, user_input: str, progress_callback: Callable = None) -> Dict[str, Any]:
        """
        处理用户命令

        Args:
            user_input: 用户输入的命令
            progress_callback: 进度回调函数

        Returns:
            包含执行结果的字典
        """
        self.is_processing = True

        try:
            # 创建任务记录
            task_id = f"task_{len(self.task_history) + 1}"
            task_record = {
                "id": task_id,
                "input": user_input,
                "status": "processing",
                "timestamp": self._get_timestamp(),
                "steps": []
            }

            # 自定义执行函数，添加进度回调
            def execute_tool_with_callback(function_name: str, function_args: dict) -> dict:
                if progress_callback:
                    progress_callback({
                        "type": "step_start",
                        "tool": function_name,
                        "args": function_args
                    })

                result = self.execute_tool(function_name, function_args)

                if progress_callback:
                    progress_callback({
                        "type": "step_complete",
                        "tool": function_name,
                        "result": result
                    })

                # 记录步骤
                task_record["steps"].append({
                    "tool": function_name,
                    "args": function_args,
                    "result": result
                })

                return result

            # 执行双层 LLM 流程
            results = self.llm_agent.run_pipeline(
                user_input=user_input,
                tools=self.tools,
                execute_tool_fn=execute_tool_with_callback
            )

            # 更新任务记录
            success_count = sum(1 for r in results if r.get("success"))
            task_record["status"] = "completed" if success_count == len(results) else "partial"
            task_record["summary"] = f"{success_count}/{len(results)} 个任务成功"
            self.task_history.append(task_record)

            return {
                "success": True,
                "task_id": task_id,
                "results": results,
                "summary": task_record["summary"],
                "steps": task_record["steps"]
            }

        except Exception as e:
            task_record["status"] = "failed"
            task_record["error"] = str(e)
            self.task_history.append(task_record)

            return {
                "success": False,
                "task_id": task_id,
                "error": str(e)
            }
        finally:
            self.is_processing = False

    def _get_timestamp(self) -> str:
        """获取当前时间戳"""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def get_available_tools(self) -> List[Dict]:
        """获取可用工具列表"""
        return self.tools

    def get_task_history(self) -> List[Dict]:
        """获取任务历史"""
        return self.task_history

    def get_status(self) -> Dict:
        """获取服务状态"""
        return {
            "is_processing": self.is_processing,
            "total_tasks": len(self.task_history),
            "api_configured": bool(self.api_key),
            "available_tools": len(self.tools)
        }


# 单例实例
_service_instance = None


def get_robot_service() -> RobotService:
    """获取机器人服务单例"""
    global _service_instance
    if _service_instance is None:
        _service_instance = RobotService()
    return _service_instance
