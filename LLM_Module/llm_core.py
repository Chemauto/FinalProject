#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Core - 双层LLM架构核心模块 (兼容层)

包含任务规划和任务执行的通用逻辑
内部使用新的模块化架构（HighLevelLLM + LowLevelLLM + AdaptiveController）
"""
import os
import sys
import json
import yaml
from openai import OpenAI
from typing import Callable, Dict, List, Any

# 导入新的模块化架构
from .high_level_llm import HighLevelLLM
from .low_level_llm import LowLevelLLM, ExecutionStatus
from .task_queue import TaskQueue, Task, TaskStatus
from .adaptive_controller import AdaptiveController


class LLMAgent:
    """
    双层LLM代理 (兼容层)

    这是旧版本的LLMAgent类，现在内部使用新的模块化架构：
    - HighLevelLLM: 任务规划
    - LowLevelLLM: 执行控制
    - AdaptiveController: 自适应控制（可选）

    保持向后兼容，现有代码无需修改即可使用新功能。
    """

    def __init__(self,
                 api_key: str,
                 base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                 prompt_path: str = None,
                 enable_adaptive: bool = False):
        """
        初始化LLM代理

        Args:
            api_key: API密钥
            base_url: API基础URL
            prompt_path: 规划提示词文件路径
            enable_adaptive: 是否启用自适应控制（重新规划功能）
        """
        self.api_key = api_key
        self.base_url = base_url
        self.enable_adaptive = enable_adaptive

        # 初始化新的模块化架构
        self.high_level_llm = HighLevelLLM(
            api_key=api_key,
            base_url=base_url,
            prompt_path=prompt_path
        )

        self.low_level_llm = LowLevelLLM(
            api_key=api_key,
            base_url=base_url
        )

        # 可选：初始化自适应控制器
        if enable_adaptive:
            from .execution_monitor import ExecutionMonitor
            from .adaptive_controller import AdaptiveController

            self.adaptive_controller = AdaptiveController(
                high_level_llm=self.high_level_llm,
                low_level_llm=self.low_level_llm,
                execution_monitor=ExecutionMonitor()
            )
        else:
            self.adaptive_controller = None

        # 兼容旧代码：保存prompt_path
        self.prompt_path = prompt_path
        self._planning_prompt_template = None

    @property
    def client(self):
        """获取OpenAI客户端（转发到high_level_llm）"""
        return self.high_level_llm.client

    @property
    def model(self):
        """获取模型名称（转发到high_level_llm）"""
        return self.high_level_llm.model

    @property
    def planning_prompt_template(self):
        """获取规划提示词模板"""
        return self.high_level_llm.prompt_template

    @planning_prompt_template.setter
    def planning_prompt_template(self, value):
        """设置规划提示词模板（会更新到high_level_llm）"""
        self.high_level_llm.prompt_template = value
        self._planning_prompt_template = value

    def load_prompt(self, prompt_path: str) -> str:
        """
        从YAML文件加载规划Prompt (兼容方法)

        已弃用：请直接使用HighLevelLLM类
        """
        if not prompt_path or not os.path.exists(prompt_path):
            print("⚠️ 警告: Prompt文件路径未提供或不存在，将使用默认的内置Prompt。")
            return "你是一个机器人任务规划助手。请将用户的复杂指令分解为简单的子任务。用户输入：{user_input}"

        try:
            with open(prompt_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                return data.get("prompt", "")
        except Exception as e:
            print(f"❌ 错误: 加载Prompt文件失败: {e}")
            return ""

    def plan_tasks(self, user_input: str, tools: List[Dict]) -> List[Dict]:
        """
        上层LLM：将用户输入分解为子任务序列 (兼容方法)

        已弃用：请直接使用HighLevelLLM.plan_tasks()

        Args:
            user_input: 用户输入
            tools: 工具列表

        Returns:
            任务列表
        """
        # 提取技能名称
        available_skills = [tool.get("function", {}).get("name", "unknown") for tool in tools]

        # 调用新的HighLevelLLM
        return self.high_level_llm.plan_tasks(
            user_input=user_input,
            available_skills=available_skills
        )

    def execute_single_task(self,
                            task_description: str,
                            tools: List[Dict],
                            execute_tool_fn: Callable,
                            previous_result: Any = None) -> Dict:
        """
        下层LLM：执行单个子任务 (兼容方法)

        已弃用：请直接使用LowLevelLLM.execute_task()

        Args:
            task_description: 任务描述
            tools: 可用工具列表
            execute_tool_fn: 工具执行函数
            previous_result: 上一步的执行结果

        Returns:
            执行结果
        """
        # 调用新的LowLevelLLM
        result = self.low_level_llm.execute_task(
            task_description=task_description,
            tools=tools,
            execute_tool_fn=execute_tool_fn,
            previous_result=previous_result
        )

        # 转换返回格式以兼容旧代码
        if result.get("status") == ExecutionStatus.SUCCESS.value:
            return {
                "success": True,
                "action": result.get("action"),
                "task": result.get("task"),
                "result": result.get("result")
            }
        else:
            return {
                "success": False,
                "error": result.get("error"),
                "task": result.get("task")
            }

    def run_pipeline(self,
                     user_input: str,
                     tools: List[Dict],
                     execute_tool_fn: Callable) -> List[Dict]:
        """
        运行完整的双层LLM流程 (兼容方法)

        新功能：如果enable_adaptive=True，将使用自适应控制器

        Args:
            user_input: 用户输入
            tools: 可用工具列表
            execute_tool_fn: 工具执行函数

        Returns:
            执行结果列表
        """
        print("\n" + "█"*60)
        print(f"📥 [用户输入] {user_input}")
        print("█"*60)

        # 如果启用自适应控制，使用新的AdaptiveController
        if self.enable_adaptive and self.adaptive_controller:
            import asyncio

            # 提取技能名称
            available_skills = [tool.get("function", {}).get("name", "unknown") for tool in tools]

            # 运行异步控制器
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

            results = loop.run_until_complete(
                self.adaptive_controller.run(
                    user_input=user_input,
                    tools=tools,
                    execute_tool_fn=execute_tool_fn,
                    available_skills=available_skills
                )
            )

            return results

        # 否则，使用旧的同步流程（向后兼容）
        try:
            tasks = self.plan_tasks(user_input, tools)

            if not tasks:
                return []

            print("\n" + "█"*60)
            print("🚀 [开始执行] 按顺序执行子任务")
            print("█"*60)

            results = []
            previous_result = None

            for idx, task in enumerate(tasks, 1):
                print(f"\n【步骤 {idx}/{len(tasks)}】")
                result = self.execute_single_task(task["task"], tools, execute_tool_fn, previous_result)
                results.append(result)

                # 保存结果供下一步使用
                if result.get("success") and result.get("result"):
                    previous_result = result["result"].get("result")
                else:
                    previous_result = None

                if not result.get("success"):
                    print(f"\n⚠️  [警告] 步骤 {idx} 失败，但继续执行后续任务")

            print("\n" + "█"*60)
            print("✅ [执行完成] 任务总结")
            print("█"*60)

            for idx, (task, result) in enumerate(zip(tasks, results), 1):
                status = "✅ 成功" if result.get("success") else "❌ 失败"
                print(f"  {idx}. {task['task']} - {status}")

            return results

        except Exception as e:
            print(f"\n❌ [错误] {type(e).__name__}: {e}")
            import traceback
            traceback.print_exc()
            return []


# 便捷函数（向后兼容）
def create_llm_agent(api_key: str,
                     base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                     prompt_path: str = None,
                     enable_adaptive: bool = False) -> LLMAgent:
    """
    创建LLM代理实例

    Args:
        api_key: API密钥
        base_url: API基础URL
        prompt_path: 规划提示词文件路径
        enable_adaptive: 是否启用自适应控制（重新规划功能）

    Returns:
        LLMAgent实例
    """
    return LLMAgent(
        api_key=api_key,
        base_url=base_url,
        prompt_path=prompt_path,
        enable_adaptive=enable_adaptive
    )
